// Package opensky fetches live ADS-B state vectors from the OpenSky Network.
//
// Two properties of the API shape most of this file. Authentication is OAuth2
// client credentials — username and password stopped working in March 2026 —
// so the client carries a bearer token and renews it behind your back. And a
// state vector arrives as a positional JSON array of mixed types rather than an
// object, with most fields nullable, which is why StateVector decodes itself by
// index and uses pointers for anything that can legitimately be absent.
//
// Nothing here filters or interprets: a vector with no position still comes
// back, because deciding what is worth tracking belongs upstream.
package opensky

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/track"
)

const (
	StatesURL = "https://opensky-network.org/api/states/all"
	TokenURL  = "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token"

	// Tokens last about 30 minutes. Renewing early costs nothing and avoids
	// spending a round trip discovering the token died in flight.
	tokenLeeway = 2 * time.Minute

	// A bounded region returns a few hundred aircraft; anything past this is a
	// malfunction, and reading it into memory is how a poller falls over.
	maxBody = 32 << 20

	// Position variances in m², the diagonal of R. ADS-B is GNSS-derived, so
	// its error is a circle rather than an ellipse. MLAT is inferred by the
	// network from arrival timing and is far worse, so it gets its own figure
	// instead of being passed off as a cooperative report.
	adsbVarianceM2 = 25.0 * 25.0
	mlatVarianceM2 = 200.0 * 200.0
)

// Position sources, as reported at index 16. Worth filtering on if you want
// only genuinely cooperative reports: MLAT positions are inferred by the
// network from timing, not broadcast by the aircraft.
const (
	SourceADSB    = 0
	SourceASTERIX = 1
	SourceMLAT    = 2
	SourceFLARM   = 3
)

// Client is safe for concurrent use. The zero value is not usable; call New or
// NewAnonymous.
type Client struct {
	HTTP *http.Client

	id     string
	secret string

	mu      sync.Mutex
	token   string
	expires time.Time
}

// New returns a client authenticating with credentials from an API client
// created on your OpenSky account page.
func New(clientID, clientSecret string) *Client {
	return &Client{
		HTTP:   &http.Client{Timeout: 30 * time.Second},
		id:     clientID,
		secret: clientSecret,
	}
}

// NewAnonymous returns a client with no credentials. Useful for getting
// something on screen before registering, but the allowance is roughly a tenth
// of an authenticated one and the resolution is coarser.
func NewAnonymous() *Client {
	return &Client{HTTP: &http.Client{Timeout: 30 * time.Second}}
}

// Box is a latitude/longitude bounding box in degrees.
type Box struct {
	LatMin, LonMin float64
	LatMax, LonMax float64
}

// Snapshot is one response. Time is the server's own timestamp for the
// snapshot, which is the one to trust: it is not when the request returned, and
// individual vectors carry their own, older, contact times.
type Snapshot struct {
	Time    time.Time
	Vectors []StateVector
}

// StateVector is one aircraft as OpenSky last saw it. Pointer fields are the
// ones the API may return as null, and null is common — an aircraft can be
// known without a current position. Distances are metres, speeds metres per
// second, angles degrees.
type StateVector struct {
	ICAO24        string // lowercase 24-bit address; the identity the tracker keys on
	Callsign      string // trailing padding already trimmed; empty if absent
	OriginCountry string

	// TimePosition is when the position was last updated, LastContact when any
	// message was last received. They differ, and for tracking you want the
	// first one.
	TimePosition *time.Time
	LastContact  time.Time

	Longitude *float64
	Latitude  *float64

	// BaroAltitude is pressure altitude and GeoAltitude is GPS-derived. Both
	// can be absent independently.
	BaroAltitude *float64
	GeoAltitude  *float64

	OnGround     bool
	Velocity     *float64 // over the ground
	TrueTrack    *float64 // degrees clockwise from north
	VerticalRate *float64 // positive climbing

	Squawk         string
	SPI            bool
	PositionSource int
	Category       *int // only populated when the extended flag is set
}

// RateLimitError reports that the daily or hourly credit allowance is spent.
// RetryAfter is zero when the server did not say.
type RateLimitError struct {
	RetryAfter time.Duration
}

func (e *RateLimitError) Error() string {
	if e.RetryAfter > 0 {
		return fmt.Sprintf("opensky: rate limited, retry after %s", e.RetryAfter)
	}
	return "opensky: rate limited"
}

// States fetches every aircraft currently known inside box, or worldwide when
// box is nil.
func (c *Client) States(ctx context.Context, box *Box) (Snapshot, error) {
	u, err := url.Parse(StatesURL)
	if err != nil {
		return Snapshot{}, err
	}
	if box != nil {
		q := u.Query()
		q.Set("lamin", strconv.FormatFloat(box.LatMin, 'f', -1, 64))
		q.Set("lomin", strconv.FormatFloat(box.LonMin, 'f', -1, 64))
		q.Set("lamax", strconv.FormatFloat(box.LatMax, 'f', -1, 64))
		q.Set("lomax", strconv.FormatFloat(box.LonMax, 'f', -1, 64))
		u.RawQuery = q.Encode()
	}

	body, err := c.get(ctx, u.String())
	if err != nil {
		return Snapshot{}, err
	}

	var payload struct {
		Time   int64         `json:"time"`
		States []StateVector `json:"states"`
	}
	if err := json.Unmarshal(body, &payload); err != nil {
		return Snapshot{}, fmt.Errorf("opensky: decoding states: %w", err)
	}

	return Snapshot{
		Time:    time.Unix(payload.Time, 0).UTC(),
		Vectors: payload.States,
	}, nil
}

// get performs an authenticated GET, renewing the token once if the server
// rejects it. A token can die earlier than its stated expiry, so a single 401
// is treated as staleness rather than as bad credentials.
func (c *Client) get(ctx context.Context, u string) ([]byte, error) {
	for attempt := range 2 {
		req, err := http.NewRequestWithContext(ctx, http.MethodGet, u, nil)
		if err != nil {
			return nil, err
		}

		if c.id != "" {
			tok, err := c.bearer(ctx)
			if err != nil {
				return nil, err
			}
			req.Header.Set("Authorization", "Bearer "+tok)
		}

		resp, err := c.HTTP.Do(req)
		if err != nil {
			return nil, fmt.Errorf("opensky: fetching states: %w", err)
		}
		body, readErr := io.ReadAll(io.LimitReader(resp.Body, maxBody))
		resp.Body.Close()
		if readErr != nil {
			return nil, fmt.Errorf("opensky: reading response: %w", readErr)
		}

		switch resp.StatusCode {
		case http.StatusOK:
			return body, nil

		case http.StatusUnauthorized:
			if attempt == 0 && c.id != "" {
				c.discard()
				continue
			}
			return nil, fmt.Errorf("opensky: unauthorized: %s", snippet(body))

		case http.StatusTooManyRequests:
			return nil, &RateLimitError{RetryAfter: retryAfter(resp)}

		default:
			return nil, fmt.Errorf("opensky: %s: %s", resp.Status, snippet(body))
		}
	}

	return nil, fmt.Errorf("opensky: unauthorized after renewing the token")
}

// bearer returns a live access token, fetching one if the current is missing or
// close to expiry.
func (c *Client) bearer(ctx context.Context) (string, error) {
	c.mu.Lock()
	defer c.mu.Unlock()

	if c.token != "" && time.Now().Before(c.expires.Add(-tokenLeeway)) {
		return c.token, nil
	}

	form := url.Values{
		"grant_type":    {"client_credentials"},
		"client_id":     {c.id},
		"client_secret": {c.secret},
	}
	req, err := http.NewRequestWithContext(ctx, http.MethodPost, TokenURL, strings.NewReader(form.Encode()))
	if err != nil {
		return "", err
	}
	req.Header.Set("Content-Type", "application/x-www-form-urlencoded")

	resp, err := c.HTTP.Do(req)
	if err != nil {
		return "", fmt.Errorf("opensky: requesting token: %w", err)
	}
	body, readErr := io.ReadAll(io.LimitReader(resp.Body, 1<<20))
	resp.Body.Close()
	if readErr != nil {
		return "", fmt.Errorf("opensky: reading token response: %w", readErr)
	}
	// The response body echoes nothing secret, but the request form does, so
	// errors here quote the reply and never the request.
	if resp.StatusCode != http.StatusOK {
		return "", fmt.Errorf("opensky: token request failed: %s: %s", resp.Status, snippet(body))
	}

	var payload struct {
		AccessToken string `json:"access_token"`
		ExpiresIn   int64  `json:"expires_in"`
	}
	if err := json.Unmarshal(body, &payload); err != nil {
		return "", fmt.Errorf("opensky: decoding token: %w", err)
	}
	if payload.AccessToken == "" {
		return "", fmt.Errorf("opensky: token response carried no access_token")
	}

	c.token = payload.AccessToken
	c.expires = time.Now().Add(time.Duration(payload.ExpiresIn) * time.Second)

	return c.token, nil
}

func (c *Client) discard() {
	c.mu.Lock()
	c.token = ""
	c.expires = time.Time{}
	c.mu.Unlock()
}

// UnmarshalJSON reads the positional array OpenSky sends in place of an object.
// Index 12 (sensors) is deliberately skipped: it is null unless you own the
// receivers that produced the report.
func (s *StateVector) UnmarshalJSON(data []byte) error {
	var raw []json.RawMessage
	if err := json.Unmarshal(data, &raw); err != nil {
		return fmt.Errorf("state vector: want an array: %w", err)
	}
	// Index 17 (category) only appears when the extended flag is set, so 17
	// entries is a complete vector and 18 is a complete extended one.
	if len(raw) < 17 {
		return fmt.Errorf("state vector: got %d fields, want at least 17", len(raw))
	}

	d := decoder{raw: raw}

	s.ICAO24 = d.str(0, "icao24")
	// Callsigns are padded out to eight characters on the wire.
	s.Callsign = strings.TrimSpace(d.str(1, "callsign"))
	s.OriginCountry = d.str(2, "origin_country")
	s.TimePosition = d.unix(3, "time_position")
	s.LastContact = d.unixValue(4, "last_contact")
	s.Longitude = d.f64(5, "longitude")
	s.Latitude = d.f64(6, "latitude")
	s.BaroAltitude = d.f64(7, "baro_altitude")
	s.OnGround = d.boolean(8, "on_ground")
	s.Velocity = d.f64(9, "velocity")
	s.TrueTrack = d.f64(10, "true_track")
	s.VerticalRate = d.f64(11, "vertical_rate")
	s.GeoAltitude = d.f64(13, "geo_altitude")
	s.Squawk = d.str(14, "squawk")
	s.SPI = d.boolean(15, "spi")
	s.PositionSource = d.intValue(16, "position_source")
	if len(raw) > 17 {
		s.Category = d.integer(17, "category")
	}

	return d.err
}

// decoder pulls typed values out of the positional array, holding the first
// failure so the field list above reads as a list rather than as error
// handling.
type decoder struct {
	raw []json.RawMessage
	err error
}

func (d *decoder) fail(name string, err error) {
	if d.err == nil {
		d.err = fmt.Errorf("state vector: field %s: %w", name, err)
	}
}

func isNull(raw json.RawMessage) bool {
	return len(raw) == 0 || bytes.Equal(bytes.TrimSpace(raw), []byte("null"))
}

func (d *decoder) str(i int, name string) string {
	if isNull(d.raw[i]) {
		return ""
	}
	var v string
	if err := json.Unmarshal(d.raw[i], &v); err != nil {
		d.fail(name, err)
		return ""
	}
	return v
}

func (d *decoder) boolean(i int, name string) bool {
	if isNull(d.raw[i]) {
		return false
	}
	var v bool
	if err := json.Unmarshal(d.raw[i], &v); err != nil {
		d.fail(name, err)
		return false
	}
	return v
}

func (d *decoder) f64(i int, name string) *float64 {
	if isNull(d.raw[i]) {
		return nil
	}
	var v float64
	if err := json.Unmarshal(d.raw[i], &v); err != nil {
		d.fail(name, err)
		return nil
	}
	return &v
}

func (d *decoder) integer(i int, name string) *int {
	if isNull(d.raw[i]) {
		return nil
	}
	var v int
	if err := json.Unmarshal(d.raw[i], &v); err != nil {
		d.fail(name, err)
		return nil
	}
	return &v
}

func (d *decoder) intValue(i int, name string) int {
	if v := d.integer(i, name); v != nil {
		return *v
	}
	return 0
}

func (d *decoder) unix(i int, name string) *time.Time {
	if isNull(d.raw[i]) {
		return nil
	}
	var v int64
	if err := json.Unmarshal(d.raw[i], &v); err != nil {
		d.fail(name, err)
		return nil
	}
	t := time.Unix(v, 0).UTC()
	return &t
}

func (d *decoder) unixValue(i int, name string) time.Time {
	if t := d.unix(i, name); t != nil {
		return *t
	}
	return time.Time{}
}

func retryAfter(resp *http.Response) time.Duration {
	secs, err := strconv.Atoi(resp.Header.Get("Retry-After"))
	if err != nil || secs < 0 {
		return 0
	}
	return time.Duration(secs) * time.Second
}

func snippet(body []byte) string {
	const max = 200
	s := strings.TrimSpace(string(body))
	if len(s) > max {
		return s[:max] + "..."
	}
	return s
}

// Reject says why a state vector produced no measurement. The zero value means
// it produced one, so a caller reads `if r != Accepted`. These are routine
// rather than errors — every response carries some — which is why they are
// counted instead of returned as one.
type Reject int

const (
	Accepted Reject = iota

	// NoPosition: the network knows the aircraft but holds no fix for it.
	NoPosition

	// NoPositionTime: a fix with no time is unusable. Substituting any other
	// timestamp would tell the filter that a stale position is current, and
	// nothing downstream can detect that.
	NoPositionTime

	// TooOld: the fix predates the snapshot by more than the caller allows.
	TooOld

	// OnGround: taxiing aircraft sit close enough together to be their own
	// association problem, and are dropped when the caller asks.
	OnGround

	numRejects
)

func (r Reject) String() string {
	switch r {
	case Accepted:
		return "accepted"
	case NoPosition:
		return "no-position"
	case NoPositionTime:
		return "no-position-time"
	case TooOld:
		return "too-old"
	case OnGround:
		return "on-ground"
	}
	return "unknown"
}

// ToMeasurement projects one state vector into the tracking frame. It rejects
// only what it cannot convert; dropping aircraft for being stale or on the
// ground is policy and lives in Snapshot.Measurements.
func (s *StateVector) ToMeasurement(f geo.Frame) (track.Measurement, Reject) {
	// A missing coordinate is not zero. Latitude 0, longitude 0 is a real
	// place in the Gulf of Guinea, and defaulting to it would hand the tracker
	// a confident phantom several thousand kilometres outside the airspace.
	if s.Latitude == nil || s.Longitude == nil {
		return track.Measurement{}, NoPosition
	}
	if s.TimePosition == nil {
		return track.Measurement{}, NoPositionTime
	}

	pos := geo.Geodetic{Lat: *s.Latitude, Lon: *s.Longitude}

	switch {
	case s.GeoAltitude != nil:
		pos.Alt = *s.GeoAltitude
	case s.BaroAltitude != nil:
		pos.Alt = *s.BaroAltitude
	}

	variance := adsbVarianceM2
	if s.PositionSource == SourceMLAT {
		variance = mlatVarianceM2
	}

	return track.Measurement{
		TimeStamp: *s.TimePosition,
		Position:  f.ToENU(pos),
		Identity:  s.ICAO24,
		R:         [2][2]float64{{variance, 0}, {0, variance}},
	}, Accepted
}

// Options is the caller's policy on what is worth tracking. The zero value
// keeps every vector that can be converted at all.
type Options struct {
	// MaxAge drops fixes older than this relative to the snapshot's own time.
	// Zero keeps every age. An old fix is not wrong — the measurement carries
	// its own timestamp and the filter accounts for it — but past some point
	// the aircraft has manoeuvred and the constant-velocity model bridging
	// that gap is fiction.
	MaxAge time.Duration

	// SkipOnGround drops aircraft reporting themselves on the ground.
	SkipOnGround bool
}

// Tally counts vectors by outcome, indexed by Reject. Accepted holds the
// number that produced a measurement.
type Tally [numRejects]int

func (t Tally) String() string {
	parts := make([]string, 0, numRejects)
	for r := range numRejects {
		if t[r] > 0 {
			parts = append(parts, fmt.Sprintf("%s=%d", r, t[r]))
		}
	}
	if len(parts) == 0 {
		return "empty"
	}
	return strings.Join(parts, " ")
}

// Measurements projects every usable vector in the snapshot and reports what
// was dropped and why. The tally is the point of the second return: a climbing
// no-position count is a feed problem and a climbing too-old count is a polling
// problem, and neither is visible from the measurements alone.
func (s Snapshot) Measurements(f geo.Frame, opts Options) ([]track.Measurement, Tally) {
	var tally Tally
	out := make([]track.Measurement, 0, len(s.Vectors))

	for i := range s.Vectors {
		v := &s.Vectors[i]

		if opts.SkipOnGround && v.OnGround {
			tally[OnGround]++
			continue
		}

		m, reason := v.ToMeasurement(f)
		if reason != Accepted {
			tally[reason]++
			continue
		}

		if opts.MaxAge > 0 && s.Time.Sub(m.TimeStamp) > opts.MaxAge {
			tally[TooOld]++
			continue
		}

		tally[Accepted]++
		out = append(out, m)
	}

	return out, tally
}
