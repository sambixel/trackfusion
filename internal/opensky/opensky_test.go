package opensky

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/track"
)

// A response shaped the way the API actually sends one: positional arrays, a
// padded callsign, an aircraft with no position fix at all, and no category
// because the extended flag was not set.
const statesJSON = `{
  "time": 1756704000,
  "states": [
    ["4b1806","SWR123  ","Switzerland",1756703998,1756703999,8.5456,47.4502,10972.8,false,231.5,87.3,0.33,null,11017.25,"1000",false,0],
    ["a0f1bb",null,"United States",null,1756703990,null,null,null,true,null,null,null,null,null,null,false,0]
  ]
}`

func decodeStates(t *testing.T, body string) Snapshot {
	t.Helper()
	var payload struct {
		Time   int64         `json:"time"`
		States []StateVector `json:"states"`
	}
	if err := json.Unmarshal([]byte(body), &payload); err != nil {
		t.Fatalf("decode: %v", err)
	}
	return Snapshot{Time: time.Unix(payload.Time, 0).UTC(), Vectors: payload.States}
}

func TestDecodesAPopulatedVector(t *testing.T) {
	got := decodeStates(t, statesJSON)

	if len(got.Vectors) != 2 {
		t.Fatalf("want 2 vectors, got %d", len(got.Vectors))
	}
	v := got.Vectors[0]

	if v.ICAO24 != "4b1806" {
		t.Errorf("icao24 = %q", v.ICAO24)
	}
	// The wire format pads callsigns out to eight characters.
	if v.Callsign != "SWR123" {
		t.Errorf("callsign = %q, want the padding trimmed", v.Callsign)
	}
	if v.Latitude == nil || *v.Latitude != 47.4502 {
		t.Errorf("latitude = %v", v.Latitude)
	}
	if v.Longitude == nil || *v.Longitude != 8.5456 {
		t.Errorf("longitude = %v", v.Longitude)
	}
	if v.BaroAltitude == nil || *v.BaroAltitude != 10972.8 {
		t.Errorf("baro altitude = %v", v.BaroAltitude)
	}
	if v.GeoAltitude == nil || *v.GeoAltitude != 11017.25 {
		t.Errorf("geo altitude = %v", v.GeoAltitude)
	}
	if v.OnGround {
		t.Error("on ground should be false")
	}
	// vertical_rate is 0.33 and must not be confused with absent.
	if v.VerticalRate == nil || *v.VerticalRate != 0.33 {
		t.Errorf("vertical rate = %v", v.VerticalRate)
	}
	if v.TimePosition == nil || v.TimePosition.Unix() != 1756703998 {
		t.Errorf("time position = %v", v.TimePosition)
	}
	if v.LastContact.Unix() != 1756703999 {
		t.Errorf("last contact = %v", v.LastContact)
	}
	if v.PositionSource != SourceADSB {
		t.Errorf("position source = %d", v.PositionSource)
	}
	if v.Category != nil {
		t.Errorf("category should be absent without the extended flag, got %v", v.Category)
	}
}

// An aircraft known to the network but without a current fix. This is common,
// and the distinction that matters is absent versus zero: a nil latitude is not
// the equator.
func TestDecodesAVectorWithNoPosition(t *testing.T) {
	got := decodeStates(t, statesJSON)
	v := got.Vectors[1]

	if v.Callsign != "" {
		t.Errorf("callsign = %q, want empty", v.Callsign)
	}
	if v.Latitude != nil || v.Longitude != nil {
		t.Errorf("want no position, got %v %v", v.Latitude, v.Longitude)
	}
	if v.BaroAltitude != nil || v.GeoAltitude != nil {
		t.Error("want no altitude")
	}
	if v.TimePosition != nil {
		t.Errorf("want no position time, got %v", v.TimePosition)
	}
	if !v.OnGround {
		t.Error("on ground should be true")
	}
	// Still identifiable, which is why nothing here is filtered out for you.
	if v.ICAO24 != "a0f1bb" {
		t.Errorf("icao24 = %q", v.ICAO24)
	}
}

func TestDecodesCategoryWhenExtended(t *testing.T) {
	body := `{"time":1,"states":[["4b1806","X       ","CH",1,2,8.0,47.0,100.0,false,200.0,90.0,0.0,null,110.0,"1000",false,0,3]]}`

	got := decodeStates(t, body)

	if v := got.Vectors[0]; v.Category == nil || *v.Category != 3 {
		t.Errorf("category = %v, want 3", v.Category)
	}
}

func TestRejectsAShortVector(t *testing.T) {
	var v StateVector
	err := v.UnmarshalJSON([]byte(`["4b1806","X","CH",1,2]`))

	if err == nil {
		t.Fatal("want an error for a truncated vector")
	}
	if !strings.Contains(err.Error(), "want at least 17") {
		t.Errorf("error should say what was missing, got %v", err)
	}
}

func TestNullStatesListIsNotAnError(t *testing.T) {
	got := decodeStates(t, `{"time":1756704000,"states":null}`)

	if len(got.Vectors) != 0 {
		t.Errorf("want no vectors, got %d", len(got.Vectors))
	}
	if got.Time.Unix() != 1756704000 {
		t.Errorf("time = %v", got.Time)
	}
}

// The full path against a stand-in server: token first, then the states call
// carrying it, and the bounding box on the query string.
func TestFetchesATokenThenTheStates(t *testing.T) {
	var tokens, states atomic.Int32
	var sawAuth, sawQuery string

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		switch {
		case strings.Contains(r.URL.Path, "token"):
			tokens.Add(1)
			if err := r.ParseForm(); err != nil {
				t.Errorf("token form: %v", err)
			}
			if got := r.PostForm.Get("grant_type"); got != "client_credentials" {
				t.Errorf("grant_type = %q", got)
			}
			if got := r.PostForm.Get("client_id"); got != "id" {
				t.Errorf("client_id = %q", got)
			}
			w.Write([]byte(`{"access_token":"tok-1","expires_in":1800,"token_type":"Bearer"}`))
		default:
			states.Add(1)
			sawAuth = r.Header.Get("Authorization")
			sawQuery = r.URL.RawQuery
			w.Write([]byte(statesJSON))
		}
	}))
	defer srv.Close()

	c := newTestClient(srv, "id", "secret")
	box := &Box{LatMin: 47.0, LonMin: 8.0, LatMax: 48.0, LonMax: 9.0}

	got, err := c.States(context.Background(), box)
	if err != nil {
		t.Fatalf("States: %v", err)
	}

	if len(got.Vectors) != 2 {
		t.Errorf("want 2 vectors, got %d", len(got.Vectors))
	}
	if sawAuth != "Bearer tok-1" {
		t.Errorf("authorization = %q", sawAuth)
	}
	for _, want := range []string{"lamin=47", "lomin=8", "lamax=48", "lomax=9"} {
		if !strings.Contains(sawQuery, want) {
			t.Errorf("query %q missing %q", sawQuery, want)
		}
	}

	// A second call must reuse the token rather than pay for another round trip.
	if _, err := c.States(context.Background(), box); err != nil {
		t.Fatalf("second States: %v", err)
	}
	if tokens.Load() != 1 {
		t.Errorf("want 1 token fetch, got %d", tokens.Load())
	}
	if states.Load() != 2 {
		t.Errorf("want 2 states calls, got %d", states.Load())
	}
}

// A token can die before its stated expiry. One rejection should be read as
// staleness and retried, not surfaced as bad credentials.
func TestRenewsOnceWhenTheTokenIsRejected(t *testing.T) {
	var tokens, attempts atomic.Int32

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if strings.Contains(r.URL.Path, "token") {
			n := tokens.Add(1)
			w.Write([]byte(`{"access_token":"tok-` + string(rune('0'+n)) + `","expires_in":1800}`))
			return
		}
		if attempts.Add(1) == 1 {
			w.WriteHeader(http.StatusUnauthorized)
			w.Write([]byte(`expired`))
			return
		}
		if got := r.Header.Get("Authorization"); got != "Bearer tok-2" {
			t.Errorf("retry should carry the new token, got %q", got)
		}
		w.Write([]byte(statesJSON))
	}))
	defer srv.Close()

	c := newTestClient(srv, "id", "secret")

	if _, err := c.States(context.Background(), nil); err != nil {
		t.Fatalf("States: %v", err)
	}
	if tokens.Load() != 2 {
		t.Errorf("want the token refetched once, got %d fetches", tokens.Load())
	}
	if attempts.Load() != 2 {
		t.Errorf("want 2 states attempts, got %d", attempts.Load())
	}
}

func TestGivesUpAfterASecondRejection(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if strings.Contains(r.URL.Path, "token") {
			w.Write([]byte(`{"access_token":"tok","expires_in":1800}`))
			return
		}
		w.WriteHeader(http.StatusUnauthorized)
	}))
	defer srv.Close()

	c := newTestClient(srv, "id", "secret")

	if _, err := c.States(context.Background(), nil); err == nil {
		t.Fatal("want an error when the credentials are genuinely bad")
	}
}

func TestRateLimitIsATypedError(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Retry-After", "120")
		w.WriteHeader(http.StatusTooManyRequests)
	}))
	defer srv.Close()

	c := newTestClient(srv, "", "")

	_, err := c.States(context.Background(), nil)

	var limit *RateLimitError
	if !asRateLimit(err, &limit) {
		t.Fatalf("want a RateLimitError, got %v", err)
	}
	if limit.RetryAfter != 2*time.Minute {
		t.Errorf("retry after = %v", limit.RetryAfter)
	}
}

// Without credentials nothing should be sent, and no token call attempted.
func TestAnonymousSendsNoAuthorization(t *testing.T) {
	var tokens atomic.Int32
	var sawAuth string
	sawAuthSet := false

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if strings.Contains(r.URL.Path, "token") {
			tokens.Add(1)
			w.WriteHeader(http.StatusBadRequest)
			return
		}
		sawAuth, sawAuthSet = r.Header.Get("Authorization"), true
		w.Write([]byte(statesJSON))
	}))
	defer srv.Close()

	c := newTestClient(srv, "", "")

	if _, err := c.States(context.Background(), nil); err != nil {
		t.Fatalf("States: %v", err)
	}
	if !sawAuthSet {
		t.Fatal("states was never called")
	}
	if sawAuth != "" {
		t.Errorf("anonymous client sent %q", sawAuth)
	}
	if tokens.Load() != 0 {
		t.Errorf("anonymous client asked for a token %d times", tokens.Load())
	}
}

func TestContextCancellationPropagates(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		<-r.Context().Done()
	}))
	defer srv.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 50*time.Millisecond)
	defer cancel()

	if _, err := newTestClient(srv, "", "").States(ctx, nil); err == nil {
		t.Fatal("want an error when the context expires")
	}
}

// --- projection into the tracking frame ---

func testFrame() geo.Frame {
	return geo.NewFrame(geo.Geodetic{Lat: 47.0, Lon: 8.0, Alt: 0})
}

func TestConvertsAUsableVector(t *testing.T) {
	snap := decodeStates(t, statesJSON)

	m, reason := snap.Vectors[0].ToMeasurement(testFrame())

	if reason != Accepted {
		t.Fatalf("reason = %v, want accepted", reason)
	}
	if m.Identity != "4b1806" {
		t.Errorf("identity = %q, want the icao24 address", m.Identity)
	}
	// The fix time, not the snapshot time: the filter builds dt from this.
	if m.TimeStamp.Unix() != 1756703998 {
		t.Errorf("timestamp = %v, want the position time", m.TimeStamp)
	}
	if m.R != [2][2]float64{{625, 0}, {0, 625}} {
		t.Errorf("R = %v, want an isotropic 25 m", m.R)
	}
	if m.Position.E == 0 || m.Position.N == 0 {
		t.Errorf("position was not projected: %+v", m.Position)
	}
}

// The failure that started this: a missing coordinate is not zero, and must
// never reach the tracker as a confident position off West Africa.
func TestRejectsAVectorWithNoPosition(t *testing.T) {
	snap := decodeStates(t, statesJSON)

	m, reason := snap.Vectors[1].ToMeasurement(testFrame())

	if reason != NoPosition {
		t.Fatalf("reason = %v, want no-position", reason)
	}
	if m != (track.Measurement{}) {
		t.Errorf("a rejected vector must carry nothing usable, got %+v", m)
	}
}

// A position with no time is unusable: any substitute timestamp would claim a
// stale fix is current, and nothing downstream could tell.
func TestRejectsAPositionWithNoTime(t *testing.T) {
	body := `{"time":1,"states":[["dead01","X","C",null,2,8.1,47.1,1000.0,false,200,90,0,null,null,"1000",false,0]]}`
	snap := decodeStates(t, body)

	_, reason := snap.Vectors[0].ToMeasurement(testFrame())

	if reason != NoPositionTime {
		t.Fatalf("reason = %v, want no-position-time", reason)
	}
}

func TestAltitudePrefersGeometricThenFallsBack(t *testing.T) {
	body := `{"time":1,"states":[
	  ["aaa111","A","C",1,2,8.1,47.1,9000.0,false,200,90,0,null,11000.0,"1000",false,0],
	  ["bbb222","B","C",1,2,8.1,47.1,9000.0,false,200,90,0,null,null,"1000",false,0],
	  ["ccc333","C","C",1,2,8.1,47.1,null,false,200,90,0,null,null,"1000",false,0]
	]}`
	snap := decodeStates(t, body)
	f := testFrame()

	var up []float64
	for i := range snap.Vectors {
		m, reason := snap.Vectors[i].ToMeasurement(f)
		if reason != Accepted {
			t.Fatalf("vector %d rejected: %v", i, reason)
		}
		up = append(up, m.Position.U)
	}

	// Geometric altitude wins where both are present, so the first sits higher
	// than the second, which falls back to the pressure reading.
	if !(up[0] > up[1]) {
		t.Errorf("geo altitude should have been preferred: %v vs %v", up[0], up[1])
	}
	// Neither present is still usable; it just sits on the reference surface.
	if up[2] > up[1] {
		t.Errorf("missing altitude should sit lowest, got %v", up[2])
	}
}

// Same feed, different sensor. An MLAT fix is inferred from arrival timing and
// deserves to say so, or the filter will over-trust it.
func TestMLATGetsItsOwnUncertainty(t *testing.T) {
	body := `{"time":1,"states":[["aaa111","A","C",1,2,8.1,47.1,1000.0,false,200,90,0,null,null,"1000",false,2]]}`
	snap := decodeStates(t, body)

	m, reason := snap.Vectors[0].ToMeasurement(testFrame())

	if reason != Accepted {
		t.Fatalf("reason = %v", reason)
	}
	if m.R[0][0] <= 625 {
		t.Errorf("MLAT R = %v, should be well above the ADS-B figure", m.R[0][0])
	}
}

func TestMeasurementsCountsWhatItDropped(t *testing.T) {
	snap := decodeStates(t, statesJSON)

	ms, tally := snap.Measurements(testFrame(), Options{})

	if len(ms) != 1 {
		t.Fatalf("want 1 measurement, got %d", len(ms))
	}
	if tally[Accepted] != 1 || tally[NoPosition] != 1 {
		t.Errorf("tally = %v, want one of each", tally)
	}
	if got := tally.String(); !strings.Contains(got, "no-position=1") {
		t.Errorf("tally should read clearly in a log, got %q", got)
	}
}

func TestMaxAgeDropsStaleFixes(t *testing.T) {
	// Snapshot at 1000; one fix 2s old, one 30s old.
	body := `{"time":1000,"states":[
	  ["fresh1","A","C",998,999,8.1,47.1,1000.0,false,200,90,0,null,null,"1000",false,0],
	  ["stale1","B","C",970,999,8.2,47.2,1000.0,false,200,90,0,null,null,"1000",false,0]
	]}`
	snap := decodeStates(t, body)

	ms, tally := snap.Measurements(testFrame(), Options{MaxAge: 10 * time.Second})

	if len(ms) != 1 || ms[0].Identity != "fresh1" {
		t.Fatalf("want only the fresh fix, got %+v", ms)
	}
	if tally[TooOld] != 1 {
		t.Errorf("tally = %v, want one too-old", tally)
	}

	// Zero MaxAge is not "drop everything".
	all, _ := snap.Measurements(testFrame(), Options{})
	if len(all) != 2 {
		t.Errorf("zero MaxAge should keep every age, got %d", len(all))
	}
}

func TestSkipOnGround(t *testing.T) {
	body := `{"time":1000,"states":[
	  ["flying","A","C",999,999,8.1,47.1,1000.0,false,200,90,0,null,null,"1000",false,0],
	  ["taxiin","B","C",999,999,8.2,47.2,400.0,true,5,90,0,null,null,"1000",false,0]
	]}`
	snap := decodeStates(t, body)

	ms, tally := snap.Measurements(testFrame(), Options{SkipOnGround: true})

	if len(ms) != 1 || ms[0].Identity != "flying" {
		t.Fatalf("want only the airborne aircraft, got %+v", ms)
	}
	if tally[OnGround] != 1 {
		t.Errorf("tally = %v, want one on-ground", tally)
	}
}
