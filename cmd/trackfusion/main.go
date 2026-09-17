// Command trackfusion maintains one set of tracks over a bounded region from
// two sensors: live ADS-B state vectors from the OpenSky Network, and a
// simulated radar over the same airspace.
//
// It runs in one of two modes. Live polls the network and drives the simulated
// radar off what the feed says is up there, writing every scan to a recording
// as it goes. Replay reads such a recording back and runs the identical
// pipeline over it, with no network and no dice, so the same traffic can be put
// through two versions of the tracker and the difference attributed to the
// change rather than to the sky.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"os/signal"
	"sort"
	"syscall"
	"time"

	"github.com/sambixel/trackfusion/internal/assoc"
	"github.com/sambixel/trackfusion/internal/filter"
	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/opensky"
	"github.com/sambixel/trackfusion/internal/radar"
	"github.com/sambixel/trackfusion/internal/record"
	"github.com/sambixel/trackfusion/internal/track"
)

type config struct {
	lat, lon float64
	radiusKM float64

	poll      time.Duration
	sweep     time.Duration
	duration  time.Duration
	maxAge    time.Duration
	truthLife time.Duration

	recordPath string
	replayPath string
	seed       int64

	processNoise float64
	gate         float64
	margin       float64
	maxMisses    int64

	sigmaRange   float64
	sigmaBearing float64
	pd           float64
	clutter      float64

	quiet bool
}

func main() {
	log.SetFlags(0)

	var cfg config
	flag.Float64Var(&cfg.lat, "lat", 51.4700, "latitude of the tracking frame origin")
	flag.Float64Var(&cfg.lon, "lon", -0.4543, "longitude of the tracking frame origin")
	flag.Float64Var(&cfg.radiusKM, "radius", 120, "half-width of the region, km")

	flag.DurationVar(&cfg.poll, "poll", 10*time.Second, "how often to ask OpenSky for state vectors")
	flag.DurationVar(&cfg.sweep, "sweep", 4*time.Second, "simulated radar sweep period")
	flag.DurationVar(&cfg.duration, "for", 0, "stop after this long; zero runs until interrupted")
	flag.DurationVar(&cfg.maxAge, "max-age", 30*time.Second, "discard ADS-B fixes older than this")
	flag.DurationVar(&cfg.truthLife, "truth-life", 60*time.Second,
		"stop simulating radar returns for an aircraft the feed has not mentioned in this long")

	flag.StringVar(&cfg.recordPath, "record", "", "write every scan to this JSONL file")
	flag.StringVar(&cfg.replayPath, "replay", "", "run from this recording instead of the network")
	flag.Int64Var(&cfg.seed, "seed", 1, "radar noise seed; the same seed replays the same returns")

	flag.Float64Var(&cfg.processNoise, "q", 3.0, "process noise, m²/s³: how much manoeuvring the filter expects")
	flag.Float64Var(&cfg.gate, "gate", 13.8, "gate threshold; 13.8 keeps about 99.9% of correct pairings")
	flag.Float64Var(&cfg.margin, "margin", 2.0, "how much worse the runner-up pairing must be to trust the winner")
	flag.Int64Var(&cfg.maxMisses, "max-misses", 4, "scans a track may go unmatched before it is dropped")

	flag.Float64Var(&cfg.sigmaRange, "radar-range-sigma", 40, "radar down-range error, m")
	flag.Float64Var(&cfg.sigmaBearing, "radar-bearing-sigma", 0.5, "radar bearing error, degrees")
	flag.Float64Var(&cfg.pd, "radar-pd", 0.9, "probability the radar sees a given aircraft on a given sweep")
	flag.Float64Var(&cfg.clutter, "radar-clutter", 2, "mean false returns per sweep")

	flag.BoolVar(&cfg.quiet, "quiet", false, "only print the periodic track table, not every scan")
	flag.Parse()

	if err := run(cfg); err != nil {
		log.Fatalf("trackfusion: %v", err)
	}
}

func run(cfg config) error {
	frame := geo.NewFrame(geo.Geodetic{Lat: cfg.lat, Lon: cfg.lon})

	engine := &Engine{
		Filter: filter.Filter{ProcessNoise: cfg.processNoise},
		Store:  track.NewStore(),
		Assoc: assoc.Config{
			GateThreshold:   cfg.gate,
			AmbiguityMargin: cfg.margin,
		},
		MaxMisses: cfg.maxMisses,
	}

	// Interrupt has to reach the recording, not just the process: the writer
	// buffers, so a run killed outright loses its tail and the capture ends
	// mid-scan.
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	if cfg.duration > 0 {
		var cancel context.CancelFunc
		ctx, cancel = context.WithTimeout(ctx, cfg.duration)
		defer cancel()
	}

	if cfg.replayPath != "" {
		return replay(ctx, cfg, engine)
	}
	return live(ctx, cfg, engine, frame)
}

// replay runs the pipeline over a recording. No network, no sensor, no
// randomness: the same file through the same tracker gives the same answer
// every time, and through a changed tracker gives a difference that is the
// change and nothing else.
func replay(ctx context.Context, cfg config, engine *Engine) error {
	f, err := os.Open(cfg.replayPath)
	if err != nil {
		return fmt.Errorf("open recording: %w", err)
	}
	defer f.Close()

	r := record.NewReader(f)
	scans := 0

	for {
		if ctx.Err() != nil {
			break
		}

		s, err := r.Next()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			return err
		}

		scans++
		report(cfg, engine.Step(s))
	}

	log.Printf("replayed %d scans from %s", scans, cfg.replayPath)
	dumpTracks(engine.Store)
	return nil
}

// live polls the network and drives the simulated radar off it.
//
// The radar needs to know what is actually up there, and the only source of
// that here is the same ADS-B feed. So the simulator is told the truth and the
// tracker is not: returns reach the tracker stripped of identity, blurred, and
// mixed with clutter, and whether it can reassemble them is the question.
func live(ctx context.Context, cfg config, engine *Engine, frame geo.Frame) error {
	client := clientFromEnv()
	box := boxAround(cfg.lat, cfg.lon, cfg.radiusKM)

	sensor := radar.New(geo.ENU{}, radar.Config{
		Period:          cfg.sweep,
		MaxRangeM:       cfg.radiusKM * 1000,
		SigmaRangeM:     cfg.sigmaRange,
		SigmaBearingRad: cfg.sigmaBearing * math.Pi / 180,
		Pd:              cfg.pd,
		ClutterRate:     cfg.clutter,
	}, cfg.seed)

	var writer *record.Writer
	if cfg.recordPath != "" {
		f, err := os.Create(cfg.recordPath)
		if err != nil {
			return fmt.Errorf("create recording: %w", err)
		}
		defer f.Close()

		writer = record.NewWriter(f)
		defer func() {
			if err := writer.Close(); err != nil {
				log.Printf("flushing recording: %v", err)
			}
		}()
	}

	step := func(s record.Scan) {
		if writer != nil {
			if err := writer.Write(s); err != nil {
				log.Printf("recording: %v", err)
			}
		}
		report(cfg, engine.Step(s))
	}

	// truth is what the radar is allowed to know, keyed by address so an
	// aircraft persists between polls. Entries age out: an aircraft that has
	// left the feed has probably left the region, and continuing to invent
	// returns for it would have the radar tracking a ghost the tracker cannot
	// ever confirm.
	truth := map[string]radar.Truth{}

	poll := time.NewTicker(cfg.poll)
	defer poll.Stop()
	sweep := time.NewTicker(cfg.sweep)
	defer sweep.Stop()

	fetch := func() {
		reqCtx, cancel := context.WithTimeout(ctx, 15*time.Second)
		defer cancel()

		snap, err := client.States(reqCtx, &box)
		if err != nil {
			var limit *opensky.RateLimitError
			if errors.As(err, &limit) {
				log.Printf("rate limited; next attempt in %v", limit.RetryAfter)
				return
			}
			if ctx.Err() == nil {
				log.Printf("poll: %v", err)
			}
			return
		}

		meas, tally := snap.Measurements(frame, opensky.Options{
			MaxAge:       cfg.maxAge,
			SkipOnGround: true,
		})
		if !cfg.quiet {
			log.Printf("adsb  %s  %s", snap.Time.UTC().Format("15:04:05"), tally)
		}

		updateTruth(truth, snap, frame, cfg.truthLife)

		// ADS-B is cooperative, so each measurement's truth is simply the
		// address it arrived with. Recording it keeps the two sensors' scans
		// the same shape.
		ids := make([]string, len(meas))
		for i, m := range meas {
			ids[i] = m.Identity
		}
		step(record.NewScan(record.ADSB, snap.Time, meas, ids))
	}

	fetch()

	for {
		select {
		case <-ctx.Done():
			log.Printf("stopping")
			dumpTracks(engine.Store)
			return nil

		case <-poll.C:
			fetch()

		case now := <-sweep.C:
			at := now.UTC()

			live := make([]radar.Truth, 0, len(truth))
			for _, t := range truth {
				if at.Sub(t.Time) <= cfg.truthLife {
					live = append(live, t)
				}
			}
			// Map order is random, and a scan whose measurements arrive in a
			// different order is a scan the associator may settle differently.
			// Sorting costs nothing here and keeps a recording reproducible.
			sort.Slice(live, func(i, j int) bool { return live[i].ID < live[j].ID })

			returns := sensor.Sweep(at, live)

			meas := make([]track.Measurement, len(returns))
			ids := make([]string, len(returns))
			for i, r := range returns {
				meas[i] = r.Meas
				ids[i] = r.Truth
			}
			step(record.NewScan(record.Radar, at, meas, ids))
		}
	}
}

// updateTruth folds a snapshot into what the radar is allowed to know, and
// forgets aircraft the feed has gone quiet about.
func updateTruth(truth map[string]radar.Truth, snap opensky.Snapshot, frame geo.Frame, life time.Duration) {
	for i := range snap.Vectors {
		v := &snap.Vectors[i]
		if v.OnGround || v.Latitude == nil || v.Longitude == nil || v.TimePosition == nil {
			continue
		}

		pos := geo.Geodetic{Lat: *v.Latitude, Lon: *v.Longitude}
		switch {
		case v.GeoAltitude != nil:
			pos.Alt = *v.GeoAltitude
		case v.BaroAltitude != nil:
			pos.Alt = *v.BaroAltitude
		}

		// TrueTrack is degrees clockwise from north, so east is the sine and
		// north the cosine — the reverse of the usual convention, and an easy
		// place to send every simulated aircraft off at a right angle to where
		// it is actually going.
		var vel [2]float64
		if v.Velocity != nil && v.TrueTrack != nil {
			hdg := *v.TrueTrack * math.Pi / 180
			vel = [2]float64{*v.Velocity * math.Sin(hdg), *v.Velocity * math.Cos(hdg)}
		}

		truth[v.ICAO24] = radar.Truth{
			ID:       v.ICAO24,
			Position: frame.ToENU(pos),
			Velocity: vel,
			Time:     *v.TimePosition,
		}
	}

	for id, t := range truth {
		if snap.Time.Sub(t.Time) > life {
			delete(truth, id)
		}
	}
}

// clientFromEnv builds an authenticated client when credentials are present and
// an anonymous one otherwise. Anonymous access works but the allowance is small
// enough that a long run will hit it, which is why the poll interval defaults
// to ten seconds rather than the five the feed updates at.
func clientFromEnv() *opensky.Client {
	id := os.Getenv("OPENSKY_CLIENT_ID")
	secret := os.Getenv("OPENSKY_CLIENT_SECRET")

	if id == "" || secret == "" {
		log.Printf("no OPENSKY_CLIENT_ID/OPENSKY_CLIENT_SECRET set; polling anonymously")
		return opensky.NewAnonymous()
	}
	return opensky.New(id, secret)
}

// boxAround turns a centre and a half-width in kilometres into the bounding box
// the API wants. A degree of latitude is a fixed distance; a degree of
// longitude shrinks toward the poles, so the two are not the same number.
func boxAround(lat, lon, radiusKM float64) opensky.Box {
	const kmPerDegLat = 111.32

	dLat := radiusKM / kmPerDegLat
	dLon := radiusKM / (kmPerDegLat * math.Cos(lat*math.Pi/180))

	return opensky.Box{
		LatMin: lat - dLat,
		LatMax: lat + dLat,
		LonMin: lon - dLon,
		LonMax: lon + dLon,
	}
}

func report(cfg config, s Stats) {
	if cfg.quiet {
		return
	}
	log.Printf("%-5s %s  meas=%-3d matched=%-3d (id=%-3d) withheld=%-2d missed=%-3d new=%-3d pruned=%-2d live=%d",
		s.Scan, s.At.UTC().Format("15:04:05"),
		s.Meas, s.Matched, s.ByID, s.Withheld, s.Missed, s.Spawned, s.Pruned, s.Live)
}

// dumpTracks prints the picture as it stands. A track with no identity was
// built from radar alone and has never been confirmed by a cooperative report,
// which makes it either an aircraft not on the feed or an accumulation of
// clutter — and telling those apart by eye is exactly what this project is
// meant to make unnecessary.
func dumpTracks(store *track.Store) {
	tracks := store.All()
	log.Printf("%d live tracks", len(tracks))

	sort.Slice(tracks, func(i, j int) bool { return tracks[i].ID < tracks[j].ID })

	for _, t := range tracks {
		id := t.Identity
		if id == "" {
			id = "-"
		}
		log.Printf("  %5d  %-8s  e=%9.0f n=%9.0f  v=(%6.0f,%6.0f) m/s  alt=%6.0f  misses=%d",
			t.ID, id,
			t.State.X[0], t.State.X[1], t.State.X[2], t.State.X[3],
			t.LastAlt, t.MissCounter)
	}
}
