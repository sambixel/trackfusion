package main

import (
	"math"
	"testing"
	"time"

	"github.com/sambixel/trackfusion/internal/assoc"
	"github.com/sambixel/trackfusion/internal/filter"
	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/radar"
	"github.com/sambixel/trackfusion/internal/record"
	"github.com/sambixel/trackfusion/internal/track"
)

var epoch = time.Date(2026, 9, 10, 12, 0, 0, 0, time.UTC)

func newEngine() *Engine {
	return &Engine{
		Filter:    filter.Filter{ProcessNoise: 3},
		Store:     track.NewStore(),
		Assoc:     assoc.Config{GateThreshold: 13.8, AmbiguityMargin: 2},
		MaxMisses: 4,
	}
}

// adsbScan builds a cooperative report: every measurement names its aircraft.
func adsbScan(at time.Time, ids []string, pos [][2]float64) record.Scan {
	meas := make([]track.Measurement, len(ids))
	for i := range ids {
		meas[i] = track.Measurement{
			TimeStamp: at,
			Position:  geo.ENU{E: pos[i][0], N: pos[i][1], U: 10000},
			Identity:  ids[i],
			R:         [2][2]float64{{100, 0}, {0, 100}},
		}
	}
	return record.NewScan(record.ADSB, at, meas, ids)
}

// radarScan builds the anonymous version of the same thing.
func radarScan(at time.Time, truth []string, pos [][2]float64) record.Scan {
	meas := make([]track.Measurement, len(pos))
	for i := range pos {
		meas[i] = track.Measurement{
			TimeStamp: at,
			Position:  geo.ENU{E: pos[i][0], N: pos[i][1], U: 10000},
			R:         [2][2]float64{{400, 0}, {0, 2500}},
		}
	}
	return record.NewScan(record.Radar, at, meas, truth)
}

func findTrack(s *track.Store, identity string) *track.Track {
	for _, t := range s.All() {
		if t.Identity == identity {
			return t
		}
	}
	return nil
}

// The simplest complete pass: a measurement matching nothing starts a track,
// and the same aircraft next scan updates it rather than starting a second one.
func TestFirstScanStartsTracksAndSecondUpdatesThem(t *testing.T) {
	e := newEngine()

	first := e.Step(adsbScan(epoch, []string{"abc123", "def456"}, [][2]float64{{0, 0}, {10000, 0}}))
	if first.Spawned != 2 || first.Live != 2 {
		t.Fatalf("first scan: %+v, want 2 spawned and 2 live", first)
	}
	if first.Matched != 0 {
		t.Errorf("first scan matched %d tracks that did not exist yet", first.Matched)
	}

	second := e.Step(adsbScan(epoch.Add(5*time.Second), []string{"abc123", "def456"},
		[][2]float64{{1000, 0}, {11000, 0}}))

	if second.Spawned != 0 {
		t.Errorf("second scan started %d duplicate tracks", second.Spawned)
	}
	if second.Matched != 2 || second.ByID != 2 {
		t.Errorf("second scan: %+v, want both matched by identity", second)
	}
	if second.Live != 2 {
		t.Errorf("%d live tracks, want 2", second.Live)
	}
}

// The aircraft moved 1 km east in 5 s and the filter has to work that out from
// two positions and nothing else, since neither sensor reports velocity.
func TestVelocityIsLearnedFromSuccessiveScans(t *testing.T) {
	e := newEngine()

	for i := range 12 {
		at := epoch.Add(time.Duration(i) * 5 * time.Second)
		e.Step(adsbScan(at, []string{"abc123"}, [][2]float64{{float64(i) * 1000, 0}}))
	}

	got := findTrack(e.Store, "abc123")
	if got == nil {
		t.Fatal("track went missing")
	}
	if math.Abs(got.State.X[2]-200) > 10 {
		t.Errorf("east velocity = %g m/s, want about 200", got.State.X[2])
	}
	if math.Abs(got.State.X[3]) > 10 {
		t.Errorf("north velocity = %g m/s, want about 0", got.State.X[3])
	}
}

// A track built from radar has no name. When a cooperative report finally lands
// on it the track adopts one, and that is the moment the two sensors stop being
// two pictures.
func TestRadarTrackAdoptsIdentityFromADSB(t *testing.T) {
	e := newEngine()

	// Three sweeps of an unnamed aircraft moving steadily east.
	for i := range 3 {
		at := epoch.Add(time.Duration(i) * 4 * time.Second)
		e.Step(radarScan(at, []string{"abc123"}, [][2]float64{{float64(i) * 800, 5000}}))
	}

	if len(e.Store.All()) != 1 {
		t.Fatalf("want 1 track from the radar sweeps, got %d", len(e.Store.All()))
	}
	if id := e.Store.All()[0].Identity; id != "" {
		t.Fatalf("radar track named itself %q", id)
	}

	// The feed catches up, reporting the same aircraft where it should be.
	got := e.Step(adsbScan(epoch.Add(12*time.Second), []string{"abc123"}, [][2]float64{{2400, 5000}}))

	if got.Spawned != 0 {
		t.Errorf("the cooperative report started a second track for an aircraft already held")
	}
	if len(e.Store.All()) != 1 {
		t.Fatalf("want 1 track, got %d", len(e.Store.All()))
	}
	if id := e.Store.All()[0].Identity; id != "abc123" {
		t.Errorf("identity = %q, want the track to have adopted abc123", id)
	}
}

// A track that already claims an address and is now matching a different one by
// position has gone wrong. Relabelling it would hide exactly the failure this
// project exists to measure.
func TestIdentityIsNeverOverwritten(t *testing.T) {
	e := newEngine()
	e.Step(adsbScan(epoch, []string{"abc123"}, [][2]float64{{0, 0}}))

	// A report from a different aircraft, close enough to win the gate.
	e.Step(adsbScan(epoch.Add(5*time.Second), []string{"def456"}, [][2]float64{{20, 0}}))

	if got := findTrack(e.Store, "abc123"); got == nil {
		t.Error("the original track lost its identity")
	}
}

// Below Pd = 1 an aircraft vanishes from sweeps it has not gone anywhere
// during. Tracks have to coast through that, and only die once the silence
// outlasts the tolerance.
func TestTracksCoastThroughMissesAndThenPrune(t *testing.T) {
	e := newEngine()
	e.Step(adsbScan(epoch, []string{"abc123"}, [][2]float64{{0, 0}}))

	for i := 1; i <= int(e.MaxMisses); i++ {
		got := e.Step(radarScan(epoch.Add(time.Duration(i)*4*time.Second), nil, nil))
		if got.Missed != 1 {
			t.Fatalf("sweep %d: %+v, want the track to have missed", i, got)
		}
		if got.Live != 1 {
			t.Fatalf("sweep %d pruned a track still within tolerance", i)
		}
	}

	final := e.Step(radarScan(epoch.Add(time.Duration(e.MaxMisses+1)*4*time.Second), nil, nil))
	if final.Pruned != 1 || final.Live != 0 {
		t.Errorf("final sweep: %+v, want the track dropped once the silence outlasted the tolerance", final)
	}
}

// Uncertainty grows while a track coasts, which is what lets a stale track
// accept a return further out than a freshly corrected one would. A gate that
// did not widen would strand every track that missed a sweep.
func TestGateWidensWhileCoasting(t *testing.T) {
	e := newEngine()
	e.Step(adsbScan(epoch, []string{"abc123"}, [][2]float64{{0, 0}}))

	fresh := findTrack(e.Store, "abc123").State.P[0][0]

	e.Step(radarScan(epoch.Add(4*time.Second), nil, nil))
	e.Step(radarScan(epoch.Add(8*time.Second), nil, nil))

	coasted := findTrack(e.Store, "abc123").State.P[0][0]
	if coasted <= fresh {
		t.Errorf("position variance %g after two missed sweeps, was %g when fresh", coasted, fresh)
	}
}

// The failure the whole project is built around: converging aircraft produce
// returns closer to each other's prediction than to their own, and a tracker
// that takes the nearest one swaps them and then grows confident about it. The
// geometry here is clean and the returns exact, so a swap would be the
// assignment failing rather than bad luck.
func TestCrossingAircraftDoNotSwap(t *testing.T) {
	e := newEngine()

	const speed = 200.0
	const step = 4 * time.Second

	// One heading east along N=+1000, the other west along N=-1000, meeting
	// over the origin partway through.
	posAt := func(i int) [][2]float64 {
		s := speed * float64(i) * step.Seconds()
		return [][2]float64{
			{-24000 + s, 1000},
			{24000 - s, -1000},
		}
	}

	e.Step(adsbScan(epoch, []string{"aaa111", "bbb222"}, posAt(0)))

	for i := 1; i <= 60; i++ {
		at := epoch.Add(time.Duration(i) * step)
		got := e.Step(radarScan(at, []string{"aaa111", "bbb222"}, posAt(i)))

		if got.Spawned != 0 {
			t.Fatalf("sweep %d started %d spurious tracks", i, got.Spawned)
		}
		if got.Live != 2 {
			t.Fatalf("sweep %d: %d live tracks, want 2", i, got.Live)
		}
	}

	final := posAt(60)

	a := findTrack(e.Store, "aaa111")
	b := findTrack(e.Store, "bbb222")
	if a == nil || b == nil {
		t.Fatal("one of the two tracks lost its identity")
	}

	// aaa111 was heading east and must have ended up east; if the tracks
	// swapped at the crossing, each would now be following the other aircraft.
	if math.Abs(a.State.X[0]-final[0][0]) > 2000 || a.State.X[2] < 0 {
		t.Errorf("aaa111 at e=%g heading %g, want near %g heading east",
			a.State.X[0], a.State.X[2], final[0][0])
	}
	if math.Abs(b.State.X[0]-final[1][0]) > 2000 || b.State.X[2] > 0 {
		t.Errorf("bbb222 at e=%g heading %g, want near %g heading west",
			b.State.X[0], b.State.X[2], final[1][0])
	}
}

// Clutter starts tracks, because there is no way to tell at the time, and those
// tracks then die of silence. What must not happen is the count growing without
// bound or an established track being pulled off its aircraft. Also the only
// test running the real sensor, so it catches radar and engine disagreeing
// about what a scan is.
func TestClutterStartsTracksThatDieWithoutStealingRealOnes(t *testing.T) {
	e := newEngine()

	const step = 4 * time.Second
	sensor := radar.New(geo.ENU{}, radar.Config{
		Period:          step,
		MaxRangeM:       120000,
		SigmaRangeM:     40,
		SigmaBearingRad: 0.5 * math.Pi / 180,
		Pd:              1,
		ClutterRate:     3,
	}, 99)

	truth := []radar.Truth{{
		ID:       "abc123",
		Position: geo.ENU{E: -40000, N: 20000},
		Velocity: [2]float64{220, 0},
		Time:     epoch,
	}}

	e.Step(adsbScan(epoch, []string{"abc123"}, [][2]float64{{-40000, 20000}}))

	peak := 0
	for i := 1; i <= 80; i++ {
		at := epoch.Add(time.Duration(i) * step)

		returns := sensor.Sweep(at, truth)
		meas := make([]track.Measurement, len(returns))
		ids := make([]string, len(returns))
		for j, r := range returns {
			meas[j] = r.Meas
			ids[j] = r.Truth
		}

		got := e.Step(record.NewScan(record.Radar, at, meas, ids))
		peak = max(peak, got.Live)
	}

	// The real aircraft is still held, and still where it actually is.
	held := findTrack(e.Store, "abc123")
	if held == nil {
		t.Fatal("clutter cost the tracker its only real aircraft")
	}
	wantE := -40000 + 220*80*step.Seconds()
	if math.Abs(held.State.X[0]-wantE) > 3000 {
		t.Errorf("abc123 at e=%g, want near %g: a false return dragged it off", held.State.X[0], wantE)
	}

	// False tracks come and go, but with roughly three false returns a sweep
	// and a tolerance of four misses, the standing population has to settle.
	// Unbounded growth here means clutter is spawning tracks that never die.
	if peak > 40 {
		t.Errorf("peaked at %d live tracks against 1 real aircraft: false tracks are not dying", peak)
	}
	if final := len(e.Store.All()); final > 25 {
		t.Errorf("ended with %d live tracks against 1 real aircraft", final)
	}
}
