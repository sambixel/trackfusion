package radar

import (
	"math"
	"testing"
	"time"

	"github.com/sambixel/trackfusion/internal/geo"
)

var epoch = time.Date(2026, 9, 10, 12, 0, 0, 0, time.UTC)

// perfect is a sensor with nothing wrong with it: everything in range is seen,
// placed exactly, and nothing is invented. Tests that care about geometry use
// it so a failure means the geometry is wrong and not that the dice came up
// badly.
func perfect() Config {
	return Config{
		Period:    4 * time.Second,
		MaxRangeM: 200000,
		Pd:        1,
	}
}

func closeTo(t *testing.T, name string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s = %g, want %g (tolerance %g)", name, got, want, tol)
	}
}

// The one invariant the whole package exists to uphold. A radar that leaked the
// address would let every return match by identity, association would never be
// exercised, and the project would be testing nothing.
func TestReturnsNeverCarryIdentity(t *testing.T) {
	cfg := perfect()
	cfg.SigmaRangeM = 50
	cfg.SigmaBearingRad = 0.02
	cfg.ClutterRate = 3

	r := New(geo.ENU{}, cfg, 1)
	truth := []Truth{
		{ID: "abc123", Position: geo.ENU{E: 10000, N: 5000}, Time: epoch},
		{ID: "def456", Position: geo.ENU{E: -8000, N: 22000}, Time: epoch},
	}

	for _, ret := range r.Sweep(epoch, truth) {
		if ret.Meas.Identity != "" {
			t.Fatalf("return leaked identity %q", ret.Meas.Identity)
		}
	}
}

// Due north the line of sight runs along the north axis, so the small range
// error lands entirely in north and the large bearing error entirely in east.
// Nothing leans, so the off-diagonal terms vanish.
func TestNoiseDueNorthSeparatesTheAxes(t *testing.T) {
	cfg := perfect()
	cfg.SigmaRangeM = 30
	cfg.SigmaBearingRad = 0.01

	const rng = 50000
	R := noiseAt(0, rng, cfg)

	wantCross := (rng * cfg.SigmaBearingRad) * (rng * cfg.SigmaBearingRad)
	wantRange := cfg.SigmaRangeM * cfg.SigmaRangeM

	closeTo(t, "R[0][0] (east, across the beam)", R[0][0], wantCross, 1e-6)
	closeTo(t, "R[1][1] (north, along the beam)", R[1][1], wantRange, 1e-6)
	closeTo(t, "R[0][1]", R[0][1], 0, 1e-9)
	closeTo(t, "R[1][0]", R[1][0], 0, 1e-9)

	if R[0][0] <= R[1][1] {
		t.Errorf("cross-range %g should dwarf down-range %g at 50 km", R[0][0], R[1][1])
	}
}

// Off the cardinal directions the ellipse leans, and the off-diagonal terms are
// what say so. A tracker gating on a radius cannot see this and will accept
// returns that sit well outside the beam.
func TestNoiseOffAxisLeans(t *testing.T) {
	cfg := perfect()
	cfg.SigmaRangeM = 30
	cfg.SigmaBearingRad = 0.01

	const rng = 50000
	leg := rng / math.Sqrt2
	R := noiseAt(leg, leg, cfg)

	varCross := (rng * cfg.SigmaBearingRad) * (rng * cfg.SigmaBearingRad)
	varRange := cfg.SigmaRangeM * cfg.SigmaRangeM

	closeTo(t, "R[0][1]", R[0][1], (varRange-varCross)/2, 1e-6)
	closeTo(t, "R[1][0]", R[1][0], (varRange-varCross)/2, 1e-6)

	if R[0][1] >= 0 {
		t.Errorf("R[0][1] = %g, want negative: northeast of the site the "+
			"uncertainty runs northwest-southeast", R[0][1])
	}

	// However it leans, it is still a covariance and has to stay usable: a
	// non-positive determinant would make the gate refuse the pair outright.
	if det := R[0][0]*R[1][1] - R[0][1]*R[1][0]; det <= 0 {
		t.Errorf("determinant %g is not positive", det)
	}
}

// Bearing error is an angle, so what it costs in metres doubles when the range
// doubles, and the area of doubt goes up fourfold. Down-range error does not
// move at all.
func TestCrossRangeErrorGrowsWithRange(t *testing.T) {
	cfg := perfect()
	cfg.SigmaRangeM = 30
	cfg.SigmaBearingRad = 0.01

	near := noiseAt(0, 25000, cfg)
	far := noiseAt(0, 50000, cfg)

	closeTo(t, "cross-range variance ratio", far[0][0]/near[0][0], 4, 1e-6)
	closeTo(t, "down-range variance ratio", far[1][1]/near[1][1], 1, 1e-9)
}

// The antenna turns on its own schedule and aircraft report on theirs, so truth
// almost always arrives stale. The sensor reports where the aircraft should
// have got to, not where it last said it was.
func TestSweepExtrapolatesStaleTruth(t *testing.T) {
	r := New(geo.ENU{}, perfect(), 1)

	// Reported 10 s before the sweep, heading east at 200 m/s.
	got := r.Sweep(epoch.Add(10*time.Second), []Truth{{
		ID:       "abc123",
		Position: geo.ENU{E: 0, N: 40000},
		Velocity: [2]float64{200, 0},
		Time:     epoch,
	}})

	if len(got) != 1 {
		t.Fatalf("want 1 return, got %d", len(got))
	}
	closeTo(t, "east", got[0].Meas.Position.E, 2000, 1e-6)
	closeTo(t, "north", got[0].Meas.Position.N, 40000, 1e-6)

	// The stamp is the sweep, not the aircraft's own report time: the sensor is
	// asserting where the target is now.
	if !got[0].Meas.TimeStamp.Equal(epoch.Add(10 * time.Second)) {
		t.Errorf("stamped %v, want the sweep time", got[0].Meas.TimeStamp)
	}
}

// Outside coverage is silence for a structural reason, not a coin flip, so it
// holds even at Pd = 1.
func TestBeyondRangeIsNeverSeen(t *testing.T) {
	cfg := perfect()
	cfg.MaxRangeM = 100000

	r := New(geo.ENU{}, cfg, 1)
	got := r.Sweep(epoch, []Truth{
		{ID: "inside", Position: geo.ENU{E: 99000, N: 0}, Time: epoch},
		{ID: "outside", Position: geo.ENU{E: 101000, N: 0}, Time: epoch},
	})

	if len(got) != 1 {
		t.Fatalf("want 1 return, got %d", len(got))
	}
	if got[0].Truth != "inside" {
		t.Errorf("kept %q, want the aircraft inside coverage", got[0].Truth)
	}
}

// Below Pd = 1 an aircraft that has not moved and has not gone anywhere still
// vanishes from some sweeps. Tracks have to survive that without being pruned,
// which is the whole reason MissCounter tolerates more than one.
func TestMissedDetectionsHappenAtRoughlyPd(t *testing.T) {
	cfg := perfect()
	cfg.Pd = 0.7

	r := New(geo.ENU{}, cfg, 7)
	truth := []Truth{{ID: "abc123", Position: geo.ENU{E: 10000, N: 10000}, Time: epoch}}

	const sweeps = 4000
	seen := 0
	for i := range sweeps {
		seen += len(r.Sweep(epoch.Add(time.Duration(i)*cfg.Period), truth))
	}

	// Wide enough that a fixed seed will not trip it, tight enough to catch Pd
	// being inverted or ignored.
	closeTo(t, "detection rate", float64(seen)/sweeps, 0.7, 0.03)
}

// Clutter is a return that came from nothing. It has to be indistinguishable
// from a real one at the point of use, or the tracker gets a hint it would not
// have in reality.
func TestClutterIsAnonymousAndInRange(t *testing.T) {
	cfg := perfect()
	cfg.ClutterRate = 5
	cfg.MaxRangeM = 80000

	site := geo.ENU{E: 1000, N: -2000}
	r := New(site, cfg, 3)

	total := 0
	for i := range 200 {
		for _, ret := range r.Sweep(epoch.Add(time.Duration(i)*cfg.Period), nil) {
			total++
			if ret.Truth != "" {
				t.Fatalf("clutter claimed truth %q", ret.Truth)
			}
			if ret.Meas.Identity != "" {
				t.Fatalf("clutter leaked identity %q", ret.Meas.Identity)
			}
			d := math.Hypot(ret.Meas.Position.E-site.E, ret.Meas.Position.N-site.N)
			if d > cfg.MaxRangeM {
				t.Fatalf("clutter at %g m, outside %g m of coverage", d, cfg.MaxRangeM)
			}
		}
	}

	closeTo(t, "clutter per sweep", float64(total)/200, cfg.ClutterRate, 0.5)
}

// Drawing the radius uniformly would pile clutter near the antenna and leave
// the outer ring empty, which is backwards: gates are widest far out, so that
// is where false returns do the damage. Half the disc's area lies beyond
// r/sqrt(2), so half the returns should too.
func TestClutterSpreadsByAreaNotRadius(t *testing.T) {
	cfg := perfect()
	cfg.ClutterRate = 20
	cfg.MaxRangeM = 100000

	r := New(geo.ENU{}, cfg, 11)

	inner, outer := 0, 0
	half := cfg.MaxRangeM / math.Sqrt2

	for i := range 500 {
		for _, ret := range r.Sweep(epoch.Add(time.Duration(i)*cfg.Period), nil) {
			if math.Hypot(ret.Meas.Position.E, ret.Meas.Position.N) < half {
				inner++
			} else {
				outer++
			}
		}
	}

	closeTo(t, "fraction in the inner half-area", float64(inner)/float64(inner+outer), 0.5, 0.03)
}

// Replay is worth nothing if the sensor does not repeat. Two runs from one seed
// must agree exactly, or a change in the tracker cannot be told apart from a
// change in the dice.
func TestSameSeedReplaysExactly(t *testing.T) {
	cfg := perfect()
	cfg.Pd = 0.8
	cfg.SigmaRangeM = 40
	cfg.SigmaBearingRad = 0.015
	cfg.ClutterRate = 4

	truth := []Truth{
		{ID: "abc123", Position: geo.ENU{E: 12000, N: 4000}, Velocity: [2]float64{180, -20}, Time: epoch},
		{ID: "def456", Position: geo.ENU{E: -6000, N: 30000}, Velocity: [2]float64{-150, 60}, Time: epoch},
	}

	run := func() []Return {
		r := New(geo.ENU{}, cfg, 42)
		var all []Return
		for i := range 50 {
			all = append(all, r.Sweep(epoch.Add(time.Duration(i)*cfg.Period), truth)...)
		}
		return all
	}

	a, b := run(), run()
	if len(a) != len(b) {
		t.Fatalf("run lengths differ: %d vs %d", len(a), len(b))
	}
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("return %d differs:\n %+v\n %+v", i, a[i], b[i])
		}
	}
}

// Noise has to be applied in the coordinates the sensor measures in. Nudging
// east and north independently would spread a distant target into a circle;
// perturbing an angle sweeps it along the arc, so almost all of the scatter
// lands across the beam and almost none along it.
func TestNoiseFollowsTheBeamNotTheAxes(t *testing.T) {
	cfg := perfect()
	cfg.SigmaRangeM = 20
	cfg.SigmaBearingRad = 0.01

	r := New(geo.ENU{}, cfg, 5)
	const rng = 100000
	truth := []Truth{{ID: "abc123", Position: geo.ENU{E: 0, N: rng}, Time: epoch}}

	var sumSqE, sumSqN float64
	const sweeps = 3000
	for i := range sweeps {
		got := r.Sweep(epoch.Add(time.Duration(i)*cfg.Period), truth)
		dE := got[0].Meas.Position.E
		dN := got[0].Meas.Position.N - rng
		sumSqE += dE * dE
		sumSqN += dN * dN
	}

	spreadE := math.Sqrt(sumSqE / sweeps)
	spreadN := math.Sqrt(sumSqN / sweeps)

	closeTo(t, "spread across the beam", spreadE, rng*cfg.SigmaBearingRad, 60)
	closeTo(t, "spread along the beam", spreadN, cfg.SigmaRangeM, 5)
}
