// Package radar simulates a non-cooperative sensor over the same airspace as
// the ADS-B feed. It manufactures the hard case: a return is a position and a
// timestamp and nothing else, so association has to infer identity from
// geometry. Truth is degraded along four axes — identity stripped, position
// blurred, detections missed, false returns invented — kept separate so a
// change in behaviour can be blamed on one of them.
package radar

import (
	"math"
	"math/rand/v2"
	"time"

	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/track"
)

// Truth is one aircraft as it actually is. The caller assembles these, which is
// why this package does not import opensky: a test needs to put two aircraft on
// a converging course without a live feed.
type Truth struct {
	ID       string
	Position geo.ENU
	Velocity [2]float64 // east, north, m/s

	// Time is when Position was true, which is not the sweep time. Aircraft
	// report when they like and the antenna turns when it likes.
	Time time.Time
}

// Return is a detection and what caused it. Truth sits beside the measurement
// rather than inside it so there is no field for it to leak through: anything
// handed to the tracker is a Measurement, and a Measurement has nowhere to put
// it.
type Return struct {
	Meas  track.Measurement
	Truth string // ICAO24, or empty for clutter
}

// Config is the sensor's character.
type Config struct {
	// Period is how often the antenna comes back around, unrelated to the
	// ADS-B cadence.
	Period time.Duration

	// MaxRangeM is the edge of coverage. Beyond it an aircraft produces
	// nothing, which is not the same event as a missed detection even though
	// both look like silence.
	MaxRangeM float64

	// SigmaRangeM is the error in how far away the target is, from timing an
	// echo. Small, and it does not grow with distance.
	SigmaRangeM float64

	// SigmaBearingRad is the error in which direction it lies, from where the
	// antenna was pointing. Being an angle, it costs more metres the further
	// out the target is: 1 degree at 100 km is 1.7 km across the beam against
	// tens of metres along it. That asymmetry is why R is a matrix and not a
	// radius.
	SigmaBearingRad float64

	// Pd is the chance of seeing a given aircraft on a given sweep.
	Pd float64

	// ClutterRate is the mean number of returns per sweep that came from
	// nothing. Poisson around it.
	ClutterRate float64
}

// Radar is one sensor at a fixed point in the tracking plane.
type Radar struct {
	Site geo.ENU

	cfg Config
	rng *rand.Rand
}

// New returns a sensor seeded for repeatable runs. The generator is held here
// rather than taken from the package source so a change in behaviour can be
// told apart from a change in the dice.
func New(site geo.ENU, cfg Config, seed int64) *Radar {
	return &Radar{
		Site: site,
		cfg:  cfg,
		rng:  rand.New(rand.NewPCG(uint64(seed), 0x9E3779B97F4A7C15)),
	}
}

// Config returns the sensor's settings, so a caller driving the sweep clock can
// read Period without holding a second copy.
func (r *Radar) Config() Config { return r.cfg }

// Sweep returns what the sensor saw at at. Aircraft are extrapolated from their
// own timestamps first, on the same constant-velocity assumption the filter
// makes.
func (r *Radar) Sweep(at time.Time, truth []Truth) []Return {
	out := make([]Return, 0, len(truth))

	for _, t := range truth {
		if ret, ok := r.detect(at, t); ok {
			out = append(out, ret)
		}
	}

	return append(out, r.clutter(at)...)
}

func (r *Radar) detect(at time.Time, t Truth) (Return, bool) {
	dt := at.Sub(t.Time).Seconds()
	pos := geo.ENU{
		E: t.Position.E + t.Velocity[0]*dt,
		N: t.Position.N + t.Velocity[1]*dt,
		U: t.Position.U,
	}

	dE := pos.E - r.Site.E
	dN := pos.N - r.Site.N
	rng := math.Hypot(dE, dN)

	if rng > r.cfg.MaxRangeM {
		return Return{}, false
	}
	if r.rng.Float64() >= r.cfg.Pd {
		return Return{}, false
	}

	// Error goes on in the coordinates the sensor measures in: nudging east and
	// north independently would spread a distant target into a circle.
	bearing := math.Atan2(dE, dN)
	noisyRng := rng + r.rng.NormFloat64()*r.cfg.SigmaRangeM
	noisyBearing := bearing + r.rng.NormFloat64()*r.cfg.SigmaBearingRad

	// A negative range would put the target behind the antenna.
	if noisyRng < 0 {
		noisyRng = 0
	}

	return Return{
		Meas: track.Measurement{
			// Stamped with the sweep, not with the aircraft's own report time:
			// it is the extrapolation above that may be stale.
			TimeStamp: at,
			Position: geo.ENU{
				E: r.Site.E + noisyRng*math.Sin(noisyBearing),
				N: r.Site.N + noisyRng*math.Cos(noisyBearing),
				U: pos.U,
			},
			Identity: "", // always, and the whole reason this package exists
			R:        noiseAt(dE, dN, r.cfg),
		},
		Truth: t.ID,
	}, true
}

func (r *Radar) clutter(at time.Time) []Return {
	n := r.poisson(r.cfg.ClutterRate)
	if n == 0 {
		return nil
	}

	out := make([]Return, 0, n)
	for range n {
		// Square root, not a flat radius: area grows with the square, so a flat
		// draw would pile clutter round the antenna and leave the outer ring
		// empty, which is where false returns actually do damage.
		rng := r.cfg.MaxRangeM * math.Sqrt(r.rng.Float64())
		bearing := 2 * math.Pi * r.rng.Float64()

		dE := rng * math.Sin(bearing)
		dN := rng * math.Cos(bearing)

		out = append(out, Return{
			Meas: track.Measurement{
				TimeStamp: at,
				Position:  geo.ENU{E: r.Site.E + dE, N: r.Site.N + dN},
				Identity:  "",
				// Same confidence as a real detection: the sensor cannot tell
				// the difference, so nothing downstream may either.
				R: noiseAt(dE, dN, r.cfg),
			},
		})
	}

	return out
}

// poisson draws a count by Knuth's method. Linear in the result, which suits
// the handful of false returns a sweep produces and would not suit a large mean.
func (r *Radar) poisson(mean float64) int {
	if mean <= 0 {
		return 0
	}

	limit := math.Exp(-mean)
	k, p := 0, 1.0
	for {
		p *= r.rng.Float64()
		if p <= limit {
			return k
		}
		k++
	}
}

// noiseAt rotates the sensor's error ellipse — one axis along the line of
// sight, one across it — into east/north. Off the cardinal directions the
// off-diagonal terms are what tell the gate the uncertainty leans. It is a
// linearisation; the returns themselves bend along an arc.
func noiseAt(dE, dN float64, cfg Config) [2][2]float64 {
	rng := math.Hypot(dE, dN)

	varRange := cfg.SigmaRangeM * cfg.SigmaRangeM
	varCross := rng * cfg.SigmaBearingRad * rng * cfg.SigmaBearingRad

	// Directly overhead there is no line of sight to align to. A circle keeps
	// the matrix invertible and the gate usable.
	if rng == 0 {
		return [2][2]float64{{varRange, 0}, {0, varRange}}
	}

	uE, uN := dE/rng, dN/rng  // along the beam
	cE, cN := -dN/rng, dE/rng // across it

	return [2][2]float64{
		{
			varRange*uE*uE + varCross*cE*cE,
			varRange*uE*uN + varCross*cE*cN,
		},
		{
			varRange*uN*uE + varCross*cN*cE,
			varRange*uN*uN + varCross*cN*cN,
		},
	}
}
