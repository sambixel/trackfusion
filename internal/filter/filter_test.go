package filter

import (
	"math"
	"math/rand/v2"
	"testing"
	"time"
)

// The functions under test are hand-unrolled: every entry of every matrix
// product is written out by index. That is fast and it is also the easiest kind
// of code in which to transpose a subscript and get an answer that still looks
// plausible. So the tests below mostly do not assert specific numbers. They
// build the same matrices the slow, obvious way with loops, and check the
// unrolled version agrees.

type mat [][]float64

func newMat(r, c int) mat {
	m := make(mat, r)
	for i := range m {
		m[i] = make([]float64, c)
	}
	return m
}

func (m mat) mul(o mat) mat {
	out := newMat(len(m), len(o[0]))
	for i := range out {
		for j := range out[i] {
			for k := range o {
				out[i][j] += m[i][k] * o[k][j]
			}
		}
	}
	return out
}

func (m mat) transpose() mat {
	out := newMat(len(m[0]), len(m))
	for i := range m {
		for j := range m[i] {
			out[j][i] = m[i][j]
		}
	}
	return out
}

func (m mat) add(o mat) mat {
	out := newMat(len(m), len(m[0]))
	for i := range out {
		for j := range out[i] {
			out[i][j] = m[i][j] + o[i][j]
		}
	}
	return out
}

func from4(p [4][4]float64) mat {
	m := newMat(4, 4)
	for i := range 4 {
		copy(m[i], p[i][:])
	}
	return m
}

// stateTransition advances position by velocity over t and leaves velocity
// alone: the constant-velocity assumption, written out.
func stateTransition(t float64) mat {
	return mat{
		{1, 0, t, 0},
		{0, 1, 0, t},
		{0, 0, 1, 0},
		{0, 0, 0, 1},
	}
}

// processNoise is the covariance the model earns by being wrong. An
// unmodelled acceleration over t moves position by t^2/2 and velocity by t,
// and Q is the covariance of that pair scaled by the spectral density q.
func processNoise(q, t float64) mat {
	pp := q * t * t * t * t / 4
	pv := q * t * t * t / 2
	vv := q * t * t
	return mat{
		{pp, 0, pv, 0},
		{0, pp, 0, pv},
		{pv, 0, vv, 0},
		{0, pv, 0, vv},
	}
}

// observation picks position out of the state and ignores velocity, which is
// the whole of what either sensor reports.
var observation = mat{
	{1, 0, 0, 0},
	{0, 1, 0, 0},
}

func identity(n int) mat {
	m := newMat(n, n)
	for i := range m {
		m[i][i] = 1
	}
	return m
}

// randomState builds an arbitrary but legitimate estimate. P is formed as
// A*A' plus a diagonal so it comes out symmetric and positive definite, which
// is what a real covariance always is; feeding in an arbitrary grid of numbers
// would test the arithmetic against inputs the filter can never see.
func randomState(rng *rand.Rand) State {
	a := newMat(4, 4)
	for i := range 4 {
		for j := range 4 {
			a[i][j] = rng.NormFloat64() * 40
		}
	}
	p := a.mul(a.transpose()).add(identity(4))

	var s State
	for i := range 4 {
		s.X[i] = rng.NormFloat64() * 1000
		for j := range 4 {
			s.P[i][j] = p[i][j]
		}
	}
	return s
}

func checkMat(t *testing.T, name string, got [4][4]float64, want mat, tol float64) {
	t.Helper()
	for i := range 4 {
		for j := range 4 {
			if diff := math.Abs(got[i][j] - want[i][j]); diff > tol*math.Max(1, math.Abs(want[i][j])) {
				t.Errorf("%s[%d][%d] = %g, want %g", name, i, j, got[i][j], want[i][j])
			}
		}
	}
}

// The covariance after a prediction is F*P*F' + Q. Transposing a subscript
// anywhere in the unrolled version shifts an off-diagonal term, which changes
// the shape of the gate without changing anything visible in the position, so
// it is checked against the loop form over arbitrary inputs.
func TestPredictMatchesTheLoopForm(t *testing.T) {
	rng := rand.New(rand.NewPCG(1, 2))
	f := Filter{ProcessNoise: 3.5}

	for _, dt := range []time.Duration{0, 100 * time.Millisecond, time.Second, 5 * time.Second, 47 * time.Second} {
		for range 200 {
			s := randomState(rng)
			secs := dt.Seconds()

			F := stateTransition(secs)
			want := F.mul(from4(s.P)).mul(F.transpose()).add(processNoise(f.ProcessNoise, secs))

			got := f.Predict(s, dt)
			checkMat(t, "P", got.P, want, 1e-9)

			for i := range 4 {
				var w float64
				for k := range 4 {
					w += F[i][k] * s.X[k]
				}
				if math.Abs(got.X[i]-w) > 1e-9*math.Max(1, math.Abs(w)) {
					t.Fatalf("dt=%v X[%d] = %g, want %g", dt, i, got.X[i], w)
				}
			}
		}
	}
}

// The obvious sanity check, kept because it is the one a reader can verify by
// eye: a track at 200 m/s east is 1 km further east five seconds later.
func TestPredictCarriesPositionForward(t *testing.T) {
	f := Filter{ProcessNoise: 1}
	s := State{X: [4]float64{0, 0, 200, -50}}

	got := f.Predict(s, 5*time.Second)

	if got.X[0] != 1000 || got.X[1] != -250 {
		t.Errorf("position = (%g, %g), want (1000, -250)", got.X[0], got.X[1])
	}
	if got.X[2] != 200 || got.X[3] != -50 {
		t.Errorf("velocity changed to (%g, %g)", got.X[2], got.X[3])
	}
}

// Uncertainty only ever grows while coasting, and it grows faster the longer
// the gap. This is what widens the gate for a track that has not been heard
// from, and it is the reason a stale track can still accept a return that a
// freshly corrected one would refuse.
func TestPredictAlwaysWidensPosition(t *testing.T) {
	f := Filter{ProcessNoise: 2}
	s := State{P: [4][4]float64{{100, 0, 0, 0}, {0, 100, 0, 0}, {0, 0, 25, 0}, {0, 0, 0, 25}}}

	prev := s.P[0][0]
	for _, dt := range []time.Duration{time.Second, 5 * time.Second, 30 * time.Second, 120 * time.Second} {
		got := f.Predict(s, dt).P[0][0]
		if got <= prev {
			t.Errorf("at dt=%v position variance %g did not exceed %g", dt, got, prev)
		}
		prev = got
	}
}

// Predicting by nothing is not quite the identity: the state does not move,
// but with dt of zero every term of Q is zero too, so P must come back
// untouched rather than picking up noise for a gap that did not happen.
func TestPredictByZeroChangesNothing(t *testing.T) {
	f := Filter{ProcessNoise: 9}
	rng := rand.New(rand.NewPCG(3, 4))
	s := randomState(rng)

	got := f.Predict(s, 0)

	if got.X != s.X {
		t.Errorf("state moved: %v -> %v", s.X, got.X)
	}
	if got.P != s.P {
		t.Errorf("covariance changed over a zero-length gap")
	}
}

// The residual is what the gate measures and S is what it measures against, so
// an error here mis-sizes every gate in the system rather than moving any one
// track visibly.
func TestInnovationMatchesTheLoopForm(t *testing.T) {
	rng := rand.New(rand.NewPCG(5, 6))
	f := Filter{ProcessNoise: 1}

	for range 500 {
		s := randomState(rng)
		z := [2]float64{rng.NormFloat64() * 1000, rng.NormFloat64() * 1000}
		r := [2][2]float64{{80, -5}, {-5, 300}}

		v, S := f.Innovation(s, z, r)

		if want := z[0] - s.X[0]; math.Abs(v[0]-want) > 1e-9 {
			t.Fatalf("v[0] = %g, want %g", v[0], want)
		}
		if want := z[1] - s.X[1]; math.Abs(v[1]-want) > 1e-9 {
			t.Fatalf("v[1] = %g, want %g", v[1], want)
		}

		rm := mat{{r[0][0], r[0][1]}, {r[1][0], r[1][1]}}
		want := observation.mul(from4(s.P)).mul(observation.transpose()).add(rm)
		for i := range 2 {
			for j := range 2 {
				if math.Abs(S[i][j]-want[i][j]) > 1e-9*math.Max(1, math.Abs(want[i][j])) {
					t.Fatalf("S[%d][%d] = %g, want %g", i, j, S[i][j], want[i][j])
				}
			}
		}
	}
}

// Update is the longest stretch of unrolled arithmetic in the project: a 4x2
// gain, a state correction and a 4x4 covariance update, all written by index.
// The loop form is K = P*H'*inv(S), x + K*v, and (I - K*H)*P.
func TestUpdateMatchesTheLoopForm(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 8))
	f := Filter{ProcessNoise: 1}

	for range 500 {
		s := randomState(rng)
		z := [2]float64{rng.NormFloat64() * 500, rng.NormFloat64() * 500}
		r := [2][2]float64{{120, 15}, {15, 900}}

		_, S := f.Innovation(s, z, r)
		det := S[0][0]*S[1][1] - S[0][1]*S[1][0]
		invS := mat{
			{S[1][1] / det, -S[0][1] / det},
			{-S[1][0] / det, S[0][0] / det},
		}

		P := from4(s.P)
		K := P.mul(observation.transpose()).mul(invS)

		wantP := identity(4).add(scale(K.mul(observation), -1)).mul(P)

		got := f.Update(s, z, r)
		checkMat(t, "P", got.P, wantP, 1e-7)

		v := mat{{z[0] - s.X[0]}, {z[1] - s.X[1]}}
		corr := K.mul(v)
		for i := range 4 {
			want := s.X[i] + corr[i][0]
			if math.Abs(got.X[i]-want) > 1e-7*math.Max(1, math.Abs(want)) {
				t.Fatalf("X[%d] = %g, want %g", i, got.X[i], want)
			}
		}
	}
}

func scale(m mat, k float64) mat {
	out := newMat(len(m), len(m[0]))
	for i := range out {
		for j := range out[i] {
			out[i][j] = m[i][j] * k
		}
	}
	return out
}

// A sensor claiming no error at all should be believed completely. This pins
// the direction of the gain: a gain built the wrong way round would move the
// state away from a measurement it is supposed to snap to.
func TestPerfectMeasurementIsBelievedEntirely(t *testing.T) {
	f := Filter{ProcessNoise: 1}
	s := State{
		X: [4]float64{0, 0, 100, 100},
		P: [4][4]float64{{500, 0, 0, 0}, {0, 500, 0, 0}, {0, 0, 900, 0}, {0, 0, 0, 900}},
	}

	// Not exactly zero: a singular S has no inverse, and Update does not guard
	// against one. Small enough that the filter should follow the measurement
	// to within centimetres.
	got := f.Update(s, [2]float64{300, -400}, [2][2]float64{{1e-9, 0}, {0, 1e-9}})

	if math.Abs(got.X[0]-300) > 0.01 || math.Abs(got.X[1]+400) > 0.01 {
		t.Errorf("position = (%g, %g), want (300, -400)", got.X[0], got.X[1])
	}
	if got.P[0][0] > 1e-6 {
		t.Errorf("position variance %g should have collapsed to the measurement's", got.P[0][0])
	}
}

// The mirror case. A sensor that admits it has almost no idea should barely
// move the estimate, which is what keeps a bad measurement from dragging a
// well-established track off course.
func TestWorthlessMeasurementIsAlmostIgnored(t *testing.T) {
	f := Filter{ProcessNoise: 1}
	s := State{
		X: [4]float64{0, 0, 0, 0},
		P: [4][4]float64{{25, 0, 0, 0}, {0, 25, 0, 0}, {0, 0, 100, 0}, {0, 0, 0, 100}},
	}

	got := f.Update(s, [2]float64{50000, 50000}, [2][2]float64{{1e12, 0}, {0, 1e12}})

	if math.Abs(got.X[0]) > 5 || math.Abs(got.X[1]) > 5 {
		t.Errorf("position = (%g, %g), want to have barely moved", got.X[0], got.X[1])
	}
}

// Correcting can only ever narrow the estimate, and the covariance has to stay
// symmetric and positive definite as it does so. A covariance that drifts
// asymmetric or goes negative makes the gate meaningless, and because the gate
// then admits the wrong return the failure compounds instead of showing up.
func TestUpdateShrinksAndStaysWellFormed(t *testing.T) {
	rng := rand.New(rand.NewPCG(9, 10))
	f := Filter{ProcessNoise: 2}

	for range 500 {
		s := randomState(rng)
		z := [2]float64{rng.NormFloat64() * 300, rng.NormFloat64() * 300}
		r := [2][2]float64{{150, 0}, {0, 150}}

		got := f.Update(s, z, r)

		for i := range 4 {
			if got.P[i][i] > s.P[i][i]+1e-6 {
				t.Fatalf("variance %d grew on update: %g -> %g", i, s.P[i][i], got.P[i][i])
			}
			if got.P[i][i] <= 0 {
				t.Fatalf("variance %d went non-positive: %g", i, got.P[i][i])
			}
			for j := range 4 {
				gap := math.Abs(got.P[i][j] - got.P[j][i])
				if gap > 1e-6*math.Max(1, math.Abs(got.P[i][j])) {
					t.Fatalf("P is not symmetric at [%d][%d]: %g vs %g", i, j, got.P[i][j], got.P[j][i])
				}
			}
		}
	}
}

// Position is all either sensor reports, so velocity is only ever inferred
// from how the position estimate has had to move. A single correction has to
// push the velocity in the direction of the residual, or a new track would
// never work out where its aircraft is going.
func TestUpdateInfersVelocityFromPositionAlone(t *testing.T) {
	f := Filter{ProcessNoise: 1}
	s := State{
		X: [4]float64{0, 0, 0, 0},
		P: [4][4]float64{
			{100, 0, 500, 0},
			{0, 100, 0, 500},
			{500, 0, 90000, 0},
			{0, 500, 0, 90000},
		},
	}

	got := f.Update(s, [2]float64{400, -400}, [2][2]float64{{50, 0}, {0, 50}})

	if got.X[2] <= 0 {
		t.Errorf("east velocity = %g, want positive: the aircraft was found east of the estimate", got.X[2])
	}
	if got.X[3] >= 0 {
		t.Errorf("north velocity = %g, want negative", got.X[3])
	}
}

// Predict then update over a straight track has to converge on the truth. This
// is the only test that runs the two together, and it is the one that would
// catch the pair being individually right but inconsistent about what the
// state vector means.
func TestFilterConvergesOnASteadyTrack(t *testing.T) {
	f := Filter{ProcessNoise: 0.5}

	const vE, vN = 220.0, -140.0
	const step = 4.0

	s := State{
		X: [4]float64{0, 0, 0, 0},
		P: [4][4]float64{{900, 0, 0, 0}, {0, 900, 0, 0}, {0, 0, 90000, 0}, {0, 0, 0, 90000}},
	}

	for i := 1; i <= 60; i++ {
		s = f.Predict(s, time.Duration(step*float64(time.Second)))
		s = f.Update(s, [2]float64{vE * step * float64(i), vN * step * float64(i)}, [2][2]float64{{100, 0}, {0, 100}})
	}

	if math.Abs(s.X[2]-vE) > 1 || math.Abs(s.X[3]-vN) > 1 {
		t.Errorf("velocity settled at (%g, %g), want (%g, %g)", s.X[2], s.X[3], vE, vN)
	}
	if s.P[2][2] > 90000 {
		t.Errorf("velocity variance %g never came down from its initial guess", s.P[2][2])
	}
}
