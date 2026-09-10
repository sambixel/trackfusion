// Package filter implements a constant-velocity Kalman filter over the state
// [x, y, vx, vy] in the local ENU plane. Reports arrive irregularly, so F is
// built from dt each call rather than fixed.
package filter

import "time"

// State is a track's estimate and its uncertainty. X is [x, y, vx, vy] in
// meters and meters per second; P is the 4x4 covariance.
type State struct {
	X [4]float64
	P [4][4]float64
}

// Filter holds tuning shared by every track. The zero value is not usable.
type Filter struct {
	// ProcessNoise is q, the acceleration spectral density in m²/s³. It sets
	// how wrong the constant-velocity assumption is allowed to be, so it is
	// the knob that decides whether turns are tracked or smoothed away.
	ProcessNoise float64
}

// Predict advances a state by dt.
func (f Filter) Predict(s State, dt time.Duration) State {

	t := dt.Seconds()
	q := f.ProcessNoise

	qpp := q * t * t * t * t / 4
	qpv := q * t * t * t / 2
	qvv := q * t * t

	return State{
		X: [4]float64{s.X[0] + s.X[2]*t,
			s.X[1] + s.X[3]*t,
			s.X[2],
			s.X[3],
		},
		P: [4][4]float64{
			{
				s.P[0][0] + t*s.P[2][0] + t*s.P[0][2] + t*t*s.P[2][2] + qpp,
				s.P[0][1] + t*s.P[2][1] + t*s.P[0][3] + t*t*s.P[2][3],
				s.P[0][2] + t*s.P[2][2] + qpv,
				s.P[0][3] + t*s.P[2][3],
			},
			{
				s.P[1][0] + t*s.P[3][0] + t*s.P[1][2] + t*t*s.P[3][2],
				s.P[1][1] + t*s.P[3][1] + t*s.P[1][3] + t*t*s.P[3][3] + qpp,
				s.P[1][2] + t*s.P[3][2],
				s.P[1][3] + t*s.P[3][3] + qpv,
			},
			{
				s.P[2][0] + t*s.P[2][2] + qpv,
				s.P[2][1] + t*s.P[2][3],
				s.P[2][2] + qvv,
				s.P[2][3],
			},
			{
				s.P[3][0] + t*s.P[3][2],
				s.P[3][1] + t*s.P[3][3] + qpv,
				s.P[3][2],
				s.P[3][3] + qvv,
			},
		},
	}
}

// Innovation returns the residual v = z - Hx and its covariance S = HPH' + R
// for an already-predicted state, without applying any correction.
func (f Filter) Innovation(s State, z [2]float64, r [2][2]float64) (v [2]float64, S [2][2]float64) {

	return [2]float64{
			z[0] - s.X[0],
			z[1] - s.X[1],
		},
		[2][2]float64{
			{s.P[0][0] + r[0][0], s.P[0][1] + r[0][1]},
			{s.P[1][0] + r[1][0], s.P[1][1] + r[1][1]},
		}
}

// Update corrects a predicted state with a position measurement. r is the
// measurement noise covariance and differs per sensor, which is what lets
// tracks stay sensor-agnostic.
func (f Filter) Update(s State, z [2]float64, r [2][2]float64) State {

	v, S := f.Innovation(s, z, r)
	det := S[0][0]*S[1][1] - S[0][1]*S[1][0]

	inverseS := [2][2]float64{
		{S[1][1] / det, -1 * S[0][1] / det},
		{-1 * S[0][1] / det, S[0][0] / det},
	}

	K := [4][2]float64{
		{
			s.P[0][0]*inverseS[0][0] + s.P[0][1]*inverseS[1][0],
			s.P[0][0]*inverseS[0][1] + s.P[0][1]*inverseS[1][1],
		},
		{
			s.P[1][0]*inverseS[0][0] + s.P[1][1]*inverseS[1][0],
			s.P[1][0]*inverseS[0][1] + s.P[1][1]*inverseS[1][1],
		},
		{
			s.P[2][0]*inverseS[0][0] + s.P[2][1]*inverseS[1][0],
			s.P[2][0]*inverseS[0][1] + s.P[2][1]*inverseS[1][1],
		},
		{
			s.P[3][0]*inverseS[0][0] + s.P[3][1]*inverseS[1][0],
			s.P[3][0]*inverseS[0][1] + s.P[3][1]*inverseS[1][1],
		},
	}

	X := [4]float64{
		s.X[0] + K[0][0]*v[0] + K[0][1]*v[1],
		s.X[1] + K[1][0]*v[0] + K[1][1]*v[1],
		s.X[2] + K[2][0]*v[0] + K[2][1]*v[1],
		s.X[3] + K[3][0]*v[0] + K[3][1]*v[1],
	}

	var P [4][4]float64
	for i := range 4 {
		for j := range 4 {
			P[i][j] = s.P[i][j] - K[i][0]*s.P[0][j] - K[i][1]*s.P[1][j]
		}
	}

	return State{X: X, P: P}
}
