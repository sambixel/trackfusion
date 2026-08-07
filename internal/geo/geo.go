// Package geo converts between WGS84 geodetic coordinates, Earth-centered
// Earth-fixed (ECEF) coordinates, and a local east-north-up (ENU) tangent
// plane.
//
// The tracker works entirely in ENU. A constant-velocity motion model has to
// be linear in the coordinates it operates on, and latitude/longitude is not:
// a degree of longitude is a different distance at every latitude, so constant
// ground speed does not produce constant coordinate rates. ENU is metric and
// Cartesian, so straight-line flight is a straight line in the state space.
package geo

// Geodetic is a WGS84 position. Lat and Lon are degrees, Alt is meters above
// the ellipsoid.
//
// ADS-B reports two altitudes and neither is exactly this one: baro_altitude
// is pressure altitude and geo_altitude is only loosely ellipsoidal. Decide
// which one feeds in here and write it down.
type Geodetic struct {
	Lat float64
	Lon float64
	Alt float64
}

// ECEF is a position in the Earth-centered, Earth-fixed frame, in meters. X
// points at the intersection of the equator and the prime meridian, Z at the
// north pole, Y completes the right-handed set.
type ECEF struct {
	X float64
	Y float64
	Z float64
}

// ENU is a position in a local tangent plane, in meters, relative to the
// origin of the Frame that produced it.
type ENU struct {
	E float64
	N float64
	U float64
}

// Frame is a local east-north-up tangent plane anchored at a fixed geodetic
// origin, in practice the center of the region being tracked.
//
// TODO: build it once and treat it as immutable. Caching the origin's ECEF
// position and the sin/cos of its lat/lon at construction means converting a
// measurement costs no trig, which matters because ToENU runs once per
// measurement per sensor.
type Frame struct{}

// NewFrame returns the tangent plane anchored at the given geodetic origin.
func NewFrame(origin Geodetic) Frame {
	panic("TODO: internal/geo NewFrame")
}

// ECEF converts a geodetic position to Earth-centered Earth-fixed coordinates.
//
// TODO: closed form. The one derived quantity is the prime vertical radius of
// curvature N = a / sqrt(1 - e² sin²φ).
func (g Geodetic) ECEF() ECEF {
	panic("TODO: internal/geo Geodetic.ECEF")
}

// Geodetic converts an ECEF position back to WGS84 lat, lon, and ellipsoidal
// height.
//
// TODO: there is no closed-form inverse, so this is the one that takes
// thought. Options are Bowring's approximation, Bowring seeded then refined by
// fixed-point iteration, or Ferrari's closed-form solution. Notes in
// _reference-geo/NOTES.md measure how accurate each is with altitude; for
// aircraft the bar is low enough that the simplest option is defensible.
//
// Two things worth getting right: use a height formula that does not divide by
// cos(lat), or it blows up near the poles, and prefer atan2 throughout so the
// polar axis needs no special case.
func (c ECEF) Geodetic() Geodetic {
	panic("TODO: internal/geo ECEF.Geodetic")
}

// ECEFToENU expresses an ECEF position in the frame's tangent plane.
func (f Frame) ECEFToENU(c ECEF) ENU {
	panic("TODO: internal/geo Frame.ECEFToENU")
}

// ENUToECEF is the inverse of ECEFToENU. The rotation is orthonormal, so
// undoing it is its transpose.
func (f Frame) ENUToECEF(p ENU) ECEF {
	panic("TODO: internal/geo Frame.ENUToECEF")
}

// ToENU converts a geodetic position into the frame's tangent plane. This is
// the path every incoming measurement takes.
func (f Frame) ToENU(g Geodetic) ENU {
	panic("TODO: internal/geo Frame.ToENU")
}

// FromENU converts a tangent-plane position back to geodetic, for display and
// for writing tracks out.
func (f Frame) FromENU(p ENU) Geodetic {
	panic("TODO: internal/geo Frame.FromENU")
}
