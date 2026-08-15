// Package geo converts between WGS84 geodetic coordinates, ECEF, and a local
// east-north-up tangent plane. The tracker works in ENU because a
// constant-velocity model needs a Cartesian frame, and lat/lon is angular.
package geo

import "math"

const (
	semiMajorAxis = 6378137.0
	flattening    = 1.0 / 298.257223563
	semiMinorAxis = semiMajorAxis * (1.0 - flattening)
	ecc2          = flattening * (2.0 - flattening)

	// ecc2Prime is the second eccentricity squared, used only by Bowring.
	ecc2Prime = ecc2 / (1.0 - ecc2)
)

// Convergence limits for the latitude iteration in ECEF.Geodetic. The
// tolerance is roughly 60 nanometers of arc, and the seed is good enough that
// the loop normally exits after one or two passes.
const (
	latTolerance = 1e-14
	latMaxIter   = 8
)

// Geodetic is a WGS84 position: Lat and Lon in degrees, Alt in meters above
// the ellipsoid. ADS-B's baro_altitude and geo_altitude are neither of these
// exactly, so decide which one feeds in here and write it down.
type Geodetic struct {
	Lat float64
	Lon float64
	Alt float64
}

// ECEF is a position in the Earth-centered, Earth-fixed frame, in meters. X
// points where the equator meets the prime meridian, Z at the north pole.
type ECEF struct {
	X float64
	Y float64
	Z float64
}

// ENU is a position in meters in a local tangent plane, relative to the origin
// of the Frame that produced it.
type ENU struct {
	E float64
	N float64
	U float64
}

// Frame is a local east-north-up tangent plane anchored at a fixed geodetic
// origin. Immutable once built and safe for concurrent use; caching the
// origin's ECEF and trig means ToENU costs no trig per measurement.
type Frame struct {
	origin                         Geodetic
	center                         ECEF
	sinLat, cosLat, sinLon, cosLon float64
}

// NewFrame returns the tangent plane anchored at the given geodetic origin.
func NewFrame(origin Geodetic) Frame {
	// Go's math package expects radians not degrees.
	sinLat, cosLat := math.Sincos(rad(origin.Lat))
	sinLon, cosLon := math.Sincos(rad(origin.Lon))

	return Frame{
		origin: origin,
		center: origin.ECEF(),
		sinLat: sinLat,
		cosLat: cosLat,
		sinLon: sinLon,
		cosLon: cosLon,
	}
}

// Origin returns the geodetic position the frame is anchored at.
func (f Frame) Origin() Geodetic { return f.origin }

// ECEF converts a geodetic position to Earth-centered Earth-fixed coordinates.
// N is the prime vertical radius of curvature: the distance to the polar axis
// measured along the ellipsoid normal.
func (g Geodetic) ECEF() ECEF {
	sinLat, cosLat := math.Sincos(rad(g.Lat))
	sinLon, cosLon := math.Sincos(rad(g.Lon))

	N := semiMajorAxis / math.Sqrt(1-ecc2*sinLat*sinLat)

	// Distance from polar axis
	P := (N + g.Alt) * cosLat

	return ECEF{
		X: P * cosLon,
		Y: P * sinLon,
		Z: (N*(1-ecc2) + g.Alt) * sinLat,
	}
}

// Geodetic converts an ECEF position back to WGS84. There is no closed form,
// since latitude appears inside N, so this seeds with Bowring's estimate and
// refines by iteration. atan2 throughout, so the polar axis needs no case.
func (c ECEF) Geodetic() Geodetic {
	// p is the distance from the polar axis.
	p := math.Hypot(c.X, c.Y)

	// Bowring's estimate: exact on a sphere, and good to under a microradian
	// on the ellipsoid at any altitude an aircraft flies.
	theta := math.Atan2(c.Z*semiMajorAxis, p*semiMinorAxis)
	sinTheta, cosTheta := math.Sincos(theta)
	lat := math.Atan2(
		c.Z+ecc2Prime*semiMinorAxis*sinTheta*sinTheta*sinTheta,
		p-ecc2*semiMajorAxis*cosTheta*cosTheta*cosTheta,
	)

	for range latMaxIter {
		sinLat := math.Sin(lat)
		n := semiMajorAxis / math.Sqrt(1-ecc2*sinLat*sinLat)
		next := math.Atan2(c.Z+ecc2*n*sinLat, p)
		converged := math.Abs(next-lat) < latTolerance
		lat = next
		if converged {
			break
		}
	}

	// Height projected onto the local vertical. Exact given lat, and unlike
	// the p/cos(lat) - N form it does not blow up near the poles.
	sinLat, cosLat := math.Sincos(lat)
	alt := p*cosLat + c.Z*sinLat - semiMajorAxis*math.Sqrt(1-ecc2*sinLat*sinLat)

	return Geodetic{
		Lat: deg(lat),
		Lon: deg(math.Atan2(c.Y, c.X)),
		Alt: alt,
	}
}

// ECEFToENU expresses an ECEF position in the frame's tangent plane.
func (f Frame) ECEFToENU(c ECEF) ENU {
	dx := c.X - f.center.X
	dy := c.Y - f.center.Y
	dz := c.Z - f.center.Z

	return ENU{
		E: -f.sinLon*dx + f.cosLon*dy,
		N: -f.sinLat*f.cosLon*dx - f.sinLat*f.sinLon*dy + f.cosLat*dz,
		U: f.cosLat*f.cosLon*dx + f.cosLat*f.sinLon*dy + f.sinLat*dz,
	}
}

// ENUToECEF is the inverse of ECEFToENU: rotate the offset back onto the ECEF
// axes, then add the origin. It reuses the same nine coefficients because
// east, north and up are mutually perpendicular and each one meter long.
func (f Frame) ENUToECEF(p ENU) ECEF {
	dx := -f.sinLon*p.E - f.sinLat*f.cosLon*p.N + f.cosLat*f.cosLon*p.U
	dy := f.cosLon*p.E - f.sinLat*f.sinLon*p.N + f.cosLat*f.sinLon*p.U
	dz := f.cosLat*p.N + f.sinLat*p.U

	return ECEF{
		X: f.center.X + dx,
		Y: f.center.Y + dy,
		Z: f.center.Z + dz,
	}
}

// ToENU converts a geodetic position into the frame's tangent plane. Every
// incoming measurement takes this path.
func (f Frame) ToENU(g Geodetic) ENU { return f.ECEFToENU(g.ECEF()) }

// FromENU converts a tangent-plane position back to geodetic, for display and
// for writing tracks out.
func (f Frame) FromENU(p ENU) Geodetic { return f.ENUToECEF(p).Geodetic() }

func rad(d float64) float64 { return d * math.Pi / 180 }
func deg(r float64) float64 { return r * 180 / math.Pi }
