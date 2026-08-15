package geo

import (
	"math"
	"testing"
)

// These are exact rigid transforms, so the only error should be float64
// roundoff, about a nanometer at Earth radius. A micrometer of slack is
// generous; anything past it is a bug, not noise.
const (
	tolMeters  = 1e-6
	tolDegrees = 1e-9
)

// Expected values come from testdata/refgen.py, which inverts the ellipsoid
// with Ferrari's closed form rather than the Bowring iteration used here. The
// two agree to 7e-15 degrees, so this checks the math, not the transcription.
var ellipsoidCases = []struct {
	name string
	g    Geodetic
	c    ECEF
	// lonUndefined marks the polar axis, where longitude carries no meaning.
	lonUndefined bool
}{
	{name: "equator prime meridian", g: Geodetic{0.0, 0.0, 0.0}, c: ECEF{6378137.0, 0.0, 0.0}},
	{name: "equator 90E", g: Geodetic{0.0, 90.0, 0.0}, c: ECEF{3.905482530786651e-10, 6378137.0, 0.0}},
	{name: "equator 180", g: Geodetic{0.0, 180.0, 0.0}, c: ECEF{-6378137.0, 7.810965061573302e-10, 0.0}},
	{name: "equator 90W", g: Geodetic{0.0, -90.0, 0.0}, c: ECEF{3.905482530786651e-10, -6378137.0, 0.0}},
	{name: "north pole", g: Geodetic{90.0, 0.0, 0.0}, c: ECEF{3.9186209248144716e-10, 0.0, 6356752.314245179}, lonUndefined: true},
	{name: "south pole", g: Geodetic{-90.0, 0.0, 0.0}, c: ECEF{3.9186209248144716e-10, 0.0, -6356752.314245179}, lonUndefined: true},
	{name: "equator with altitude", g: Geodetic{0.0, 0.0, 1000.0}, c: ECEF{6379137.0, 0.0, 0.0}},
	{name: "45N 45E", g: Geodetic{45.0, 45.0, 0.0}, c: ECEF{3194419.1450605746, 3194419.145060574, 4487348.408865919}},
	{name: "Columbus OH", g: Geodetic{39.9612, -82.9988, 275.0}, c: ECEF{596735.8670062379, -4859182.274880405, 4074861.040214553}},
	{name: "Columbus OH at cruise", g: Geodetic{39.9612, -82.9988, 11000.0}, c: ECEF{597737.8640048217, -4867341.472817389, 4081749.37209099}},
	{name: "Los Angeles", g: Geodetic{33.9425, -118.4081, 38.0}, c: ECEF{-2519970.5495130317, -4659013.629316602, 3541178.3850251455}},
	{name: "Sydney", g: Geodetic{-33.8688, 151.2093, 58.0}, c: ECEF{-4646093.477288303, 2553229.5358170713, -3534404.710910369}},
	{name: "Dead Sea below ellipsoid", g: Geodetic{31.5, 35.5, -400.0}, c: ECEF{4431142.0419520065, 3160702.901391348, 3313078.018134642}},
	{name: "near north pole", g: Geodetic{89.9, 45.0, 100.0}, c: ECEF{7898.0763589197895, 7898.076358919789, 6356842.566957005}},
	{name: "near south pole", g: Geodetic{-89.9, -135.0, 100.0}, c: ECEF{-7898.076358919789, -7898.0763589197895, -6356842.566957005}},
}

func TestGeodeticToECEF(t *testing.T) {
	for _, tc := range ellipsoidCases {
		t.Run(tc.name, func(t *testing.T) {
			got := tc.g.ECEF()
			assertClose(t, "X", got.X, tc.c.X, tolMeters)
			assertClose(t, "Y", got.Y, tc.c.Y, tolMeters)
			assertClose(t, "Z", got.Z, tc.c.Z, tolMeters)
		})
	}
}

func TestECEFToGeodetic(t *testing.T) {
	for _, tc := range ellipsoidCases {
		t.Run(tc.name, func(t *testing.T) {
			got := tc.c.Geodetic()
			assertClose(t, "Lat", got.Lat, tc.g.Lat, tolDegrees)
			if !tc.lonUndefined {
				assertClose(t, "Lon", got.Lon, tc.g.Lon, tolDegrees)
			}
			assertClose(t, "Alt", got.Alt, tc.g.Alt, tolMeters)
		})
	}
}

type enuCase struct {
	name string
	g    Geodetic
	p    ENU
}

// Two origins on purpose. St. Louis is the real frame but sits a fifth of a
// degree off the 90th meridian, so its cos(lon) is 0.0035 and three of the
// nine coefficients are nearly muted. Columbus has all four values distinct.
var (
	columbusOrigin = Geodetic{39.9612, -82.9988, 275.0}
	stLouisOrigin  = Geodetic{38.6270, -90.1994, 142.0}
)

var columbusCases = []enuCase{
	{"origin itself", Geodetic{39.9612, -82.9988, 275.0}, ENU{0.0, 0.0, 0.0}},
	{"directly above origin", Geodetic{39.9612, -82.9988, 11275.0}, ENU{-2.6489033189136535e-11, 1.9281287677586079e-10, 10999.999999999794}},
	{"one degree north", Geodetic{40.9612, -82.9988, 275.0}, ENU{-5.4569682106375694e-11, 111042.67470396207, -969.0827711210222}},
	{"one degree east", Geodetic{39.9612, -81.9988, 275.0}, ENU{85441.51017614038, 478.8991593328037, -571.5153346586494}},
	{"one degree south", Geodetic{38.9612, -82.9988, 275.0}, ENU{2.9103830456733704e-11, -111023.465655559, -968.8592579999022}},
	{"one degree west", Geodetic{39.9612, -83.9988, 275.0}, ENU{-85441.51017614047, 478.89915933288466, -571.5153346587504}},
	{"airliner at cruise 200km NE", Geodetic{41.2, -80.6, 11000.0}, ENU{201499.81616449796, 140500.19863879707, 6001.739457669231}},
	{"edge of a 2 degree box", Geodetic{40.9612, -81.9988, 9500.0}, ENU{84305.43313565961, 111676.204607585, 7690.596062443627}},
}

var stLouisCases = []enuCase{
	{"origin itself", Geodetic{38.627, -90.1994, 142.0}, ENU{0.0, 0.0, 0.0}},
	{"overhead at cruise", Geodetic{38.627, -90.1994, 10668.0}, ENU{8.419931418757187e-13, -2.828528522513807e-10, 10526.000000000258}},
	{"due north one degree", Geodetic{39.627, -90.1994, 142.0}, ENU{1.0231815394945443e-12, 111014.75341941937, -968.8388561971369}},
	{"due east one degree", Geodetic{38.627, -89.1994, 142.0}, ENU{87076.89524019102, 474.3712498358642, -593.6606145986386}},
	{"NE of the city", Geodetic{39.4, -89.3, 11277.6}, ENU{77604.81720607875, 86344.948855298, 10079.831306476997}},
	{"SW near the box edge", Geodetic{37.4, -91.8, 9448.8}, ENU{-141913.52936160073, -135147.44356835316, 6297.769589876902}},
	{"NE corner of the box", Geodetic{40.127, -88.1994, 0.0}, ENU{170436.27555112465, 168372.14561968177, -4646.024949622006}},
}

var enuTables = []struct {
	name   string
	origin Geodetic
	cases  []enuCase
}{
	{"Columbus", columbusOrigin, columbusCases},
	{"StLouis", stLouisOrigin, stLouisCases},
}

func TestFrameToENU(t *testing.T) {
	for _, tbl := range enuTables {
		t.Run(tbl.name, func(t *testing.T) {
			f := NewFrame(tbl.origin)
			for _, tc := range tbl.cases {
				t.Run(tc.name, func(t *testing.T) {
					got := f.ToENU(tc.g)
					assertClose(t, "E", got.E, tc.p.E, tolMeters)
					assertClose(t, "N", got.N, tc.p.N, tolMeters)
					assertClose(t, "U", got.U, tc.p.U, tolMeters)
				})
			}
		})
	}
}

func TestFrameFromENU(t *testing.T) {
	for _, tbl := range enuTables {
		t.Run(tbl.name, func(t *testing.T) {
			f := NewFrame(tbl.origin)
			for _, tc := range tbl.cases {
				t.Run(tc.name, func(t *testing.T) {
					got := f.FromENU(tc.p)
					assertClose(t, "Lat", got.Lat, tc.g.Lat, tolDegrees)
					assertClose(t, "Lon", got.Lon, tc.g.Lon, tolDegrees)
					assertClose(t, "Alt", got.Alt, tc.g.Alt, tolMeters)
				})
			}
		})
	}
}

// Weak on its own: the origin subtracts to a zero displacement, and rotating
// zero gives zero however wrong the rotation is.
func TestFrameOriginIsZero(t *testing.T) {
	for _, tbl := range enuTables {
		t.Run(tbl.name, func(t *testing.T) {
			f := NewFrame(tbl.origin)
			if f.Origin() != tbl.origin {
				t.Errorf("Origin() = %+v, want %+v", f.Origin(), tbl.origin)
			}
			got := f.ToENU(tbl.origin)
			assertClose(t, "E", got.E, 0, tolMeters)
			assertClose(t, "N", got.N, 0, tolMeters)
			assertClose(t, "U", got.U, 0, tolMeters)
		})
	}
}

// Guards against a transposed or reordered rotation. The diagonal case matters
// most: a point moving along one axis cannot exercise the other coefficients.
func TestAxisDirections(t *testing.T) {
	for _, tbl := range enuTables {
		t.Run(tbl.name, func(t *testing.T) {
			o := tbl.origin
			f := NewFrame(o)

			north := f.ToENU(Geodetic{o.Lat + 0.5, o.Lon, o.Alt})
			if north.N <= 0 {
				t.Errorf("a position north of the origin has N = %g, want positive", north.N)
			}
			if math.Abs(north.E) > tolMeters {
				t.Errorf("a position due north of the origin has E = %g, want zero", north.E)
			}

			east := f.ToENU(Geodetic{o.Lat, o.Lon + 0.5, o.Alt})
			if east.E <= 0 {
				t.Errorf("a position east of the origin has E = %g, want positive", east.E)
			}

			diag := f.ToENU(Geodetic{o.Lat + 0.5, o.Lon + 0.5, o.Alt})
			if diag.E <= 0 || diag.N <= 0 {
				t.Errorf("a position northeast of the origin has E = %g, N = %g, want both positive", diag.E, diag.N)
			}

			up := f.ToENU(Geodetic{o.Lat, o.Lon, o.Alt + 1000})
			assertClose(t, "U", up.U, 1000, tolMeters)
			assertClose(t, "E", up.E, 0, tolMeters)
			assertClose(t, "N", up.N, 0, tolMeters)
		})
	}
}

// If the rotation is not orthonormal then ENUToECEF is not the inverse of
// ECEFToENU, distances in the plane are not distances on the ground, and the
// filter's covariance stops meaning anything.
func TestFrameAxesOrthonormal(t *testing.T) {
	for _, tbl := range enuTables {
		t.Run(tbl.name, func(t *testing.T) {
			f := NewFrame(tbl.origin)

			// A kilometer, not a meter: recovering the axis differences
			// against an ECEF magnitude of 6.4e6 where one ulp is a nanometer.
			const scale = 1000.0
			names := []string{"east", "north", "up"}
			axes := []ENU{{E: scale}, {N: scale}, {U: scale}}

			var v [3][3]float64
			for i, ax := range axes {
				c := f.ENUToECEF(ax)
				v[i] = [3]float64{
					(c.X - f.center.X) / scale,
					(c.Y - f.center.Y) / scale,
					(c.Z - f.center.Z) / scale,
				}
			}

			const tolUnit = 1e-11
			for i := range v {
				for j := i; j < len(v); j++ {
					dot := v[i][0]*v[j][0] + v[i][1]*v[j][1] + v[i][2]*v[j][2]
					want := 0.0
					if i == j {
						want = 1.0
					}
					if math.Abs(dot-want) > tolUnit {
						t.Errorf("%s . %s = %.15g, want %g", names[i], names[j], dot, want)
					}
				}
			}
		})
	}
}

// Quantifies the approximation the design rests on: a tangent plane departs
// from the ellipsoid by d²/2R, so a point flat in the plane reads that much
// higher than the origin. Confirms the plane is oriented correctly.
func TestTangentPlaneRisesAboveEllipsoid(t *testing.T) {
	o := columbusOrigin
	f := NewFrame(o)

	// Meridional radius of curvature, the relevant R for a due-north offset.
	sinLat := math.Sin(rad(o.Lat))
	w := 1.0 - ecc2*sinLat*sinLat
	m := semiMajorAxis * (1.0 - ecc2) / (w * math.Sqrt(w))

	for _, d := range []float64{1e3, 1e4, 5e4, 1e5, 2e5} {
		got := f.FromENU(ENU{N: d}).Alt - o.Alt
		want := d * d / (2 * m)

		// Next term is O(d⁴/R³), 6e-5 of the leading term at 200 km.
		if math.Abs(got-want) > 1e-3*want {
			t.Errorf("at %g m north: rise = %.3f m, want ~%.3f m (d²/2R)", d, got, want)
		}
	}
}

func TestGeodeticECEFRoundTrip(t *testing.T) {
	for lat := -89.0; lat <= 89.0; lat += 7.0 {
		for lon := -180.0; lon < 180.0; lon += 13.0 {
			for _, alt := range []float64{-400, 0, 275, 11000, 40000} {
				want := Geodetic{lat, lon, alt}
				got := want.ECEF().Geodetic()
				assertClose(t, "Lat", got.Lat, want.Lat, tolDegrees)
				assertClose(t, "Lon", got.Lon, want.Lon, tolDegrees)
				assertClose(t, "Alt", got.Alt, want.Alt, tolMeters)
			}
		}
	}
}

// Sweeps a region several times larger than the tracker will use, since the
// conversion should not be what limits how big a region can be.
func TestFrameRoundTrip(t *testing.T) {
	for _, tbl := range enuTables {
		t.Run(tbl.name, func(t *testing.T) {
			o := tbl.origin
			f := NewFrame(o)
			for dLat := -3.0; dLat <= 3.0; dLat += 0.75 {
				for dLon := -3.0; dLon <= 3.0; dLon += 0.75 {
					for _, alt := range []float64{0, 275, 11000} {
						want := Geodetic{o.Lat + dLat, o.Lon + dLon, alt}
						got := f.FromENU(f.ToENU(want))
						assertClose(t, "Lat", got.Lat, want.Lat, tolDegrees)
						assertClose(t, "Lon", got.Lon, want.Lon, tolDegrees)
						assertClose(t, "Alt", got.Alt, want.Alt, tolMeters)
					}
				}
			}
		})
	}
}

func TestENURoundTrip(t *testing.T) {
	for _, tbl := range enuTables {
		t.Run(tbl.name, func(t *testing.T) {
			f := NewFrame(tbl.origin)
			for _, want := range []ENU{
				{},
				{E: 1, N: 2, U: 3},
				{E: -150000, N: 90000, U: 11000},
				{E: 250000, N: -250000, U: -300},
			} {
				got := f.ECEFToENU(f.ENUToECEF(want))
				assertClose(t, "E", got.E, want.E, tolMeters)
				assertClose(t, "N", got.N, want.N, tolMeters)
				assertClose(t, "U", got.U, want.U, tolMeters)
			}
		})
	}
}

func assertClose(t *testing.T, field string, got, want, tol float64) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s = %.12g, want %.12g (off by %.3g, tolerance %g)", field, got, want, got-want, tol)
	}
}

// ToENU runs once per measurement per sensor, so it sits on the hot path.
func BenchmarkFrameToENU(b *testing.B) {
	f := NewFrame(stLouisOrigin)
	g := Geodetic{39.4, -89.3, 11277.6}
	var sink ENU
	for b.Loop() {
		sink = f.ToENU(g)
	}
	_ = sink
}

func BenchmarkFrameFromENU(b *testing.B) {
	f := NewFrame(stLouisOrigin)
	p := ENU{E: 77604.8, N: 86344.9, U: 10079.8}
	var sink Geodetic
	for b.Loop() {
		sink = f.FromENU(p)
	}
	_ = sink
}
