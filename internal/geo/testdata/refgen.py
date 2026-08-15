"""Independent reference implementation used to generate golden values for
internal/geo tests.

The inverse (ECEF -> geodetic) uses Ferrari's closed-form solution, which is a
different algorithm from the Bowring-seeded fixed-point iteration in the Go
code. Agreement between the two is a real cross-check, not a transcription
check.
"""
import math

A = 6378137.0
F = 1.0 / 298.257223563
B = A * (1.0 - F)
E2 = F * (2.0 - F)
EP2 = E2 / (1.0 - E2)


def geodetic_to_ecef(lat_deg, lon_deg, h):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sp, cp = math.sin(lat), math.cos(lat)
    sl, cl = math.sin(lon), math.cos(lon)
    n = A / math.sqrt(1.0 - E2 * sp * sp)
    return (
        (n + h) * cp * cl,
        (n + h) * cp * sl,
        (n * (1.0 - E2) + h) * sp,
    )


def ecef_to_geodetic_ferrari(x, y, z):
    """Ferrari's closed-form solution. Undefined on the polar axis."""
    p = math.hypot(x, y)
    if p == 0.0:
        return (90.0 if z >= 0 else -90.0), 0.0, abs(z) - B
    zz = z * z
    ff = 54.0 * B * B * zz
    g = p * p + (1.0 - E2) * zz - E2 * (A * A - B * B)
    c = E2 * E2 * ff * p * p / (g * g * g)
    s = (1.0 + c + math.sqrt(c * c + 2.0 * c)) ** (1.0 / 3.0)
    k = s + 1.0 + 1.0 / s
    pp = ff / (3.0 * k * k * g * g)
    q = math.sqrt(1.0 + 2.0 * E2 * E2 * pp)
    r0 = -(pp * E2 * p) / (1.0 + q) + math.sqrt(
        abs(0.5 * A * A * (1.0 + 1.0 / q)
            - pp * (1.0 - E2) * zz / (q * (1.0 + q))
            - 0.5 * pp * p * p)
    )
    t = p - E2 * r0
    u = math.sqrt(t * t + zz)
    v = math.sqrt(t * t + (1.0 - E2) * zz)
    z0 = B * B * z / (A * v)
    h = u * (1.0 - B * B / (A * v))
    lat = math.atan2(z + EP2 * z0, p)
    lon = math.atan2(y, x)
    return math.degrees(lat), math.degrees(lon), h


def ecef_to_enu(x, y, z, olat_deg, olon_deg, oh):
    ox, oy, oz = geodetic_to_ecef(olat_deg, olon_deg, oh)
    dx, dy, dz = x - ox, y - oy, z - oz
    lat = math.radians(olat_deg)
    lon = math.radians(olon_deg)
    sp, cp = math.sin(lat), math.cos(lat)
    sl, cl = math.sin(lon), math.cos(lon)
    return (
        -sl * dx + cl * dy,
        -sp * cl * dx - sp * sl * dy + cp * dz,
        cp * cl * dx + cp * sl * dy + sp * dz,
    )


def g(v):
    return repr(float(v))


ELLIPSOID_CASES = [
    ("equator prime meridian", 0.0, 0.0, 0.0),
    ("equator 90E", 0.0, 90.0, 0.0),
    ("equator 180", 0.0, 180.0, 0.0),
    ("equator 90W", 0.0, -90.0, 0.0),
    ("north pole", 90.0, 0.0, 0.0),
    ("south pole", -90.0, 0.0, 0.0),
    ("equator with altitude", 0.0, 0.0, 1000.0),
    ("45N 45E", 45.0, 45.0, 0.0),
    ("Columbus OH", 39.9612, -82.9988, 275.0),
    ("Columbus OH at cruise", 39.9612, -82.9988, 11000.0),
    ("Los Angeles", 33.9425, -118.4081, 38.0),
    ("Sydney", -33.8688, 151.2093, 58.0),
    ("Dead Sea below ellipsoid", 31.5, 35.5, -400.0),
    ("near north pole", 89.9, 45.0, 100.0),
    ("near south pole", -89.9, -135.0, 100.0),
]

ORIGIN = (39.9612, -82.9988, 275.0)  # Columbus, OH

ENU_CASES = [
    ("origin itself", 39.9612, -82.9988, 275.0),
    ("directly above origin", 39.9612, -82.9988, 11275.0),
    ("one degree north", 40.9612, -82.9988, 275.0),
    ("one degree east", 39.9612, -81.9988, 275.0),
    ("one degree south", 38.9612, -82.9988, 275.0),
    ("one degree west", 39.9612, -83.9988, 275.0),
    ("airliner at cruise 200km NE", 41.2, -80.6, 11000.0),
    ("edge of a 2 degree box", 40.9612, -81.9988, 9500.0),
]

print("// ---- ellipsoid cases ----")
for name, lat, lon, h in ELLIPSOID_CASES:
    x, y, z = geodetic_to_ecef(lat, lon, h)
    blat, blon, bh = ecef_to_geodetic_ferrari(x, y, z)
    # Report how far Ferrari lands from the input, as a check on the pair.
    dlat = abs(blat - lat)
    dlon = abs(blon - lon) if abs(lat) < 89.999 else 0.0
    dh = abs(bh - h)
    print('\t{{"%s", Geodetic{%s, %s, %s}, ECEF{%s, %s, %s}}}, // ferrari back-err lat %.3g deg lon %.3g deg h %.3g m'
          % (name, g(lat), g(lon), g(h), g(x), g(y), g(z), dlat, dlon, dh))

print()
print("// ---- ENU cases, origin %r ----" % (ORIGIN,))
for name, lat, lon, h in ENU_CASES:
    x, y, z = geodetic_to_ecef(lat, lon, h)
    e, n, u = ecef_to_enu(x, y, z, *ORIGIN)
    print('\t{{"%s", Geodetic{%s, %s, %s}, ENU{%s, %s, %s}}},'
          % (name, g(lat), g(lon), g(h), g(e), g(n), g(u)))

print()
print("// ---- derived constants ----")
print("b  =", g(B))
print("e2 =", g(E2))
print("ep2 =", g(EP2))
