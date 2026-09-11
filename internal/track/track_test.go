package track

import (
	"testing"
	"time"

	"github.com/sambixel/trackfusion/internal/geo"
)

var epoch = time.Date(2026, 9, 10, 12, 0, 0, 0, time.UTC)

func mkMeas(id string, e, n float64) Measurement {
	return Measurement{
		TimeStamp: epoch,
		Position:  geo.ENU{E: e, N: n, U: 10000},
		Identity:  id,
		R:         [2][2]float64{{90, 0}, {0, 400}},
	}
}

func TestNewStoreIsEmpty(t *testing.T) {
	if got := NewStore().All(); len(got) != 0 {
		t.Errorf("want no tracks, got %d", len(got))
	}
}

// A track starts where the measurement was, as uncertain about that position as
// the sensor was, and with no idea at all which way the aircraft is going.
func TestSpawnSeedsFromTheMeasurement(t *testing.T) {
	s := NewStore()
	got := s.Spawn(mkMeas("abc123", 5000, -2500))

	if got.State.X[0] != 5000 || got.State.X[1] != -2500 {
		t.Errorf("position = (%g, %g), want (5000, -2500)", got.State.X[0], got.State.X[1])
	}
	if got.State.X[2] != 0 || got.State.X[3] != 0 {
		t.Errorf("velocity = (%g, %g), want zero: one point says nothing about motion",
			got.State.X[2], got.State.X[3])
	}
	if got.State.P[0][0] != 90 || got.State.P[1][1] != 400 {
		t.Errorf("position variance = (%g, %g), want the sensor's own (90, 400)",
			got.State.P[0][0], got.State.P[1][1])
	}
	if !got.LastUpdate.Equal(epoch) || got.MissCounter != 0 {
		t.Errorf("new track started stale: %v, %d misses", got.LastUpdate, got.MissCounter)
	}
}

// Velocity starts at zero, so the initial variance has to be wide enough to
// cover any aircraft that could have produced the report. Too narrow and the
// track insists it is stationary, the gate stays centred where the aircraft
// was, and the second report falls outside it — the track dies at two points
// having never been wrong about anything except its own confidence.
func TestSpawnAdmitsItDoesNotKnowTheVelocity(t *testing.T) {
	got := NewStore().Spawn(mkMeas("abc123", 0, 0))

	const wantVar = 300.0 * 300.0
	if got.State.P[2][2] != wantVar || got.State.P[3][3] != wantVar {
		t.Errorf("velocity variance = (%g, %g), want %g: about 300 m/s of doubt either way",
			got.State.P[2][2], got.State.P[3][3], wantVar)
	}

	// Nothing yet links position to velocity, and claiming otherwise would
	// have the first correction move the velocity for a reason the filter has
	// not earned.
	for _, ij := range [][2]int{{0, 2}, {2, 0}, {1, 3}, {3, 1}, {0, 3}, {1, 2}} {
		if v := got.State.P[ij[0]][ij[1]]; v != 0 {
			t.Errorf("P[%d][%d] = %g, want 0", ij[0], ij[1], v)
		}
	}
}

// A cooperative report names its aircraft and the new track inherits that name,
// which is what lets the next report from it skip association entirely.
func TestSpawnCarriesIdentityWhenThereIsOne(t *testing.T) {
	s := NewStore()

	named := s.Spawn(mkMeas("abc123", 0, 0))
	if named.Identity != "abc123" {
		t.Errorf("identity = %q, want abc123", named.Identity)
	}

	// A radar return names nothing, and the track it starts is a position and
	// a hope. It has to be able to exist anyway: an aircraft the ADS-B feed
	// has never mentioned is exactly the one worth tracking.
	anon := s.Spawn(mkMeas("", 100, 100))
	if anon.Identity != "" {
		t.Errorf("identity = %q, want empty", anon.Identity)
	}
}

// IDs are handed out in order and never reused. They are not aircraft
// identities: a track can be holding the wrong aircraft, and an ID that got
// recycled would make that impossible to see in a log.
func TestIDsAreUniqueAndOrdered(t *testing.T) {
	s := NewStore()

	var ids []int64
	for range 5 {
		ids = append(ids, s.Spawn(mkMeas("", 0, 0)).ID)
	}

	s.tracks[0].MissCounter = 99
	s.Prune(3)
	after := s.Spawn(mkMeas("", 0, 0)).ID

	for i := 1; i < len(ids); i++ {
		if ids[i] <= ids[i-1] {
			t.Fatalf("IDs not increasing: %v", ids)
		}
	}
	if after <= ids[len(ids)-1] {
		t.Errorf("ID %d reused after a prune; last handed out was %d", after, ids[len(ids)-1])
	}
}

// Association returns indices into this slice rather than IDs, so the order has
// to hold still between the call that reads it and the call that acts on it.
func TestAllIsStable(t *testing.T) {
	s := NewStore()
	for i := range 4 {
		s.Spawn(mkMeas("", float64(i)*1000, 0))
	}

	first := s.All()
	for range 10 {
		again := s.All()
		for i := range first {
			if first[i] != again[i] {
				t.Fatalf("track %d moved between calls", i)
			}
		}
	}
}

// The boundary is where a lifecycle rule is usually wrong by one. A track on
// exactly the limit has not yet exceeded it and stays.
func TestPruneKeepsTracksOnTheLimit(t *testing.T) {
	s := NewStore()
	for _, misses := range []int64{0, 2, 3, 4} {
		s.Spawn(mkMeas("", 0, 0)).MissCounter = misses
	}

	s.Prune(3)

	got := s.All()
	if len(got) != 3 {
		t.Fatalf("want 3 survivors, got %d", len(got))
	}
	if got[2].MissCounter != 3 {
		t.Errorf("the track on exactly 3 misses was dropped")
	}
}

// Pruning filters in place, which reuses the backing array. That is worth doing
// and it is also the version of this loop that quietly reorders the survivors
// if written carelessly, so the order is checked rather than assumed.
func TestPrunePreservesOrder(t *testing.T) {
	s := NewStore()

	var want []int64
	for i := range 10 {
		tr := s.Spawn(mkMeas("", float64(i), 0))
		if i%3 == 0 {
			tr.MissCounter = 100
			continue
		}
		want = append(want, tr.ID)
	}

	s.Prune(5)

	got := s.All()
	if len(got) != len(want) {
		t.Fatalf("want %d survivors, got %d", len(want), len(got))
	}
	for i := range want {
		if got[i].ID != want[i] {
			t.Fatalf("survivor %d is track %d, want %d (order changed)", i, got[i].ID, want[i])
		}
	}
}

func TestPruneCanEmptyTheStore(t *testing.T) {
	s := NewStore()
	for range 3 {
		s.Spawn(mkMeas("", 0, 0)).MissCounter = 9
	}

	s.Prune(2)

	if got := s.All(); len(got) != 0 {
		t.Errorf("want an empty store, got %d tracks", len(got))
	}
}

// Pruning nothing must not disturb anything, which is the common case: most
// scans match most tracks.
func TestPruneIsANoOpWhenNothingIsStale(t *testing.T) {
	s := NewStore()
	for range 4 {
		s.Spawn(mkMeas("", 0, 0))
	}

	before := append([]*Track(nil), s.All()...)
	s.Prune(3)
	after := s.All()

	if len(after) != len(before) {
		t.Fatalf("want %d tracks, got %d", len(before), len(after))
	}
	for i := range before {
		if before[i] != after[i] {
			t.Errorf("track %d was replaced", i)
		}
	}
}
