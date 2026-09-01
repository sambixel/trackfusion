package assoc

import (
	"testing"

	"github.com/sambixel/trackfusion/internal/filter"
	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/track"
)

// Every track in these tests carries the same position uncertainty and every
// measurement the same noise, so the scaled distance is just the squared
// separation over 100. That keeps the arithmetic checkable by hand and puts the
// focus on which pairing gets chosen rather than on the filter.
const (
	posVar  = 50.0
	measVar = 50.0
	scale   = posVar + measVar
)

func mkTrack(id int64, identity string, e, n float64) *track.Track {
	return &track.Track{
		ID:       id,
		Identity: identity,
		State: filter.State{
			X: [4]float64{e, n, 0, 0},
			P: [4][4]float64{
				{posVar, 0, 0, 0},
				{0, posVar, 0, 0},
				{0, 0, 1, 0},
				{0, 0, 0, 1},
			},
		},
	}
}

func mkMeas(identity string, e, n float64) track.Measurement {
	return track.Measurement{
		Identity: identity,
		Position: geo.ENU{E: e, N: n},
		R:        [2][2]float64{{measVar, 0}, {0, measVar}},
	}
}

func pairsOf(t *testing.T, ps []Pair) map[[2]int]Pair {
	t.Helper()
	out := make(map[[2]int]Pair, len(ps))
	for _, p := range ps {
		out[[2]int{p.TrackIdx, p.MeasIdx}] = p
	}
	return out
}

// A named aircraft is not an inference problem. The report is taken even though
// it lands nowhere near where the track expected it, because a gap in the feed
// is a much likelier explanation than the aircraft misreporting its own address.
func TestIdentitySkipsTheGate(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "abc123", 0, 0)}
	meas := []track.Measurement{mkMeas("abc123", 50000, 50000)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8, AmbiguityMargin: 1.0})

	if len(got.Matched) != 1 {
		t.Fatalf("want 1 match, got %d (%+v)", len(got.Matched), got)
	}
	if got.Matched[0].Reason != ByIdentity {
		t.Errorf("want ByIdentity, got %v", got.Matched[0].Reason)
	}
	if len(got.UnmatchedTracks) != 0 || len(got.UnmatchedMeas) != 0 {
		t.Errorf("nothing should be left over: %+v", got)
	}
}

// An address nobody is holding yet is still a position problem for the tracks
// already up, and a candidate for a new track if it matches none of them.
func TestUnknownIdentityFallsThroughToPosition(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "abc123", 0, 0)}
	meas := []track.Measurement{mkMeas("zzz999", 5, 0)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8})

	if len(got.Matched) != 1 || got.Matched[0].Reason != ByPosition {
		t.Fatalf("want one position match, got %+v", got)
	}
}

func TestGateRejectsWhatIsTooFar(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", 0, 0)}
	meas := []track.Measurement{mkMeas("", 5000, 0)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8, AmbiguityMargin: 1.0})

	if len(got.Matched) != 0 || len(got.Withheld) != 0 {
		t.Fatalf("nothing should have paired: %+v", got)
	}
	if len(got.UnmatchedTracks) != 1 {
		t.Errorf("the track saw nothing, so it missed: %+v", got.UnmatchedTracks)
	}
	if len(got.UnmatchedMeas) != 1 {
		t.Errorf("the return belongs to nothing held, so it can start a track: %+v", got.UnmatchedMeas)
	}
}

// The case the package exists for. Track B sits almost on top of return 0, so
// taking the closest pair first hands B that return and strands A with the far
// one. Settling the scan as a whole is cheaper the other way round.
//
//	A(0,0)      B(10,0)  r0(11,0)                        r1(30,0)
//
//	     greedy: B-r0 = 0.01, A-r1 = 9.00   total 9.01
//	     global: A-r0 = 1.21, B-r1 = 4.00   total 5.21
func TestGlobalAssignmentBeatsTakingTheClosestFirst(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", 0, 0), mkTrack(2, "", 10, 0)}
	meas := []track.Measurement{mkMeas("", 11, 0), mkMeas("", 30, 0)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8})

	if len(got.Matched) != 2 {
		t.Fatalf("want both paired, got %+v", got)
	}
	ps := pairsOf(t, got.Matched)
	if _, ok := ps[[2]int{0, 0}]; !ok {
		t.Errorf("track A should have taken return 0, got %+v", got.Matched)
	}
	if _, ok := ps[[2]int{1, 1}]; !ok {
		t.Errorf("track B should have taken return 1, got %+v", got.Matched)
	}
}

// Two tracks, two returns, arranged so that both readings of the scan cost
// exactly the same. There is no information here that favours either pairing,
// so committing to one would be a coin flip that the filter would then treat as
// fact. Both are handed back untouched.
//
//	           r1(0,5)
//	A(-5,0)              B(5,0)
//	           r0(0,-5)
func TestPerfectlyAmbiguousScanIsWithheld(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", -5, 0), mkTrack(2, "", 5, 0)}
	meas := []track.Measurement{mkMeas("", 0, -5), mkMeas("", 0, 5)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8, AmbiguityMargin: 0.25})

	if len(got.Matched) != 0 {
		t.Errorf("nothing here is decidable, but got matches: %+v", got.Matched)
	}
	if len(got.Withheld) != 2 {
		t.Errorf("want both pairs withheld, got %+v", got.Withheld)
	}
	// The distinction the caller depends on: a crowded track has not missed
	// anything, and a return that could not be attributed must not be allowed
	// to start a second track for an aircraft already held.
	if len(got.UnmatchedTracks) != 0 {
		t.Errorf("withheld is not a miss, so pruning must not see it: %+v", got.UnmatchedTracks)
	}
	if len(got.UnmatchedMeas) != 0 {
		t.Errorf("withheld must not spawn a duplicate track: %+v", got.UnmatchedMeas)
	}
}

// The same scan with the check off. This is the comparison the ambiguity margin
// is meant to be judged against, and the reason zero disables it rather than
// meaning something.
func TestAmbiguityCheckOffCommitsToTheCoinFlip(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", -5, 0), mkTrack(2, "", 5, 0)}
	meas := []track.Measurement{mkMeas("", 0, -5), mkMeas("", 0, 5)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8})

	if len(got.Matched) != 2 || len(got.Withheld) != 0 {
		t.Fatalf("want two committed matches and no withholding, got %+v", got)
	}
}

// A clear winner must survive the ambiguity check, or the tracker would never
// commit to anything in traffic.
func TestClearWinnerIsNotWithheld(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", 0, 0), mkTrack(2, "", 200, 0)}
	meas := []track.Measurement{mkMeas("", 2, 0), mkMeas("", 202, 0)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8, AmbiguityMargin: 1.0})

	if len(got.Matched) != 2 || len(got.Withheld) != 0 {
		t.Fatalf("want both committed, got %+v", got)
	}
}

// More returns than tracks: the surplus is a spawn candidate, not an error.
func TestSurplusMeasurementsBecomeSpawnCandidates(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", 0, 0)}
	meas := []track.Measurement{mkMeas("", 2, 0), mkMeas("", 4000, 0)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8})

	if len(got.Matched) != 1 || got.Matched[0].MeasIdx != 0 {
		t.Fatalf("want the near return matched, got %+v", got)
	}
	if len(got.UnmatchedMeas) != 1 || got.UnmatchedMeas[0] != 1 {
		t.Fatalf("want the far return left to spawn, got %+v", got.UnmatchedMeas)
	}
}

// More tracks than returns: the surplus missed, and will eventually prune.
func TestSurplusTracksMiss(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", 0, 0), mkTrack(2, "", 4000, 0)}
	meas := []track.Measurement{mkMeas("", 2, 0)}

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8})

	if len(got.Matched) != 1 {
		t.Fatalf("want one match, got %+v", got)
	}
	if len(got.UnmatchedTracks) != 1 || got.UnmatchedTracks[0] != 1 {
		t.Fatalf("want the far track to miss, got %+v", got.UnmatchedTracks)
	}
}

func TestEmptyScanMissesEveryTrack(t *testing.T) {
	tracks := []*track.Track{mkTrack(1, "", 0, 0), mkTrack(2, "", 10, 0)}

	got := Associate(filter.Filter{}, tracks, nil, Config{GateThreshold: 13.8, AmbiguityMargin: 1.0})

	if len(got.UnmatchedTracks) != 2 {
		t.Fatalf("want both to miss, got %+v", got)
	}
}

func TestNoTracksMakesEveryReturnACandidate(t *testing.T) {
	meas := []track.Measurement{mkMeas("", 0, 0), mkMeas("", 10, 0)}

	got := Associate(filter.Filter{}, nil, meas, Config{GateThreshold: 13.8, AmbiguityMargin: 1.0})

	if len(got.UnmatchedMeas) != 2 {
		t.Fatalf("want both to spawn, got %+v", got)
	}
}

// A track whose uncertainty has collapsed cannot be judged, and must be refused
// rather than divided by. Without the guard this pairs on a NaN distance and
// poisons the track's state from then on.
func TestDegenerateUncertaintyIsRefusedNotDividedBy(t *testing.T) {
	tr := mkTrack(1, "", 0, 0)
	tr.State.P = [4][4]float64{}
	tracks := []*track.Track{tr}
	meas := []track.Measurement{{Position: geo.ENU{E: 0, N: 0}}} // R is zero too

	got := Associate(filter.Filter{}, tracks, meas, Config{GateThreshold: 13.8})

	if len(got.Matched) != 0 || len(got.Withheld) != 0 {
		t.Fatalf("an unjudgeable pair must not be paired: %+v", got)
	}
}

// Association must not depend on map ordering, or a replay of one recording
// would not reproduce the same picture twice.
func TestResultIsDeterministic(t *testing.T) {
	build := func() ([]*track.Track, []track.Measurement) {
		return []*track.Track{
				mkTrack(1, "aaa", 0, 0),
				mkTrack(2, "bbb", 100, 0),
				mkTrack(3, "", 200, 0),
				mkTrack(4, "", 205, 5),
			}, []track.Measurement{
				mkMeas("bbb", 101, 1),
				mkMeas("", 203, 2),
				mkMeas("aaa", 2, 1),
				mkMeas("", 900, 0),
			}
	}
	cfg := Config{GateThreshold: 13.8, AmbiguityMargin: 0.5}

	tr, ms := build()
	want := Associate(filter.Filter{}, tr, ms, cfg)

	for i := range 200 {
		tr, ms := build()
		got := Associate(filter.Filter{}, tr, ms, cfg)
		if len(got.Matched) != len(want.Matched) ||
			len(got.Withheld) != len(want.Withheld) ||
			len(got.UnmatchedTracks) != len(want.UnmatchedTracks) ||
			len(got.UnmatchedMeas) != len(want.UnmatchedMeas) {
			t.Fatalf("run %d differed in shape:\n want %+v\n got  %+v", i, want, got)
		}
		for k := range got.Matched {
			if got.Matched[k] != want.Matched[k] {
				t.Fatalf("run %d differed at match %d: want %+v got %+v", i, k, want.Matched[k], got.Matched[k])
			}
		}
	}
}

// The solver on its own, against a matrix small enough to check by hand. The
// cheapest pairing is (0,1) + (1,0) + (2,2) = 1 + 2 + 2 = 5; taking the zero at
// (1,1) first leads to 6.
func TestHungarianFindsTheHandWorkedOptimum(t *testing.T) {
	a := [][]float64{
		{4, 1, 3},
		{2, 0, 5},
		{3, 2, 2},
	}

	got := hungarian(a)

	total := 0.0
	seen := map[int]bool{}
	for i, j := range got {
		if j < 0 || seen[j] {
			t.Fatalf("not a one-to-one pairing: %v", got)
		}
		seen[j] = true
		total += a[i][j]
	}
	if total != 5 {
		t.Errorf("want total 5, got %v from %v", total, got)
	}
}

// Brute force over every permutation, to confirm the solver is actually finding
// the optimum and not just something plausible.
func TestHungarianMatchesBruteForce(t *testing.T) {
	mats := [][][]float64{
		{{1}},
		{{5, 9}, {9, 1}},
		{{7, 7, 7}, {7, 7, 7}, {7, 7, 7}},
		{{10, 19, 8, 15}, {10, 18, 7, 17}, {13, 16, 9, 14}, {12, 19, 8, 18}},
		{{0.01, 1.21, 9.0}, {4.0, 0.02, 8.5}, {6.0, 3.0, 0.03}},
		{{1e6, 1e6, 2.5}, {1e6, 3.5, 1e6}, {4.5, 1e6, 1e6}},
	}

	for n, a := range mats {
		got := hungarian(a)
		total := 0.0
		for i, j := range got {
			total += a[i][j]
		}

		size := len(a)
		perm := make([]int, size)
		for i := range perm {
			perm[i] = i
		}
		want := brute(a, perm, 0)

		if total > want+1e-9 {
			t.Errorf("matrix %d: solver found %v, brute force found %v", n, total, want)
		}
	}
}

func brute(a [][]float64, perm []int, at int) float64 {
	if at == len(perm) {
		total := 0.0
		for i, j := range perm {
			total += a[i][j]
		}
		return total
	}
	best := 0.0
	first := true
	for k := at; k < len(perm); k++ {
		perm[at], perm[k] = perm[k], perm[at]
		if got := brute(a, perm, at+1); first || got < best {
			best, first = got, false
		}
		perm[at], perm[k] = perm[k], perm[at]
	}
	return best
}
