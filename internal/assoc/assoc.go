// Package assoc decides which measurements belong to which tracks.
//
// A track corrected with the wrong aircraft's measurement sees a small
// residual, shrinks its uncertainty, and gates more tightly around the wrong
// aircraft next scan: confidence and error grow together. So it gates against
// each track's own uncertainty, settles the whole scan at once, and hands back
// nothing when the answer is too close to call.
package assoc

import (
	"math"

	"github.com/sambixel/trackfusion/internal/filter"
	"github.com/sambixel/trackfusion/internal/track"
)

// Reason separates a match that required no inference from one that did. Only
// ByPosition can swap.
type Reason int

const (
	ByIdentity Reason = iota // the aircraft named itself
	ByPosition               // inferred from geometry alone
)

func (r Reason) String() string {
	switch r {
	case ByIdentity:
		return "identity"
	case ByPosition:
		return "position"
	}
	return "unknown"
}

// Config is the tuning that decides how willing the associator is to commit.
type Config struct {
	// GateThreshold is the furthest a pair may sit and still be considered,
	// scaled by the track's own uncertainty and so unitless. 13.8 keeps about
	// 99.9% of correct pairings, 9.21 about 99%.
	GateThreshold float64

	// AmbiguityMargin is how much worse the best competing reading of a scan
	// must be before a pairing is trusted. Zero disables the check.
	AmbiguityMargin float64
}

// Pair holds indices into the slices passed to Associate, not IDs. Cost is
// unset for ByIdentity.
type Pair struct {
	TrackIdx int
	MeasIdx  int
	Cost     float64
	Reason   Reason
}

// Result is a decision, not an action: nothing here has been applied, which is
// what lets two versions of the associator run over one recording and be
// compared.
type Result struct {
	// Matched is stable: identity pairs in measurement order, then position
	// pairs in track order.
	Matched []Pair

	// Withheld won the assignment but not by enough to believe. The track was
	// crowded rather than absent, so it must not count toward pruning, and the
	// measurement must not start a track for an aircraft already held.
	Withheld []Pair

	UnmatchedTracks []int // nothing was near: a real miss
	UnmatchedMeas   []int // belongs to nothing held: may start a track
}

// Associate settles one scan. Tracks must already be predicted to the time of
// the measurements; this package reads their state and never advances it.
func Associate(f filter.Filter, tracks []*track.Track, meas []track.Measurement, cfg Config) Result {
	var res Result

	trackUsed := make([]bool, len(tracks))
	measUsed := make([]bool, len(meas))

	// A cooperative report names its aircraft, so its track does not compete
	// for it. Not gated: a report far from the prediction means the track
	// drifted, not that the aircraft lied about its address.
	holder := make(map[string]int, len(tracks))
	for i, t := range tracks {
		if t.Identity == "" {
			continue
		}
		// Two tracks should never claim one address; if they do, the older wins.
		if _, taken := holder[t.Identity]; !taken {
			holder[t.Identity] = i
		}
	}
	for j, m := range meas {
		if m.Identity == "" {
			continue
		}
		i, known := holder[m.Identity]
		if !known || trackUsed[i] {
			continue
		}
		trackUsed[i] = true
		measUsed[j] = true
		res.Matched = append(res.Matched, Pair{TrackIdx: i, MeasIdx: j, Reason: ByIdentity})
	}

	// What is left is a geometry problem: radar returns, which carry no
	// identity at all, and reports from aircraft not yet tracked.
	var rows, cols []int
	for i := range tracks {
		if !trackUsed[i] {
			rows = append(rows, i)
		}
	}
	for j := range meas {
		if !measUsed[j] {
			cols = append(cols, j)
		}
	}

	cost := make([][]float64, len(rows))
	open := make([][]bool, len(rows))
	perRow := make([]int, len(rows))
	perCol := make([]int, len(cols))

	for a, i := range rows {
		cost[a] = make([]float64, len(cols))
		open[a] = make([]bool, len(cols))
		for b, j := range cols {
			d, ok := gate(f, tracks[i], meas[j], cfg.GateThreshold)
			if !ok {
				continue
			}
			cost[a][b] = d
			open[a][b] = true
			perRow[a]++
			perCol[b]++
		}
	}

	// blocked stands in for a disallowed pairing and for the padding cells. It
	// must outweigh any full set of real pairings, or the solver would buy a
	// cheap row by leaving a measurement unclaimed.
	n := len(rows)
	if len(cols) > n {
		n = len(cols)
	}
	blocked := float64(n+1)*math.Abs(cfg.GateThreshold) + 1

	assign, best := solve(cost, open, blocked)

	for a, b := range assign {
		if b < 0 {
			continue
		}
		pair := Pair{
			TrackIdx: rows[a],
			MeasIdx:  cols[b],
			Cost:     cost[a][b],
			Reason:   ByPosition,
		}

		// Forbid the winning cell and re-solve: how much worse is the best
		// reading that disagrees about this pair? Barely worse means picking
		// one is a guess. Only a contested pair is worth the second solve.
		contested := perRow[a] > 1 || perCol[b] > 1
		if cfg.AmbiguityMargin > 0 && contested {
			open[a][b] = false
			_, alt := solve(cost, open, blocked)
			open[a][b] = true

			if alt-best < cfg.AmbiguityMargin {
				res.Withheld = append(res.Withheld, pair)
				trackUsed[rows[a]] = true
				measUsed[cols[b]] = true
				continue
			}
		}

		res.Matched = append(res.Matched, pair)
		trackUsed[rows[a]] = true
		measUsed[cols[b]] = true
	}

	for i := range tracks {
		if !trackUsed[i] {
			res.UnmatchedTracks = append(res.UnmatchedTracks, i)
		}
	}
	for j := range meas {
		if !measUsed[j] {
			res.UnmatchedMeas = append(res.UnmatchedMeas, j)
		}
	}

	return res
}

// gate measures how far a measurement sits from a track's prediction in units
// of that track's own uncertainty. Scaled rather than in metres so a coasting
// track gets a forgiving gate and a just-corrected one a tight gate.
func gate(f filter.Filter, t *track.Track, m track.Measurement, threshold float64) (float64, bool) {
	v, S := f.Innovation(t.State, [2]float64{m.Position.E, m.Position.N}, m.R)

	det := S[0][0]*S[1][1] - S[0][1]*S[1][0]

	// Collapsed or non-finite uncertainty leaves no honest way to judge the
	// pair, so refuse it rather than spread a NaN through every track it
	// later touches.
	if !(det > 0) || !(S[0][0] > 0) || math.IsInf(det, 0) {
		return 0, false
	}

	d := (S[1][1]*v[0]*v[0] - (S[0][1]+S[1][0])*v[0]*v[1] + S[0][0]*v[1]*v[1]) / det
	if math.IsNaN(d) || d < 0 || d > threshold {
		return 0, false
	}

	return d, true
}

// solve settles the whole grid at once, returning the column given to each row
// or -1 where nothing survived gating. The total covers the padded square,
// placeholder cells included, which is what makes two solves comparable.
func solve(cost [][]float64, open [][]bool, blocked float64) ([]int, float64) {
	rows := len(cost)
	cols := 0
	if rows > 0 {
		cols = len(cost[0])
	}

	n := rows
	if cols > n {
		n = cols
	}
	if n == 0 {
		return nil, 0
	}

	square := make([][]float64, n)
	for i := range square {
		square[i] = make([]float64, n)
		for j := range square[i] {
			if i < rows && j < cols && open[i][j] {
				square[i][j] = cost[i][j]
			} else {
				square[i][j] = blocked
			}
		}
	}

	taken := hungarian(square)

	assign := make([]int, rows)
	for i := range assign {
		assign[i] = -1
	}

	total := 0.0
	for i, j := range taken {
		if j < 0 {
			continue
		}
		total += square[i][j]
		if i < rows && j < cols && open[i][j] {
			assign[i] = j
		}
	}

	return assign, total
}

// hungarian returns the cheapest one-to-one pairing over a square matrix as
// taken[row] = col, by shortest augmenting path. It knows nothing about
// tracking, so it can be checked against a matrix worked out by hand.
func hungarian(a [][]float64) []int {
	n := len(a)
	inf := math.Inf(1)

	// Index 0 is a sentinel, so everything below is one-based. owner[j] holds
	// column j's row; prev[j] the column it was reached from.
	rowCut := make([]float64, n+1)
	colCut := make([]float64, n+1)
	owner := make([]int, n+1)
	prev := make([]int, n+1)

	for i := 1; i <= n; i++ {
		owner[0] = i
		at := 0

		best := make([]float64, n+1)
		seen := make([]bool, n+1)
		for j := range best {
			best[j] = inf
		}

		// Walk out from row i until the path reaches a free column.
		for {
			seen[at] = true
			from := owner[at]

			step := inf
			next := 0
			for j := 1; j <= n; j++ {
				if seen[j] {
					continue
				}
				if cur := a[from-1][j-1] - rowCut[from] - colCut[j]; cur < best[j] {
					best[j] = cur
					prev[j] = at
				}
				if best[j] < step {
					step = best[j]
					next = j
				}
			}

			// Shift the discounts so the cheapest reachable column comes free.
			for j := 0; j <= n; j++ {
				if seen[j] {
					rowCut[owner[j]] += step
					colCut[j] -= step
				} else {
					best[j] -= step
				}
			}

			at = next
			if owner[at] == 0 {
				break
			}
		}

		// Walk back, handing each column to the row behind it.
		for at != 0 {
			p := prev[at]
			owner[at] = owner[p]
			at = p
		}
	}

	taken := make([]int, n)
	for i := range taken {
		taken[i] = -1
	}
	for j := 1; j <= n; j++ {
		if owner[j] > 0 {
			taken[owner[j]-1] = j - 1
		}
	}

	return taken
}
