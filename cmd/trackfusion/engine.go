package main

import (
	"time"

	"github.com/sambixel/trackfusion/internal/assoc"
	"github.com/sambixel/trackfusion/internal/filter"
	"github.com/sambixel/trackfusion/internal/record"
	"github.com/sambixel/trackfusion/internal/track"
)

// Engine runs one scan through the pipeline. It makes no decisions of its own,
// it only puts the packages that do in the right order, and the order is the
// part that matters: association compares measurements against predictions, so
// tracks move to the scan's time first, and pruning happens last so a track
// that was merely crowded out is not mistaken for one that is gone.
type Engine struct {
	Filter filter.Filter
	Store  *track.Store
	Assoc  assoc.Config

	// MaxMisses is how many scans a track may go unmatched before it is
	// dropped. At zero, a Pd below one deletes healthy tracks on the sensor's
	// first bad sweep.
	MaxMisses int64
}

// Stats is what one scan did. Withheld is the one worth watching: the times the
// associator had a winner and declined to trust it.
type Stats struct {
	Scan     record.Sensor
	At       time.Time
	Meas     int
	Matched  int
	ByID     int
	Withheld int
	Missed   int
	Spawned  int
	Pruned   int
	Live     int
}

// Step advances the picture by one scan.
func (e *Engine) Step(s record.Scan) Stats {
	meas := s.Measurements()
	tracks := e.Store.All()

	// One time for the whole scan, so its measurements are treated as
	// simultaneous. For a sweep they are; for a snapshot they are not, but
	// association settles a scan at once and cannot do that against tracks
	// sitting at different times. The poller's MaxAge bounds the error.
	for _, t := range tracks {
		dt := s.Time.Sub(t.LastUpdate)
		if dt < 0 {
			// Out-of-order scans would run the filter backwards, shrinking the
			// covariance on the strength of time travel.
			dt = 0
		}
		t.State = e.Filter.Predict(t.State, dt)

		// LastUpdate is when the state is valid, not when it was last
		// corrected; leaving it behind would advance the track twice over the
		// same gap. Staleness is MissCounter's job.
		t.LastUpdate = s.Time
	}

	res := assoc.Associate(e.Filter, tracks, meas, e.Assoc)

	stats := Stats{
		Scan:     s.Sensor,
		At:       s.Time,
		Meas:     len(meas),
		Matched:  len(res.Matched),
		Withheld: len(res.Withheld),
		Missed:   len(res.UnmatchedTracks),
	}

	for _, p := range res.Matched {
		t, m := tracks[p.TrackIdx], meas[p.MeasIdx]

		t.State = e.Filter.Update(t.State, [2]float64{m.Position.E, m.Position.N}, m.R)
		t.MissCounter = 0
		if m.Position.U != 0 {
			t.LastAlt = m.Position.U
		}

		// A radar-born track has no name until a cooperative report lands on
		// it, and that is the moment the two sensors become one picture. Never
		// overwritten: a track already claiming an address and now matching a
		// different one by position has gone wrong, and relabelling it would
		// erase the evidence.
		if t.Identity == "" && m.Identity != "" {
			t.Identity = m.Identity
		}

		if p.Reason == assoc.ByIdentity {
			stats.ByID++
		}
	}

	// Withheld pairs are left alone on purpose: the track was crowded rather
	// than absent so it takes no miss, and the measurement belongs to something
	// already held so it starts nothing. Either would turn a moment of
	// ambiguity into a duplicate track competing for every future return.

	for _, i := range res.UnmatchedTracks {
		tracks[i].MissCounter++
	}

	for _, j := range res.UnmatchedMeas {
		e.Store.Spawn(meas[j])
		stats.Spawned++
	}

	before := len(e.Store.All())
	e.Store.Prune(e.MaxMisses)
	stats.Live = len(e.Store.All())
	stats.Pruned = before - stats.Live

	return stats
}
