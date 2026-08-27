// Package track holds the representation of a track, its lifecycle, and the
// store that owns the live set. Tracks are sensor-agnostic: nothing here knows
// which sensor produced the measurement that updated it.
package track

import (
	"time"

	"github.com/sambixel/trackfusion/internal/filter"
	"github.com/sambixel/trackfusion/internal/geo"
)

// Measurement is one observation from any sensor, already projected into the
// tracking frame. Identity holds the ADS-B address and is empty for radar,
// which is the asymmetry the tracker exists to handle. R is in m².
type Measurement struct {
	TimeStamp time.Time
	Position  geo.ENU
	Identity  string
	R         [2][2]float64
}

// Track is one hypothesis about one object. Id is not an aircraft identity: a
// track can be holding the wrong aircraft, and counting how often that happens
// is the point of the project.
type Track struct {
	ID          int64
	State       filter.State
	LastUpdate  time.Time
	MissCounter int64
	LastAlt     float64
}

// Store owns the live set. A slice rather than a map because every scan
// iterates all of them, association wants stable indices, and Go randomises
// map order, which would make replay non-deterministic.
type Store struct {
	tracks []*Track
	nextID int64
}

// NewStore returns an empty store.
func NewStore() *Store {
	return &Store{}
}

// All returns the live tracks in a stable order, for Predict and association.
func (s *Store) All() []*Track {
	return s.tracks
}

// Spawn starts a track from a measurement that matched nothing. Velocity is
// unknown at one point, so it starts at zero with a variance wide enough to
// cover any aircraft; too small and the next gate misses the second report.
func (s *Store) Spawn(m Measurement) *Track {
	id := s.nextID
	s.nextID = id + 1
	const velVar = 300.0 * 300.0

	state := filter.State{
		X: [4]float64{m.Position.E, m.Position.N, 0, 0},
		P: [4][4]float64{
			{m.R[0][0], 0, 0, 0},
			{0, m.R[1][1], 0, 0},
			{0, 0, velVar, 0},
			{0, 0, 0, velVar},
		},
	}

	t := &Track{
		ID:          id,
		State:       state,
		LastUpdate:  m.TimeStamp,
		MissCounter: 0,
	}
	s.tracks = append(s.tracks, t)

	return t
}

// Prune drops tracks that have gone unmatched too many scans running. Filters
// in place to reuse the backing array and preserve order.
func (s *Store) Prune(maxMisses int64) {
	result := s.tracks[:0]

	for _, track := range s.tracks {
		if track.MissCounter <= maxMisses {
			result = append(result, track)
		}
	}

	s.tracks = result
}
