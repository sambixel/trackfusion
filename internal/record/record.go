// Package record captures every scan that enters the system and plays it back.
// Without it, "did that change improve anything" is answered by watching a map
// and forming an impression.
//
// The unit written is a scan and not a measurement, because an empty scan
// carries information: a sweep that saw nothing means every track missed, and a
// format holding only measurements would drop the line and rewrite history into
// one where the sweep never happened.
package record

import (
	"bufio"
	"encoding/json"
	"fmt"
	"io"
	"time"

	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/track"
)

// Sensor names the source of a scan. Recorded because the two fail differently
// and a log that cannot tell them apart cannot be diagnosed.
type Sensor string

const (
	ADSB  Sensor = "adsb"
	Radar Sensor = "radar"
)

// Scan is one sensor reporting once, including the case where it had nothing.
type Scan struct {
	Sensor Sensor `json:"sensor"`

	// Time is when the sensor reported, which is not any observation's own
	// timestamp: a sweep stamps every return alike, a snapshot is a collection
	// of contacts of differing ages.
	Time time.Time `json:"t"`

	Obs []Observation `json:"obs"`
}

// Observation is one measurement as it entered the system, already in the
// tracking frame. Projecting again on replay would make the run depend on the
// geo code being unchanged, and the point of a recording is to hold one
// variable still.
type Observation struct {
	Time time.Time `json:"t"`

	E float64 `json:"e"`
	N float64 `json:"n"`
	U float64 `json:"u,omitempty"`

	Identity string `json:"id,omitempty"` // empty for radar

	// Truth is what actually produced this, or empty for clutter. The tracker
	// never sees it; it is here so a recording can be asked afterwards what
	// really happened.
	Truth string `json:"truth,omitempty"`

	// R is the sensor's own statement of confidence, in m². Recorded rather
	// than recomputed because on replay the geometry that produced it is gone.
	R [2][2]float64 `json:"r"`
}

// NewScan assembles a scan. truth may be nil, and is otherwise parallel to meas.
func NewScan(sensor Sensor, at time.Time, meas []track.Measurement, truth []string) Scan {
	obs := make([]Observation, len(meas))

	for i, m := range meas {
		obs[i] = Observation{
			Time:     m.TimeStamp,
			E:        m.Position.E,
			N:        m.Position.N,
			U:        m.Position.U,
			Identity: m.Identity,
			R:        m.R,
		}
		if i < len(truth) {
			obs[i].Truth = truth[i]
		}
	}

	return Scan{Sensor: sensor, Time: at, Obs: obs}
}

// Measurements returns what the tracker should see. Truth is deliberately left
// behind: this is the only way in, so there is no path by which a recording
// could hand the tracker the answer.
func (s Scan) Measurements() []track.Measurement {
	out := make([]track.Measurement, len(s.Obs))

	for i, o := range s.Obs {
		out[i] = track.Measurement{
			TimeStamp: o.Time,
			Position:  geo.ENU{E: o.E, N: o.N, U: o.U},
			Identity:  o.Identity,
			R:         o.R,
		}
	}

	return out
}

// Truth returns the labels parallel to Measurements, for inspecting a recording
// after the fact.
func (s Scan) Truth() []string {
	out := make([]string, len(s.Obs))
	for i, o := range s.Obs {
		out[i] = o.Truth
	}
	return out
}

// Writer appends scans as one JSON object per line.
type Writer struct {
	buf *bufio.Writer
	enc *json.Encoder
}

// NewWriter wraps w. Nothing reaches the stream until Flush or Close, so a run
// killed outright loses its tail.
func NewWriter(w io.Writer) *Writer {
	buf := bufio.NewWriter(w)
	return &Writer{buf: buf, enc: json.NewEncoder(buf)}
}

func (w *Writer) Write(s Scan) error {
	if err := w.enc.Encode(s); err != nil {
		return fmt.Errorf("record: write scan: %w", err)
	}
	return nil
}

func (w *Writer) Flush() error {
	if err := w.buf.Flush(); err != nil {
		return fmt.Errorf("record: flush: %w", err)
	}
	return nil
}

// Close flushes. The underlying stream belongs to whoever opened it.
func (w *Writer) Close() error { return w.Flush() }

// Reader replays a recording in the order it was written.
type Reader struct {
	dec *json.Decoder
}

// NewReader wraps r. It decodes from the stream rather than scanning lines
// because a busy snapshot runs well past the 64 KB a bufio.Scanner returns by
// default, which would end the replay early and look like the end of the file.
func NewReader(r io.Reader) *Reader {
	return &Reader{dec: json.NewDecoder(bufio.NewReader(r))}
}

// Next returns the next scan, or io.EOF unwrapped so callers can compare
// against it directly.
func (r *Reader) Next() (Scan, error) {
	var s Scan
	if err := r.dec.Decode(&s); err != nil {
		if err == io.EOF {
			return Scan{}, io.EOF
		}
		return Scan{}, fmt.Errorf("record: read scan: %w", err)
	}
	return s, nil
}

// All reads the rest into memory. Fine for tests and short captures; a long one
// should be streamed with Next.
func (r *Reader) All() ([]Scan, error) {
	var out []Scan
	for {
		s, err := r.Next()
		if err == io.EOF {
			return out, nil
		}
		if err != nil {
			return out, err
		}
		out = append(out, s)
	}
}
