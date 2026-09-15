package record

import (
	"bytes"
	"fmt"
	"io"
	"strings"
	"testing"
	"time"

	"github.com/sambixel/trackfusion/internal/geo"
	"github.com/sambixel/trackfusion/internal/track"
)

var epoch = time.Date(2026, 9, 10, 12, 0, 0, 0, time.UTC)

func mkMeas(id string, e, n float64, at time.Time) track.Measurement {
	return track.Measurement{
		TimeStamp: at,
		Position:  geo.ENU{E: e, N: n, U: 9500},
		Identity:  id,
		R:         [2][2]float64{{120.5, -3.25}, {-3.25, 4400.75}},
	}
}

func writeAll(t *testing.T, scans []Scan) *bytes.Buffer {
	t.Helper()
	var buf bytes.Buffer
	w := NewWriter(&buf)
	for _, s := range scans {
		if err := w.Write(s); err != nil {
			t.Fatalf("write: %v", err)
		}
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	return &buf
}

// A replay that does not reproduce the measurement exactly is not a replay. The
// comparison is on equality rather than a tolerance on purpose: a rounded
// coordinate would move a marginal pairing across the gate boundary, which is
// precisely the case a recording exists to re-examine.
func TestRoundTripIsExact(t *testing.T) {
	want := []track.Measurement{
		mkMeas("abc123", 12345.678901234, -9876.543210987, epoch.Add(-3*time.Second)),
		mkMeas("", 0.1, 0.2, epoch),
	}

	buf := writeAll(t, []Scan{NewScan(Radar, epoch, want, []string{"abc123", ""})})

	got, err := NewReader(buf).All()
	if err != nil {
		t.Fatalf("read: %v", err)
	}
	if len(got) != 1 {
		t.Fatalf("want 1 scan, got %d", len(got))
	}
	if got[0].Sensor != Radar || !got[0].Time.Equal(epoch) {
		t.Errorf("scan header round-tripped as %v at %v", got[0].Sensor, got[0].Time)
	}

	back := got[0].Measurements()
	if len(back) != len(want) {
		t.Fatalf("want %d measurements, got %d", len(want), len(back))
	}
	for i := range want {
		if back[i].Position != want[i].Position || back[i].R != want[i].R ||
			back[i].Identity != want[i].Identity || !back[i].TimeStamp.Equal(want[i].TimeStamp) {
			t.Errorf("measurement %d differs:\n got %+v\nwant %+v", i, back[i], want[i])
		}
	}
}

// A sweep that saw nothing is a fact about the world: every track missed. If
// the format dropped it, replay would skip the sweep entirely and the tracks
// would never accumulate the misses that eventually prune them.
func TestEmptyScanSurvives(t *testing.T) {
	buf := writeAll(t, []Scan{
		NewScan(Radar, epoch, nil, nil),
		NewScan(Radar, epoch.Add(4*time.Second), []track.Measurement{mkMeas("", 100, 200, epoch)}, nil),
		NewScan(Radar, epoch.Add(8*time.Second), nil, nil),
	})

	got, err := NewReader(buf).All()
	if err != nil {
		t.Fatalf("read: %v", err)
	}
	if len(got) != 3 {
		t.Fatalf("want 3 scans, got %d: empty sweeps were dropped", len(got))
	}
	if len(got[0].Obs) != 0 || len(got[2].Obs) != 0 {
		t.Errorf("empty scans came back non-empty")
	}
	if len(got[1].Obs) != 1 {
		t.Errorf("scan 1 lost its observation")
	}
}

// Truth is recorded but must not be reachable from the path the tracker uses,
// or a replay would quietly hand it the answer and every associator would look
// perfect.
func TestTruthIsRecordedButNotHandedToTheTracker(t *testing.T) {
	meas := []track.Measurement{mkMeas("", 500, 600, epoch)}
	buf := writeAll(t, []Scan{NewScan(Radar, epoch, meas, []string{"abc123"})})

	got, err := NewReader(buf).All()
	if err != nil {
		t.Fatalf("read: %v", err)
	}

	if got[0].Truth()[0] != "abc123" {
		t.Errorf("truth lost in the round trip: %q", got[0].Truth()[0])
	}
	if id := got[0].Measurements()[0].Identity; id != "" {
		t.Errorf("Measurements leaked truth as identity %q", id)
	}
}

// Replay depends on order. Scans come back in the order written, including when
// two sensors report at the same instant.
func TestOrderIsPreserved(t *testing.T) {
	var scans []Scan
	for i := range 20 {
		sensor := ADSB
		if i%2 == 1 {
			sensor = Radar
		}
		scans = append(scans, NewScan(sensor, epoch.Add(time.Duration(i)*time.Second), nil, nil))
	}

	got, err := NewReader(writeAll(t, scans)).All()
	if err != nil {
		t.Fatalf("read: %v", err)
	}
	if len(got) != len(scans) {
		t.Fatalf("want %d scans, got %d", len(scans), len(got))
	}
	for i := range scans {
		if got[i].Sensor != scans[i].Sensor || !got[i].Time.Equal(scans[i].Time) {
			t.Fatalf("scan %d out of order: %v at %v", i, got[i].Sensor, got[i].Time)
		}
	}
}

// A busy snapshot runs to hundreds of aircraft, which puts one line well past
// the 64 KB a bufio.Scanner returns by default. That limit is why this reads
// through a decoder: it would not corrupt the file, it would just end the
// replay early and look like the end of it.
func TestBusyScanExceeds64KBAndStillReads(t *testing.T) {
	var meas []track.Measurement
	for i := range 600 {
		meas = append(meas, mkMeas(fmt.Sprintf("%06x", i), float64(i)*137.5, float64(i)*-91.25, epoch))
	}

	buf := writeAll(t, []Scan{NewScan(ADSB, epoch, meas, nil)})
	if buf.Len() < 64*1024 {
		t.Fatalf("test is not exercising the limit: line is only %d bytes", buf.Len())
	}

	got, err := NewReader(buf).All()
	if err != nil {
		t.Fatalf("read: %v", err)
	}
	if len(got) != 1 || len(got[0].Obs) != len(meas) {
		t.Fatalf("want %d observations, got %v scans", len(meas), len(got))
	}
	if got[0].Obs[599].Identity != "000257" {
		t.Errorf("last observation came back as %+v", got[0].Obs[599])
	}
}

// Next reports the end of a recording as io.EOF unwrapped, so a replay loop can
// compare against it directly rather than reaching for errors.Is.
func TestNextReportsPlainEOF(t *testing.T) {
	r := NewReader(writeAll(t, []Scan{NewScan(ADSB, epoch, nil, nil)}))

	if _, err := r.Next(); err != nil {
		t.Fatalf("first scan: %v", err)
	}
	if _, err := r.Next(); err != io.EOF {
		t.Errorf("want io.EOF at the end, got %v", err)
	}
}

// A truncated or hand-edited recording has to fail loudly. Returning the
// partial run without a word would look like a short capture.
func TestCorruptLineIsAnError(t *testing.T) {
	_, err := NewReader(strings.NewReader(`{"sensor":"adsb","t":"2026-09-10T12:00:00Z","obs":[]}` + "\n" + `{"sensor":`)).All()
	if err == nil {
		t.Fatal("want an error on a truncated recording, got nil")
	}
	if err == io.EOF {
		t.Fatal("truncation was reported as a clean end of file")
	}
}

// Buffering means a scan is not on disk when Write returns. Close is what makes
// it durable, and a capture that is never closed loses its tail.
func TestCloseFlushes(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)

	if err := w.Write(NewScan(ADSB, epoch, nil, nil)); err != nil {
		t.Fatalf("write: %v", err)
	}
	if buf.Len() != 0 {
		t.Errorf("want the scan still buffered, %d bytes already written", buf.Len())
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	if buf.Len() == 0 {
		t.Error("Close did not flush")
	}
}
