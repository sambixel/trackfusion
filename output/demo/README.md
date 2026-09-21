# TrackFusion recruiter demo

**Play `trackfusion-recruiter-demo.mp4`.** It is a 51-second, 1280×720 H.264 video with all explanations on screen. No audio, account, or network connection is needed.

The video shows actual output from the repository's Go tracking engine. The display is a presentation of exported results, not a screen recording of an existing application UI.

## What it shows

- 0–5 seconds: the problem and project introduction.
- 5–14 seconds: anonymous radar observations, clutter, and aircraft labels.
- 14–24 seconds: estimated motion and track association as aircraft pass nearby.
- 24–34 seconds: three deliberately missing radar detections; prediction continues and uncertainty grows.
- 34–40 seconds: incoming radar observations update the existing track.
- 40–46 seconds: the Go components and deterministic replay workflow.
- 46–51 seconds: Samuel Bixel and the project repository.

## Data and limitations

This is a synthetic, reproducible demonstration. Two aircraft receive an initial simulated ADS-B position/identity fix, followed by four-second simulated radar sweeps with bearing/range noise and Poisson clutter. Detection probability is set to 1 for the controlled scenario; ALPHA's detections are deliberately omitted at simulation times 100, 104, and 108 seconds. The radar seed is 21.

The unchanged production `Engine`, radar, filter, associator, track store, and recorder generate the results. Ground-truth labels remain outside the estimator. The video interpolates track positions between exported scans for smooth playback and varies playback speed to linger on the gap. Grey circles are unnamed tracks, including those initiated by clutter. Ellipses use the recorded position covariance and a 95% scale. The uncertainty chart displays the square root of the summed east/north position variances.

This demonstrates behavior on one scenario; it is not a general accuracy benchmark or a guarantee against identity switches. It does not demonstrate live OpenSky connectivity. Existing identity-conflict and out-of-order-scan issues identified in the review have not been changed by making this video.

`trace.json` contains the exported scan-by-scan states and scenario metadata. `scenario.jsonl` contains the corresponding replayable inputs. The 93-test / 8-package statement describes the existing repository tests; the export harness is separate.

## Share it

Send the MP4 directly or upload it as a project video. Keep the burned-in synthetic-scenario label visible. Use `poster.jpg` as its thumbnail. The video is silent intentionally so it works on a noisy career-fair floor and on muted autoplay.

## Replay the inputs

From the repository root:

```sh
go run ./cmd/trackfusion -replay output/demo/scenario.jsonl
```

See `scripts/demo/README.md` for rebuilding the video.
