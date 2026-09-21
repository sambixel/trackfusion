# Technical recruiter demo

Play **trackfusion-technical-demo.mp4**. This is the revised 51-second video, with a subdued engineering display, exact scan snapshots, numeric track states, an uncertainty history, and actual scan logs. The original video remains available separately.

The data and engine are unchanged. Unlike the original video, the revised plot does not interpolate positions between scans. Playback pauses on each exported state and slows during the deliberate detection gap. This is a rendered inspection display of engine exports, not a screen recording of an interactive application.

The closing summary reports position RMSE of 71.9 m across 76 identified-track states at simulation times 12–160 seconds. That statistic comes from `trace.json` and describes only this one synthetic scenario. Unnamed/clutter tracks are shown separately and are not part of that RMSE. The uncertainty plot shows `sqrt(Pee + Pnn)` in meters, with the deliberate missing-detection interval shaded.

Use **poster-technical.jpg** as the video thumbnail.

Rebuild on macOS after exporting `trace.json` using the original exporter:

```sh
python3 scripts/demo/render_technical.py --preview
python3 scripts/demo/render_technical.py
swiftc -module-cache-path /tmp/trackfusion-video/swift-cache scripts/demo/encode.swift -o /tmp/trackfusion-video/encode
/tmp/trackfusion-video/encode /tmp/trackfusion-video/technical-frames output/demo/trackfusion-technical-demo.mp4
```

The encoder may need permission to access the macOS video encoding service when invoked inside a restricted sandbox. Source frames are in `/tmp/trackfusion-video/technical-frames`; decoded review images are written to their parent directory.
