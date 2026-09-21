# Rebuild the recruiter video

The exporter runs the existing production engine using a temporary Go test overlay. It does not modify production files or the checked-in test suite.

Requirements: Go, Python 3 with Pillow and numpy, and macOS with Swift/AVFoundation and the Avenir Next system font. No third-party encoder or network service is used.

From the repository root:

```sh
python3 scripts/demo/export.py
python3 scripts/demo/render.py --preview
python3 scripts/demo/render.py
swiftc -module-cache-path /tmp/trackfusion-video/swift-cache scripts/demo/encode.swift -o /tmp/trackfusion-video/encode
/tmp/trackfusion-video/encode /tmp/trackfusion-video/frames output/demo/trackfusion-recruiter-demo.mp4
```

Frames are stored in `/tmp/trackfusion-video/frames` by default; override with `TRACKFUSION_FRAMES` and pass the same directory to the encoder. Final data, previews, and the video are under `output/demo`. The encoder checks duration and extracts five decoded frames beside the temporary frame directory for visual review. Rebuilding replaces the generated artifacts at those locations.

The captions and storyboard are in `render.py`; the repeatable synthetic scenario is in `export_test.go.txt`.
