# Recommended upload: 1440p / 60 fps

Use **trackfusion-technical-demo-1440p60.mp4**. This supersedes the earlier 720p exports for sharing.

- 2560×1440, rendered directly at that resolution from text and geometry.
- 60 fps with interpolated track motion between the production engine's recorded states.
- Larger airspace display; smaller telemetry and uncertainty sidebar.
- First 43 seconds show the running scenario; the last 8 seconds cover implementation and results.
- Actual observations, scan logs, and sidebar values update at recorded scan times. The rendering interpolation does not add measurements or change the engine results.
- H.264 High, 4:2:0, BT.709, quality-based CRF 16 encoding.
- MP4 fast start, no edit lists, no reordered frames, and a constant frame rate.
- Silent, with on-screen captions.

The original technical video was a valid 720p/24 fps stream, but its deliberately held scan states made the motion appear choppy. The original MP4 also contained an edit list, contrary to YouTube's recommended upload settings. Neither observation proves the cause of a specific YouTube processing rejection. The new export removes that container feature and has its entire stream decoded locally as a validation step. Actual YouTube acceptance still requires an upload.

The validation results are recorded in `1440p60-validation.json`. The matching thumbnail is `poster-technical-1440p60.jpg`.

## Rebuild

Python dependencies: Pillow, numpy, imageio-ffmpeg. macOS supplies the Menlo font used by the renderer. Existing `trace.json` is the input.

```sh
TRACKFUSION_RENDER_SCALE=2 python3 scripts/demo/render_smooth.py
python3 scripts/demo/encode_youtube.py
```

The encoder streams newly rendered RGB frames directly to FFmpeg, then verifies a complete decode, frame count, fast-start placement, and the absence of edit lists. It also extracts representative encoded frames for visual review.

Upload guidance: https://support.google.com/youtube/answer/1722171
Processing-error guidance: https://support.google.com/youtube/answer/10383400
