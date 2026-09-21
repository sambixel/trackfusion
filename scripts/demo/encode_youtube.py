"""Encode native-resolution lossless frames with a simple, verified MP4 timeline."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

import imageio_ffmpeg

os.environ['TRACKFUSION_RENDER_SCALE'] = '2'
import render_smooth

ROOT = Path(__file__).resolve().parents[2]
TMP = Path('/tmp/trackfusion-video')
OUT = ROOT / 'output/demo/trackfusion-technical-demo-1440p60.mp4'
FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
FPS, COUNT = 60, 3060

command = [FFMPEG, '-y', '-hide_banner', '-f', 'rawvideo', '-pixel_format', 'rgb24',
           '-video_size', '2560x1440', '-framerate', str(FPS), '-i', 'pipe:0', '-frames:v', str(COUNT), '-an',
           '-vf', 'scale=in_range=full:out_range=tv:out_color_matrix=bt709,format=yuv420p',
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '16', '-profile:v', 'high',
           '-level:v', '5.1', '-r', str(FPS), '-g', '30', '-keyint_min', '30', '-bf', '0',
           '-sc_threshold', '0', '-x264-params', 'open-gop=0:force-cfr=1',
           '-color_primaries', 'bt709', '-color_trc', 'bt709', '-colorspace', 'bt709',
           '-bsf:v', 'h264_metadata=colour_primaries=1:transfer_characteristics=1:matrix_coefficients=1:video_full_range_flag=0',
           '-color_range', 'tv', '-map_metadata', '-1', '-movflags', '+faststart',
           '-use_editlist', '0', '-video_track_timescale', '60000', str(OUT)]
with (TMP / 'encode-1440p60.log').open('w') as log:
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
    try:
        for frame in range(COUNT):
            image = render_smooth.render(frame / FPS)
            assert image.size == (2560, 1440)
            process.stdin.write(image.tobytes())
            if frame % 300 == 0:
                print(f'Rendered and encoded {frame}/{COUNT} frames', flush=True)
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError((TMP / 'encode-1440p60.log').read_text()[-4000:])
    except BaseException:
        process.terminate()
        process.wait()
        raise

# Check the entire stream, not just thumbnail extraction or container duration.
check = subprocess.run([FFMPEG, '-hide_banner', '-v', 'error', '-xerror', '-i', str(OUT),
                        '-progress', 'pipe:1', '-f', 'null', '-'], capture_output=True, text=True)
check.check_returncode()
assert not check.stderr.strip(), check.stderr
assert f'frame={COUNT}' in check.stdout and 'progress=end' in check.stdout, check.stdout

# Inspect actual MP4 boxes; don't mistake marker bytes inside compressed data for boxes.
data = OUT.read_bytes()
atoms = []
def parse_boxes(start, end, depth=0):
    cursor = start
    while cursor < end:
        assert cursor + 8 <= end
        length, kind = struct.unpack_from('>I4s', data, cursor)
        header = 8
        if length == 1:
            length = struct.unpack_from('>Q', data, cursor + 8)[0]
            header = 16
        if length == 0:
            length = end - cursor
        assert header <= length <= end - cursor
        atoms.append((kind.decode(), cursor, length, depth))
        if kind in (b'moov', b'trak', b'edts', b'mdia', b'minf', b'stbl'):
            parse_boxes(cursor + header, cursor + length, depth + 1)
        cursor += length
    assert cursor == end
parse_boxes(0, len(data))
assert not any(k in ('edts', 'elst', 'ctts') for k, _, _, _ in atoms)
assert next(o for k, o, _, _ in atoms if k == 'moov') < next(o for k, o, _, _ in atoms if k == 'mdat')

for seconds in (2, 20, 29, 45, 49):
    subprocess.run([FFMPEG, '-y', '-v', 'error', '-ss', str(seconds), '-i', str(OUT),
                    '-frames:v', '1', str(TMP / f'1440p60-decoded-{seconds}.png')], check=True)
report = {'file': OUT.name, 'resolution': '2560x1440', 'fps': FPS, 'frames_decoded': COUNT,
          'duration_seconds': 51, 'codec': 'H.264 High / yuv420p / BT.709',
          'quality': 'CRF 16 / medium', 'edit_lists': False, 'frame_reordering': False,
          'fast_start': True, 'bytes': len(data), 'full_decode_errors': 0}
(OUT.parent / '1440p60-validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
