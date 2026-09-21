"""Animation-first 60fps composition; interpolate display states, not observations."""
import copy
import math
from pathlib import Path

import render_technical as ui

FPS, DURATION = 60, 51


def simulation_time(t):
    if t < 3: return t * 4
    if t < 14: return 12 + (t - 3) / 11 * 40
    if t < 26: return 52 + (t - 14) / 12 * 44
    if t < 36: return 96 + (t - 26) * 2
    return min(160, 116 + (t - 36) / 7 * 44)


def display_state(seconds):
    """Interpolate X/P only for the plot; sidebar and scan log use raw states."""
    index = min(40, int(seconds // 4))
    current = ui.TRACE[index]
    following = ui.TRACE[min(40, index + 1)]
    fraction = (seconds - current['Seconds']) / 4
    displayed = copy.deepcopy(current)
    for track in displayed['Tracks']:
        next_track = next((z for z in following['Tracks'] if z['ID'] == track['ID']), None)
        if next_track:
            for j in range(4):
                track['State']['X'][j] += fraction * (next_track['State']['X'][j] - track['State']['X'][j])
                for k in range(4):
                    track['State']['P'][j][k] += fraction * (next_track['State']['P'][j][k] - track['State']['P'][j][k])
    return current, displayed


def sidebar(d, frame):
    track = next(z for z in frame['Tracks'] if z['Identity'] == 'aaa111')
    x = track['State']['X']
    missed = track['MissCounter']
    ui.panel(d, (1004, 122, 1256, 322), 'TRACK 000 / aaa111')
    ui.text(d, (1020, 169), 'COAST' if missed else 'RADAR UPDATE', 17,
            ui.AMBER if missed else ui.GREEN, True)
    ui.text(d, (1020, 204), f'E {x[0]:>9.1f} m', 16)
    ui.text(d, (1020, 231), f'N {x[1]:>9.1f} m', 16)
    ui.text(d, (1020, 269), f'missed scans: {missed}', 15, ui.DIM)
    ui.text(d, (1020, 296), f'last scan: {frame["Seconds"]:3d} s', 13, ui.DIM)

    ui.panel(d, (1004, 334, 1256, 568), 'POSITION UNCERTAINTY')
    left, top, right, bottom = 1042, 399, 1236, 507
    ui.text(d, (1016, 389), '250', 13, ui.DIM)
    ui.text(d, (1029, 499), '0', 13, ui.DIM)
    ui.rule(d, (left, top), (right, top), ui.GRID)
    ui.rule(d, (left, bottom), (right, bottom), ui.BORDER)
    d.rectangle((left + 100 / 160 * (right-left), top,
                 left + 112 / 160 * (right-left), bottom), fill='#343027')
    points = []
    for past in ui.TRACE[:frame['Seconds'] // 4 + 1]:
        tr = next(z for z in past['Tracks'] if z['Identity'] == 'aaa111')
        p = tr['State']['P']
        sigma = math.sqrt(p[0][0] + p[1][1])
        points.append((left + past['Seconds'] / 160 * (right-left),
                       bottom - sigma / 250 * (bottom-top)))
    if len(points)>1: d.line(points, fill=ui.GREEN, width=2)
    p = track['State']['P']
    sigma = math.sqrt(p[0][0] + p[1][1])
    ui.text(d, (1020, 524), f'{sigma:.1f} m', 19, ui.GREEN, True)
    ui.text(d, (1182, 527), '160 s', 13, ui.DIM)
    ui.text(d, (1020, 548), 'sqrt(Pee + Pnn)', 13, ui.DIM)


def render(t):
    if t >= 43:
        if t < 47:
            im, d = ui.base(t, '05 / IMPLEMENTATION')
            ui.methods(d)
        else:
            im, d = ui.base(t, '06 / RUN SUMMARY')
            ui.summary(d)
        return im
    seconds = simulation_time(t)
    actual, displayed = display_state(seconds)
    if t < 3:
        title = '01 / MULTI-SENSOR TRACKING'
        cap = 'ADS-B initializes identity. Radar supplies noisy, anonymous observations.'
    elif t < 14:
        title = '01 / RADAR OBSERVATIONS'
        cap = 'Two aircraft, measurement noise, and clutter. The engine estimates position and velocity.'
    elif t < 26:
        title = '02 / TRACK ASSOCIATION'
        cap = 'Global assignment associates each radar scan with the predicted tracks.'
    elif seconds < 100:
        title = '03 / DETECTION GAP'
        cap = 'Next: three radar detections are deliberately omitted for track 000.'
    elif seconds < 112:
        title = '03 / DETECTION GAP'
        cap = 'Track 000 keeps moving through the gap. Its position uncertainty grows.'
    else:
        title = '04 / MEASUREMENT REACQUIRED'
        cap = 'Radar updates resume at 112 s. The existing track is corrected; uncertainty decreases.'
    im, d = ui.base(t, title)
    ui.text(d, (902, 80), f't={seconds:05.1f}s  scan {actual["Seconds"]//4+2:02d}/42', 16, ui.DIM)
    ui.scope(d, (24, 122, 984, 568), displayed)
    sidebar(d, actual)
    s = actual['Stats']
    ui.panel(d, (24, 580, 1256, 628), 'RADAR SCAN')
    # Single compact scan row leaves the animation the majority of the screen.
    d.rectangle((25, 581, 1255, 627), fill=ui.PANEL)
    ui.text(d, (38, 594),
            f't={actual["Seconds"]:03d}s  obs={s["Meas"]:02d}  matched={s["Matched"]:02d}  '
            f'withheld={s["Withheld"]:02d}  new={s["Spawned"]:02d}  pruned={s["Pruned"]:02d}  live={s["Live"]:02d}', 16)
    ui.caption(d, cap)
    # Explain the rendering distinction in the footer rather than implying
    # the engine computes new estimates at the presentation frame rate.
    d.rectangle((24, 677, 1080, 704), fill=ui.BG)
    ui.text(d, (24, 682), 'SYNTHETIC INPUT / ACTUAL ENGINE / INTERPOLATED MOTION; SCAN-BASED TELEMETRY', 13, ui.DIM)
    return im


if __name__ == '__main__':
    from PIL import Image
    sheet = Image.new('RGB', (1920, 1080), ui.BG)
    for i, t in enumerate([2, 8, 20, 29, 31, 35, 39, 45, 49]):
        frame = render(t)
        frame.save(ui.TMP / f'smooth-preview-{i}.png')
        sheet.paste(frame.resize((640, 360)), ((i % 3)*640, (i // 3)*360))
    sheet.save(ui.TMP / 'smooth-storyboard.jpg', quality=95)
    render(29).save(ui.OUT / 'poster-technical-1440p60.jpg', quality=98, subsampling=0)
