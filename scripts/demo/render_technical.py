"""Engineering-style replay video using the same unmodified engine export."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/demo'
TMP = Path('/tmp/trackfusion-video')
SCALE = int(os.environ.get('TRACKFUSION_RENDER_SCALE', '1'))
FRAMES = Path(os.environ.get('TRACKFUSION_FRAMES', str(TMP / 'technical-frames')))
FRAMES.mkdir(parents=True, exist_ok=True)
DATA = json.loads((OUT / 'trace.json').read_text())
TRACE = DATA['frames']
W, H, FPS, DURATION = 1280, 720, 24, 51
BG = '#131618'
PANEL = '#181d20'
GRID = '#242b2e'
BORDER = '#41494d'
TEXT = '#e1e5e4'
DIM = '#b0babc'
FAINT = '#79878b'
GREEN = '#a6c9ac'
BLUE = '#92b4c4'
AMBER = '#cdb47f'
COLORS = {'aaa111': GREEN, 'bbb222': BLUE}
FONT = '/System/Library/Fonts/Menlo.ttc'
FONTS = {}


def font(size, bold=False):
    size = max(13, size)
    key = size, bold
    if key not in FONTS:
        FONTS[key] = ImageFont.truetype(FONT, size * SCALE, index=1 if bold else 0)
    return FONTS[key]


def text(d, xy, s, size=16, color=TEXT, bold=False):
    d.text(xy, s, fill=color, font=font(size, bold))


class ScaledDraw:
    """Rasterize vector geometry and fonts at the target resolution directly."""
    def __init__(self, image):
        self.draw = ImageDraw.Draw(image)

    def text(self, xy, text, **kwargs):
        self.draw.text(tuple(v * SCALE for v in xy), text, **kwargs)

    def line(self, xy, **kwargs):
        kwargs['width'] = kwargs.get('width', 1) * SCALE
        self.draw.line([(x * SCALE, y * SCALE) for x, y in xy], **kwargs)

    def rectangle(self, xy, **kwargs):
        kwargs['width'] = kwargs.get('width', 1) * SCALE
        self.draw.rectangle(tuple(v * SCALE for v in xy), **kwargs)


def rule(d, a, b, color=BORDER, width=1):
    d.line([a, b], fill=color, width=width)


def panel(d, rect, label):
    x0, y0, x1, y1 = rect
    d.rectangle(rect, fill=PANEL, outline=BORDER)
    text(d, (x0 + 12, y0 + 8), label, 13, DIM)
    rule(d, (x0, y0 + 32), (x1, y0 + 32))


def scan_at(seconds):
    return TRACE[min(40, max(0, int(seconds // 4)))]


def simtime(t):
    if t < 14:
        return max(0, (t - 5) / 9 * 48)
    if t < 24:
        return 48 + (t - 14) / 10 * 48
    if t < 34:
        return 96 + (t - 24) / 10 * 20
    return min(160, 116 + (t - 34) / 6 * 44)


def base(t, section):
    im = Image.new('RGB', (W * SCALE, H * SCALE), BG)
    d = ScaledDraw(im)
    text(d, (24, 18), 'trackfusion', 25, TEXT, True)
    text(d, (225, 26), '/ replay inspector', 15, DIM)
    text(d, (964, 24), 'Samuel Bixel  |  Go', 16, DIM)
    rule(d, (24, 60), (1256, 60))
    text(d, (24, 76), section, 20, TEXT, True)
    text(d, (24, 682), 'SYNTHETIC INPUT / PRODUCTION ENGINE / OFFLINE REPLAY', 13, DIM)
    text(d, (1111, 682), f'{int(t):02d}s / 51s', 13, DIM)
    rule(d, (24, 712), (1256, 712), GRID)
    rule(d, (24, 712), (24 + 1232 * t / DURATION, 712), GREEN, 2)
    return im, d


def scope(d, rect, f):
    x0, y0, x1, y1 = rect
    panel(d, rect, 'POSITION / LOCAL ENU [km]')
    text(d, (x0 + 345, y0 + 8), 'x return   [] track   grey: unnamed', 11, DIM)
    # Equal E/N scale; plotted values are exact scan snapshots, not interpolated.
    left, top, right, bottom = x0 + 57, y0 + 48, x1 - 22, y1 - 42
    scale = min((right - left) / 44000, (bottom - top) / 22000)
    cx, cy = (left + right) / 2, (top + bottom) / 2

    def pos(e, n):
        return cx + e * scale, cy - n * scale

    def inside(p):
        return left < p[0] < right and top < p[1] < bottom

    for km in range(-20, 21, 5):
        x, _ = pos(km * 1000, 0)
        rule(d, (x, top), (x, bottom), BORDER if km == 0 else GRID)
        text(d, (x - 13, bottom + 9), f'{km:>3}', 12, DIM)
    for km in range(-10, 11, 5):
        _, y = pos(0, km * 1000)
        rule(d, (left, y), (right, y), BORDER if km == 0 else GRID)
        text(d, (x0 + 15, y - 7), f'{km:>3}', 12, DIM)
    text(d, (right - 10, bottom + 9), 'E', 12, DIM)
    text(d, (x0 + 16, top - 15), 'N', 12, DIM)
    idx = f['Seconds'] // 4
    for o in f['Observations']:
        p = pos(o['e'], o['n'])
        if inside(p):
            x, y = p
            rule(d, (x - 4, y - 4), (x + 4, y + 4), AMBER)
            rule(d, (x - 4, y + 4), (x + 4, y - 4), AMBER)
    for tr in f['Tracks']:
        x = tr['State']['X']
        p = pos(*x[:2])
        if not inside(p):
            continue
        identity = tr['Identity']
        color = COLORS.get(identity, FAINT)
        if not identity:
            px, py = p
            d.rectangle((px - 3, py - 3, px + 3, py + 3), outline=FAINT)
            continue
        points = []
        for prior in TRACE[max(0, idx - 12):idx + 1]:
            old = next((z for z in prior['Tracks'] if z['ID'] == tr['ID']), None)
            if old:
                points.append(pos(*old['State']['X'][:2]))
        if points and points[-1] != p:
            points.append(p)
        if len(points) > 1:
            d.line(points, fill=color, width=2)
        for pp in points[::2]:
            d.rectangle((pp[0] - 1, pp[1] - 1, pp[0] + 1, pp[1] + 1), fill=color)
        cov = np.array(tr['State']['P'])[:2, :2]
        values, vec = np.linalg.eigh(cov)
        ellipse = []
        for theta in np.linspace(0, math.tau, 60):
            delta = vec @ (np.sqrt(np.maximum(values, 0) * 5.991) * np.array([math.cos(theta), math.sin(theta)]))
            ellipse.append(pos(x[0] + delta[0], x[1] + delta[1]))
        d.line(ellipse, fill=color, width=1)
        px, py = p
        d.rectangle((px - 4, py - 4, px + 4, py + 4), outline=color, width=2)
        speed = math.hypot(*x[2:])
        rule(d, p, (px + x[2] / max(speed, 1) * 18, py - x[3] / max(speed, 1) * 18), color, 2)
        ly = py + 15 if identity == 'aaa111' else py - 44
        lx = min(px + 12, x1 - 166) if identity == 'aaa111' else max(x0 + 15, px - 155)
        d.rectangle((lx - 3, ly - 2, lx + 151, ly + 37), fill=PANEL)
        text(d, (lx, ly), f'{tr["ID"]:03d} / {identity}', 14, color, True)
        status = 'COAST' if tr['MissCounter'] else 'UPDATE'
        text(d, (lx, ly + 20), f'{speed:.0f} m/s  {status}', 12, AMBER if tr['MissCounter'] else DIM)


def track_state(d, f):
    tr = next(z for z in f['Tracks'] if z['Identity'] == 'aaa111')
    x, p = tr['State']['X'], tr['State']['P']
    panel(d, (836, 122, 1256, 362), 'SELECTED TRACK / 000 / aaa111')
    status = 'COAST / NO OBSERVATION' if tr['MissCounter'] else 'MEASUREMENT UPDATE'
    text(d, (852, 170), status, 16, AMBER if tr['MissCounter'] else GREEN, True)
    text(d, (852, 204), 'state     east          north', 13, DIM)
    text(d, (852, 231), f'pos   {x[0]:>9.1f} m   {x[1]:>8.1f} m', 17)
    text(d, (852, 259), f'vel   {x[2]:>9.2f} m/s {x[3]:>7.2f} m/s', 17)
    rule(d, (852, 290), (1240, 290), GRID)
    text(d, (852, 305), f'misses {tr["MissCounter"]}    prune when > 4', 14, DIM)
    text(d, (852, 333), f'Ppos diag [{p[0][0]:.0f}, {p[1][1]:.0f}] m^2', 13, DIM)


def uncertainty(d, f):
    panel(d, (836, 374, 1256, 520), 'POSITION UNCERTAINTY / TRACK 000')
    left, top, right, bottom = 881, 429, 1238, 485
    text(d, (850, 413), '250', 11, DIM)
    text(d, (865, 476), '0', 11, DIM)
    rule(d, (left, top), (right, top), GRID)
    rule(d, (left, bottom), (right, bottom), BORDER)
    # Shaded interval is the deliberate omission of three target detections.
    gx0 = left + 100 / 160 * (right - left)
    gx1 = left + 112 / 160 * (right - left)
    d.rectangle((gx0, top, gx1, bottom), fill='#343027')
    points = []
    for fr in TRACE[:f['Seconds'] // 4 + 1]:
        tr = next(z for z in fr['Tracks'] if z['Identity'] == 'aaa111')
        p = tr['State']['P']
        sigma = math.sqrt(p[0][0] + p[1][1])
        points.append((left + fr['Seconds'] / 160 * (right - left), bottom - sigma / 250 * (bottom - top)))
    if len(points) > 1:
        d.line(points, fill=GREEN, width=2)
    tr = next(z for z in f['Tracks'] if z['Identity'] == 'aaa111')
    val = math.sqrt(tr['State']['P'][0][0] + tr['State']['P'][1][1])
    text(d, (852, 495), f'sqrt(Pee + Pnn) = {val:5.1f} m', 12, DIM)
    text(d, (1196, 495), '160s', 11, DIM)


def event_log(d, f):
    panel(d, (24, 532, 1256, 618), 'SCAN LOG / LAST 2 RADAR SCANS')
    idx = f['Seconds'] // 4
    for row, fr in enumerate(TRACE[max(0, idx - 1):idx + 1]):
        s = fr['Stats']
        txt = (f't={fr["Seconds"]:03d}s  sensor=radar  obs={s["Meas"]:02d}  matched={s["Matched"]:02d}  '
               f'withheld={s["Withheld"]:02d}  spawned={s["Spawned"]:02d}  pruned={s["Pruned"]:02d}  live={s["Live"]:02d}')
        text(d, (38, 570 + row * 22), txt, 16, TEXT if row else DIM)


def caption(d, s):
    text(d, (24, 638), s, 18, TEXT)


def config(d):
    panel(d, (836, 122, 1256, 520), 'SCENARIO CONFIGURATION')
    rows = [('targets', '2 / constant velocity'), ('frame', 'local ENU [m]'),
            ('initialization', 'ADS-B identity + position'), ('radar period', '4 s'),
            ('range sigma', '65 m'), ('bearing sigma', '0.5 deg'),
            ('clutter', 'Poisson / mean 1.5'), ('random seed', '21'),
            ('target dropout', '100, 104, 108 s')]
    for i, (k, v) in enumerate(rows):
        y = 174 + i * 35
        text(d, (852, y), k, 13, DIM)
        text(d, (988, y), v, 12, AMBER if k == 'target dropout' else TEXT)


def methods(d):
    panel(d, (24, 122, 808, 520), 'SCAN PROCESSING / PRODUCTION ENGINE')
    rows = [
        ('01', 'PREDICT', 'x = [east, north, v_east, v_north]', 'Constant-velocity Kalman model; dt from scan timestamps.'),
        ('02', 'GATE + ASSIGN', 'd^2 = residual^T S^-1 residual <= 13.8', 'Hungarian assignment; ambiguity margin = 2.0.'),
        ('03', 'UPDATE / COAST', 'matched -> correction     unmatched -> miss + 1', 'Withheld pairs take no correction and no miss.'),
        ('04', 'SPAWN + PRUNE', 'unmatched observation -> new track', 'Remove tracks after more than 4 missed scans.'),
    ]
    for i, (num, label, code, detail) in enumerate(rows):
        y = 169 + i * 87
        text(d, (40, y), num, 16, DIM)
        text(d, (88, y), label, 16, GREEN, True)
        text(d, (88, y + 26), code, 16, TEXT)
        text(d, (88, y + 51), detail, 15, DIM)
    panel(d, (836, 122, 1256, 520), 'MODULES / GO')
    rows = [('opensky', 'OAuth2 + ADS-B polling'), ('radar', 'noise / clutter / dropout'),
            ('geo', 'WGS84 <-> ENU'), ('filter', 'state + covariance'),
            ('assoc', 'gating + assignment'), ('track', 'identity + lifecycle'),
            ('record', 'JSONL capture / replay')]
    for i, (name, role) in enumerate(rows):
        text(d, (852, 169 + i * 43), name, 15, TEXT)
        text(d, (970, 171 + i * 43), role, 12, DIM)
    panel(d, (24, 532, 1256, 618), 'VERIFICATION')
    text(d, (40, 576), '93 tests / 8 packages   |   race checks passed   |   repeat export: byte-identical', 17, TEXT)
    caption(d, 'The display reads exported states; estimation and association run in Go.')


def summary(d):
    panel(d, (24, 122, 808, 520), 'RUN SUMMARY / SEED 21')
    metrics = [('Input', '1 ADS-B scan + 41 radar scans'),
               ('Simulated duration', '160 s'),
               ('Identified tracks at end', '2 / 2'),
               ('Position RMSE', f'{DATA["position_rmse_m"]:.1f} m'),
               ('RMSE sample scope', '76 identified-track states; t >= 12 s'),
               ('Injected dropout', '3 radar detections omitted for aaa111'),
               ('Reacquisition', '112 s / same track ID 000')]
    for i, (k, v) in enumerate(metrics):
        y = 172 + i * 43
        text(d, (40, y), k, 14, DIM)
        text(d, (315, y), v, 14, GREEN if k == 'Position RMSE' else TEXT)
    text(d, (40, 486), 'One controlled scenario; not a general accuracy benchmark.', 13, DIM)
    panel(d, (836, 122, 1256, 520), 'REPLAY / LOCAL')
    for i, s in enumerate(['$ go run ./cmd/trackfusion \\', '  -replay output/demo/scenario.jsonl', '',
                            'No API credentials.', 'No network requests.', 'Recorded sensor inputs.', '',
                            'Same seed and inputs;', 'same engine output.']):
        text(d, (852, 177 + i * 31), s, 14, TEXT if i < 2 else DIM)
    panel(d, (24, 532, 1256, 618), 'SOURCE / SAMUEL BIXEL')
    text(d, (40, 572), 'github.com/sambixel/trackfusion', 23, TEXT, True)
    caption(d, 'Go / state estimation / sensor integration / deterministic replay')


def render(t):
    if t < 5:
        im, d = base(t, '00 / RUN SETUP')
        f = TRACE[0]
        scope(d, (24, 122, 808, 520), f)
        config(d)
        event_log(d, f)
        caption(d, 'Two aircraft initialized from ADS-B; subsequent observations come from simulated radar.')
    elif t < 40:
        seconds = simtime(t)
        f = scan_at(seconds)
        if t < 14:
            title = '01 / RADAR OBSERVATIONS'
            cap = 'Radar returns have no identity. Clutter can initiate temporary unnamed tracks.'
        elif t < 24:
            title = '02 / TRACK ASSOCIATION'
            cap = 'Each scan is assigned globally after gating against predicted track uncertainty.'
        elif f['Seconds'] < 112:
            title = '03 / DETECTION GAP'
            cap = 'Track 000 coasts through three missed sweeps; position uncertainty increases.'
        else:
            title = '04 / MEASUREMENT REACQUIRED'
            cap = 'At t=112 s, a radar observation updates the existing track and reduces uncertainty.'
        im, d = base(t, title)
        text(d, (828, 80), f't={f["Seconds"]:03d} s  |  scan={f["Seconds"]//4+2:02d}/42', 16, DIM)
        scope(d, (24, 122, 808, 520), f)
        track_state(d, f)
        uncertainty(d, f)
        event_log(d, f)
        caption(d, cap)
    elif t < 46:
        im, d = base(t, '05 / IMPLEMENTATION')
        methods(d)
    else:
        im, d = base(t, '06 / REPRODUCIBLE OUTPUT')
        summary(d)
    return im


if __name__ == '__main__':
    if '--preview' in sys.argv:
        times = [2, 7, 20, 28, 30, 33, 37, 43, 48]
        sheet = Image.new('RGB', (1920, 1080), BG)
        for i, t in enumerate(times):
            im = render(t)
            im.save(TMP / f'technical-preview-{i}.png')
            sheet.paste(im.resize((640, 360)), ((i % 3) * 640, (i // 3) * 360))
        sheet.save(TMP / 'technical-storyboard.jpg', quality=95)
        render(28).save(OUT / ('poster-technical-1440p.jpg' if SCALE == 2 else 'poster-technical.jpg'), quality=98, subsampling=0)
    else:
        for i in range(FPS * DURATION):
            if SCALE > 1:
                render(i / FPS).save(FRAMES / f'{i:05d}.png')
            else:
                render(i / FPS).save(FRAMES / f'{i:05d}.jpg', quality=95, subsampling=0)
            if i % 240 == 0:
                print(f'Rendered {i}/{FPS * DURATION} technical frames', flush=True)
