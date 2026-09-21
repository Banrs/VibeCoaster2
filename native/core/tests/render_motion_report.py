"""Render synchronized diagnostic SVGs from a retained CLI trace and report.

Pure Python; the plots are display-rate diagnostics. Acceptance peaks come
from the report's native-rate metrics, not these downsampled polylines.
"""
import html
import json
import math
from pathlib import Path
import sys


def render(base, seat_index=0):
    trace = json.loads(Path(str(base) + "-trace.json").read_text(encoding="utf-8"))
    report = json.loads(base.with_suffix(".json").read_text(encoding="utf-8"))
    frames, geometry = trace["frames"], trace["geometry"]
    colors = ["#007c83", "#c34c23", "#7049aa"]
    width, height = 1600, 2140
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="#fafcfd"/>',
           '<style>text{font-family:Segoe UI,Arial,sans-serif;fill:#1c3040;font-size:15px}.title{font-size:24px;font-weight:600}.small{font-size:13px}</style>']

    def text(x, y, value, cls=""):
        svg.append(f'<text x="{x}" y="{y}" class="{cls}">{html.escape(str(value))}</text>')

    text(40, 38, f'VibeCoaster · seed {report["seed"]} · {report["generatorVersion"]}', "title")
    text(40, 65, f"Synchronized 60 Hz display trace · {('front', 'middle', 'rear')[seat_index]} seat · native 960/1920 Hz acceptance extrema in JSON")
    text(40, 90, f'300 km/h baseline · measured {report["metrics"]["maxSpeed"] * 3.6:.2f} km/h · first 180 km/h {report["metrics"]["launchTo180"]:.6f} s · accepted: {report["accepted"]}')
    end = frames[-1]["time"]
    times = [f["time"] for f in frames]

    def panel(y, title, series, limits=None, labels=(), guides=()):
        x0, x1, top, bottom = 95, 1530, y + 30, y + 182
        values = [v for _, data, _ in series for v in data]
        low, high = limits or (min(values), max(values))
        if high <= low:
            high = low + 1
        text(40, y + 16, title)
        for index, label in enumerate(labels):
            text(1050 + 150 * index, y + 16, label)
            svg.append(f'<path d="M {1025 + 150 * index} {y+11} h 18" stroke="{colors[index]}" stroke-width="3"/>')
        def point(t, value):
            return x0 + (x1 - x0) * t / end, bottom - (value - low) / (high - low) * (bottom - top)
        for tick in range(5):
            value = low + (high - low) * tick / 4
            _, yy = point(0, value)
            svg.append(f'<path d="M {x0} {yy:.2f} H {x1}" stroke="#dbe4e9"/>')
            text(42, yy + 5, f'{value:.1f}', "small")
        for tick in range(0, int(end) + 1, 20):
            xx, _ = point(tick, low)
            svg.append(f'<path d="M {xx:.2f} {top} V {bottom}" stroke="#e5ebee"/>')
            text(xx - 10, bottom + 20, tick, "small")
        for value in guides:
            _, yy = point(0, value)
            svg.append(f'<path d="M {x0} {yy:.2f} H {x1}" stroke="#ab3548" stroke-dasharray="5,4"/>')
        for xvalues, data, color in series:
            points = " ".join(f'{x:.2f},{yy:.2f}' for x, yy in (point(t, v) for t, v in zip(xvalues, data)))
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.3"/>')

    def series(values, color):
        return times, values, color

    panel(115, 'Speed · km/h', [series([f['speed'] * 3.6 for f in frames], colors[0])], (0, max(330, report['metrics']['maxSpeed']*3.7)), guides=(180, 300))
    for index, key in enumerate(('roll', 'pitch', 'yaw')):
        panel(340 + index * 195, key.capitalize() + ' · continuous degrees (physical vectors resolve vertical-pitch ambiguity)',
              [series([math.degrees(f['dynamics'][seat_index][key]) for f in frames], colors[index])])
    panel(925, 'Rider specific force · g', [series([f['seats'][seat_index][i] for f in frames], colors[i]) for i in range(3)], (-1.5, 5.0), ('Gz / up', 'Gy / right', 'Gx / forward'), (-1.5, 5.0))
    panel(1150, 'Absolute rider force rates · g/s', [series([abs(f['dynamics'][seat_index]['forceRateGps'][i]) for f in frames], colors[i]) for i in range(3)], (0, 20), ('Gz rate', 'Gy rate', 'Gx rate'), (20,))
    magnitude = lambda v: math.sqrt(sum(x*x for x in v))
    panel(1375, 'Inertial jerk · m/s³ (vector magnitude; includes gravity-independent acceleration change)', [series([magnitude(f['dynamics'][seat_index]['inertialJerkMps3']) for f in frames], colors[0])])
    panel(1600, 'Angular jerk · rad/s³ (world vector magnitude)', [series([magnitude(f['dynamics'][seat_index]['angularJerkRadps3']) for f in frames], colors[2])])
    text(730, 1845, 'Elapsed ride time · seconds')
    native = report['seatStatistics']
    text(40, 1880, 'Native-rate maxima across front / middle / rear seats', "title")
    for i, seat in enumerate(native):
        text(40, 1915 + i * 29, f'{("Front", "Middle", "Rear")[i]}: inertial jerk {seat["maxInertialJerkMps3"]:.3f} m/s³ · angular jerk {seat["maxAngularJerkRadps3"]:.3f} rad/s³ · Gz/Gy/Gx rates ' + ' / '.join(f'{axis["maxRateGps"]:.3f}' for axis in seat['axes']) + ' g/s')
    text(40, 2080, 'Force and rate limits are provisional game assumptions. These charts do not establish rider comfort, reference cadence equivalence or engineering certification.')
    svg.append('</svg>')
    Path(str(base) + '-motion' + ('', '-middle', '-rear')[seat_index] + '.svg').write_text('\n'.join(svg), encoding='utf-8')


if __name__ == '__main__':
    for seat in range(3):
        render(Path(sys.argv[1]), seat)
