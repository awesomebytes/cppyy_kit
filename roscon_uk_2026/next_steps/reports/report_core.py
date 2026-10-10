"""Integer-clock ranking and alignment, independent of native bindings."""
from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass
import html
import math
import numbers

MAX_SAMPLES = 1_000_000


def integer_times(values, *, strict=False):
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            raise TypeError('timestamps must be integers in nanoseconds')
        result.append(int(value))
    if len(result) > MAX_SAMPLES:
        raise ValueError('sample limit exceeded')
    if any(b <= a if strict else b < a for a, b in zip(result, result[1:])):
        raise ValueError('timestamps must increase' if strict else 'timestamps must be sorted')
    return result


@dataclass(frozen=True)
class Window:
    start_ns: int
    end_ns: int
    score: float
    samples: int
    representative_ns: int
    representative_index: int

    def as_dict(self):
        return asdict(self)


def rank_windows(times, scores, duration_ns, count=5):
    """Greedy disjoint windows, descending mean score (rounded to 1e-12) then earliest start.

    Candidates start at each sample with a full duration before/equal to the
    last timestamp. Membership is [start, end). Touching windows do not overlap.
    Representative is the highest-score member, breaking ties by earliest time.
    """
    times = integer_times(times, strict=True)
    if not isinstance(duration_ns, numbers.Integral) or isinstance(duration_ns, bool) or duration_ns <= 0:
        raise ValueError('duration_ns must be a positive integer')
    if not isinstance(count, int) or isinstance(count, bool) or not 1 <= count <= 5:
        raise ValueError('count must be in [1, 5]')
    scores = [float(x) for x in scores]
    if len(times) != len(scores) or any(not math.isfinite(x) or x < 0 for x in scores):
        raise ValueError('scores must be finite, nonnegative, and match timestamps')
    prefix = [0.0]
    for score in scores:
        prefix.append(prefix[-1] + score)
        if not math.isfinite(prefix[-1]):
            raise ValueError('score accumulation overflow')
    candidates = []
    for i, start in enumerate(times):
        end = start + int(duration_ns)
        if end > times[-1]:
            break
        j = bisect_left(times, end)
        score = (prefix[j] - prefix[i]) / (j - i)
        candidates.append((score, start, end, i, j))
    candidates.sort(key=lambda row: (-round(row[0], 12), row[1]))
    selected = []
    for score, start, end, i, j in candidates:
        if any(start < window.end_ns and window.start_ns < end for window in selected):
            continue
        peak = max(range(i, j), key=lambda index: (scores[index], -times[index]))
        selected.append(Window(start, end, score, j-i, times[peak], peak))
        if len(selected) == count:
            break
    return selected


def match_images(queries, image_times, *, query_clock, image_clock,
                 direction='nearest', tolerance_ns=60_000_000):
    """Integer as-of join. Nearest ties prefer earlier time, then source order.

    Exact duplicate image times: nearest/forward use first source record;
    backward uses last. An unmatched query is retained with null image fields.
    """
    checked = []
    for value in queries:
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            raise TypeError('query timestamps must be integers')
        checked.append(int(value))
    queries = checked
    if len(queries) > MAX_SAMPLES:
        raise ValueError('query limit exceeded')
    image_times = integer_times(image_times)
    if query_clock != image_clock:
        raise ValueError('clock mismatch; supply timestamps from the same declared clock')
    if direction not in ('nearest', 'backward', 'forward'):
        raise ValueError('direction must be nearest, backward, or forward')
    if isinstance(tolerance_ns, bool) or not isinstance(tolerance_ns, numbers.Integral) or tolerance_ns < 0:
        raise ValueError('tolerance_ns must be a nonnegative integer')
    result = []
    for query in queries:
        left = bisect_left(image_times, query)
        if direction == 'forward':
            index = left if left < len(image_times) else None
        elif direction == 'backward':
            index = bisect_right(image_times, query)-1
            index = index if index >= 0 else None
        else:
            candidates = []
            if left < len(image_times):
                candidates.append(left)
            if left:
                candidates.append(bisect_left(image_times, image_times[left-1]))
            index = min(candidates, key=lambda i: (abs(image_times[i]-query), image_times[i], i)) if candidates else None
        delta = None if index is None else image_times[index]-query
        if delta is not None and abs(delta) > tolerance_ns:
            index, delta = None, None
        result.append({'query_ns': int(query), 'image_index': index,
                       'image_ns': None if index is None else image_times[index],
                       'delta_ns': delta, 'status': 'unmatched' if index is None else 'matched'})
    return result


def check_frames(pose_frame, reference_frame, expected_frame):
    if not pose_frame or pose_frame != reference_frame or pose_frame != expected_frame:
        raise ValueError('pose, reference and configuration frames must agree; transforms are not inferred')


def trace_svg(times, series, *, width=760, height=230):
    """Standalone bounded SVG trace. Convert relative ns only after subtraction."""
    if not times:
        return '<p>No trace samples.</p>'
    values = [value for _, rows in series for value in rows]
    if not all(math.isfinite(x) for x in values):
        raise ValueError('plot values must be finite')
    ymin, ymax = min(values), max(values)
    span = ymax-ymin or 1.0
    start, end = int(times[0]), int(times[-1])
    span_ns = end-start or 1
    colors = ['#2266aa', '#cc3311', '#228833']
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img" aria-label="Position metric trace in metres">',
             '<rect width="100%" height="100%" fill="white"/>']
    stride = max(1, (len(times)+1499)//1500)
    for index, (name, rows) in enumerate(series):
        indices = list(range(0, len(times), stride))
        if indices[-1] != len(times)-1:
            indices.append(len(times)-1)
        points = ' '.join(f'{40+(int(times[i])-start)/span_ns*(width-60):.3f},{height-35-(rows[i]-ymin)/span*(height-65):.3f}' for i in indices)
        color = colors[index % len(colors)]
        parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.5" points="{points}"/>')
        parts.append(f'<text x="45" y="{18+index*16}" fill="{color}">{html.escape(name)}</text>')
    parts.append(f'<text x="40" y="{height-8}">0 s</text><text x="{width-130}" y="{height-8}">{span_ns/1e9:.6g} s</text>')
    parts.append(f'<text x="5" y="50">{ymax:.4g} m</text><text x="5" y="{height-40}">{ymin:.4g} m</text></svg>')
    return ''.join(parts)
