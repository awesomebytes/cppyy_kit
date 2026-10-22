"""Measure browser image decodes and animation callbacks against a running Viser."""
import argparse
import json
import os
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[5]
SAMPLE = """async (seconds) => {
  const started = performance.now();
  const loads = {};
  const raf = [];
  let running = true;
  function loaded(event) {
    const image = event.target;
    if (!(image instanceof HTMLImageElement) || image.naturalWidth < 100) return;
    const index = Array.from(document.images).indexOf(image);
    const key = `image_${index}_${image.naturalWidth}x${image.naturalHeight}`;
    (loads[key] ??= []).push(performance.now() - started);
  }
  function animate(time) {
    if (running) {raf.push(time); requestAnimationFrame(animate);}
  }
  document.addEventListener('load', loaded, true);
  requestAnimationFrame(animate);
  await new Promise(resolve => setTimeout(resolve, seconds * 1000));
  running = false;
  document.removeEventListener('load', loaded, true);
  const elapsed = (performance.now() - started) / 1000;
  const canvas = document.createElement('canvas');
  const gl = canvas.getContext('webgl');
  const debug = gl && gl.getExtension('WEBGL_debug_renderer_info');
  return {
    elapsed_s: elapsed,
    image_load_events_ms: loads,
    image_load_hz: Object.fromEntries(Object.entries(loads).map(([k,v]) => [k,v.length/elapsed])),
    animation_callbacks: raf.length,
    animation_callback_hz: raf.length / elapsed,
    webgl_renderer: debug ? gl.getParameter(debug.UNMASKED_RENDERER_WEBGL) : null,
    images: Array.from(document.images).map(i => ({width:i.naturalWidth,height:i.naturalHeight,complete:i.complete})),
  };
}"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8768')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--headed', action='store_true')
    parser.add_argument('--chrome-vulkan', action='store_true', help='Headed system Chrome with NVIDIA Vulkan ANGLE')
    parser.add_argument('--output', type=Path, default=ROOT / 'build/webcam_session/rendering_evidence/viser_browser.json')
    args = parser.parse_args()
    with sync_playwright() as playwright:
        options = {'headless': not args.headed, 'args': ['--no-sandbox']}
        if args.chrome_vulkan:
            options.update(headless=False, executable_path='/usr/bin/google-chrome',
                           args=['--no-sandbox', '--ignore-gpu-blocklist', '--use-gl=angle', '--use-angle=vulkan'],
                           env={**os.environ, '__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'})
        browser = playwright.chromium.launch(**options)
        page = browser.new_page(viewport={'width':1440, 'height':1000})
        page.goto(args.url)
        page.wait_for_timeout(3000)
        result = page.evaluate(SAMPLE, args.seconds)
        result['url'] = args.url
        result['headless'] = options['headless']
        result['meaning'] = (
            'Image load events count newly decoded GUI image resources. Animation callbacks '
            'count browser scheduling opportunities, not unique cloud renders or GPU throughput. '
            'Neither measures camera-to-display latency.'
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        page.screenshot(path=str(args.output.with_suffix('.png')))
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps({key: value for key, value in result.items() if key != 'image_load_events_ms'}, indent=2))
        browser.close()


if __name__ == '__main__':
    main()
