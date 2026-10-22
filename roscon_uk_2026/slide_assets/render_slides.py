"""Render the Markdown draft as a local, syntax-highlighted slide deck."""
from pathlib import Path
import html
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'build/slide_render_deps'))
import markdown
from pygments.formatters import HtmlFormatter

source = ROOT / 'roscon_uk_2026/SLIDE_DRAFT.md'
parts = re.split(r'(?m)^## (?=Slide\s+\d)', source.read_text())
slides = []
def render_markdown(text):
    diagrams = []
    def diagram(match):
        diagrams.append('<pre class="mermaid">' + html.escape(match.group(1)) + '</pre>')
        return f'\n\nMERMAIDPLACEHOLDER{len(diagrams)-1}\n\n'
    text = re.sub(r'```mermaid\n(.*?)```', diagram, text, flags=re.S)
    rendered = markdown.markdown(text, extensions=['fenced_code', 'tables', 'codehilite'])
    for index, content in enumerate(diagrams):
        rendered = rendered.replace(f'<p>MERMAIDPLACEHOLDER{index}</p>', content)
    return rendered

for part in parts[1:]:
    screen, separator, notes = part.partition('**Say**')
    rendered = render_markdown('## ' + screen)
    if separator:
        rendered += '<details><summary>Speaker notes and media cues</summary>'
        rendered += render_markdown('**Say**' + notes)
        rendered += '</details>'
    slides.append('<section class="slide">' + rendered + '</section>')
if not slides:
    raise ValueError('Expected numbered Slide headings')
css = HtmlFormatter(style='monokai').get_style_defs('.codehilite')
mermaid_js = (source.parent / 'slide_assets/vendor/mermaid.min.js').read_text()
page = '''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>cppyy_kit: slide content draft</title><style>
body{margin:0;background:#eef1f5;color:#162334;font:20px/1.5 system-ui,sans-serif}
nav{position:sticky;top:0;background:#162334;color:white;padding:10px 24px;z-index:2;display:flex;gap:18px;align-items:center}
button{padding:8px 14px;font:inherit;font-size:15px;cursor:pointer}
.slide{box-sizing:border-box;background:white;max-width:1180px;margin:24px auto;padding:36px 56px;min-height:660px;box-shadow:0 2px 12px #0001}
h2{font-size:38px;line-height:1.2;color:#173f68;margin-top:0}h3{font-size:21px;color:#35638b;margin-top:28px}
p,li{max-width:1000px}a{color:#126397}pre{font:17px/1.45 ui-monospace,monospace;overflow:auto;padding:18px;border-radius:6px;background:#272822;color:#f8f8f2}
.mermaid{background:white;color:#162334;text-align:center;white-space:normal}.mermaid svg{max-width:100%;height:auto}code{font-family:ui-monospace,monospace}table{border-collapse:collapse;font-size:18px}td,th{padding:9px 14px;border:1px solid #c9d3df;text-align:left}
blockquote{border-left:4px solid #6393bd;margin-left:0;padding-left:20px;color:#324d66}
details{margin-top:32px;border-top:1px solid #c9d3df;padding-top:12px;font-size:18px}summary{cursor:pointer;color:#35638b}img{max-width:100%;max-height:440px}body.single .slide{display:none}body.single .slide.active{display:block}
@media print{nav{display:none}.slide,body.single .slide{display:block;box-shadow:none;margin:0;break-before:page;max-width:none}pre{white-space:pre-wrap}}
''' + css + '''</style></head><body class="single"><nav>
<button onclick="go(-1)">Previous</button><span id="count"></span><button onclick="go(1)">Next</button>
<button onclick="document.body.classList.toggle('single')">Slides / all content</button>
<button onclick="document.querySelectorAll('details').forEach(d=>d.open=!d.open)">Notes</button>
<span style="font-size:14px">Draft with speaker notes · ← → to navigate · print for PDF</span></nav>
''' + '\n'.join(slides) + '<script>' + mermaid_js + '</script>' + '''<script>
const slides=[...document.querySelectorAll('.slide')];let current=Math.max(0,(parseInt(location.hash.slice(1))||1)-1);
function go(delta){current=Math.max(0,Math.min(slides.length-1,current+delta));slides.forEach((s,i)=>s.classList.toggle('active',i===current));document.querySelector('#count').textContent=(current+1)+' / '+slides.length;window.scrollTo(0,0)}
document.addEventListener('keydown',e=>{if(e.key==='ArrowRight')go(1);if(e.key==='ArrowLeft')go(-1)});
document.body.classList.remove('single');
mermaid.initialize({startOnLoad:false, securityLevel:'strict', theme:'neutral'});
mermaid.run({querySelector:'.mermaid'}).then(()=>{document.body.classList.add('single');go(0)}).catch(error=>{console.error(error);go(0)});
</script></body></html>'''
output = source.with_suffix('.html')
output.write_text(page)
print(f'{output}: {len(slides)} slides')
