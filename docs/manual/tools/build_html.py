#!/usr/bin/env python3
"""Build a single self-contained HTML file from docs/manual/*.md for offline viewing.

Usage: python3 build_html.py [output_path]
Embeds all images as base64 data URIs so the result is one file, no network needed.
"""
import base64
import mimetypes
import re
import sys
from pathlib import Path

import markdown

MANUAL_DIR = Path(__file__).resolve().parent.parent
OUT_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else MANUAL_DIR / "Intelijet_PPS_Manual.html"

CHAPTERS = [
    "00_contents.md",
    "01_introduction.md",
    "02_safety.md",
    "03_installation.md",
    "04_startup_shutdown.md",
    "05_screens.md",
    "06_projects_and_jobs.md",
    "07_scanning.md",
    "08_reports.md",
    "09_monitoring_alarms.md",
    "10_troubleshooting.md",
    "11_maintenance.md",
    "12_data_sync.md",
    "13_usb_copier.md",
    "14_appendix.md",
]


def embed_images(html: str) -> str:
    def repl(m):
        prefix, src, suffix = m.group(1), m.group(2), m.group(3)
        if src.startswith(("http://", "https://", "data:")):
            return m.group(0)
        img_path = (MANUAL_DIR / src).resolve()
        if not img_path.exists():
            return m.group(0)
        mime = mimetypes.guess_type(str(img_path))[0] or "image/png"
        data = base64.b64encode(img_path.read_bytes()).decode("ascii")
        return f'{prefix}data:{mime};base64,{data}{suffix}'

    return re.sub(r'(<img[^>]*src=")([^"]+)("[^>]*>)', repl, html)


def internal_link_to_anchor(href: str):
    m = re.match(r'^(\d\d_[a-z_]+)\.md(#.*)?$', href)
    if m:
        return f"#{m.group(1)}"
    return None


def fix_links(html: str, slug: str) -> str:
    def repl(m):
        prefix, href, suffix = m.group(1), m.group(2), m.group(3)
        anchor = internal_link_to_anchor(href)
        if anchor:
            return f'{prefix}{anchor}{suffix}'
        return m.group(0)

    return re.sub(r'(<a[^>]*href=")([^"]+)("[^>]*>)', repl, html)


def build():
    md = markdown.Markdown(extensions=["tables", "fenced_code", "toc", "sane_lists"])
    sections = []
    nav_items = []

    for fname in CHAPTERS:
        path = MANUAL_DIR / fname
        if not path.exists():
            continue
        slug = fname[:-3]
        text = path.read_text(encoding="utf-8")
        md.reset()
        html = md.convert(text)
        html = embed_images(html)
        html = fix_links(html, slug)

        title_match = re.search(r"^#\s+(.+)$", text, re.MULTILINE)
        title = title_match.group(1) if title_match else slug
        nav_items.append((slug, title))
        sections.append(f'<section id="{slug}" class="chapter">{html}</section>')

    nav_html = "\n".join(
        f'<li><a href="#{slug}" onclick="selectChapter(\'{slug}\')">{title}</a></li>'
        for slug, title in nav_items
    )

    body = "\n".join(sections)

    html_doc = f"""<!doctype html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Jacon Intelijet PPS — Operator Manual</title>
<style>
  :root {{
    --bg: #ffffff;
    --fg: #1a1a1a;
    --muted: #5a5a5a;
    --accent: #b3261e;
    --border: #d8d8d8;
    --nav-bg: #f4f2ee;
    --code-bg: #f0f0f0;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{
      --bg: #1c1c1c;
      --fg: #eaeaea;
      --muted: #a8a8a8;
      --accent: #ff6b60;
      --border: #3a3a3a;
      --nav-bg: #262626;
      --code-bg: #2a2a2a;
    }}
  }}
  * {{ box-sizing: border-box; }}
  html, body {{ margin: 0; padding: 0; background: var(--bg); color: var(--fg); }}
  body {{
    font-family: Georgia, "Noto Serif", "Times New Roman", serif;
    font-size: 17px;
    line-height: 1.6;
  }}
  #layout {{ display: flex; min-height: 100vh; }}
  #sidebar {{
    width: 280px;
    flex-shrink: 0;
    background: var(--nav-bg);
    border-right: 1px solid var(--border);
    padding: 20px 16px;
    position: sticky;
    top: 0;
    height: 100vh;
    overflow-y: auto;
  }}
  #sidebar h1 {{
    font-size: 16px;
    margin: 0 0 4px;
    font-family: Arial, sans-serif;
  }}
  #sidebar .sub {{
    font-size: 12px;
    color: var(--muted);
    margin-bottom: 16px;
    font-family: Arial, sans-serif;
  }}
  #sidebar ul {{
    list-style: none;
    padding: 0;
    margin: 0;
  }}
  #sidebar li {{ margin-bottom: 2px; }}
  #sidebar a {{
    display: block;
    padding: 7px 8px;
    border-radius: 4px;
    color: var(--fg);
    text-decoration: none;
    font-family: Arial, sans-serif;
    font-size: 14px;
  }}
  #sidebar a:hover {{ background: var(--border); }}
  #sidebar a.active {{ background: var(--accent); color: #fff; }}
  #search {{
    width: 100%;
    padding: 8px;
    margin-bottom: 14px;
    border: 1px solid var(--border);
    border-radius: 4px;
    background: var(--bg);
    color: var(--fg);
    font-family: Arial, sans-serif;
    font-size: 13px;
  }}
  #content {{
    flex: 1;
    max-width: 900px;
    padding: 32px 48px 80px;
  }}
  .chapter {{ display: none; }}
  .chapter.active {{ display: block; }}
  h1, h2, h3 {{ font-family: Arial, sans-serif; }}
  h1 {{ border-bottom: 3px solid var(--accent); padding-bottom: 10px; }}
  h2 {{ border-bottom: 1px solid var(--border); padding-bottom: 6px; margin-top: 36px; }}
  strong {{ color: var(--fg); }}
  img {{ max-width: 100%; border: 1px solid var(--border); border-radius: 4px; margin: 12px 0; }}
  table {{ border-collapse: collapse; width: 100%; margin: 16px 0; font-family: Arial, sans-serif; font-size: 14px; }}
  th, td {{ border: 1px solid var(--border); padding: 8px 10px; text-align: left; vertical-align: top; }}
  th {{ background: var(--nav-bg); }}
  code {{ background: var(--code-bg); padding: 1px 5px; border-radius: 3px; font-size: 0.9em; }}
  pre {{ background: var(--code-bg); padding: 12px; border-radius: 6px; overflow-x: auto; }}
  blockquote {{ border-left: 4px solid var(--accent); margin: 12px 0; padding: 4px 16px; color: var(--muted); }}
  a {{ color: var(--accent); }}
  #topbar {{
    display: none;
  }}
  .prevnext {{
    display: flex;
    justify-content: space-between;
    margin-top: 48px;
    padding-top: 16px;
    border-top: 1px solid var(--border);
    font-family: Arial, sans-serif;
    font-size: 14px;
  }}
  .prevnext a {{ text-decoration: none; }}
  @media print {{
    #sidebar {{ display: none; }}
    .chapter {{ display: block !important; page-break-after: always; }}
    #content {{ max-width: 100%; padding: 0; }}
  }}
  @media (max-width: 800px) {{
    #layout {{ flex-direction: column; }}
    #sidebar {{ width: 100%; height: auto; position: static; }}
    #content {{ padding: 20px; }}
  }}
</style>
</head>
<body>
<div id="layout">
  <nav id="sidebar">
    <h1>Jacon Intelijet PPS</h1>
    <div class="sub">Operator Manual — offline copy</div>
    <input type="text" id="search" placeholder="Tìm trong mục lục…" oninput="filterNav()">
    <ul id="navlist">
      {nav_html}
    </ul>
  </nav>
  <main id="content">
    {body}
  </main>
</div>
<script>
  var chapters = {[slug for slug, _ in nav_items]!r};

  function selectChapter(slug) {{
    document.querySelectorAll('.chapter').forEach(function(el) {{
      el.classList.toggle('active', el.id === slug);
    }});
    document.querySelectorAll('#sidebar a').forEach(function(a) {{
      a.classList.toggle('active', a.getAttribute('href') === '#' + slug);
    }});
    window.scrollTo(0, 0);
    if (history.replaceState) history.replaceState(null, '', '#' + slug);
    renderPrevNext(slug);
  }}

  function renderPrevNext(slug) {{
    var idx = chapters.indexOf(slug);
    var container = document.querySelector('#' + slug + ' .prevnext');
    if (container) container.remove();
    var links = document.createElement('div');
    links.className = 'prevnext';
    var prevHtml = idx > 0 ? '<a href="#' + chapters[idx-1] + '" onclick="selectChapter(\\'' + chapters[idx-1] + '\\')">&larr; Chương trước</a>' : '<span></span>';
    var nextHtml = idx < chapters.length - 1 ? '<a href="#' + chapters[idx+1] + '" onclick="selectChapter(\\'' + chapters[idx+1] + '\\')">Chương tiếp &rarr;</a>' : '<span></span>';
    links.innerHTML = prevHtml + nextHtml;
    document.getElementById(slug).appendChild(links);
  }}

  function filterNav() {{
    var q = document.getElementById('search').value.toLowerCase();
    document.querySelectorAll('#navlist li').forEach(function(li) {{
      var text = li.textContent.toLowerCase();
      li.style.display = text.indexOf(q) === -1 ? 'none' : '';
    }});
  }}

  (function init() {{
    var hash = window.location.hash.replace('#', '');
    var start = chapters.indexOf(hash) !== -1 ? hash : chapters[0];
    selectChapter(start);
  }})();
</script>
</body>
</html>
"""
    OUT_PATH.write_text(html_doc, encoding="utf-8")
    print(f"Wrote {OUT_PATH} ({OUT_PATH.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    build()
