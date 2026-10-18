"""Regenerate the embedded page after editing web/index.html. Python stdlib only."""
from pathlib import Path

root = Path(__file__).resolve().parent
html = (root / 'web/index.html').read_text(encoding='utf-8')
assert ')AMB82HTML"' not in html, 'Raw string delimiter conflict'
(root / 'GestureLight/WebPage.h').write_text(
    '#pragma once\n// Generated from web/index.html by build_web.py.\n'
    'static const char WEB_PAGE[] = R"AMB82HTML(' + html + ')AMB82HTML";\n',
    encoding='utf-8',
)
print('Updated GestureLight/WebPage.h')
