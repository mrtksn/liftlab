#!/usr/bin/env python3
"""Fingerprint static CSS/JS references after editing UI assets; no bundler required."""
import argparse
import hashlib,re
from pathlib import Path
root=Path(__file__).resolve().parent.parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--check',action='store_true',help='Fail if references need updating, without writing files')
args=parser.parse_args()
index=root/'index.html'
style=root/'css/style.css'
original_css=style.read_text()
css=original_css
css=re.sub(r'url\("([^"?]+\.css)(?:\?[^"]*)?"\)',lambda m:'url("'+m[1]+'?v='+hashlib.sha256((style.parent/m[1]).read_bytes()).hexdigest()[:10]+'")',css)
original_index=index.read_text()
text=original_index
def reference(match):
    path=match[2]
    target=root/path
    if path=='css/style.css':
        content=b''.join(css.encode() if p==style else p.read_bytes() for p in sorted((root/'css').glob('*.css')))
    else:content=target.read_bytes()
    version=hashlib.sha256(content).hexdigest()[:10]
    return match[1]+path+'?v='+version+match[3]
text=re.sub(r'((?:src|href)=")((?:js|css)/[^"?]+)(?:\?[^\"]*)?(\")',reference,text)
if args.check:
    if css!=original_css or text!=original_index:
        parser.exit(1,'UI asset fingerprints are stale. Run python3 tools/stamp_ui_assets.py\n')
    print('UI asset fingerprints match')
else:
    style.write_text(css)
    index.write_text(text)
    print('UI asset fingerprints updated')
