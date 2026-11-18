#!/usr/bin/env python3
"""Compile the firmware sources with cc21k in several configurations, without CCES and without the board.

Each variant copies the firmware sources at the repo root (*.c, *.h) to a temporary folder, rewrites some #defines
(APP_MODE, ANC_REF_BAND, ANC_DIAG, ...) and compiles Audio_Talkthrough_I2S.c and anc_fxnlms.c with the Release flags
of the SHARC Core1 project (.cproject: -proc ADSP-SC598 -si-revision any -O -DCORE1 -DNDEBUG -DADI_MCAPI, include
folder system/). It only compiles: nothing is linked, so memory placement is not checked.

  python tools/compile_check.py [--cces DIR] [--only NAME ...] [--keep DIR] [--list]

The CrossCore Embedded Studio folder (cc21k.exe) comes from --cces or the CCES_ROOT environment variable. The exit
status is 1 when a variant fails or prints a warning. Run it before every firmware commit; a header-only change still
needs Clean + Build in CCES (see the -absolute-path-dependencies note in the project).
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PROJECT = REPO / 'EV-SOMCRR-EZKIT' / 'SC598' / 'sharc' / 'AD_Talkthrough_I2S_SC598_SHARC_Core1'
SOURCES = ('Audio_Talkthrough_I2S.c', 'anc_fxnlms.c')
FLAGS = ['-proc', 'ADSP-SC598', '-si-revision', 'any', '-O', '-DCORE1', '-DNDEBUG', '-DADI_MCAPI']

# name: {define: value}. The first one is the tree as it is.
VARIANTS = {
    'as-is': {},
    'anc-diag-off': {'ANC_DIAG': '0'},
    'band-none': {'ANC_REF_BAND': 'ANC_REF_BAND_NONE'},
    'band-400-5000': {'ANC_REF_BAND': 'ANC_REF_BAND_400_5000'},
    'prints-5': {'ANC_PRINTS_AFTER_START': '5'},
    'peak-meter': {'ENABLE_SLOT_PEAK_METER': '1'},
    'adc-capture': {'ENABLE_ADC_CAPTURE': '1'},
    'dma-probe': {'ENABLE_DMA_PROBE': '1'},
    'path-measure': {'APP_MODE': 'APP_MODE_PATH_MEASURE'},
    'diag-mic': {'APP_MODE': 'APP_MODE_DIAG_MIC'},
    'diag-line-in': {'APP_MODE': 'APP_MODE_DIAG_LINE_IN'},
    'dac-id': {'APP_MODE': 'APP_MODE_DIAG_DAC_ID'},
    'speaker-test': {'APP_MODE': 'APP_MODE_SPEAKER_TEST'},
}
DIAG_RE = re.compile(r'\b(warning|error)\b', re.I)


def cc21k_path(arg):
    root = arg or os.environ.get('CCES_ROOT')
    if not root:
        sys.exit('Give the CrossCore Embedded Studio folder with --cces or the CCES_ROOT environment variable.')
    exe = Path(root) / 'cc21k.exe'
    if not exe.exists():
        sys.exit(f'Not found: {exe}')
    return exe


def apply(folder, overrides):
    """Rewrite '#define NAME value' in the copied sources; each name must be defined exactly once."""
    files = sorted(folder.glob('*.[ch]'))
    for name, value in overrides.items():
        pat = re.compile(rf'^(#define\s+{re.escape(name)}\s+)(\S+)', re.M)
        hits = [(f, t) for f in files for t in [f.read_text(encoding='utf-8')] if pat.search(t)]
        if len(hits) != 1 or len(pat.findall(hits[0][1])) != 1:
            sys.exit(f'#define {name} is not defined exactly once in the sources')
        f, t = hits[0]
        f.write_text(pat.sub(lambda m: m.group(1) + value, t), encoding='utf-8')


def run(exe, name, overrides, work):
    folder = work / name
    folder.mkdir(parents=True)
    for f in REPO.glob('*.[ch]'):
        shutil.copy2(f, folder / f.name)
    apply(folder, overrides)
    out, ok = [], True
    for src in SOURCES:
        cmd = [str(exe), *FLAGS, '-I', str(PROJECT / 'system'), '-c', str(folder / src),
               '-o', str(folder / (Path(src).stem + '.doj'))]
        p = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=folder)
        text = (p.stdout + p.stderr).strip()
        diags = [l for l in text.splitlines() if DIAG_RE.search(l)]
        if p.returncode != 0 or diags:
            ok = False
            out.append(f'  {src}: exit {p.returncode}\n' + '\n'.join('    ' + l for l in text.splitlines()[:40]))
    tag = ', '.join(f'{k}={v}' for k, v in overrides.items()) or 'sources as they are'
    print(f'{"OK  " if ok else "FAIL"} {name:14s} ({tag})')
    for l in out:
        print(l)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cces', help='CrossCore Embedded Studio folder (default: CCES_ROOT)')
    ap.add_argument('--only', nargs='+', choices=list(VARIANTS), help='compile only these variants')
    ap.add_argument('--keep', help='folder for the variant copies and objects (default: a temporary folder)')
    ap.add_argument('--list', action='store_true', help='list the variants and exit')
    a = ap.parse_args()
    if a.list:
        for k, v in VARIANTS.items():
            print(f'{k:14s} {v}')
        return
    exe = cc21k_path(a.cces)
    names = a.only or list(VARIANTS)
    tmp = None
    if a.keep:
        work = Path(a.keep)
        if work.exists() and any(work.iterdir()):
            sys.exit(f'{work} is not empty')
        work.mkdir(parents=True, exist_ok=True)
    else:
        tmp = tempfile.TemporaryDirectory(prefix='compile_check_')
        work = Path(tmp.name)
    try:
        results = [run(exe, n, VARIANTS[n], work) for n in names]
    finally:
        if tmp:
            tmp.cleanup()
    print(f'{sum(results)} of {len(results)} variants compiled without diagnostics')
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
