#!/usr/bin/env python3
"""Compare a new S or F measurement (pathmeas.py .npz) with an older one and with the header the firmware uses.

  python tools/compare_paths.py S <new npz> <old npz> [--header-rev REV | --header FILE] [--band 400 1500]
                                [--f0 600] [--freqs 447 ...] [--out PNG]

Prints, at the fan tones (f0, 2 f0, 5 f0, 6 f0, plus --freqs) and over the band, |new/old| and the phase of new/old,
and the same against the header, which is the S-hat/F-hat the firmware applies (default: second_path.h or
feedback_path.h in the repo now; --header-rev reads it from a git revision, e.g. the one a test ran with).
For S, FxNLMS pushes the error up wherever the header's phase is more than 90 deg off the true path: the report
lists those stretches of the band. F-hat tap 0 multiplies y(n-1) (export_paths.py), so the F header gets one core
sample of delay before the comparison.

Responses are smoothed over 1/12 octave (complex, phase kept). The run-to-run spread of the new measurement shows
what is noise. The figure goes next to the new npz unless --out is given: "<new stem> vs <old stem>.png".

Example (2026-11-17, the chassis moved from the desk to the lab):
  python tools/compare_paths.py S "measurements/sweep/2026-11-17/1117 S -3 4M lab.npz"
      "measurements/sweep/2026-11-06/1106 S -3 4M away48k fix.npz" --header-rev da8e2d9 --freqs 447
  gives -155 deg against the header at 447 Hz, where the first lab ANC test diverged, and over 90 deg in
  434-524 Hz and 894-922 Hz.
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
HEADER = {'S': 'second_path.h', 'F': 'feedback_path.h'}
CORE_FS = 48000
INK, OLD, NEW, HDR, SPREAD = '#222222', '#7a7a7a', '#2a6fdb', '#d9822b', '#9db8e8'


def load(p):
    z = np.load(p, allow_pickle=True)
    return z['f'], z['X'], z['X_runs'], json.loads(str(z['meta']))


def header_taps(text):
    """Coefficients of the C array in an export_paths.py header."""
    body = text[text.index('{', text.index('[')):text.rindex('}')]
    num = r'[-+]?\d*\.\d+(?:[eE][-+]?\d+)?[fF]?|[-+]?\d+[eE][-+]?\d+[fF]?'
    return np.array([float(v.rstrip('fF')) for v in re.findall(num, body)])


def header_text(path, file=None, rev=None):
    if file:
        return Path(file).read_text(encoding='utf-8'), str(file)
    name = HEADER[path]
    if rev:
        out = subprocess.run(['git', '-C', str(REPO), 'show', f'{rev}:{name}'], capture_output=True, text=True,
                             encoding='utf-8', check=True).stdout
        return out, f'{name} at {rev}'
    return (REPO / name).read_text(encoding='utf-8'), f'{name} (working tree)'


def response(taps, f, path):
    n = np.arange(taps.size)
    H = np.exp(-2j * np.pi * np.outer(f, n) / CORE_FS) @ taps
    if path == 'F':      # tap 0 multiplies y(n-1)
        H = H * np.exp(-2j * np.pi * f / CORE_FS)
    return H


def smooth_c(f, X, frac=12):
    """1/frac-octave complex moving average (keeps the phase)."""
    c = np.concatenate([[0], np.cumsum(X)])
    il = np.searchsorted(f, f * 2 ** (-0.5 / frac))
    ih = np.maximum(np.searchsorted(f, f * 2 ** (0.5 / frac), side='right'), il + 1)
    return (c[ih] - c[il]) / (ih - il)


def stretches(fv, mask, gap=2.0):
    sel = fv[mask]
    if not sel.size:
        return []
    cut = np.flatnonzero(np.diff(sel) > gap) + 1
    return [(g[0], g[-1]) for g in np.split(sel, cut)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', choices=('S', 'F'))
    ap.add_argument('new')
    ap.add_argument('old')
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--header', help='header file to compare against')
    g.add_argument('--header-rev', help='git revision whose header to compare against')
    ap.add_argument('--band', type=float, nargs=2, default=(400.0, 1500.0), help='ANC reference band, Hz')
    ap.add_argument('--f0', type=float, default=600.0, help='fan f0, Hz (tones f0, 2 f0, 5 f0, 6 f0)')
    ap.add_argument('--freqs', type=float, nargs='*', default=[], help='more frequencies to report, Hz')
    ap.add_argument('--out', help='figure path')
    a = ap.parse_args()

    f, Xn, Xn_runs, _ = load(a.new)
    fo, Xo, _, _ = load(a.old)
    if not np.allclose(f, fo):
        sys.exit('The two measurements have different frequency grids.')
    text, hdr_name = header_text(a.path, a.header, a.header_rev)
    taps = header_taps(text)
    m = (f >= 60) & (f <= 12000)
    fm = f[m]
    sn, so, sh = smooth_c(fm, Xn[m]), smooth_c(fm, Xo[m]), smooth_c(fm, response(taps, fm, a.path))
    r_old, r_hdr = sn / so, sn / sh
    dmag, dph = 20 * np.log10(np.abs(r_old)), np.degrees(np.angle(r_old))
    hmag, hph = 20 * np.log10(np.abs(r_hdr)), np.degrees(np.angle(r_hdr))
    spread = None
    if Xn_runs.shape[0] > 1:
        spread = np.max(np.abs(20 * np.log10(np.abs(np.array([smooth_c(fm, r[m]) for r in Xn_runs]) / sn))), axis=0)

    tones = [(k * a.f0, f'{k} f0' if k > 1 else 'f0') for k in (1, 2, 5, 6)] + [(v, f'{v:g} Hz') for v in a.freqs]
    L = [f'{a.path}: new {Path(a.new).name} vs old {Path(a.old).name}; header {hdr_name} ({taps.size} taps)',
         '1/12-octave smoothed; "header" = new / the S-hat or F-hat the firmware applies.',
         '   frequency            |new/old| dB  phase deg   |new/hdr| dB  phase deg']
    for ft, lab in sorted(tones):
        i = np.argmin(np.abs(fm - ft))
        L.append(f'   {lab:8s} {fm[i]:8.1f} Hz {dmag[i]:+8.1f}   {dph[i]:+8.0f}     {hmag[i]:+8.1f}   {hph[i]:+8.0f}')
    b = (fm >= a.band[0]) & (fm <= a.band[1])
    over = b & (np.abs(hph) > 90)
    L.append(f'In {a.band[0]:g}-{a.band[1]:g} Hz: |phase new/hdr| at most {np.max(np.abs(hph[b])):.0f} deg '
             f'({fm[b][np.argmax(np.abs(hph[b]))]:.0f} Hz); over 90 deg: '
             + (', '.join(f'{lo:.0f}-{hi:.0f} Hz' for lo, hi in stretches(fm, over)) or 'none'))
    L.append(f'In {a.band[0]:g}-{a.band[1]:g} Hz: |new/old| from {dmag[b].min():+.1f} to {dmag[b].max():+.1f} dB, '
             f'phase new/old from {dph[b].min():+.0f} to {dph[b].max():+.0f} deg')
    if spread is not None:
        L.append(f'Run-to-run spread of the new measurement in the band: median {np.median(spread[b]):.2f} dB')
    print('\n'.join(L))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 11, 'axes.edgecolor': '#999999', 'axes.titlelocation': 'left'})
    fig, axs = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    axs[0].plot(fm, 20 * np.log10(np.abs(so)), lw=1.0, color=OLD, label=f'old: {Path(a.old).stem}')
    axs[0].plot(fm, 20 * np.log10(np.abs(sh)), lw=0.9, color=HDR, ls='--', label=f'header: {hdr_name}')
    axs[0].plot(fm, 20 * np.log10(np.abs(sn)), lw=1.3, color=NEW, label=f'new: {Path(a.new).stem}')
    axs[0].set_ylabel(f'|{a.path}|, dB')
    axs[0].set_title(f'{a.path}: old, header and new (1/12 octave)')
    axs[0].legend(frameon=False, loc='lower left')
    axs[1].plot(fm, dmag, lw=1.2, color=NEW, label='|new / old|')
    axs[1].plot(fm, hmag, lw=0.9, color=HDR, label='|new / header|')
    if spread is not None:
        axs[1].fill_between(fm, -spread, spread, color=SPREAD, alpha=0.35, lw=0, label='run-to-run spread (new)')
    axs[1].axhline(0, color='#999999', lw=0.8)
    axs[1].set_ylabel('dB')
    axs[1].set_ylim(max(-20, min(dmag.min(), hmag.min()) - 2), min(20, max(dmag.max(), hmag.max()) + 2))
    axs[1].legend(frameon=False, loc='lower left')
    axs[2].plot(fm, dph, lw=1.0, color=OLD, label='phase new / old')
    axs[2].plot(fm, hph, lw=1.3, color=NEW, label='phase new / header')
    for y in (-90, 90):
        axs[2].axhline(y, color='#c0392b', lw=0.8, ls='--')
    axs[2].text(fm[0], 92, ' +-90 deg: FxNLMS diverges beyond', color='#c0392b', fontsize=9, va='bottom')
    axs[2].set_ylim(-185, 185)
    axs[2].set_ylabel('deg')
    axs[2].set_xlabel('Hz')
    axs[2].legend(frameon=False, loc='lower left')
    for ax in axs:
        ax.set_xscale('log')
        ax.axvspan(*a.band, color=NEW, alpha=0.05, lw=0)
        ax.grid(True, which='both', lw=0.4, color='#e6e6e6')
        for ft, _ in tones:
            ax.axvline(ft, color='#bbbbbb', lw=0.6, ls=':')
    for ft, lab in tones:
        axs[2].text(ft, 1.0, lab, transform=axs[2].get_xaxis_transform(), fontsize=8, rotation=90, va='top',
                    ha='right', color='#666666')
    fig.tight_layout()
    out = a.out or str(Path(a.new).with_name(f'{Path(a.new).stem} vs {Path(a.old).stem}.png'))
    fig.savefig(out, dpi=90)
    print('saved', out)


if __name__ == '__main__':
    main()
