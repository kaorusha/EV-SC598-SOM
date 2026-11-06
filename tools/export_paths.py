#!/usr/bin/env python3
"""Write second_path.h / feedback_path.h (S-hat, F-hat for the 48 kHz ANC core) from a pathmeas.py result (.npz).

The ANC core sees the plant through the 192 <-> 48 kHz boundary (anc_rate.h): core y -> interpolator -> J18R
-> DAC, speaker, air, mic, ADC -> decimator -> core Err (S) or core Ref (F). Firmware from 2026-11-05 on runs
APP_MODE_PATH_MEASURE through the same boundary, so its pathmeas result already is the core-domain path.

--legacy-192k converts a result measured with the older firmware (192 kHz per-frame processing, 16-frame DMA
blocks, no rate filters). Its P runs from the J18R TX write to the Err RX read, two DMA blocks (32 frames)
included. In the new firmware a core sample is taken at the last frame of each ANC_DECIM-frame group and its
output starts in the TX block two blocks later, i.e. 2 * B - (ANC_DECIM - 1) frames on with B = COUNT / 4. So

    S_core(f) = H_rate(f)^2 * P_legacy(f) * exp(j 2 pi f (32 - (2 B - ANC_DECIM + 1) - E) / 192000)

E = LEGACY_EXTRA_FRAMES is measured, not modelled, and is 0 now: 1106 S -3 4M away48k fix lags the E = 0 model by
-0.25 frames (early-IR cross-correlation against the converted 1105 S -3 4M away, 0.999), F in three boots by -0.35
to -0.95 frames. That residual is a fraction of a frame and changes from boot to boot (probably the codec phase at
init). The 8.25 frames (43 us) measured on 2026-11-05 came from a buffer-selection bug in SPORTCallback, fixed on
2026-11-06. Measure again if COUNT or the DMA setup changes. A path measured through the boundary needs no
conversion and is always preferred.

The IR band ends at the npz's f2_ir (pathmeas.py): through the boundary the reference path falls off above about
13 kHz, and X above that is noise divided by the falling reference.

Taps are core samples (48 kHz). S-hat tap k is the response at core Err sample n + k to y(n). F-hat tap 0
multiplies y(n-1) (anc_fxnlms.c computes the feedback estimate before y(n)), so F-hat is shifted by one tap.
Taps before PRE_ZERO_MS are zeroed (the PC-side leakage of the measurement and band-limit ringing sit there;
the direct sound arrives after 0.69 ms), and the last END_TAPER_MS fade out.

  python tools/export_paths.py S "measurements/sweep/2026-11-06/1106 S -3 4M away48k fix.npz"
  python tools/export_paths.py F "measurements/sweep/2026-11-05/1105 F -12 2M.npz" --legacy-192k
"""
import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy import fft as sfft
from scipy import signal

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pathmeas as pm

REPO = Path(__file__).resolve().parents[1]
CODEC_FS = 192000
TAPS = 2048
LEGACY_LATENCY_FRAMES = 32     # old firmware: two 16-frame DMA blocks between the TX write and the RX read
LEGACY_EXTRA_FRAMES = 0        # measured lag of the new firmware's outputs re the block k+2 model (see the docstring)
PRE_ZERO_MS = 0.40
PRE_FADE_MS = 0.10
END_TAPER_MS = 5.0
HEADER = {'S': ('second_path.h', 'SECOND_PATH', 'Secondary path S-hat: core y(n) -> core Err'),
          'F': ('feedback_path.h', 'FEEDBACK_PATH', 'Feedback path F-hat: core y(n-1) -> core Ref')}


def rate_filter():
    """ANC_DECIM and the biquad sections from anc_rate.h, the firmware's single source of truth."""
    src = (REPO / 'anc_rate.h').read_text(encoding='utf-8')
    decim = int(re.search(r'#define\s+ANC_DECIM\s+(\d+)', src).group(1))
    rows = re.findall(r'\{\s*([-+0-9.eE]+)f\s*,\s*([-+0-9.eE]+)f\s*,\s*([-+0-9.eE]+)f\s*,\s*([-+0-9.eE]+)f\s*,'
                      r'\s*([-+0-9.eE]+)f\s*\}', src)
    sos = np.array([[float(b0), float(b1), float(b2), 1.0, float(a1), float(a2)] for b0, b1, b2, a1, a2 in rows])
    return decim, sos


def block_frames():
    src = (REPO / 'Audio_Talkthrough_I2S.h').read_text(encoding='utf-8')
    return int(re.search(r'#define\s+COUNT\s+(\d+)', src).group(1)) // 4


def core_ir(z, legacy):
    f, X = z['f'], z['X']
    n = 2 * (f.size - 1)
    f2 = json.loads(str(z['meta'])).get('f2_ir', pm.F2)
    H = X * pm.band_weight(f, pm.F1, f2)
    note = f'measured through the 48 kHz core boundary (no conversion), IR band {pm.F1:g} Hz-{f2 / 1e3:.1f} kHz'
    if legacy:
        decim, sos = rate_filter()
        B = block_frames()
        shift = LEGACY_LATENCY_FRAMES - (2 * B - decim + 1) - LEGACY_EXTRA_FRAMES
        _, hr = signal.sosfreqz(sos, worN=f, fs=CODEC_FS)
        H = H * hr ** 2 * np.exp(2j * np.pi * f * shift / CODEC_FS)
        note = (f'converted from the 192 kHz firmware: x rate filter^2 (anc_rate.h), advanced {shift:g} frames '
                f'({shift / CODEC_FS * 1e6:.1f} us) for {B}-frame DMA blocks')
        if LEGACY_EXTRA_FRAMES:
            note += f', including the measured {LEGACY_EXTRA_FRAMES:g}-frame output lag'
    return sfft.irfft(H, n), note


def shape(ir, fs, path):
    h = ir[1:TAPS + 1].copy() if path == 'F' else ir[:TAPS].copy()
    t = (np.arange(TAPS) + (1 if path == 'F' else 0)) / fs * 1e3
    w = np.clip((t - PRE_ZERO_MS) / PRE_FADE_MS, 0, 1)
    w = 0.5 - 0.5 * np.cos(np.pi * w)
    k = int(END_TAPER_MS * 1e-3 * fs)
    w[TAPS - k:] *= 0.5 + 0.5 * np.cos(np.pi * np.arange(1, k + 1) / k)
    return (h * w).astype(np.float32) + np.float32(0)    # + 0 turns the window's -0.0 into 0.0


def write_header(path, h, source, meta, note, stats):
    fname, sym, title = HEADER[path]
    lines = [
        '/******************************************************************************',
        f' * @file {fname}',
        f' * @brief {title} (48 kHz ANC core, generated by tools/export_paths.py)',
        ' *',
        f' * Source:            {source}',
        f" * Runs:              {', '.join(meta['runs'])}",
        f" * Measured:          {meta['started']}, {meta['level_dbfs']:g} dBFS drive, {meta['length']} sweeps",
        f' * Conversion:        {note}',
        f" * Direct sound:      onset (-20 dB) tap {stats['onset']:.1f}, peak tap {stats['peak']:.1f}",
        f" * Energy kept:       {stats['kept']:.2f}% of the 0-683 ms response",
        f' * Shaping:           zero before {PRE_ZERO_MS:g} ms ({PRE_FADE_MS:g} ms fade-in), {END_TAPER_MS:g} ms fade-out',
        f' * Filter Taps:       {TAPS}',
        f' * Memory Footprint:  {TAPS * 4} bytes ({TAPS * 4 / 1024:.2f} KB)',
        ' ******************************************************************************/',
        '',
        f'#ifndef __{sym}_H__',
        f'#define __{sym}_H__',
        '',
        '#include <stdint.h>',
        '',
        '/* Number of filter taps */',
        f'#define {sym}_TAPS ({TAPS})',
        '',
        '/* Sampling rate in Hz (the ANC core rate) */',
        f'#define {sym}_TAPS_FS ((uint32_t)48000)',
        '',
        '/* FIR filter coefficients (IEEE 754 32-bit float) */',
        f'static const float {sym}[{sym}_TAPS] = {{',
    ]
    for i in range(0, TAPS, 8):
        lines.append('    ' + ', '.join(f'{v:+.8e}f' for v in h[i:i + 8]) + (',' if i + 8 < TAPS else ''))
    lines += ['};', '', f'#endif /* __{sym}_H__ */', '']
    (REPO / fname).write_text('\n'.join(lines), encoding='utf-8')


def main():
    ap = argparse.ArgumentParser(description='Write S-hat / F-hat headers for the 48 kHz ANC core from a pathmeas .npz.')
    ap.add_argument('path', choices=('S', 'F'))
    ap.add_argument('npz', help='"<MMDD> <path> <level> <length> [tag].npz" from pathmeas.py')
    ap.add_argument('--legacy-192k', action='store_true', help='the .npz comes from the pre-2026-11-05 192 kHz firmware')
    a = ap.parse_args()
    src = Path(a.npz)
    z = np.load(src)
    meta = json.loads(str(z['meta']))
    if meta['path'] != a.path:
        sys.exit(f"{src.name} is a {meta['path']} measurement, not {a.path}")
    ir, note = core_ir(z, a.legacy_192k)
    fs = meta['fs']
    e = ir[:int(0.683 * fs)] ** 2
    env = np.abs(signal.hilbert(ir[:TAPS + 1]))
    peak = int(np.argmax(env))
    stats = dict(peak=peak - (1 if a.path == 'F' else 0),
                 onset=int(np.argmax(env >= 0.1 * env[peak])) - (1 if a.path == 'F' else 0),
                 kept=float(np.sum(e[:TAPS + 1]) / np.sum(e) * 100))
    h = shape(ir, fs, a.path)
    meta['runs'] = [str(r) for r in z['runs']]
    meta['started'] = meta.get('started', '?')
    write_header(a.path, h, src.as_posix(), meta, note, stats)
    print(f"{HEADER[a.path][0]}: {note}; onset tap {stats['onset']} ({stats['onset'] / fs * 1e3:.3f} ms), peak tap "
          f"{stats['peak']} ({stats['peak'] / fs * 1e3:.3f} ms), energy kept {stats['kept']:.2f}%")


if __name__ == '__main__':
    main()
