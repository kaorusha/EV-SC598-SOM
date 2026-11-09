#!/usr/bin/env python3
"""Secondary / feedback path measurement with an embedded electrical timing reference.

Firmware: APP_MODE_PATH_MEASURE (Audio_Talkthrough_I2S.c)
  PC Left  -> J16 Left  -> x CALIBRATION_CHIRP_GAIN -> added to J17 Left and J17 Right
  PC Right -> J16 Right -> J18 Right -> speaker
  J17 Left  = Error mic + reference -> secondary path S
  J17 Right = Ref mic   + reference -> feedback path F
The US205 mic input is mono: plug in J17 Left for S or J17 Right for F, and pass --path to match.

A run plays two log sweeps on PC Left, D apart (references 1 and 2), and the same sweep on PC Right
D after reference 2 (the speaker). After deconvolution reference 1 sits at the (random) stream
latency p, reference 2 at p + D and the path at p + 2D. With equal windows,

    P(f) = CHIRP_GAIN * (A_L / A_R) * FFT(path window) / FFT(reference 2 window) * exp(j 2 pi f dJ(f))

is the path as the firmware sees it: digital J16 Right sample (the ANC output y) in, digital Err or
Ref ADC sample out. Stream latency, the US205 converters, the J17 output and the mic input gain
cancel. What does not cancel is any mismatch between the PC Left -> J16 Left and PC Right -> J16
Right input chains.

Since 2026-11-05 the firmware runs this bridge through the ANC core's 192 <-> 48 kHz boundary (anc_rate.h),
so P runs from a 48 kHz core output sample to a 48 kHz core Err / Ref sample, rate filters and DMA timing
included; tools/export_paths.py writes it to the S-hat / F-hat headers. Results from older firmware are
192 kHz-domain (export_paths.py --legacy-192k converts them).

dJ(f) undoes the US205's playback-to-capture timing wander: tens of us within a run, in plateaus
joined by ~0.3 s ramps (measured 2026-11-04; it is also what moved REW's time zero). The sweep
reaches k*f (k = exp(D / L)) exactly D after it passes f, so at k*f the two references were recorded
exactly when reference 2 and the path passed f, and their ratio there is the timing change alone.

Run with the anc_project conda env, from any directory:
  python tools/pathmeas.py devices      list WASAPI devices, check the US205 formats and Windows volumes (silent)
  python tools/pathmeas.py selftest     simulate the chain and check the analysis (silent)
  python tools/pathmeas.py measure --path S --levels -12 --repeats 2 --notes "knob mark 3"   (plays sweeps)
  python tools/pathmeas.py analyze [folder]   analyse a day folder again (default: today's)

Data go to measurements/sweep/YYYY-MM-DD/ (gitignored):
  "<MMDD> <path> <level> <length> rN".wav/.json   raw mono float32 recording, settings, notes, Windows volumes,
                                                  firmware source commit and xruns
  "<MMDD> <path> <level> <length>".npz            complex response of each run, their mean, noise floor
  "<MMDD> <path> <level> <length> IR.wav"         impulse response of the mean, for REW's Import impulse response
  "<MMDD> <path> <length>.png", "<MMDD> SF <length>.png", report.txt
"""
import argparse
import ctypes
import datetime
import json
import os
import re
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np
from scipy import fft as sfft
from scipy import ndimage
from scipy import signal
from scipy.io import wavfile

REPO = Path(__file__).resolve().parents[1]
FIRMWARE_C = REPO / 'Audio_Talkthrough_I2S.c'
DATA_ROOT = REPO / 'measurements' / 'sweep'

DEVICE = 'USB Audio Device'   # UGREEN US205 in the WASAPI device list
JACK = {'S': 'J17 Left (Error mic)', 'F': 'J17 Right (Ref mic)'}   # in the US205 mic input for each path
FS = 48000
F1, F2 = 40.0, 20000.0        # sweep range (Hz). S/F carry nothing below ~60 Hz, and starting at 40 Hz spares
                              # the speaker the low-frequency excursion suspected behind the audible doubling
LEAD_S = 0.5                  # silence before the reference sweep
DELAY_S = 0.8                 # D between reference 1, reference 2 and the speaker sweep (> WIN)
N_REFS = 2                    # reference sweeps on PC Left; two make the timing-wander correction possible
TAIL_S = 1.5                  # silence after the speaker sweep
PREROLL_S = 0.3               # recording starts this long before playback
MARGIN_S = 1.0                # and runs this much longer than the playback buffer
LATENCY_S = 0.1               # WASAPI buffer
WIN = 32768                   # analysis window (683 ms at 48 kHz), same for reference, path and noise
PRE_S = 0.15                  # windows open this long before the arrival: the zero-phase band limit of the
FADE_S = 0.10                 # deconvolution rings ~100 ms before it, so the window fades in over FADE_S
TAPER = 0.5                   # half-Hann fade over the last half of each window
NOISE_GAP_S = 0.3             # noise window opens this long after the path window ends
TONES = (189, 300, 602, 1131, 1800, 3614)                # fan tones (Hz)
OCTAVES = (63, 125, 250, 500, 1000, 2000, 4000, 8000)
DELAY_BAND = (200.0, 4000.0)  # band for the run-to-run time-zero estimate
Y_MAX = 0.2                   # ANC output limit |y|, for the "Err reach" row

# Chart colours on the light surface; S and F keep their colour in every figure
SURFACE, INK, INK2, MUTED, GRID, AXIS = '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7'
PATH_COLOR = {'S': '#2a78d6', 'F': '#eb6834'}
OCT_HEAD = ''.join(f'{(f"{fc // 1000}k" if fc >= 1000 else str(fc)):>7}' for fc in OCTAVES)


def db(x):
    return 20 * np.log10(np.maximum(np.abs(x), 1e-30))


def db2amp(d):
    return 10.0 ** (np.asarray(d, dtype=float) / 20)


def parse_length(s):
    unit = {'k': 1 << 10, 'K': 1 << 10, 'M': 1 << 20}.get(s[-1])
    return int(float(s[:-1]) * unit) if unit else int(s)


def length_tag(n):
    if n % (1 << 20) == 0:
        return f'{n >> 20}M'
    return f'{n >> 10}k' if n % (1 << 10) == 0 else str(n)


def firmware_settings():
    """CALIBRATION_CHIRP_GAIN and APP_MODE from the firmware source (the board runs them if it is up to date)."""
    try:
        src = FIRMWARE_C.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return 0.05, None
    gain = re.search(r'#define\s+CALIBRATION_CHIRP_GAIN\s+([0-9.eE+-]+)', src)
    mode = re.search(r'#define\s+APP_MODE\s+(\w+)', src)
    return (float(gain.group(1)) if gain else 0.05), (mode.group(1) if mode else None)


def git_state(paths=()):
    """Short HEAD hash and the tracked files with uncommitted changes (under paths, if given); None without git."""
    def git(*args):
        return subprocess.run(('git', '-C', str(REPO)) + args, capture_output=True, text=True, check=True).stdout
    try:
        head = git('rev-parse', '--short', 'HEAD').strip()
        changed = git('status', '--porcelain', '--untracked-files=no', '--', *paths).splitlines()
    except (OSError, subprocess.CalledProcessError):
        return None
    return dict(commit=head, modified=[line[3:] for line in changed])


def source_text(state):
    """'d1ae254 + uncommitted Audio_Talkthrough_I2S.c, APP_MODE_PATH_MEASURE' for a git_state() result (a run's
    'firmware' record adds app_mode)."""
    state = state or {}
    text = state.get('commit') or 'no git checkout'
    if state.get('modified'):
        text += ' + uncommitted ' + ', '.join(state['modified'])
    return text + (f", {state['app_mode']}" if state.get('app_mode') else '')


def import_sounddevice():
    """sounddevice finds conda's portaudio.dll through PATH, which only an activated env sets."""
    lib = Path(sys.prefix) / 'Library' / 'bin'
    if lib.is_dir() and str(lib) not in os.environ.get('PATH', ''):
        os.environ['PATH'] = str(lib) + os.pathsep + os.environ.get('PATH', '')
    import sounddevice
    return sounddevice


# ---------------------------------------------------------------- stimulus and recording

def make_sweep(fs, n, f1, f2):
    """Log sweep f1 -> f2 in n samples, 1/3-octave fade-in, 1/24-octave fade-out."""
    L = n / fs / np.log(f2 / f1)
    t = np.arange(n) / fs
    x = np.sin(2 * np.pi * f1 * L * np.expm1(t / L))
    n_in, n_out = int(L * np.log(2) / 3 * fs), int(L * np.log(2) / 24 * fs)
    x[:n_in] *= 0.5 - 0.5 * np.cos(np.pi * np.arange(n_in) / n_in)
    x[n - n_out:] *= 0.5 + 0.5 * np.cos(np.pi * np.arange(1, n_out + 1) / n_out)
    return x


def make_play(meta):
    """Stereo playback buffer: n_refs reference sweeps on Left, n_delay samples apart, then the same sweep on
    Right n_delay after the last reference. Runs from before 2026-11-04 evening have one reference."""
    s = make_sweep(meta['fs'], meta['n_sweep'], meta['f1'], meta['f2'])
    a, d, n_refs = meta['n_lead'], meta['n_delay'], meta.get('n_refs', 1)
    play = np.zeros((a + n_refs * d + s.size + meta['n_tail'], 2), np.float32)
    for i in range(n_refs):
        play[a + i * d:a + i * d + s.size, 0] += db2amp(meta['ref_dbfs']) * s
    play[a + n_refs * d:a + n_refs * d + s.size, 1] = db2amp(meta['level_dbfs']) * s
    return play


def wasapi_devices(sd, name):
    api = next(i for i, h in enumerate(sd.query_hostapis()) if 'WASAPI' in h['name'])
    devs = [d for d in sd.query_devices() if d['hostapi'] == api and name.lower() in d['name'].lower()]
    ins = [d for d in devs if d['max_input_channels'] > 0]
    outs = [d for d in devs if d['max_output_channels'] > 0]
    if len(ins) != 1 or len(outs) != 1:
        sys.exit(f'Need one WASAPI input and one output whose name contains "{name}", found '
                 f'{[d["name"] for d in devs]}. Run "pathmeas.py devices" and pass --device.')
    return ins[0], outs[0]


def record(sd, play, fs, dev_in, dev_out, exclusive):
    """Play the stereo buffer and record the mono mic input. Two streams on one device share its clock;
    their start offset does not matter because the reference rides in the same recording."""
    n_play = len(play)
    n_rec = n_play + int((PREROLL_S + MARGIN_S) * fs)
    rec = np.zeros(n_rec, np.float32)
    pos = {'in': 0, 'out': 0}
    flags = []
    done_in, done_out = threading.Event(), threading.Event()

    def cb_in(indata, frames, t, status):
        i = pos['in']
        if status:
            flags.append({'stream': 'in', 'frame': i, 'status': str(status)})
        k = min(frames, n_rec - i)
        rec[i:i + k] = indata[:k, 0]
        pos['in'] = i + k
        if pos['in'] >= n_rec:
            raise sd.CallbackStop

    def cb_out(outdata, frames, t, status):
        i = pos['out']
        if status:
            flags.append({'stream': 'out', 'frame': i, 'status': str(status)})
        k = min(frames, n_play - i)
        outdata[:k] = play[i:i + k]
        outdata[k:] = 0
        pos['out'] = i + k
        if pos['out'] >= n_play:
            raise sd.CallbackStop

    kw = dict(samplerate=fs, dtype='float32', latency=LATENCY_S,
              extra_settings=sd.WasapiSettings(exclusive=exclusive))
    with sd.InputStream(device=dev_in['index'], channels=1, callback=cb_in,
                        finished_callback=done_in.set, **kw):
        time.sleep(PREROLL_S)
        with sd.OutputStream(device=dev_out['index'], channels=2, callback=cb_out,
                             finished_callback=done_out.set, **kw):
            if not done_out.wait(n_play / fs + 10):
                raise RuntimeError('playback did not finish')
        if not done_in.wait(PREROLL_S + MARGIN_S + 10):
            raise RuntimeError('recording did not finish')
    return rec, flags


def bad_xruns(meta):
    """xruns that can corrupt a sweep: any input flag after the first buffer, output flags while a sweep plays."""
    end = meta['n_lead'] + meta.get('n_refs', 1) * meta['n_delay'] + meta['n_sweep']
    return [x for x in meta.get('xruns', [])
            if (x['stream'] == 'in' and x['frame'] > 0)
            or (x['stream'] == 'out' and meta['n_lead'] // 2 <= x['frame'] < end)]


# ---------------------------------------------------------------- Windows volume

class PropertyKey(ctypes.Structure):
    _fields_ = [('fmtid', ctypes.c_ubyte * 16), ('pid', ctypes.c_uint)]


class PropVariant(ctypes.Structure):
    _fields_ = [('vt', ctypes.c_ushort), ('reserved', ctypes.c_ushort * 3), ('ptr', ctypes.c_void_p),
                ('pad', ctypes.c_void_p)]


def guid(s):
    return (ctypes.c_ubyte * 16).from_buffer_copy(uuid.UUID(s).bytes_le)


def com(p, i, *argtypes):
    """Method i (vtable order; 0-2 are IUnknown's) of the COM interface pointer p. A failed HRESULT raises OSError."""
    vtable = ctypes.cast(p, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p)))[0]
    fn = ctypes.WINFUNCTYPE(ctypes.HRESULT, ctypes.c_void_p, *argtypes)(vtable[i])
    return lambda *args: fn(p, *args)


def windows_volume(name, capture):
    """Windows volume of the active endpoint whose friendly name is name (the WASAPI device name), from Core Audio's
    IAudioEndpointVolume through ctypes, so no extra package: master scalar and dB, dB per channel, mute, and whether
    the device applies the volume itself. If it does not, Windows applies it in software, which WASAPI exclusive
    mode bypasses. Returns {'error': ...} instead of raising."""
    ole32 = ctypes.windll.ole32
    init = ole32.CoInitialize(None)      # S_FALSE (1): PortAudio has already initialised COM on this thread
    held = []

    def ptr():
        held.append(ctypes.c_void_p())
        return held[-1]

    def get(p, i, ctype, *args):
        """Method i with UINT args and one output argument; returns the output."""
        v = ctype()
        com(p, i, *([ctypes.c_uint] * len(args)), ctypes.c_void_p)(*args, ctypes.byref(v))
        return v.value

    try:
        enum, devs = ptr(), ptr()
        ctypes.oledll.ole32.CoCreateInstance(guid('bcde0395-e52f-467c-8e3d-c4579291692e'), None, 1,  # MMDeviceEnumerator
                                             guid('a95664d2-9614-4f35-a746-de8db63617e6'), ctypes.byref(enum))
        com(enum, 3, ctypes.c_int, ctypes.c_uint, ctypes.c_void_p)(int(capture), 1, ctypes.byref(devs))  # active ones
        key = PropertyKey(guid('a45c254e-df1c-4efd-8020-67d146a850e0'), 14)                            # friendly name
        for i in range(get(devs, 3, ctypes.c_uint)):
            dev, store, vol = ptr(), ptr(), ptr()
            com(devs, 4, ctypes.c_uint, ctypes.c_void_p)(i, ctypes.byref(dev))
            com(dev, 4, ctypes.c_uint, ctypes.c_void_p)(0, ctypes.byref(store))                       # read only
            pv = PropVariant()
            com(store, 5, ctypes.c_void_p, ctypes.c_void_p)(ctypes.byref(key), ctypes.byref(pv))
            friendly = ctypes.wstring_at(pv.ptr) if pv.vt == 31 else None                                # VT_LPWSTR
            ole32.PropVariantClear(ctypes.byref(pv))
            if friendly != name:
                continue
            com(dev, 3, ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p)(
                guid('5cdf2c82-841e-4546-9722-0cf74078229a'), 23, None, ctypes.byref(vol))               # IAudioEndpointVolume
            return dict(scalar=round(get(vol, 9, ctypes.c_float), 4), db=round(get(vol, 8, ctypes.c_float), 3),
                        channels_db=[round(get(vol, 12, ctypes.c_float, c), 3) for c in range(get(vol, 5, ctypes.c_uint))],
                        mute=bool(get(vol, 15, ctypes.c_int)), hw_volume=bool(get(vol, 19, ctypes.c_uint) & 1))
        return {'error': f'no active endpoint named {name}'}
    except OSError as e:
        return {'error': str(e)}
    finally:
        for p in reversed(held):
            if p.value:
                com(p, 2)()                                                                              # Release
        if init in (0, 1):
            ole32.CoUninitialize()


def read_volumes(dev_out, dev_in):
    return {'out': windows_volume(dev_out['name'], capture=False), 'in': windows_volume(dev_in['name'], capture=True)}


def volume_text(v):
    """'out 1.000 = +0.00 dB, in 0.500 = +1.89 dB' for a read_volumes() result; channels only when they differ."""
    parts = []
    for key, e in v.items():
        if 'error' in e:
            parts.append(f"{key} unreadable ({e['error']})")
            continue
        ch = e['channels_db']
        parts.append(f"{key} {e['scalar']:.3f} = {e['db']:+.2f} dB"
                     + (f" (channels {' / '.join(f'{c:+.2f}' for c in ch)} dB)" if len(set(ch)) > 1 else '')
                     + (' muted' if e['mute'] else '') + ('' if e['hw_volume'] else ' in software'))
    return ', '.join(parts)


def volume_warnings(v):
    w = [f"Windows {'output' if key == 'out' else 'input'} is muted" for key, e in v.items() if e.get('mute')]
    ch = v['out'].get('channels_db', [])
    if len(ch) == 2 and ch[0] != ch[1]:
        w.append(f'Windows output balance: PC Right is {ch[1] - ch[0]:+.2f} dB re PC Left, which adds '
                 f'{ch[1] - ch[0]:+.2f} dB to P')
    return w


# ---------------------------------------------------------------- analysis

def band_weight(f, f1, f2):
    """1 from 2*f1 to f2*2^(-1/6), raised cosine (in log f) down to 0 at f1 and at f2. It cancels in the
    path / reference ratio as long as both windows hold all of its ringing. A causal high-pass cannot
    replace it: below f1 the sweep spectrum is 50-100 dB down, so the cut must be steep."""
    lf = np.log2(np.maximum(f, 1e-3))
    lo = np.clip(lf - np.log2(f1), 0, 1)
    hi = np.clip((np.log2(f2) - lf) * 6, 0, 1)
    return (0.5 - 0.5 * np.cos(np.pi * lo)) * (0.5 - 0.5 * np.cos(np.pi * hi))


def fade_window(n, fs):
    """Half-Hann fade-in over FADE_S, flat, half-Hann fade-out over the last TAPER of the window."""
    w = np.ones(n)
    j, k = int(FADE_S * fs), int(n * TAPER)
    w[:j] = 0.5 - 0.5 * np.cos(np.pi * np.arange(j) / j)
    w[n - k:] = 0.5 + 0.5 * np.cos(np.pi * np.arange(1, k + 1) / k)
    return w


def deconvolve(rec, meta):
    """Impulse response of the recording against the unit sweep, band-limited to the sweep range.
    Harmonic distortion lands at negative times, which wrap to the end of the buffer."""
    s = make_sweep(meta['fs'], meta['n_sweep'], meta['f1'], meta['f2'])
    n = sfft.next_fast_len(rec.size, real=True)
    H = sfft.rfft(rec.astype(np.float64), n)
    B = band_weight(sfft.rfftfreq(n, 1 / meta['fs']), meta['f1'], meta['f2'])
    m = B > 0
    H[m] *= B[m] / sfft.rfft(s, n)[m]
    H[~m] = 0
    return sfft.irfft(H, n)


def locate_ref(env, fs, n_d, n_refs=1):
    """Index of the first electrical reference. Reference IR peaks are 30-45 dB above the S and F peaks
    (REW 11/4 exports). With two references, take the spot where both it and the one D later are strong and
    nothing comparable sits D earlier (that rules out reference 2, and a path stronger than the references);
    the max filter allows for wander moving reference 2 by a sample or two. With one reference, the global
    maximum is normally the reference; if the path is the maximum, the reference is D earlier."""
    if n_refs == 2:
        em = ndimage.maximum_filter1d(env, 7)
        pair = np.minimum(em[:-n_d], em[n_d:])
        before = np.concatenate((np.zeros(n_d), em[:-2 * n_d]))
        m = int(np.argmax(np.where(before < 0.25 * pair, pair, 0)))
        return m - 3 + int(np.argmax(env[m - 3:m + 4]))
    m = int(np.argmax(env))
    w, g = int(0.04 * fs), int(0.002 * fs)
    lo = max(m - n_d - w, 0)
    before = env[lo:max(m - n_d + g, 0)]
    after = env[m + n_d - g:m + n_d + w]
    if before.size and before.max() > (after.max() if after.size else 0):
        m = lo + int(np.argmax(before))
    return m


def wander_phase(f, r1, r2, valid, meta):
    """Phase 2 pi f dJ(f) that undoes the timing change dJ between reference 2 and the path at each bin, from
    the two reference spectra at k*f (see the module docstring). Returns the phase and the bins it covers
    (k*f must stay inside the valid band)."""
    k = (meta['f2'] / meta['f1']) ** (meta['n_delay'] / meta['n_sweep'])
    idx = np.flatnonzero(valid)
    fv = f[idx]
    q = r2[idx] / r1[idx]
    ph = np.unwrap(np.angle(q))
    a = int(np.argmin(np.abs(fv - 1000)))      # timing changes stay far below 500 us, so at 1 kHz the
    ph -= 2 * np.pi * np.round((ph[a] - np.angle(q[a])) / (2 * np.pi))   # phase is on the principal branch
    ok = fv * k <= fv[-1]
    theta = np.zeros(f.size)
    covered = np.zeros(f.size, bool)
    theta[idx[ok]] = -np.interp(fv[ok] * k, fv, ph) / k
    covered[idx[ok]] = True
    return theta, covered


def analyse_run(rec, meta, correct_wander=True):
    """Deconvolve one recording; return the path response P(f), its noise floor and diagnostics.
    correct_wander=False skips the timing-wander correction (self-test comparison only)."""
    fs, n_d = meta['fs'], meta['n_delay']
    n_refs = meta.get('n_refs', 1)
    gain = meta.get('chirp_gain', 0.05)
    h = deconvolve(rec, meta)
    env = np.abs(h)
    p = locate_ref(env, fs, n_d, n_refs)
    pre, gap = int(PRE_S * fs), int(NOISE_GAP_S * fs)
    starts = {f'ref{i + 1}': p + i * n_d - pre for i in range(n_refs)}
    starts['path'] = p + n_refs * n_d - pre
    starts['noise'] = starts['path'] + WIN + gap
    if starts['ref1'] < int(0.05 * fs) or p + n_refs * n_d + meta['n_sweep'] + WIN // 2 > rec.size:
        raise ValueError(f'reference found at {p / fs:.3f} s: the path response does not fit '
                         f'in the {rec.size / fs:.1f} s recording')
    warn = []
    T = meta['n_sweep'] / fs
    for k in range(2, 6):
        a = (p + n_refs * n_d - int(T * np.log(k) / np.log(meta['f2'] / meta['f1']) * fs)) % h.size
        if a < starts['noise'] + WIN and a + WIN > starts['ref1']:
            warn.append(f'harmonic {k} of the path lands in the analysis windows: use a longer sweep')
    purity = float(db(env[p] / env[p - int(0.05 * fs):p - int(0.005 * fs)].max()))
    if purity < 30:
        warn.append(f'reference only {purity:.0f} dB above the 5-50 ms before it: check APP_MODE and the PC Left cable')

    win = fade_window(WIN, fs)
    nfft = 2 * WIN
    f = sfft.rfftfreq(nfft, 1 / fs)
    sp = {k: sfft.rfft(h[s:s + WIN] * win, nfft) for k, s in starts.items()}
    valid = band_weight(f, meta['f1'], meta['f2']) > 0.05
    ref = sp[f'ref{n_refs}']                   # the reference recorded D before the path
    k1 = int(np.argmin(np.abs(f - 1000)))
    # Top of the usable band. X = path / reference, so where the reference path itself falls off, X is noise and
    # PC-side leakage blown up by 1 / |reference|. Old firmware: -3.6 dB at 18 kHz (US205). Since 2026-11-05 the
    # reference crosses the firmware's 192 <-> 48 kHz rate filters: -6 dB near 13 kHz, -72 dB at 20 kHz.
    rmag = np.sqrt(smooth(f, np.abs(ref) ** 2, 12))
    low = valid & (f > 1000) & (rmag < db2amp(-6.0) * rmag[k1])
    f_top = float(f[np.argmax(low)]) if low.any() else float(meta['f2'])
    theta = np.zeros(f.size)
    wander_us = np.nan
    if n_refs == 2:
        theta, covered = wander_phase(f, sp['ref1'], sp['ref2'], valid, meta)
        valid &= covered
        b = valid & (f >= 300) & (f <= 16000)
        wander_us = float(np.max(np.abs(theta[b] / (2 * np.pi * f[b]))) * 1e6)
        if not correct_wander:
            theta = np.zeros(f.size)
    g = gain * db2amp(meta['ref_dbfs'] - meta['level_dbfs'])
    X = np.zeros(f.size, complex)
    N = np.zeros(f.size)
    X[valid] = g * sp['path'][valid] / ref[valid] * np.exp(1j * theta[valid])
    # Noise of X: the path window's own noise, plus the reference windows' noise, which scales with X
    N[valid] = np.abs(sp['noise'][valid] / ref[valid]) * np.sqrt(g ** 2 + n_refs * np.abs(X[valid]) ** 2)
    Nsm = np.sqrt(smooth(f, N ** 2, 12))    # one bin of N is a random draw; weights use the smoothed floor
    chain_db = float(db(sp['ref1'][k1] / (gain * db2amp(meta['ref_dbfs']))))

    # Envelope for the plot: 0.5 ms max-pooled, dB re the reference peak, time re reference 1
    blk = int(0.0005 * fs)
    i0 = starts['ref1'] - int(0.05 * fs)
    i1 = min(starts['noise'] + WIN + int(0.05 * fs), h.size)
    seg = env[i0:i0 + (i1 - i0) // blk * blk].reshape(-1, blk).max(axis=1)
    return dict(f=f, X=X, N=N, Nsm=Nsm, valid=valid, f_top=f_top, p_s=p / fs, purity_db=purity, chain_db=chain_db,
                wander_us=wander_us, rec_peak_dbfs=float(db(np.max(np.abs(rec)))), warnings=warn,
                env_t=(i0 + blk * np.arange(seg.size) + blk / 2 - p) / fs * 1e3, env_db=db(seg / env[p]),
                windows={k: ((s - p) / fs * 1e3, (s + WIN - p) / fs * 1e3) for k, s in starts.items()})


def path_ir(f, X, meta, f2=None):
    """Band-limited impulse response of X (sample 0 = zero delay), its peak and -20 dB onset in ms.
    f2: top of the band, default meta['f2']."""
    fs = meta['fs']
    ir = sfft.irfft(X * band_weight(f, meta['f1'], f2 or meta['f2']), 2 * (f.size - 1))[:WIN]
    a = np.abs(ir)
    k = int(np.argmax(a))
    frac = 0.0
    if 0 < k < a.size - 1:
        den = a[k - 1] - 2 * a[k] + a[k + 1]
        frac = 0.5 * (a[k - 1] - a[k + 1]) / den if den else 0.0
    return ir, (k + frac) / fs * 1e3, int(np.argmax(a >= 0.1 * a[k])) / fs * 1e3


def mean_group_delay(f, X, valid):
    """Energy-weighted mean group delay (s) over DELAY_BAND, from the phase step between adjacent bins."""
    m = valid[:-1] & valid[1:] & (f[:-1] >= DELAY_BAND[0]) & (f[1:] <= DELAY_BAND[1])
    w = (np.abs(X[1:]) * np.abs(X[:-1]))[m]
    dphi = np.angle(X[1:] * np.conj(X[:-1]))[m]
    return float(-np.sum(w * dphi) / np.sum(w) / (2 * np.pi * (f[1] - f[0])))


def analyse_group(items):
    """items: [(meta, rec)] of one path and level, in run order."""
    runs = [analyse_run(rec, meta) for meta, rec in items]
    Xs = np.array([r['X'] for r in runs])
    Ns = np.array([r['Nsm'] for r in runs])
    M = Xs.mean(axis=0)
    meta = items[0][0]
    f = runs[0]['f']
    # The IR keeps the flat part of band_weight below the reference's -6 dB point (old firmware: stays at f2)
    f2_ir = min(meta['f2'], min(r['f_top'] for r in runs) * 2 ** (1 / 6))
    ir, peak_ms, onset_ms = path_ir(f, M, meta, f2_ir)
    return dict(meta=meta, metas=[m for m, _ in items], names=[m['name'] for m, _ in items], runs=runs, f=f,
                valid=runs[0]['valid'], f2_ir=f2_ir,
                Xs=Xs, Ns=Ns, M=M, Nm=np.sqrt((Ns ** 2).mean(axis=0) / len(runs)), ir=ir,
                peak_ms=peak_ms, onset_ms=onset_ms, gd_ms=mean_group_delay(f, M, runs[0]['valid']) * 1e3)


def time_zero_us(ra, rb):
    """Weighted least-squares delay of run b re run a over DELAY_BAND in us (> 0: b later), and its 1 sigma."""
    f = ra['f']
    m = ra['valid'] & (f >= DELAY_BAND[0]) & (f <= DELAY_BAND[1])
    xa, xb = ra['X'][m], rb['X'][m]
    var = (ra['Nsm'][m] / np.abs(xa)) ** 2 + (rb['Nsm'][m] / np.abs(xb)) ** 2
    keep = var < 0.25
    w, fm = 1 / var[keep], f[m][keep]
    phi = np.angle(xb[keep] / xa[keep])
    s = np.sum(w * fm ** 2)
    return -np.sum(w * fm * phi) / (2 * np.pi * s) * 1e6, 1e6 / (2 * np.pi * np.sqrt(s))


def band_ratio(f, num, den, w, valid, frac=1):
    """Weighted complex mean of num/den in 1/frac-octave bands around OCTAVES: [(dB, deg)]."""
    out = []
    for fc in OCTAVES:
        m = valid & (f >= fc * 2 ** (-0.5 / frac)) & (f < fc * 2 ** (0.5 / frac))
        q = np.sum(w[m] * num[m] / den[m]) / np.sum(w[m])
        out.append((float(db(q)), float(np.degrees(np.angle(q)))))
    return out


def h1_terms(a, b):
    """Per-bin numerator and denominator of the noise-corrected H1 estimate of a['M'] / b['M'] for groups a, b;
    sum each over a band and divide. The weights come from the noise windows, not from the noisy |M|: weighting
    by |M| favours bins where noise happened to add, so a level at 10-15 dB SNR read up to 0.6 dB high."""
    v = a['Nm'] ** 2 + b['Nm'] ** 2
    w = np.divide(1.0, v, out=np.zeros_like(v), where=v > 0)
    return w * a['M'] * np.conj(b['M']), w * (np.abs(b['M']) ** 2 - b['Nm'] ** 2)


def inv_var(*pairs):
    """1 / (sum of (N/|X|)^2) for (X, N) pairs; zero where X is zero."""
    with np.errstate(divide='ignore', invalid='ignore'):
        v = sum((N / np.abs(X)) ** 2 for X, N in pairs)
        return np.where(np.isfinite(v) & (v > 0), 1 / v, 0.0)


def octave_snr(f, X, N, valid):
    out = []
    for fc in OCTAVES:
        m = valid & (f >= fc / 2 ** 0.5) & (f < fc * 2 ** 0.5)
        out.append(float(10 * np.log10(np.sum(np.abs(X[m]) ** 2) / np.sum(N[m] ** 2))))
    return out


def tone_values(f, X, N):
    """(dB, SNR dB) at each fan tone, power-averaged over 1/24 octave."""
    out = []
    for ft in TONES:
        m = (f >= ft * 2 ** (-1 / 48)) & (f <= ft * 2 ** (1 / 48))
        pw, nw = np.mean(np.abs(X[m]) ** 2), np.mean(N[m] ** 2)
        out.append((float(10 * np.log10(pw)), float(10 * np.log10(pw / nw))))
    return out


def smooth(f, y, frac):
    """Mean of y over a 1/frac-octave band centred on each bin (works for complex y)."""
    c = np.concatenate(([0], np.cumsum(y)))
    lo = np.searchsorted(f, f * 2 ** (-0.5 / frac))
    hi = np.maximum(np.searchsorted(f, f * 2 ** (0.5 / frac), side='right'), lo + 1)
    return (c[hi] - c[lo]) / (hi - lo)


def safe_ratio(a, b):
    return np.divide(a, b, out=np.zeros_like(a), where=b != 0)


def smooth_ratio(f, num, den, w, frac):
    """Weighted complex mean of num/den over a 1/frac-octave band around each bin. Weighting by SNR^2
    keeps noisy bins from biasing the mean low."""
    return smooth(f, w * safe_ratio(num, den), frac) / np.maximum(smooth(f, w, frac), 1e-300)


def band_snr(f, X, N, frac=12):
    """Power SNR over a 1/frac-octave band around each bin."""
    return smooth(f, np.abs(X) ** 2, frac) / np.maximum(smooth(f, N ** 2, frac), 1e-300)


def group_delay(f, X, frac):
    """Energy-weighted group delay (s) over a 1/frac-octave band, from the phase step between adjacent bins."""
    w = np.abs(X[1:]) * np.abs(X[:-1])
    gd = -np.angle(X[1:] * np.conj(X[:-1])) / (2 * np.pi * (f[1] - f[0]))
    fm = (f[1:] + f[:-1]) / 2
    return fm, smooth(fm, w * gd, frac) / np.maximum(smooth(fm, w, frac), 1e-300)


def group_name(m):
    return ' '.join(x for x in (m['mmdd'], m['path'], f"{m['level_dbfs']:g}", m['length'], m.get('tag')) if x)


# ---------------------------------------------------------------- reports

def run_summary(r):
    snr = octave_snr(r['f'], r['X'], r['Nsm'], r['valid'])
    return (f"ref at {r['p_s']:.4f} s, purity {r['purity_db']:.0f} dB, chain {r['chain_db']:+.2f} dB, "
            f"peak {r['rec_peak_dbfs']:.1f} dBFS, SNR 125/1k/4k Hz {snr[1]:.0f}/{snr[4]:.0f}/{snr[6]:.0f} dB")


def report_group(g, out):
    out.append(f"\n== {group_name(g['meta'])}: {len(g['runs'])} run(s) ==")
    out.append(f"{'run':<22}{'ref at s':>9}{'chain dB':>9}{'purity':>8}{'peak dBFS':>10}{'wander':>8}" + OCT_HEAD
               + '  <- SNR per octave, dB')
    for name, r in zip(g['names'], g['runs']):
        snr = octave_snr(r['f'], r['X'], r['Nsm'], r['valid'])
        wander = 'none' if np.isnan(r['wander_us']) else f"{r['wander_us']:.0f} us"
        out.append(f"{name:<22}{r['p_s']:>9.4f}{r['chain_db']:>+9.2f}{r['purity_db']:>8.0f}{r['rec_peak_dbfs']:>10.1f}"
                   f'{wander:>8}' + ''.join(f'{v:>7.0f}' for v in snr))
        out.extend(f'  warning: {w}' for w in r['warnings'])
    if any(np.isnan(r['wander_us']) for r in g['runs']):
        out.append('  wander "none": single-reference run, timing wander between reference and path not corrected')
    vols = {}                                   # runs from before 2026-11-05 13:30 have no volume record
    for m in g['metas']:
        if 'windows_volume' in m:
            t = volume_text(m['windows_volume'])
            if 'windows_volume_after' in m:
                t += f"; CHANGED during the run to {volume_text(m['windows_volume_after'])}"
            vols.setdefault(t, []).append(m['name'].split()[-1])
    for t, rs in vols.items():
        out.append(f"Windows volume{'' if len(vols) == 1 else ' ' + ', '.join(rs)}: {t}")
    for t in dict.fromkeys(source_text(m['firmware']) for m in g['metas'] if 'firmware' in m):
        out.append(f'Firmware source: {t}')
    if len(g['runs']) > 1:
        r0 = g['runs'][0]
        tz = [time_zero_us(r0, r) for r in g['runs'][1:]]
        out.append('time zero re r1, 200 Hz-4 kHz (us): '
                   + ', '.join(f'{n.split()[-1]} {t:+.3f} +-{s:.3f}' for n, (t, s) in zip(g['names'][1:], tz)))
        w = inv_var((g['M'], g['Nm']))
        dev = [band_ratio(g['f'], X, g['M'], w, g['valid']) for X in g['Xs']]
        out.append(f"{'largest run - mean, dB':<66}" + ''.join(f'{max(abs(d[i][0]) for d in dev):>7.2f}'
                                                             for i in range(len(OCTAVES))))
        out.append(f"{'largest run - mean, deg':<66}" + ''.join(f'{max(abs(d[i][1]) for d in dev):>7.1f}'
                                                              for i in range(len(OCTAVES))))
    out.append(f"path delay: onset (-20 dB re IR peak) {g['onset_ms']:.3f} ms, IR peak {g['peak_ms']:.3f} ms, "
               f"mean group delay 200 Hz-4 kHz {g['gd_ms']:.3f} ms")
    if g['f2_ir'] < g['meta']['f2']:
        out.append(f"IR band ends at {g['f2_ir'] / 1e3:.1f} kHz: the reference path falls off above that "
                   f"(firmware rate filters)")
    tv = tone_values(g['f'], g['M'], g['Nm'])
    out.append(f"{'tone (Hz)':<16}" + ''.join(f'{t:>8}' for t in TONES))
    out.append(f"{'|' + g['meta']['path'] + '| dB':<16}" + ''.join(f'{v:>8.1f}' for v, _ in tv))
    out.append(f"{'SNR dB':<16}" + ''.join(f'{s:>8.1f}' for _, s in tv))


def report_levels(gs, out):
    levels = sorted(gs)
    if len(levels) < 2:
        return
    q = gs[levels[0]]
    out.append(f"\n== {q['meta']['path']}: level difference re {levels[0]:g} dBFS (noise-corrected H1 per octave) ==")
    out.append(f"{'level':<12}" + OCT_HEAD)
    for L in levels[1:]:
        g = gs[L]
        num, den = h1_terms(g, q)
        br = []
        for fc in OCTAVES:
            m = g['valid'] & q['valid'] & (g['f'] >= fc / 2 ** 0.5) & (g['f'] < fc * 2 ** 0.5)
            r = np.sum(num[m]) / np.sum(den[m])
            br.append((float(db(r)), float(np.degrees(np.angle(r)))))
        out.append(f"{f'{L:g} dB':<12}" + ''.join(f'{d:>7.2f}' for d, _ in br))
        out.append(f"{'     deg':<12}" + ''.join(f'{a:>7.1f}' for _, a in br))


def report_sf(gS, gF, out):
    tS, tF = tone_values(gS['f'], gS['M'], gS['Nm']), tone_values(gF['f'], gF['M'], gF['Nm'])
    out.append(f"\n== S ({gS['meta']['level_dbfs']:g} dBFS) and F ({gF['meta']['level_dbfs']:g} dBFS) at the fan tones ==")
    out.append(f"{'tone (Hz)':<20}" + ''.join(f'{t:>8}' for t in TONES))
    out.append(f"{'S dB':<20}" + ''.join(f'{s:>8.1f}' for s, _ in tS))
    out.append(f"{'F dB':<20}" + ''.join(f'{v:>8.1f}' for v, _ in tF))
    out.append(f"{'F/S dB':<20}" + ''.join(f'{v - s:>+8.1f}' for (s, _), (v, _) in zip(tS, tF)))
    out.append(f"{'Err reach dBFS':<20}" + ''.join(f'{s + db(Y_MAX):>8.1f}' for s, _ in tS)
               + f'   (largest Err level the ANC can cancel with |y| = {Y_MAX})')


# ---------------------------------------------------------------- plots

def pyplot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': AXIS, 'axes.labelcolor': INK2, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titlelocation': 'left', 'axes.labelsize': 9, 'xtick.color': MUTED, 'ytick.color': MUTED,
        'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'grid.linestyle': '-',
        'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False,
        'legend.fontsize': 8, 'text.color': INK, 'font.family': 'sans-serif',
        'font.sans-serif': ['Segoe UI', 'DejaVu Sans'], 'savefig.dpi': 110,
    })
    return plt


def blend(color, t):
    """color mixed with the chart surface: t = 0 gives the colour, 1 the surface."""
    c, s = (np.array([int(x[i:i + 2], 16) for i in (1, 3, 5)]) / 255 for x in (color, SURFACE))
    return tuple((1 - t) * c + t * s)


def freq_axis(ax, bottom=True):
    ax.set_xscale('log')
    ax.set_xlim(F1, F2)
    ax.set_xticks([20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000],
                  ['20', '50', '100', '200', '500', '1k', '2k', '5k', '10k', '20k'])
    ax.minorticks_off()
    if bottom:
        ax.set_xlabel('Frequency (Hz)')


def clamp_ylim(ax, limit):
    lo, hi = ax.get_ylim()
    ax.set_ylim(max(lo, -limit), min(hi, limit))


def plot_mask(f):
    return (f >= F1 * 2 ** 0.5) & (f <= F2 * 2 ** (-1 / 6))


def plot_path(gs, png):
    """One path, all levels: response, phase, run spread, level linearity, deconvolved recording."""
    plt = pyplot()
    levels = sorted(gs, reverse=True)                           # loudest first, darkest shade
    q = gs[levels[-1]]
    path = q['meta']['path']
    shade = {L: blend(PATH_COLOR[path], 0.5 * i / max(len(levels) - 1, 1)) for i, L in enumerate(levels)}
    f = q['f']
    pm = plot_mask(f)
    fig, ax = plt.subplots(3, 2, figsize=(12, 10.5))
    for L in levels:
        g = gs[L]
        lab = f'{L:g} dBFS'
        ax[0, 0].plot(f[pm], db(np.sqrt(smooth(f, np.abs(g['M']) ** 2, 12)))[pm], color=shade[L], lw=1.5, label=lab)
        fm, gd = group_delay(f, g['M'], 12)
        gm = plot_mask(fm)
        ax[0, 1].plot(fm[gm], gd[gm] * 1e3, color=shade[L], lw=1.5, label=lab)
        w = inv_var((g['M'], g['Nm']))
        ok = (band_snr(f, g['M'], g['Nm']) > 100)[pm]
        for i, X in enumerate(g['Xs']):
            r = smooth_ratio(f, X, g['M'], w, 12)[pm]
            kw = dict(color=shade[L], lw=0.9, label=lab if i == 0 else None)
            ax[1, 0].plot(f[pm], np.where(ok, db(r), np.nan), **kw)
            ax[1, 1].plot(f[pm], np.where(ok, np.degrees(np.angle(r)), np.nan), **kw)
    ax[0, 0].plot(f[pm], db(np.sqrt(smooth(f, q['Nm'] ** 2, 12)))[pm], color=MUTED, lw=0.9,
                  label=f'noise at {levels[-1]:g} dBFS')
    ax[0, 1].axhline(q['onset_ms'], color=MUTED, lw=0.9)
    ax[0, 1].annotate(f"onset {q['onset_ms']:.2f} ms", (F1 * 1.5, q['onset_ms']), textcoords='offset points',
                      xytext=(0, 4), color=INK2, fontsize=8)
    for L in levels[:-1]:
        num, den = h1_terms(gs[L], q)
        r = (smooth(f, num, 6) / np.maximum(smooth(f, den, 6), 1e-300))[pm]
        ok = (band_snr(f, q['M'], q['Nm']) > 100)[pm]
        ax[2, 0].plot(f[pm], np.where(ok, db(r), np.nan), color=shade[L], lw=1.5, label=f'{L:g} dBFS')
    if len(levels) == 1:
        ax[2, 0].text(0.5, 0.5, 'one level measured', transform=ax[2, 0].transAxes,
                      ha='center', va='center', color=MUTED)
    run = q['runs'][0]
    ax[2, 1].plot(run['env_t'], run['env_db'], color=PATH_COLOR[path], lw=0.8)
    for key, (t0, t1) in run['windows'].items():
        ax[2, 1].axvspan(t0, t1, color=MUTED, alpha=0.12, lw=0)
        ax[2, 1].text((t0 + t1) / 2, 3, {'ref1': 'ref 1', 'ref2': 'ref 2', 'path': path, 'noise': 'noise'}[key],
                      ha='center', va='bottom', color=INK2, fontsize=8)

    titles = [(f'{path} magnitude, FS in -> FS out (1/12 oct)', 'dB'),
              (f'{path} group delay, energy-weighted (1/12 oct)', 'ms'),
              ('Each run re its level mean, magnitude (1/12 oct, where SNR > 20 dB)', 'dB'),
              ('Each run re its level mean, phase (1/12 oct, where SNR > 20 dB)', 'deg'),
              (f'Level re {levels[-1]:g} dBFS, magnitude (1/6 oct, where SNR > 20 dB)', 'dB'),
              (f"Deconvolved recording {q['names'][0]}", 'dB re reference peak')]
    for a, (title, ylabel) in zip(ax.flat, titles):
        a.set_title(title)
        a.set_ylabel(ylabel)
    for a in ax.flat[:5]:
        freq_axis(a, bottom=a in (ax[2, 0], ax[1, 1]))
    lo, hi = ax[0, 0].get_ylim()
    ax[0, 0].set_ylim(max(lo, hi - 100), hi)
    clamp_ylim(ax[1, 0], 3)
    clamp_ylim(ax[1, 1], 30)
    clamp_ylim(ax[2, 0], 6)
    ax[2, 1].set_xlim(run['env_t'][0], run['env_t'][-1])
    ax[2, 1].set_ylim(max(run['env_db'].min(), -160), 12)
    ax[2, 1].set_xlabel('Time re reference (ms)')
    for a in (ax[0, 0], ax[1, 0], ax[2, 0]):
        if len(a.get_legend_handles_labels()[0]) > 1:
            a.legend(loc='best')
    fig.suptitle(' '.join(x for x in (q['meta']['mmdd'], path, q['meta']['length'], q['meta'].get('tag')) if x),
                 x=0.01, ha='left', fontsize=12, color=INK)
    fig.tight_layout()
    fig.savefig(png)
    plt.close(fig)


def plot_sf(gS, gF, png):
    plt = pyplot()
    f = gS['f']
    pm = plot_mask(f)
    fig, ax = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    pw = {}
    for g in (gS, gF):
        path = g['meta']['path']
        pw[path] = smooth(f, np.abs(g['M']) ** 2, 24)[pm]       # same band as tone_values
        ax[0].plot(f[pm], db(np.sqrt(pw[path])), color=PATH_COLOR[path], lw=1.5,
                   label=f"{path} at {g['meta']['level_dbfs']:g} dBFS")
    with np.errstate(invalid='ignore', divide='ignore'):     # both zero above the wander-corrected band
        ax[1].plot(f[pm], db(np.sqrt(pw['F'] / pw['S'])), color=INK2, lw=1.5)
    tS, tF = tone_values(f, gS['M'], gS['Nm']), tone_values(f, gF['M'], gF['Nm'])
    for ft, (s, _), (v, _) in zip(TONES, tS, tF):
        ax[1].plot([ft], [v - s], 'o', ms=5, color=INK2, mec=SURFACE, mew=1.5)
        ax[1].annotate(f'{v - s:+.1f}', (ft, v - s), textcoords='offset points', xytext=(0, 8),
                       ha='center', fontsize=8, color=INK)
    ax[0].set_title('S and F magnitude, FS in -> FS out (1/24 oct)')
    ax[0].set_ylabel('dB')
    ax[0].legend(loc='best')
    ax[1].set_title('F/S (1/24 oct); dots: fan tones')
    ax[1].set_ylabel('dB')
    freq_axis(ax[0], bottom=False)
    freq_axis(ax[1])
    fig.tight_layout()
    fig.savefig(png)
    plt.close(fig)


# ---------------------------------------------------------------- commands

def analyze_folder(folder, use_level=None, plots=True):
    items = {}
    for js in sorted(folder.glob('*.json')):
        meta = json.loads(js.read_text(encoding='utf-8'))
        if meta.get('kind') != 'pathmeas-run':
            continue
        if meta.get('failed'):
            print(f"skipping {meta['name']}: {meta['failed']}")
            continue
        if bad_xruns(meta):
            print(f"skipping {meta['name']}: xruns while the sweeps played")
            continue
        _, rec = wavfile.read(js.with_suffix('.wav'))
        key = (meta['mmdd'], meta['length'], meta.get('tag') or '', meta['path'], meta['level_dbfs'])
        items.setdefault(key, []).append((meta, rec))
    if not items:
        sys.exit(f'No usable runs in {folder}')
    sessions = {}
    for (mmdd, length, tag, path, level), v in sorted(items.items()):
        g = analyse_group(sorted(v, key=lambda x: x[0]['run']))
        sessions.setdefault((mmdd, length, tag), {}).setdefault(path, {})[level] = g
    out = [f'Path measurement report: {folder}']
    for (mmdd, length, tag), paths in sessions.items():
        for path in (p for p in 'SF' if p in paths):
            gs = paths[path]
            for level in sorted(gs, reverse=True):
                g = gs[level]
                report_group(g, out)
                stem = folder / group_name(g['meta'])
                np.savez(f'{stem}.npz', f=g['f'], X=g['M'], N=g['Nm'], X_runs=g['Xs'], N_runs=g['Ns'],
                         runs=np.array(g['names']), meta=json.dumps(dict(g['meta'], f2_ir=g['f2_ir']), ensure_ascii=False))
                wavfile.write(f'{stem} IR.wav', g['meta']['fs'], g['ir'].astype(np.float32))
            report_levels(gs, out)
            if plots:
                plot_path(gs, folder / (' '.join(x for x in (mmdd, path, length, tag) if x) + '.png'))
        if 'S' in paths and 'F' in paths:
            gS, gF = (pick_level(paths[p], use_level) for p in 'SF')
            report_sf(gS, gF, out)
            if plots:
                plot_sf(gS, gF, folder / (' '.join(x for x in (mmdd, 'SF', length, tag) if x) + '.png'))
    text = '\n'.join(out)
    print(text)
    (folder / 'report.txt').write_text(text + '\n', encoding='utf-8')


def pick_level(gs, use_level):
    """The quietest level, or the one closest to use_level."""
    return gs[min(gs) if use_level is None else min(gs, key=lambda L: abs(L - use_level))]


def next_stem(folder, mmdd, path, level, length, tag):
    base = ' '.join(x for x in (mmdd, path, f'{level:g}', length, tag) if x)
    k = 1
    while (folder / f'{base} r{k}.json').exists():
        k += 1
    return f'{base} r{k}', k


def cmd_measure(a):
    sd = import_sounddevice()
    dev_in, dev_out = wasapi_devices(sd, a.device)
    chirp_gain, mode = firmware_settings()
    n_sweep = parse_length(a.length)
    exclusive = not a.shared
    for check, dev, ch in ((sd.check_output_settings, dev_out, 2), (sd.check_input_settings, dev_in, 1)):
        try:
            check(device=dev['index'], channels=ch, dtype='float32', samplerate=FS,
                  extra_settings=sd.WasapiSettings(exclusive=exclusive))
        except sd.PortAudioError as e:
            sys.exit(f"{dev['name']} does not take {FS} Hz in {'exclusive' if exclusive else 'shared'} mode: {e}")
    day = datetime.date.today()
    folder = DATA_ROOT / day.isoformat()
    base = dict(kind='pathmeas-run', mmdd=day.strftime('%m%d'), path=a.path, length=length_tag(n_sweep),
                tag=a.tag, fs=FS, f1=F1, f2=F2, n_sweep=n_sweep, n_lead=int(LEAD_S * FS),
                n_delay=int(DELAY_S * FS), n_tail=int(TAIL_S * FS), n_refs=N_REFS, ref_dbfs=a.ref_level,
                chirp_gain=chirp_gain, device_out=dev_out['name'], device_in=dev_in['name'],
                exclusive=exclusive, notes=a.notes, firmware=dict(git_state() or {}, app_mode=mode))
    run_s = (base['n_lead'] + N_REFS * base['n_delay'] + n_sweep + base['n_tail']) / FS + PREROLL_S + MARGIN_S
    jack = JACK[a.path]
    print(f"Output {dev_out['name']}, input {dev_in['name']}, WASAPI {'exclusive' if exclusive else 'shared'}, {FS} Hz")
    print('Firmware source (recorded with each run; say in --notes if the board runs another build): '
          + source_text(base['firmware']))
    vol = read_volumes(dev_out, dev_in)
    print(f'Windows volume (recorded with each run): {volume_text(vol)}')
    for w in volume_warnings(vol):
        print(f'WARNING: {w}')
    if any(e.get('mute') for e in vol.values()):
        sys.exit('A muted endpoint plays or records silence. Unmute it in Windows, then try again.')
    print(f'Path {a.path}: {jack} must be in the US205 mic input. Chirp gain {chirp_gain:g} from the firmware source.')
    if mode != 'APP_MODE_PATH_MEASURE':
        print(f'WARNING: the firmware source selects {mode}; the board must run APP_MODE_PATH_MEASURE.')
    print(f"Sweep {base['length']} ({n_sweep / FS:.1f} s), {F1:g} Hz-{F2 / 1000:g} kHz. {N_REFS} references on PC Left "
          f"at {a.ref_level:g} dBFS; speaker drive on PC Right at {', '.join(f'{L:g}' for L in a.levels)} dBFS; "
          f'{a.repeats} run(s) each, about {len(a.levels) * a.repeats * run_s / 60:.1f} min in total.')
    print(f'Data: {folder}')
    if not a.yes:
        try:
            input('This plays sweeps through the speaker. Press Enter to start, Ctrl+C to abort: ')
        except (EOFError, KeyboardInterrupt):
            sys.exit('\nAborted.')
    folder.mkdir(parents=True, exist_ok=True)
    first = True
    for level in a.levels:
        for _ in range(a.repeats):
            name, k = next_stem(folder, base['mmdd'], a.path, level, base['length'], a.tag)
            meta = dict(base, level_dbfs=level, run=k, name=name,
                        started=datetime.datetime.now().isoformat(timespec='seconds'),
                        windows_volume=read_volumes(dev_out, dev_in))
            print(f'{name}: {run_s:.0f} s ...', flush=True)
            try:
                rec, flags = record(sd, make_play(meta), FS, dev_in, dev_out, exclusive)
            except sd.PortAudioError as e:
                sys.exit(f'Audio error: {e}. Close REW or anything else that uses the US205, then try again.')
            meta['xruns'] = flags
            after = read_volumes(dev_out, dev_in)
            if after != meta['windows_volume']:          # e.g. a call app's automatic mic level
                meta['windows_volume_after'] = after
                print(f'  WARNING: the Windows volume changed during the run, now {volume_text(after)}')
            wavfile.write(folder / f'{name}.wav', FS, rec)
            (folder / f'{name}.json').write_text(json.dumps(meta, indent=1, ensure_ascii=False), encoding='utf-8')
            bad = bad_xruns(meta)
            if bad:
                print(f'  {len(bad)} xrun(s) while the sweeps played; this run will be skipped: {bad}')
                continue
            try:
                r = analyse_run(rec, meta)
            except ValueError as e:                     # keep the files, but out of every later analysis
                meta['failed'] = str(e)
                (folder / f'{name}.json').write_text(json.dumps(meta, indent=1, ensure_ascii=False), encoding='utf-8')
                sys.exit(f'  {e}')
            print('  ' + run_summary(r))
            for w in r['warnings']:
                print(f'  warning: {w}')
            if first and r['purity_db'] < 30:
                sys.exit('  No clean reference in the first run: check the firmware mode and the cables, then retry.')
            first = False
    analyze_folder(folder, a.use_level, not a.no_plot)


def cmd_analyze(a):
    analyze_folder(Path(a.folder), a.use_level, not a.no_plot)


def cmd_devices(a):
    sd = import_sounddevice()
    api = next(i for i, h in enumerate(sd.query_hostapis()) if 'WASAPI' in h['name'])
    for d in sd.query_devices():
        if d['hostapi'] == api:
            mark = '*' if a.device.lower() in d['name'].lower() else ' '
            print(f"{mark}{d['index']:>4}  in {d['max_input_channels']}  out {d['max_output_channels']}  "
                  f"{d['default_samplerate']:>6.0f} Hz  {d['name']}")
    dev_in, dev_out = wasapi_devices(sd, a.device)
    for exclusive in (True, False):
        res = []
        for check, dev, ch in ((sd.check_output_settings, dev_out, 2), (sd.check_input_settings, dev_in, 1)):
            try:
                check(device=dev['index'], channels=ch, dtype='float32', samplerate=FS,
                      extra_settings=sd.WasapiSettings(exclusive=exclusive))
                res.append('ok')
            except sd.PortAudioError:
                res.append('not supported')
        print(f"{FS} Hz {'exclusive' if exclusive else 'shared'}: output {res[0]}, input {res[1]}")
    print(f'Windows volume: {volume_text(read_volumes(dev_out, dev_in))}')


def conv(x, h):
    return signal.oaconvolve(x, h)[:x.size]


def synth_chain(f_hp, gain, n=32768):
    """Converter chain: 2nd-order AC-coupling high-pass and a linear-phase 21 kHz low-pass."""
    h = np.zeros(n)
    h[0] = gain
    h = signal.sosfilt(signal.butter(2, f_hp, 'highpass', fs=FS, output='sos'), h)
    return signal.oaconvolve(h, signal.firwin(255, 21000, fs=FS))[:n]


def synth_path(rng, n=8192):
    """Speaker-to-mic-like response: 0.9 ms delay, 120 Hz high-pass, 1.8 kHz peak, 15 ms decaying tail."""
    t = np.arange(n) / FS
    h = np.zeros(n)
    h[0] = 1.0
    h += 0.3 * rng.standard_normal(n) * np.exp(-t / 0.015) * (t > 0.0005)
    h = signal.sosfilt(signal.butter(2, 120, 'highpass', fs=FS, output='sos'), h)
    h = signal.sosfilt(signal.butter(2, 6000, 'lowpass', fs=FS, output='sos'), h)
    b, a = signal.iirpeak(1800, 8, fs=FS)
    h += 0.5 * signal.lfilter(b, a, h)
    h = sfft.irfft(sfft.rfft(h, 2 * n) * np.exp(-2j * np.pi * sfft.rfftfreq(2 * n) * 43.37), 2 * n)[:n]
    return h / np.max(np.abs(h))


def synth_wander(n, fs, rng, amp_us=30.0):
    """Playback-to-capture timing like the US205's, in samples: plateaus of 0.5-3 s joined by 0.3 s ramps."""
    d = np.empty(n)
    i, level = 0, 0.0
    while i < n:
        hold = int(rng.uniform(0.5, 3.0) * fs)
        d[i:i + hold] = level
        i += hold
        new = rng.uniform(-amp_us, amp_us) * 1e-6 * fs
        r = int(0.3 * fs)
        m = d[i:i + r].size
        d[i:i + m] = (level + (new - level) * (0.5 - 0.5 * np.cos(np.pi * np.arange(r) / r)))[:m]
        i += r
        level = new
    return d


def fractional_delay(x, d):
    """y[n] = x(n - d[n]) for a slowly varying delay d in samples (32-tap Kaiser-windowed sinc)."""
    pos = np.arange(x.size) - d
    i0 = np.floor(pos).astype(np.int64)
    frac = pos - i0
    y = np.zeros(x.size)
    for i in range(-15, 17):
        u = i - frac
        tap = np.sinc(u) * np.i0(8.0 * np.sqrt(np.clip(1 - (u / 16.5) ** 2, 0, None))) / np.i0(8.0)
        y += x[np.clip(i0 + i, 0, x.size - 1)] * tap
    return y


def cmd_selftest(a):
    """Simulated chain with matched input chains, a speaker with 2nd-harmonic distortion and US205-like timing
    wander, two stream latencies per case:
      1. no wander, clean: the analysis itself must be exact
      2. no wander, noisy: the reported noise floor must match the actual error
      3. wander, clean: what the correction leaves. The path's own response spans ~10 ms, and during a wander
         ramp the timing moves within it, which one correction per bin cannot undo; limits are p99 per band
      4. a path stronger than the references, with wander and noise: reference detection"""
    rng = np.random.default_rng(1)
    meta = dict(fs=FS, f1=F1, f2=F2, n_sweep=parse_length(a.length), n_lead=int(LEAD_S * FS),
                n_delay=int(DELAY_S * FS), n_tail=int(TAIL_S * FS), n_refs=N_REFS, ref_dbfs=-12.0,
                level_dbfs=-12.0, chirp_gain=0.05)
    play = make_play(meta).astype(np.float64)
    c_in, c_out = synth_chain(3.0, 1.4), synth_chain(8.0, 0.7)   # PC -> J16 ADC, J17 DAC -> mic input
    x_ref = conv(play[:, 0], c_in)
    y = conv(play[:, 1], c_in)
    y = y + 0.1 * conv(y, signal.firwin(255, 7000, fs=FS)) ** 2   # harmonic stays below Nyquist
    n_rec = play.shape[0] + int((PREROLL_S + MARGIN_S) * FS)
    lats = (int(0.31 * FS) + 17, int(0.47 * FS) + 5)
    # p99 phase error (deg) left by the wander correction; typical 2M values are about half of these
    limits = ((50, 500, 1.0), (500, 2000, 3.0), (2000, 8000, 8.0))

    def simulate(path, noise, wander):
        out = conv(conv(y, path) + noise * 1e-5 * rng.standard_normal(y.size) + meta['chirp_gain'] * x_ref, c_out)
        recs = []
        for lat in lats:
            rec = np.zeros(n_rec)
            rec[lat:lat + out.size] = out
            if wander:
                rec = fractional_delay(rec, synth_wander(n_rec, FS, rng))
            recs.append((rec + noise * 3e-5 * rng.standard_normal(n_rec)).astype(np.float32))
        return recs

    def found(runs):
        return all(0 <= r['p_s'] - (lat + meta['n_lead']) / FS < 0.01 for r, lat in zip(runs, lats))

    def p99(runs, truth, lo, hi):
        e = [np.angle(r['X'][b] / truth[b]) for r in runs for b in [r['valid'] & (r['f'] >= lo) & (r['f'] < hi)]]
        return float(np.percentile(np.abs(np.degrees(np.concatenate(e))), 99))

    ok = True
    path = 0.001 * synth_path(rng)
    truth = sfft.rfft(path, 2 * WIN)
    runs = [analyse_run(r, meta) for r in simulate(path, 0.0, False)]
    m = runs[0]['valid'] & (runs[0]['f'] >= 50) & (runs[0]['f'] <= 16000)
    err = np.concatenate([r['X'][m] / truth[m] for r in runs])
    e_db, e_deg = float(np.max(np.abs(db(err)))), float(np.max(np.abs(np.degrees(np.angle(err)))))
    tz, _ = time_zero_us(*runs)
    good = found(runs) and e_db < 0.02 and e_deg < 0.2 and abs(tz) < 0.01
    print(f'1. no wander, clean: error max {e_db:.4f} dB {e_deg:.3f} deg (50 Hz-16 kHz), '
          f'time zero r2 re r1 {tz:+.4f} us  {"ok" if good else "FAIL"}')
    ok &= good

    runs = [analyse_run(r, meta) for r in simulate(path, 1.0, False)]
    f = runs[0]['f']
    ratio = [float(np.sqrt(np.mean(np.abs(runs[0]['X'][b] - truth[b]) ** 2) / np.mean(runs[0]['Nsm'][b] ** 2)))
             for b in (runs[0]['valid'] & (f >= lo) & (f < 2 * lo) for lo in (62.5, 250, 1000, 4000))]
    tz, sig = time_zero_us(*runs)
    good = found(runs) and abs(tz) < max(4 * sig, 0.01) and all(0.7 < x < 1.4 for x in ratio)
    print(f'2. no wander, noisy: error / reported noise floor at 63/250/1k/4k Hz '
          + ' '.join(f'{x:.2f}' for x in ratio) + f', time zero {tz:+.4f} +-{sig:.4f} us  {"ok" if good else "FAIL"}')
    ok &= good

    recs = simulate(path, 0.0, True)
    fixed = [analyse_run(r, meta) for r in recs]
    raw = [analyse_run(r, meta, correct_wander=False) for r in recs]
    tz, _ = time_zero_us(*fixed)
    tz_raw, _ = time_zero_us(*raw)
    pf = [p99(fixed, truth, lo, hi) for lo, hi, _ in limits]
    pr = [p99(raw, truth, lo, hi) for lo, hi, _ in limits]
    good = (found(fixed) and all(x < lim for x, (_, _, lim) in zip(pf, limits)) and abs(tz) < 0.2
            and all(x < y / 4 for x, y in zip(pf, pr)))
    print(f'3. wander ({fixed[0]["wander_us"]:.0f}/{fixed[1]["wander_us"]:.0f} us over D), clean: p99 phase error '
          f'50-500/500-2k/2k-8k Hz corrected {"/".join(f"{x:.2f}" for x in pf)} deg, uncorrected '
          f'{"/".join(f"{x:.1f}" for x in pr)} deg; time zero {tz:+.3f} us (uncorrected {tz_raw:+.2f})  '
          f'{"ok" if good else "FAIL"}')
    ok &= good

    strong = 1.0 * synth_path(rng)
    truth = sfft.rfft(strong, 2 * WIN)
    runs = [analyse_run(r, meta) for r in simulate(strong, 1.0, True)]
    ps = [p99(runs, truth, lo, hi) for lo, hi, _ in limits]
    good = found(runs) and all(x < lim for x, (_, _, lim) in zip(ps, limits))
    print(f'4. path above the references, wander and noise: reference found {found(runs)}, p99 phase error '
          f'{"/".join(f"{x:.2f}" for x in ps)} deg  {"ok" if good else "FAIL"}')
    ok &= good
    print('PASS' if ok else 'FAIL')


def main():
    sys.stdout.reconfigure(errors='replace')
    ap = argparse.ArgumentParser(description='Secondary / feedback path measurement with an embedded electrical reference.')
    sub = ap.add_subparsers(dest='cmd', required=True)
    d = sub.add_parser('devices', help='list WASAPI devices and check the US205 formats (silent)')
    d.add_argument('--device', default=DEVICE)
    s = sub.add_parser('selftest', help='simulate the chain and check the analysis (silent, about 2 min)')
    s.add_argument('--length', default='2M', help='the wander limits assume the measurement default, 2M')
    m = sub.add_parser('measure', help='play sweeps through the speaker and record')
    m.add_argument('--path', required=True, choices=('S', 'F'), help='S: J17 Left in the mic input, F: J17 Right')
    m.add_argument('--levels', type=float, nargs='+', default=[-12.0], help='speaker drive on PC Right, dBFS (sine peak)')
    m.add_argument('--ref-level', type=float, default=-12.0, help='reference on PC Left, dBFS')
    m.add_argument('--repeats', type=int, default=2)
    m.add_argument('--length', default='2M', help='sweep length in samples: 1M, 2M, 4M (512k puts the 2nd harmonic '
                                                  'into the reference window)')
    m.add_argument('--tag', default='', help='extra word in the file names, e.g. knob3')
    m.add_argument('--notes', default='', help='free text stored with each run and copied into the exported header '
                                               '(who is in front of the outlet, knob, cables, firmware build); the '
                                               'Windows volumes and the git commit are recorded automatically')
    m.add_argument('--device', default=DEVICE)
    m.add_argument('--shared', action='store_true', help='WASAPI shared mode instead of exclusive')
    m.add_argument('--use-level', type=float, help='level (dBFS) for the S/F comparison; default the quietest')
    m.add_argument('--no-plot', action='store_true')
    m.add_argument('--yes', action='store_true', help='start without the Enter prompt')
    z = sub.add_parser('analyze', help='analyse a day folder again')
    z.add_argument('folder', nargs='?', default=str(DATA_ROOT / datetime.date.today().isoformat()))
    z.add_argument('--use-level', type=float, help='level (dBFS) for the S/F comparison; default the quietest')
    z.add_argument('--no-plot', action='store_true')
    a = ap.parse_args()
    {'devices': cmd_devices, 'selftest': cmd_selftest, 'measure': cmd_measure, 'analyze': cmd_analyze}[a.cmd](a)


if __name__ == '__main__':
    main()
