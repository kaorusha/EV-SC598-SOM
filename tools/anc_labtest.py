#!/usr/bin/env python3
"""Lab ANC run: record the Err mic while CCES_runner loads and runs the Release build, then measure the attenuation.

Setup (APP_MODE_ANC): J17 Left (the raw Err mic) into the US205 mic input through the L/R split cable (a stereo plug
in the mono mic jack bridges L and R), J18 Right to the speaker amp, fan running. CCES_runner loads the Release .dxe
with --reset-on-connect. The board keeps running ANC after the runner exits, so stop it afterwards: speaker power off
first, then the board.

The first program load after a board power-up makes a loud pop while the codecs start: keep the speaker amp off during
that load and switch it on before ANC starts (ANC_START_DELAY_S after boot). A reload gives only a small click.

  python tools/anc_labtest.py check                      silent: build time, US205, Windows volume, input level now
  python tools/anc_labtest.py run --name "1117 ANC fan100 band400-1500" [--timeout 100] [--notes "..."] [--cces DIR]
  python tools/anc_labtest.py analyze <wav> [--f0 HZ]    again, from the saved files
  python tools/anc_labtest.py report <wav> [<wav> ...] [--labels ...] [--out DIR] [--name PREFIX]
                                                         slide figures for 1-3 runs (below)
  python tools/anc_labtest.py diag <wav>                 ANC_DIAG output after a divergence trip ([DIAG] lines, snapshot)
  python tools/anc_labtest.py synth <wav>                synthetic recording to test the analysis

The CrossCore Embedded Studio folder (CCES_runner.exe, SHARC/ldr/ezkitSC598W_preload_core1.dxe) comes from --cces or
the CCES_ROOT environment variable.

Data go to measurements/anc_test/ (gitignored): "<name>".wav (mono float32), .log (runner output), .json (times,
notes, devices, Windows volumes, firmware source state), .txt and .png (analysis), and "<name> snapshot.txt" when
the ANC_DIAG build wrote one after a trip (anc_fxnlms.h ANC_DIAG_FILE).

Analysis: ANC off is the 10 s before the ANC start (runner start + ANC_AFTER_RUNNER), without the stretches where a
status print froze J17 L (a JTAG printf halts the core and the SPORT DMA repeats its last buffers). Attenuation is
off - on in dB (above 0 = quieter) for the fan tones f0, 2 f0, 5 f0, 6 f0 (power within +-6 Hz), 400-560 Hz,
7-12 kHz and the total 60 Hz-20 kHz. When the divergence guard tripped ("[ANC TRIP] Diverged ... s after ANC started"
in the log), ANC on ends 0.5 s before the trip, since the guard muted the speaker there. A timed switch-off
(ANC_AUTO_OFF_S, "[ANC OFF] Output switched off ... s after ANC started") ends it the same way. With ANC_AUTO_OFF_S,
give run a --timeout that covers the ANC start (about 28 s after the runner start), ANC_AUTO_OFF_S and the
diagnostics dump that follows.

Report slides show each result twice: as bars (easy to read) and as the Err spectrum with ANC off and on (where the
change is, including what got louder). "report" writes both, plus the attenuation over time, for up to 3 runs side
by side, e.g. for the 11/17 band-pass comparison:
  python tools/anc_labtest.py report "measurements/anc_test/1117 ANC fan100 band400-1500 labS.wav"
      "measurements/anc_test/1117 ANC fan100 band400-5000 labS diag.wav" --name "1117 band 400-1500 vs 400-5000"
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.io import wavfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pathmeas as pm  # noqa: E402  import_sounddevice, wasapi_devices, windows_volume, git_state, DEVICE, LATENCY_S

REPO = Path(__file__).resolve().parents[1]
OUT_DIR = REPO / 'measurements' / 'anc_test'
DIAG_FILE = REPO / 'measurements' / 'anc_diag' / 'anc_diag_snapshot.txt'   # ANC_DIAG_FILE in anc_fxnlms.h
DXE = (REPO / 'EV-SOMCRR-EZKIT' / 'SC598' / 'sharc' / 'AD_Talkthrough_I2S_SC598_SHARC_Core1' / 'Release'
       / 'AD_Talkthrough_I2S_SC598_SHARC_Core1.dxe')
SOURCES = [REPO / f for f in ('Audio_Talkthrough_I2S.c', 'anc_fxnlms.c', 'anc_fxnlms.h', 'anc_ref_band.h',
                              'anc_rate.h', 'second_path.h', 'feedback_path.h', 'speaker_test.h')]
FS = 48000
PRE_S = 5.0                # recording before the runner starts
AFTER_S = 20.0             # recording after the runner exits (ANC keeps running)
TIMEOUT_S = 100.0          # runner timeout: ANC runs about 70 s while the runner is attached
MAX_S = 900.0
PROG_AFTER_RUNNER = 8.2    # program start (codec setup) after the runner start, 2026-11-17 runs: 8.10-8.30 s
ANC_AFTER_RUNNER = 28.3    # ANC start after the runner start (program start + about 20 s), same runs
F0_RANGE = (560.0, 640.0)  # fan f0 at 100% duty: 602 Hz in August, 595-603 Hz on 2026-11-17
TONE_HW = 6.0              # tone level = power within +-TONE_HW Hz
SEG = int(0.5 * FS)        # PSD segment and hop
HOP = int(0.25 * FS)
COMPONENTS = ('f0', '2 f0', '5 f0', '6 f0', '400-560 Hz', '7-12 kHz', 'total')
INK, GRID = '#222222', '#e3e3e3'
C_OFF, C_ON = '#7a7a7a', '#2a6fdb'
C_COMP = {'f0': '#159a85', '2 f0': '#d9822b', '6 f0': '#8e44ad', '7-12 kHz': '#b07d12', 'total': '#7a7a7a'}
C_TRIP = '#c0392b'


def db(v):
    return 20 * np.log10(np.maximum(np.abs(v), 1e-12))


# ---------------------------------------------------------------- board and recording

def cces_paths(arg):
    root = arg or os.environ.get('CCES_ROOT')
    if not root:
        sys.exit('Give the CrossCore Embedded Studio folder with --cces or the CCES_ROOT environment variable.')
    root = Path(root)
    runner, preload = root / 'CCES_runner.exe', root / 'SHARC' / 'ldr' / 'ezkitSC598W_preload_core1.dxe'
    for p in (runner, preload):
        if not p.exists():
            sys.exit(f'Not found: {p}')
    return runner, preload


def runner_cmd(runner, preload, timeout_s):
    return [str(runner), '--target', 'Emulation Debug Target', '--platform', 'ADSP-SC598W USB EZ-KIT via Debug Agent',
            '--processor', 'ADSP-SC598W', '-v1', '--reset-on-connect', '--timeout', str(int(timeout_s * 1000)),
            '-c', f'0,CLEAR_SYMBOLS|RUN_AFTER_LOAD|{preload},{DXE}']


def build_state():
    """(ok, text): the Release .dxe exists and is newer than every firmware source (header-only edits need a Clean)."""
    if not DXE.exists():
        return False, f'no Release build at {DXE.name}'
    t_dxe = DXE.stat().st_mtime
    newest = max((p for p in SOURCES if p.exists()), key=lambda p: p.stat().st_mtime)
    ok = t_dxe > newest.stat().st_mtime
    fmt = lambda t: time.strftime('%m/%d %H:%M:%S', time.localtime(t))  # noqa: E731
    return ok, (f'.dxe {fmt(t_dxe)}, newest source {newest.name} {fmt(newest.stat().st_mtime)}'
                + ('' if ok else '  -> STALE, build first (Clean + Build after a header-only change)'))


def check(sd, listen_s=2.0):
    """Silent. Returns (ok, info dict)."""
    ok_build, build = build_state()
    print('Build:', build)
    dev_in, _ = pm.wasapi_devices(sd, pm.DEVICE)
    vol = pm.windows_volume(dev_in['name'], capture=True)
    print(f"Input: {dev_in['name']}  Windows volume {vol}")
    if 'scalar' in vol and abs(vol['scalar'] - 0.5) > 0.01:
        print('  WARNING: the US205 mic volume should stay 0.500')
    try:
        rec = sd.rec(int(listen_s * FS), samplerate=FS, channels=1, dtype='float32', device=dev_in['index'],
                     extra_settings=sd.WasapiSettings(exclusive=True), blocking=True)[:, 0]
    except Exception as e:  # noqa: BLE001
        print(f'Audio error: {e}. Close REW or anything else that uses the US205.')
        return False, {}
    print(f'Input now: peak {db(np.max(np.abs(rec))):.1f} dBFS, rms {db(np.sqrt(np.mean(rec ** 2))):.1f} dBFS '
          '(J17 L carries the Err mic only while an ANC-mode program runs)')
    return ok_build, dict(device=dev_in['name'], windows_volume=vol, build=build, dev_index=dev_in['index'])


def run(a):
    runner, preload = cces_paths(a.cces)
    sd = pm.import_sounddevice()
    ok, info = check(sd)
    if not ok:
        sys.exit('Not running: fix the build or the audio device first.')
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    DIAG_FILE.parent.mkdir(parents=True, exist_ok=True)   # the ANC_DIAG build writes there over JTAG after a trip
    base = OUT_DIR / a.name
    for ext in ('.wav', '.log', '.json', '.png', '.txt'):
        if Path(str(base) + ext).exists():
            sys.exit(f'{base.name}{ext} exists already; pick another --name.')
    dev_index = info.pop('dev_index')
    buf = np.zeros(int(MAX_S * FS), np.float32)
    pos, marks, flags = [0], [], []

    def cb(indata, frames, t, status):
        i = pos[0]
        if status:
            flags.append({'frame': i, 'status': str(status)})
        k = min(frames, buf.size - i)
        buf[i:i + k] = indata[:k, 0]
        pos[0] = i + k
        marks.append((pos[0], time.perf_counter()))
        if pos[0] >= buf.size:
            raise sd.CallbackStop

    cmd = runner_cmd(runner, preload, a.timeout)
    t_wall0 = time.time()
    print(f'Recording from {time.strftime("%H:%M:%S")}; runner at +{PRE_S:.0f} s, program about '
          f'+{PRE_S + PROG_AFTER_RUNNER:.0f} s, ANC about +{PRE_S + ANC_AFTER_RUNNER:.0f} s:',
          subprocess.list2cmdline(cmd))
    with sd.InputStream(device=dev_index, channels=1, samplerate=FS, dtype='float32', latency=pm.LATENCY_S,
                        extra_settings=sd.WasapiSettings(exclusive=True), callback=cb):
        time.sleep(PRE_S)
        t_start = time.perf_counter()
        wall_start = time.strftime('%Y-%m-%d %H:%M:%S')
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace',
                                  timeout=a.timeout + 120)
            out, rc = proc.stdout + proc.stderr, proc.returncode
        except subprocess.TimeoutExpired as e:
            out, rc = f'{e.stdout or ""}{e.stderr or ""}\n[anc_labtest] runner timed out', None
        t_exit = time.perf_counter()
        print(f'Runner exited ({rc}) after {t_exit - t_start:.1f} s; recording {AFTER_S:.0f} s more (ANC runs on)')
        time.sleep(AFTER_S)
    n = pos[0]
    ends = np.array([m[0] for m in marks], float)
    perf = np.array([m[1] for m in marks], float)
    to_s = lambda t: float(np.interp(t, perf, ends)) / FS  # noqa: E731  wall clock -> recording time (s)
    wavfile.write(str(base) + '.wav', FS, buf[:n].copy())
    Path(str(base) + '.log').write_text(out, encoding='utf-8')
    meta = dict(fs=FS, started=wall_start, runner_start_s=round(to_s(t_start), 3),
                runner_exit_s=round(to_s(t_exit), 3), runner_rc=rc, runner_timeout_s=a.timeout,
                runner_cmd=subprocess.list2cmdline(cmd), xruns=flags, source=pm.git_state(), notes=a.notes, **info)
    Path(str(base) + '.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')
    print('Saved', base.name, '.wav/.log/.json')
    if DIAG_FILE.exists() and DIAG_FILE.stat().st_mtime > t_wall0:
        dst = Path(str(base) + ' snapshot.txt')
        DIAG_FILE.replace(dst)
        print('ANC_DIAG snapshot:', dst.name)
    analyze(Path(str(base) + '.wav'))
    if Path(str(base) + ' snapshot.txt').exists():
        diag(Path(str(base) + '.wav'))


# ---------------------------------------------------------------- analysis

def frozen_free(x, a, b):
    """Mask over [a, b) s: False inside stretches whose 50 ms rms falls more than 3 dB under the median (J17 L frozen
    by a status print), with 50 ms margins."""
    seg = x[int(a * FS):int(b * FS)]
    n = int(0.05 * FS)
    k = seg.size // n
    if k < 3:
        return np.ones(seg.size, bool)
    r = 10 * np.log10(np.mean(seg[:k * n].reshape(k, n) ** 2, axis=1) + 1e-30)
    bad = np.convolve(r < np.median(r) - 3.0, np.ones(3), 'same') > 0
    m = np.repeat(~bad, n)
    return np.concatenate([m, np.ones(seg.size - m.size, bool)])


def mean_psd(x, a, b, skip_frozen=True):
    """Mean one-sided PSD (FS^2/Hz) of 0.5 s Hann segments in [a, b) s; with skip_frozen, segments that touch a frozen
    stretch are left out. Returns f, psd, number of segments."""
    seg = x[max(int(a * FS), 0):int(b * FS)]
    ok = frozen_free(x, a, b) if skip_frozen else np.ones(seg.size, bool)
    w = np.hanning(SEG)
    norm = FS * np.sum(w ** 2)
    acc, n = np.zeros(SEG // 2 + 1), 0
    for i in range(0, seg.size - SEG + 1, HOP):
        if ok[i:i + SEG].all():
            acc += 2 * np.abs(np.fft.rfft(seg[i:i + SEG] * w)) ** 2 / norm
            n += 1
    return np.fft.rfftfreq(SEG, 1 / FS), acc / max(n, 1), n


def find_f0(f, p, lo, hi):
    """Strongest narrow peak in [lo, hi] Hz (PSD over its +-40 Hz running median), refined by parabolic interpolation."""
    from scipy.signal import medfilt
    df = f[1] - f[0]
    med = medfilt(p, int(80 / df) | 1)
    m = np.flatnonzero((f >= lo) & (f <= hi))
    i = m[np.argmax(p[m] / np.maximum(med[m], 1e-30))]
    y0, y1, y2 = np.log(p[i - 1:i + 2] + 1e-30)
    d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
    return float(f[i] + d * df), float(10 * np.log10(p[i] / max(med[i], 1e-30)))


def components(f, p, f0):
    df = f[1] - f[0]
    band = lambda lo, hi: float(np.sum(p[(f >= lo) & (f < hi)]) * df)   # noqa: E731
    tone = lambda fc: band(fc - TONE_HW, fc + TONE_HW + 1e-9)           # noqa: E731
    return {'f0': tone(f0), '2 f0': tone(2 * f0), '5 f0': tone(5 * f0), '6 f0': tone(6 * f0),
            '400-560 Hz': band(400, 560), '7-12 kHz': band(7000, 12000), 'total': band(60, 20000)}


BAND_RE = re.compile(r'Ref band (\d+)-(\d+) Hz')                # firmware banner
TRIP_RE = re.compile(r'Diverged ([\d.]+) s after ANC started')   # first [ANC TRIP] line of the divergence guard
OFF_RE = re.compile(r'Output switched off ([\d.]+) s after ANC started')   # [ANC OFF] line of ANC_AUTO_OFF_S


def parse_log(text):
    lines = text.splitlines()
    band, trip, off = BAND_RE.search(text), TRIP_RE.search(text), OFF_RE.search(text)
    return dict(banner=[l for l in lines if l.startswith('>>>')],
                step0=next((l for l in lines if l.startswith('[Step 0]')), None),
                ready=any('SYSTEM READY' in l for l in lines),
                status=[l for l in lines if re.match(r'\[ANC #\d+\]', l)],
                trip=[l for l in lines if '[ANC TRIP]' in l],
                off=[l for l in lines if '[ANC OFF]' in l],
                diag=sum(1 for l in lines if l.startswith('[DIAG] t=')),
                band=(int(band[1]), int(band[2])) if band else None,
                trip_s=float(trip[1]) if trip else None,
                off_s=float(off[1]) if off else None)


def load_run(wav):
    wav = Path(wav)
    base = Path(str(wav)[:-4])
    fs, x = wavfile.read(str(wav))
    assert fs == FS, fs
    meta = json.loads(Path(str(base) + '.json').read_text(encoding='utf-8'))
    logp = Path(str(base) + '.log')
    log = parse_log(logp.read_text(encoding='utf-8', errors='replace') if logp.exists() else '')
    return base, x.astype(np.float64), meta, log


def measure(wav, f0=None):
    """PSDs, f0, the attenuation per window and the time course of one run, for analyze and report.

    ANC on ends 0.5 s before a divergence trip (the guard muted the speaker there) or a timed switch-off."""
    base, x, meta, log = load_run(wav)
    dur = x.size / FS
    t_r, t_x = meta['runner_start_s'], meta['runner_exit_s']
    ta = meta.get('anc_start_s', t_r + ANC_AFTER_RUNNER)
    stops = []   # (time in the recording, marker label, window after it, text)
    if log['trip_s'] is not None:
        stops.append((ta + log['trip_s'], 'guard trip', 'after the trip (muted)',
                      f'The guard tripped {log["trip_s"]:.1f} s after the ANC start'))
    if log['off_s'] is not None:
        stops.append((ta + log['off_s'], 'switch-off', 'after the switch-off',
                      f'The output was switched off {log["off_s"]:.1f} s after the ANC start'))
    t_trip, stop_label, after_key, stop_text = min(stops) if stops else (None, None, None, None)
    f, p_off, n_off = mean_psd(x, ta - 10.5, ta - 0.5)
    if f0 is None:
        f0, prom = find_f0(f, p_off, *F0_RANGE)
        f0_text = f'f0 {f0:.2f} Hz (strongest peak in {F0_RANGE[0]:.0f}-{F0_RANGE[1]:.0f} Hz, {prom:.0f} dB over its surroundings)'
    else:
        f0_text = f'f0 {f0:.2f} Hz (given)'
    c_off = components(f, p_off, f0)
    stop = t_x - 2.0 if t_trip is None else min(t_x - 2.0, t_trip - 0.5)
    win = {'first 5 s': (ta + 0.5, min(ta + 5.5, stop)), 'steady (attached)': (ta + 10.0, stop)}
    if t_trip is None:
        win['after the runner exit'] = (t_x + 2.0, dur - 1.0)
    else:
        win[after_key] = (t_trip + 1.0, dur - 1.0)   # status prints are back: skip frozen stretches
    win = {k: v for k, v in win.items() if v[1] - v[0] >= 3.0}
    spec, nseg, res = {'ANC off': p_off}, {'ANC off': n_off}, {}
    for k, (a, b) in win.items():
        _, p, n = mean_psd(x, a, b, skip_frozen=(k == after_key))
        spec[k], nseg[k] = p, n
        c = components(f, p, f0)
        res[k] = {q: 10 * np.log10(c_off[q] / max(c[q], 1e-30)) for q in COMPONENTS}
    on_key = next((k for k in ('steady (attached)', 'first 5 s') if k in win), None)
    # time course: 2 s windows, 1 s hop; status prints freeze J17 L before the ANC start and after a trip
    tt, rows = [], []
    for a in np.arange(ta - 10.0, dur - 2.0, 1.0):
        if ta - 2.0 < a < ta:
            continue
        _, p, n = mean_psd(x, a, a + 2.0, skip_frozen=a < ta or (t_trip is not None and a + 2.0 > t_trip))
        if n == 0:
            continue
        c = components(f, p, f0)
        tt.append(a + 1.0 - ta)
        rows.append([10 * np.log10(c_off[q] / max(c[q], 1e-30)) for q in COMPONENTS])
    rows = np.array(rows).reshape(-1, len(COMPONENTS))
    return dict(base=base, meta=meta, log=log, dur=dur, t_r=t_r, t_x=t_x, ta=ta, t_trip=t_trip, stop_label=stop_label,
                stop_text=stop_text, f=f, f0=f0, f0_text=f0_text, c_off=c_off, win=win, spec=spec, nseg=nseg, res=res, on_key=on_key,
                tt=np.array(tt), rows={q: rows[:, i] for i, q in enumerate(COMPONENTS)})


def analyze(wav, f0=None):
    m = measure(wav, f0)
    base, meta, log, ta, t_r, c_off = m['base'], m['meta'], m['log'], m['ta'], m['t_r'], m['c_off']
    L = [f'{Path(wav).name}: {m["dur"]:.1f} s; runner start {t_r:.2f} s, exit {m["t_x"]:.2f} s; ANC start taken as '
         f'{ta:.2f} s (runner + {ta - t_r:.1f} s)', f'Notes: {meta.get("notes", "")}', m['f0_text'],
         f'ANC off: {ta - 10.5:.1f}-{ta - 0.5:.1f} s, {m["nseg"]["ANC off"]} segments of 0.5 s without frozen stretches',
         'Attenuation, off - on (dB, above 0 = quieter); tones = power within +-6 Hz:',
         '  window                          ' + ''.join(f'{c:>11s}' for c in COMPONENTS)]
    for k, (a, b) in m['win'].items():
        L.append(f'  {k:22s} {a:6.1f}-{b:6.1f}' + ''.join(f'{m["res"][k][q]:+11.1f}' for q in COMPONENTS))
    if m['t_trip'] is not None:
        L.append(f'  {m["stop_text"]} ({m["t_trip"]:.1f} s): ANC on ends 0.5 s before it.')
    L.append('  ANC off levels (dBFS on the US205; tones as amplitude): '
             + ', '.join(f'{q} {10 * np.log10((2 if "f0" in q else 1) * c_off[q] + 1e-30):.1f}' for q in COMPONENTS))
    L.append('Log:')
    L += ['  ' + l for l in log['banner'][:7]]
    L += ['  ' + l for l in log['banner'][7:] if 'idle level' in l]   # pushbutton idle levels (ANC_BUTTONS)
    if log['step0']:
        L.append('  ' + log['step0'])
    L.append(f"  SYSTEM READY: {'yes' if log['ready'] else 'NO'}; runner rc {meta.get('runner_rc')}")
    L += ['  ' + l for l in log['status'][-2:]]
    L.append('  ' + ('; '.join(log['trip']) if log['trip'] else 'no [ANC TRIP] line while the runner was attached'))
    if log['off']:
        L.append('  ' + '; '.join(log['off']))
    if log['diag']:
        L.append(f'  {log["diag"]} [DIAG] lines: run "anc_labtest.py diag" on this wav')
    txt = '\n'.join(L)
    print(txt)
    Path(str(base) + '.txt').write_text(txt + '\n', encoding='utf-8')
    plot(base, m)
    print('Plot:', base.name + '.png')
    return m['res']


def plot(base, m):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    f, spec, f0, tt, t_exit = m['f'], m['spec'], m['f0'], m['tt'], m['t_x'] - m['ta']
    plt.rcParams.update({'font.size': 11, 'axes.edgecolor': '#999999', 'axes.labelcolor': INK, 'xtick.color': INK,
                         'ytick.color': INK, 'axes.titlelocation': 'left'})
    fig = plt.figure(figsize=(13, 9.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.1])
    ax = fig.add_subplot(gs[0, :])
    if len(tt):
        for q, c in C_COMP.items():
            ax.plot(tt, m['rows'][q], color=c, lw=1.6 if q != 'total' else 1.2, label=q)
    ax.axvline(0, color='#555555', lw=0.8, ls='--')
    ax.text(0, 0.98, ' ANC on', transform=ax.get_xaxis_transform(), va='top', fontsize=10, color='#444444')
    ax.axvline(t_exit, color='#999999', lw=0.8, ls=':')
    ax.text(t_exit, 0.98, ' runner exit', transform=ax.get_xaxis_transform(), va='top', fontsize=10, color='#666666')
    if m['t_trip'] is not None:
        t = m['t_trip'] - m['ta']
        ax.axvline(t, color=C_TRIP, lw=0.9, ls='--')
        ax.text(t, 0.90, ' ' + m['stop_label'], transform=ax.get_xaxis_transform(), va='top', fontsize=10,
                color='#444444')
    ax.axhline(0, color='#888888', lw=0.8)
    ax.set_xlabel('time from the ANC start (s), 2 s windows')
    ax.set_ylabel('Err attenuation, off - on (dB)')
    ax.set_title(f'Err mic attenuation over time (f0 = {f0:.1f} Hz)')
    ax.grid(True, lw=0.5, color=GRID)
    ax.legend(frameon=False, ncol=5, loc='lower left')
    on_key = m['on_key'] or list(spec)[-1]
    for j, (lo, hi, logx, title) in enumerate(((100, 12000, True, 'Err spectrum'),
                                               (f0 - 30, f0 + 30, False, f'around f0 = {f0:.1f} Hz'))):
        a2 = fig.add_subplot(gs[1, j])
        mm = (f >= lo) & (f <= hi)
        for k, c, lw in (('ANC off', C_OFF, 1.2), (on_key, C_ON, 1.0)):
            a2.plot(f[mm], 10 * np.log10(spec[k][mm] + 1e-30), color=c, lw=lw, label=f'{k}')
        if logx:
            a2.set_xscale('log')
        a2.set_xlabel('Hz')
        a2.set_ylabel('PSD, dB re FS^2/Hz')
        a2.set_title(title)
        a2.grid(True, which='both', lw=0.4, color=GRID)
        a2.legend(frameon=False, fontsize=9, loc='upper right')
    fig.tight_layout()
    fig.savefig(str(base) + '.png', dpi=90)
    plt.close(fig)


# ---------------------------------------------------------------- slide figures

C_RUN = ('#2a6fdb', '#e8702a', '#5b2c83')    # runs in this fixed order; OKLab CVD checks pass for every pair and C_OFF
C_TIME = {'f0': '#159a85', '6 f0': '#8e44ad', '7-12 kHz': '#b07d12', 'total': '#3d3d3d'}
SLIDE_RC = {'font.size': 15, 'axes.edgecolor': '#999999', 'axes.labelcolor': INK, 'xtick.color': INK,
            'ytick.color': INK, 'axes.titlelocation': 'left', 'axes.titlesize': 16}
SPEC_TICKS = (100, 200, 500, 1000, 2000, 5000, 10000, 20000)
SMOOTH_OCT = 24                              # off - on panel of the spectrum figure: 1/24-octave bands


def slide_names(f0):
    """Names of COMPONENTS on the slides: '600 Hz (f0)', '3.6 kHz (6 f0)', '7-12 kHz band', 'total'."""
    def hz(v):
        return f'{round(v, -1):.0f} Hz' if v < 1000 else f'{v / 1000:.1f} kHz'
    return {q: f'{hz((int(q.split()[0]) if " " in q else 1) * f0)} ({q})' if 'f0' in q
            else (q if q == 'total' else f'{q} band') for q in COMPONENTS}


def panel_title(ax, label, n):
    """Run label over one of n side-by-side panels, wrapped so that neighbouring titles do not overlap."""
    import textwrap
    ax.set_title(textwrap.fill(label, 60 // n), fontsize=16 if n < 3 else 13)


def octave_avg(f, p, frac=SMOOTH_OCT, lo=60.0, hi=20000.0):
    """Mean PSD over 1/frac-octave bands, centred on a 1/(2 frac)-octave grid from lo to hi Hz."""
    fc = lo * 2 ** (np.arange(int(np.log2(hi / lo) * 2 * frac) + 1) / (2 * frac))
    c = np.concatenate([[0.0], np.cumsum(p)])
    il = np.searchsorted(f, fc * 2 ** (-0.5 / frac))
    ih = np.maximum(np.searchsorted(f, fc * 2 ** (0.5 / frac)), il + 1)
    return fc, (c[ih] - c[il]) / (ih - il)


def report(wavs, labels=None, f0s=None, out=None, name='anc_lab', title=None):
    """Slide figures for 1-3 runs: <name>_bars.png (attenuation per component), <name>_spectrum.png (Err spectrum,
    ANC off and on, with off - on per 1/24 octave), <name>_time.png (attenuation over time) and <name>.txt."""
    if len(wavs) > len(C_RUN):
        sys.exit(f'At most {len(C_RUN)} runs per figure.')
    for opt, v in (('--labels', labels), ('--f0', f0s)):
        if v and len(v) != len(wavs):
            sys.exit(f'Give one {opt} value per wav.')
    runs = [measure(w, f0s[i] if f0s else None) for i, w in enumerate(wavs)]
    for m in runs:
        if m['on_key'] is None:
            sys.exit(f'{m["base"].name}: no ANC-on window of 3 s or more (the guard tripped too early).')
    if not labels:
        labels = [f'ref band-pass {m["log"]["band"][0]}-{m["log"]["band"][1]} Hz' if m['log']['band']
                  else m['base'].name for m in runs]
        if len(set(labels)) < len(labels):
            labels = [m['base'].name for m in runs]
    out = Path(out) if out else Path(wavs[0]).resolve().parent
    out.mkdir(parents=True, exist_ok=True)
    names = slide_names(float(np.mean([m['f0'] for m in runs])))
    steady = all(m['on_key'] == 'steady (attached)' for m in runs)
    L = [f'anc_labtest.py report, {time.strftime("%Y-%m-%d %H:%M")}']
    for lab, m in zip(labels, runs):
        a, b = m['win'][m['on_key']]
        L += [f'{lab}: {m["base"].name}.wav', f'  {m["f0_text"]}',
              f'  ANC off {m["ta"] - 10.5:.1f}-{m["ta"] - 0.5:.1f} s ({m["nseg"]["ANC off"]} segments of 0.5 s); '
              f'ANC on = {m["on_key"]} {a:.1f}-{b:.1f} s ({m["nseg"][m["on_key"]]} segments)']
        if m['t_trip'] is not None:
            L.append(f'  {m["stop_text"]}; ANC on ends 0.5 s before it.')
    w = max(len(l) for l in labels)
    L += ['Attenuation, off - on (dB, above 0 = quieter); tones = power within +-6 Hz:',
          '  ' + ' ' * w + ''.join(f'{q:>11s}' for q in COMPONENTS)]
    L += [f'  {lab:{w}s}' + ''.join(f'{m["res"][m["on_key"]][q]:+11.1f}' for q in COMPONENTS)
          for lab, m in zip(labels, runs)]
    L.append(f'Spectrum figure: 0.5 s Hann segments (2 Hz); off - on over 1/{SMOOTH_OCT}-octave bands. '
             'Figures: ' + ', '.join(f'{name}_{k}.png' for k in ('bars', 'spectrum', 'time')))
    txt = '\n'.join(L)
    print(txt)
    (out / f'{name}.txt').write_text(txt + '\n', encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with plt.rc_context(SLIDE_RC):
        report_bars(plt, runs, labels, names, out / f'{name}_bars.png',
                    title or ('Steady state from 10 s after the ANC start (above 0 = quieter)' if steady
                              else 'ANC on, before the trip or switch-off where there is one (above 0 = quieter)'))
        report_spectrum(plt, runs, labels, out / f'{name}_spectrum.png')
        report_time(plt, runs, labels, names, out / f'{name}_time.png')
    print('Figures in', out)


def report_bars(plt, runs, labels, names, path, title):
    keys = list(COMPONENTS)
    xs = np.arange(len(keys))
    n = len(runs)
    wbar = min(0.5, 0.76 / n)
    vals = [np.array([m['res'][m['on_key']][q] for q in keys]) for m in runs]
    fig, ax = plt.subplots(figsize=(13.2, 4.7))
    for j, (v, lab) in enumerate(zip(vals, labels)):
        xb = xs + (j - (n - 1) / 2) * wbar
        ax.bar(xb, v, wbar * 0.94, color=C_RUN[j], label=lab)
        for xx, vv in zip(xb, v):
            ax.text(xx, vv + (0.4 if vv >= 0 else -0.4), f'{vv:+.1f}', ha='center', va='bottom' if vv >= 0 else 'top',
                    fontsize=12.5 if n < 3 else 11, color=INK)
    ax.axhline(0, color='#888888', lw=0.9)
    ax.set_xticks(xs)
    ax.set_xticklabels([names[q].replace(' (', '\n(').replace(' band', '\nband') for q in keys])
    ax.set_ylabel('Err attenuation, off - on (dB)')
    lo, hi = min(v.min() for v in vals), max(v.max() for v in vals)
    ax.set_ylim(min(-8, lo - 2.5), hi + 3)
    ax.set_title(title)
    ax.grid(True, axis='y', lw=0.6, color=GRID)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, loc='upper right')
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def report_spectrum(plt, runs, labels, path):
    from matplotlib.ticker import FixedFormatter, FixedLocator, MultipleLocator, NullLocator
    n = len(runs)
    fig, axs = plt.subplots(2, n, figsize=(13.2, 5.2), sharex=True, sharey='row', squeeze=False,   # slide + 3 bullets
                            gridspec_kw={'height_ratios': [1.8, 1]})
    for j, (m, lab) in enumerate(zip(runs, labels)):
        f, p_off, p_on = m['f'], m['spec']['ANC off'], m['spec'][m['on_key']]
        sel = (f >= 60) & (f <= 20000)
        top, bot = axs[0, j], axs[1, j]
        for ax in (top, bot):
            if m['log']['band']:
                ax.axvspan(*m['log']['band'], color=INK, alpha=0.06, lw=0)
            for k in (1, 2, 5, 6):
                ax.axvline(k * m['f0'], color='#bbbbbb', lw=0.8, ls=':', zorder=0)
            ax.grid(True, lw=0.5, color=GRID)
            for s in ('top', 'right'):
                ax.spines[s].set_visible(False)
        top.plot(f[sel], 10 * np.log10(p_off[sel] + 1e-30), color=C_OFF, lw=0.9, label='ANC off')
        top.plot(f[sel], 10 * np.log10(p_on[sel] + 1e-30), color=C_RUN[j], lw=0.9, label='ANC on')
        for k in (1, 2, 5, 6):          # 6 f0 is labelled right of its line, the others left, so 5 f0 and 6 f0 part
            top.text(k * m['f0'], 0.99, f'{k} f0' if k > 1 else 'f0', transform=top.get_xaxis_transform(),
                     rotation=90, ha='left' if k == 6 else 'right', va='top', fontsize=11, color='#666666')
        if m['log']['band']:
            top.text(np.sqrt(np.prod(m['log']['band'])), 0.02, 'ref band', transform=top.get_xaxis_transform(),
                     ha='center', va='bottom', fontsize=11, color='#666666')
        panel_title(top, lab, n)
        top.legend(frameon=False, loc='lower left', fontsize=13)
        fc, a_off = octave_avg(f, p_off)
        _, a_on = octave_avg(f, p_on)
        d = 10 * np.log10(a_off / np.maximum(a_on, 1e-30))
        bot.fill_between(fc, 0, d, color=C_RUN[j], alpha=0.18, lw=0)
        bot.plot(fc, d, color=C_RUN[j], lw=1.5)
        bot.axhline(0, color='#888888', lw=0.8)
        bot.text(0.99, 0.97, f'1/{SMOOTH_OCT} octave, above 0 = quieter', transform=bot.transAxes, ha='right',
                 va='top', fontsize=11, color='#666666')
        bot.set_xlabel('Hz')
    ax = axs[1, 0]
    ax.set_xscale('log')
    ax.set_xlim(60, 20000)
    ax.xaxis.set_major_locator(FixedLocator(SPEC_TICKS))
    ax.xaxis.set_major_formatter(FixedFormatter([f'{t // 1000}k' if t >= 1000 else str(t) for t in SPEC_TICKS]))
    ax.xaxis.set_minor_locator(NullLocator())
    axs[0, 0].yaxis.set_major_locator(MultipleLocator(10))
    axs[0, 0].set_ylabel('Err PSD (dB re FS²/Hz)')
    axs[1, 0].set_ylabel('off - on (dB)')
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def report_time(plt, runs, labels, names, path):
    n = len(runs)
    spans = [max(float(m['tt'][-1] - m['tt'][0]), 1.0) if len(m['tt']) else 1.0 for m in runs]
    fig, axs = plt.subplots(1, n, figsize=(13.2, 4.7), sharey=True, squeeze=False,
                            gridspec_kw={'width_ratios': spans})
    lo, hi = -9.0, 24.0
    for ax, m, lab in zip(axs[0], runs, labels):
        for q, c in C_TIME.items():
            v = m['rows'][q]
            ax.plot(m['tt'], v, color=c, lw=1.8 if q != 'total' else 1.4, label=names[q])
            if len(v):
                lo, hi = min(lo, float(v.min()) - 2), max(hi, float(v.max()) + 2)
        ax.axvline(0, color='#555555', lw=0.9, ls='--')
        ax.text(0.5, 0.97, ' ANC on', transform=ax.get_xaxis_transform(), va='top', fontsize=13, color='#444444')
        if m['t_trip'] is not None:
            t = m['t_trip'] - m['ta']
            ax.axvline(t, color=C_TRIP, lw=0.9, ls='--')
            ax.text(t, 0.97, ' ' + m['stop_label'], transform=ax.get_xaxis_transform(), va='top', fontsize=13,
                    color='#444444')
        ax.axhline(0, color='#888888', lw=0.8)
        panel_title(ax, lab, n)
        ax.set_xlabel('time from the ANC start (s)')
        ax.grid(True, lw=0.6, color=GRID)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
    axs[0, 0].set_ylabel('Err attenuation, off - on (dB)')
    axs[0, 0].set_ylim(lo, hi)
    h, l = axs[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc='lower center', ncol=4, frameon=False, fontsize=14)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ---------------------------------------------------------------- ANC_DIAG output

DIAG_RE = re.compile(r'\[DIAG\] t=\s*([-\d.]+) y_pk=\s*([-\d.]+) clip=\s*([-\d.]+) x=\s*([-\d.]+) Fy=\s*([-\d.]+) '
                     r'u=\s*([-\d.]+) y=\s*([-\d.]+) e=\s*([-\d.]+) W2=\s*([-\d.eE+]+)')
DIAG_NAMES = ['x (Ref)', 'Fy (feedback est.)', 'u (into W)', 'y (before clamp)', 'e (Err)']
DIAG_COLS = ['#7a7a7a', '#d9822b', '#2a6fdb', '#8e44ad', '#17a589']


def diag(wav, clamp=0.9):
    """[DIAG] status history from the log and the snapshot file of an ANC_DIAG build after a trip."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    base = Path(str(wav)[:-4])
    log = Path(str(base) + '.log').read_text(encoding='utf-8', errors='replace')
    H = np.array([[float(v) for v in m.groups()] for m in DIAG_RE.finditer(log)])
    L = [f'{base.name}: {len(H)} [DIAG] records']
    if len(H):
        L.append('   t (s)   y_pk   clip    x     Fy     u      y      e     (dB re FS)   W2')
        for i in sorted(set(list(range(min(4, len(H)))) + list(range(max(4, len(H) - 12), len(H))))):
            t, ypk, clip, x, fy, u, y, e, w2 = H[i]
            L.append(f'  {t:6.2f}  {ypk:6.3f}  {clip:4.2f}  {x:6.1f} {fy:6.1f} {u:6.1f} {y:6.1f} {e:6.1f}   {w2:.3e}')
    sp = Path(str(base) + ' snapshot.txt')
    S, expected = None, None
    if sp.exists():
        # The runner may end while the board is still writing (JTAG file I/O is slow): drop an unfinished last line
        text = sp.read_text(encoding='utf-8', errors='replace')
        lines = text.splitlines() if text.endswith('\n') else text.splitlines()[:-1]
        head = re.search(r'the last (\d+) ANC core samples', text)
        expected = int(head[1]) if head else None
        rows = []
        for l in lines:
            parts = l.split()
            if l.startswith('#') or len(parts) != 5:
                continue
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                continue
        S = np.array(rows) if rows else None
    if S is not None:
        n = len(S)
        L.append(f'Snapshot: {n} samples ({1e3 * n / FS:.1f} ms) up to the trip or switch-off'
                 + (f'; PARTIAL, the board was writing {expected} when the runner ended' if expected and n < expected
                    else ''))
        for k, nm in enumerate(DIAG_NAMES):
            v = S[:, k]
            L.append(f'  {nm:20s} rms {db(np.sqrt(np.mean(v ** 2))):6.1f} dB  peak {np.max(np.abs(v)):.4f}')
        y = S[:, 3]
        L.append(f'  y over the clamp ({clamp}): {100 * np.mean(np.abs(y) > clamp):.1f}% of the snapshot, '
                 f'{100 * np.mean(np.abs(y[-2400:]) > clamp):.1f}% of the last 50 ms')
        w = np.hanning(n)
        f = np.fft.rfftfreq(n, 1 / FS)
        spec = [np.abs(np.fft.rfft(S[:, k] * w)) * 2 / np.sum(w) for k in range(5)]
        m = (f > 100) & (f < 12000)
        L.append('  strongest components of y (amplitude), with u, x, Fy, e at the same bin:')
        seen = []
        for i in np.argsort(-np.where(m, spec[3], 0)):
            if any(abs(f[i] - s) < 20 for s in seen):
                continue
            seen.append(f[i])
            L.append(f'    {f[i]:7.1f} Hz: y {spec[3][i]:.4f}  u {spec[2][i]:.2e}  x {spec[0][i]:.2e}  '
                     f'Fy {spec[1][i]:.2e}  e {spec[4][i]:.2e}   |y/u| {db(spec[3][i] / max(spec[2][i], 1e-30)):5.1f} dB')
            if len(seen) >= 6:
                break
    txt = '\n'.join(L)
    print(txt)
    Path(str(base) + ' diag.txt').write_text(txt + '\n', encoding='utf-8')
    fig = plt.figure(figsize=(13, 12))
    gs = fig.add_gridspec(4, 2, height_ratios=[1, 1, 1.2, 1])
    if len(H):
        t = H[:, 0]
        panels = [(gs[0, 0], H[:, 1], 'pre-clamp |y| peak per record'), (gs[0, 1], 100 * H[:, 2], 'samples clamped, %')]
        for g, v, title in panels:
            a = fig.add_subplot(g)
            a.plot(t, v, color=DIAG_COLS[3], lw=1.2)
            a.set_title(title, loc='left')
            a.set_xlabel('s after the ANC start')
            a.grid(True, lw=0.5, color=GRID)
        a3 = fig.add_subplot(gs[1, 0])
        for k, c in zip(range(3, 8), DIAG_COLS):
            a3.plot(t, H[:, k], color=c, lw=1.1, label=DIAG_NAMES[k - 3].split()[0])
        a3.set_title('levels per record, dB re FS (y after the clamp)', loc='left')
        a3.legend(frameon=False, ncol=5, fontsize=8, loc='lower left')
        a3.grid(True, lw=0.5, color=GRID)
        a4 = fig.add_subplot(gs[1, 1])
        a4.semilogy(t, H[:, 8], color=INK, lw=1.2)
        a4.set_title('W2 = sum w^2', loc='left')
        a4.grid(True, lw=0.5, color=GRID)
    if S is not None:
        a5 = fig.add_subplot(gs[2, :])
        for k in range(5):
            a5.semilogx(f[m], db(spec[k][m]), color=DIAG_COLS[k], lw=1.0, label=DIAG_NAMES[k])
        top = max(db(np.max(s[m])) for s in spec)
        a5.set_ylim(top - 130, top + 5)
        a5.set_title(f'snapshot spectra, last {1e3 * n / FS:.0f} ms before the trip or switch-off (amplitude, dB re FS)',
                     loc='left')
        a5.set_xlabel('Hz')
        a5.legend(frameon=False, ncol=5, fontsize=8, loc='lower left')
        a5.grid(True, which='both', lw=0.4, color=GRID)
        a6 = fig.add_subplot(gs[3, :])
        k0 = max(0, n - 960)
        tt = (np.arange(k0, n) - n) / FS * 1e3
        a6.plot(tt, S[k0:, 3], color=DIAG_COLS[3], lw=0.9, label='y before the clamp')
        a6.plot(tt, np.clip(S[k0:, 3], -clamp, clamp), color=INK, lw=0.7, label='y sent (clamped)')
        a6.set_title('y in the last 20 ms before the trip or switch-off', loc='left')
        a6.set_xlabel('ms before the trip or switch-off')
        a6.legend(frameon=False, fontsize=8, loc='lower left')
        a6.grid(True, lw=0.5, color=GRID)
    fig.tight_layout()
    fig.savefig(str(base) + ' diag.png', dpi=90)
    plt.close(fig)
    print('Plot:', base.name + ' diag.png')


# ---------------------------------------------------------------- synthetic test

def synth(path):
    """Fan-like noise with f0 and 6 f0 tones; f0 drops 22 dB with a 0.4 s time constant at the ANC start; two
    0.45 s print freezes in the ANC-off window, which the analysis must leave out."""
    from scipy.signal import butter, lfilter
    rng = np.random.default_rng(1)
    t_r, f0 = PRE_S, 602.37
    t_x = t_r + TIMEOUT_S + 7.6
    n = int((t_x + AFTER_S) * FS)
    t = np.arange(n) / FS
    b, a_ = butter(2, [80, 8000], 'bandpass', fs=FS)
    x = lfilter(b, a_, rng.standard_normal(n)) * 10 ** (-58 / 20)
    t_anc = t_r + ANC_AFTER_RUNNER
    g = np.where(t < t_anc, 1.0, 10 ** (-22 / 20) + (1 - 10 ** (-22 / 20)) * np.exp(-(t - t_anc) / 0.4))
    x += 10 ** (-60 / 20) * g * np.sin(2 * np.pi * f0 * t) + 10 ** (-57 / 20) * np.sin(2 * np.pi * 6 * f0 * t + 1)
    for tf in (t_anc - 3.2, t_anc - 1.2):    # print freezes: the held output decays through the US205 AC coupling
        i0, i1 = int(tf * FS), int((tf + 0.45) * FS)
        x[i0:i1] = x[i0] * np.exp(-np.arange(i1 - i0) / (0.0015 * FS))
    p = Path(path)
    wavfile.write(str(p), FS, x.astype(np.float32))
    base = Path(str(p)[:-4])
    Path(str(base) + '.json').write_text(json.dumps(dict(fs=FS, runner_start_s=t_r, runner_exit_s=t_x, runner_rc=0,
                                                         notes='synthetic: f0 -22 dB, 6 f0 unchanged')),
                                         encoding='utf-8')
    Path(str(base) + '.log').write_text('[Step 0] SPORT4/DMA10-11 idle at boot\n>>> SYSTEM READY\n'
                                        '>>> Ref band 400-1500 Hz, mu 0.01, eps 1e-11, clamp 0.9\n', encoding='utf-8')
    print('wrote', p, '(expect f0 about +21 dB, limited by the noise within +-6 Hz; 6 f0 and the bands 0 dB; '
          'total about +1 dB)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('check')
    r = sub.add_parser('run')
    r.add_argument('--name', required=True, help='"MMDD ANC ..." file name in measurements/anc_test/')
    r.add_argument('--notes', default='', help='conditions: fan duty, band, who is in front of the outlet, ...')
    r.add_argument('--timeout', type=float, default=TIMEOUT_S, help='runner timeout, s (the board runs on after it)')
    r.add_argument('--cces', help='CrossCore Embedded Studio folder (default: CCES_ROOT)')
    z = sub.add_parser('analyze')
    z.add_argument('wav')
    z.add_argument('--f0', type=float, help='fan f0 in Hz (default: the strongest peak in 560-640 Hz)')
    p = sub.add_parser('report', help='slide figures: bars, Err spectrum off/on, time course')
    p.add_argument('wav', nargs='+', help='1-3 runs, compared in this order')
    p.add_argument('--labels', nargs='+', help='one per wav (default: "ref band-pass <band>" from the log)')
    p.add_argument('--f0', type=float, nargs='+', help='fan f0 in Hz, one per wav (default: found per run)')
    p.add_argument('--out', help='folder for the figures (default: the folder of the first wav)')
    p.add_argument('--name', default='anc_lab', help='file prefix: <name>_bars.png, _spectrum.png, _time.png, .txt')
    p.add_argument('--title', help='title of the bar chart')
    d = sub.add_parser('diag')
    d.add_argument('wav')
    s = sub.add_parser('synth')
    s.add_argument('wav')
    a = ap.parse_args()
    if a.cmd == 'check':
        ok, _ = check(pm.import_sounddevice())
        print('READY' if ok else 'NOT READY')
    elif a.cmd == 'run':
        run(a)
    elif a.cmd == 'analyze':
        analyze(a.wav, a.f0)
    elif a.cmd == 'report':
        report(a.wav, a.labels, a.f0, a.out, a.name, a.title)
    elif a.cmd == 'diag':
        diag(a.wav)
    else:
        synth(a.wav)


if __name__ == '__main__':
    main()
