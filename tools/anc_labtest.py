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
                                  [--buttons]            pushbutton test: SW4 / SW3 edges and convergence (below)
  python tools/anc_labtest.py analyze <wav> [--f0 HZ] [--buttons]   again, from the saved files
  python tools/anc_labtest.py report <wav> [<wav> ...] [--labels ...] [--out DIR] [--name PREFIX]
                                                         slide figures for 1-3 runs (below)
  python tools/anc_labtest.py diag <wav>                 ANC_DIAG output after a divergence trip ([DIAG] lines, snapshot)
  python tools/anc_labtest.py track <wav> [--step S]     second by second: fan f0 and tones, frozen J17 L (dumps)
  python tools/anc_labtest.py pickup <wav> [--f0 HZ]     fan lines inside vs outside the wait-phase print freezes:
                                                         through the core (ADC side) or after the DAC
  python tools/anc_labtest.py listen --name "..." [--seconds 10] [--f0 HZ] [--notes "..."] [--windows a-b,c-d]
                                     [--course]          record J17 L without loading anything: the fan lines now,
                                                         per stretch (e.g. output off, then on) and second by second
  python tools/anc_labtest.py listen --wav <wav> [--windows a-b,c-d] [--course]   the same for an earlier recording
  python tools/anc_labtest.py synth <wav>                synthetic recording to test the analysis

The CrossCore Embedded Studio folder (CCES_runner.exe, SHARC/ldr/ezkitSC598W_preload_core1.dxe) comes from --cces or
the CCES_ROOT environment variable.

Data go to measurements/anc_test/ (gitignored): "<name>".wav (mono float32), .log (runner output), .json (times,
notes, devices, Windows volumes, firmware source state), .txt and .png (analysis), and "<name> snapshot.txt" when
the ANC_DIAG build wrote one after a trip (anc_fxnlms.h ANC_DIAG_FILE).

Analysis: the ANC start comes from the recording. During the ANC_START_DELAY_S wait the firmware prints a status line
every 2 s ("[ANC #n] wait t=-1.0 s ..."), and each print freezes J17 L for about 0.4 s (a JTAG printf halts the core
and the SPORT DMA repeats its last buffers). ANC starts |t| of the last such line after its freeze ends (see
find_anc_start). Without the freezes (fan off: Err too quiet) it falls back to runner start + ANC_AFTER_RUNNER, which
depends on the build. ANC off is the 10 s before the ANC start, without the frozen stretches. Attenuation is
off - on in dB (above 0 = quieter) for the fan tones f0, 2 f0, 5 f0, 6 f0 (power within +-6 Hz), 400-560 Hz,
7-12 kHz and the total 60 Hz-20 kHz. When the divergence guard tripped ("[ANC TRIP] Diverged ... s after ANC started"
in the log), ANC on ends 0.5 s before the trip, since the guard muted the speaker there. A timed switch-off
(ANC_AUTO_OFF_S, "[ANC OFF] Output switched off ... s after ANC started") ends it the same way. With ANC_AUTO_OFF_S,
give run a --timeout that covers the ANC start (about 31 s after the runner start), ANC_AUTO_OFF_S and the
diagnostics dump that follows.

Output switches (SW4 output off/on, SW3 restart) leave no line in the log: the firmware prints nothing while ANC runs.
They show in the recording instead. The tones that ANC reduced go back to their ANC-off level while the output is off
and come down again when it is on. switch_analysis follows the envelopes of those tones (complex demodulation, 20 Hz
low-pass) and lists each edge. For each rise it gives the time each tone took to get 10 dB quieter (or its steady
attenuation - 3 dB, when that is less), counted from:
- the ANC start;
- the cut of an SW3 restart: W = 0 sends y to 0 at once, so the tones jump up first; an off stretch shorter than
  1.5 s counts as a restart;
- the moment the tones left their off level, for SW4 on. SW3 pressed while off looks the same, since its press
  leaves no trace.
It also checks each edge for a transient (2 ms rms in 5-20 kHz against the largest one in the 2 s before) and gives
the levels of each on and off stretch. The ANC-on windows leave out the off stretches and 1 s after each rise.
Edges after the start are looked for only in a pushbutton test: run --buttons, which the .json keeps, or
analyze --buttons. Otherwise only the start counts, because with one or two reduced tones the attenuation dips to
the ANC-off level now and then while ANC runs. With the 400-4000 band (three tones) no such dip was seen in 120 s.
For a pushbutton test, also give run a --timeout that covers the presses (e.g. 150). Wait 10 s after the ANC start
(LED9 steady) before the first press, and about 10 s between presses.

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
ANC_AFTER_RUNNER = 31.0    # ANC start after the runner start, only for the run message and as the fallback of
                           # find_anc_start. It grows with the boot prints: 28.1-28.6 s on 2026-11-17, 30.2 s with
                           # ANC_DIAG, 30.9-31.1 s with the 2026-11-18 builds (buttons, timed switch-off)
F0_RANGE = (560.0, 640.0)  # fan f0 at 100% duty: 602 Hz in August, 595-603 Hz on 2026-11-17
TONE_HW = 6.0              # tone level = power within +-TONE_HW Hz
SEG = int(0.5 * FS)        # PSD segment and hop
HOP = int(0.25 * FS)
COMPONENTS = ('f0', '2 f0', '5 f0', '6 f0', '400-560 Hz', '7-12 kHz', 'total')
INK, GRID = '#222222', '#e3e3e3'
C_OFF, C_ON = '#7a7a7a', '#2a6fdb'
C_COMP = {'f0': '#159a85', '2 f0': '#d9822b', '6 f0': '#8e44ad', '7-12 kHz': '#b07d12', 'total': '#7a7a7a'}
C_TRIP = '#c0392b'
ENV_LP = 20.0              # tone envelopes: complex demodulation, then a zero-phase low-pass at this many Hz
ENV_FS = 1000              # envelope sample rate (Hz)
SW_PICK_DB = 5.0           # tones this much quieter 1-9 s after the ANC start mark the output state
SW_OFF_DB = 3.0            # output off: those tones within this of their ANC-off level (their mean in dB), ...
SW_FLOOR_DB = 1.5          # ... coming this close at least once and staying within SW_OFF_DB for SW_MIN_OFF_S;
SW_MIN_OFF_S = 0.01        # anything less is a dip of the attenuation, not an off. An SW3 press while on keeps y at 0
                           # only for the 50 ms fade and W converges within tens of ms: in the 20 Hz envelopes the
                           # tones stay within 3 dB of the ANC-off level for just 13-19 ms (2026-11-19)
SW_MED_S = 0.02            # running median of the mean attenuation before the on/off decision
SW_DIP_S = 1.5             # off for less than this, then on again: a restart (SW3) while the output was on
CONV_DB = 10.0             # convergence: CONV_DB quieter, or the steady attenuation - 3 dB when that is less
CLICK_BAND = (5000.0, 20000.0)   # edge transient: 2 ms rms in this band
C_TONE = {'f0': '#159a85', '2 f0': '#2a6fdb', '5 f0': '#b07d12', '6 f0': '#8e44ad', 'mean': '#3d3d3d'}
SW_KINDS = ('output off', 'output on', 'restart (SW3)')   # edges made by the pushbuttons


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


def runner_cmd(runner, preload, timeout_s, dxe=DXE):
    """CCES_runner loads dxe (default the Release build) on SHARC core 1 and runs it; the board runs on after it."""
    return [str(runner), '--target', 'Emulation Debug Target', '--platform', 'ADSP-SC598W USB EZ-KIT via Debug Agent',
            '--processor', 'ADSP-SC598W', '-v1', '--reset-on-connect', '--timeout', str(int(timeout_s * 1000)),
            '-c', f'0,CLEAR_SYMBOLS|RUN_AFTER_LOAD|{preload},{dxe}']


def build_state():
    """(ok, text): the Release .dxe exists and is newer than every firmware source."""
    if not DXE.exists():
        return False, f'no Release build at {DXE.name}'
    t_dxe = DXE.stat().st_mtime
    newest = max((p for p in SOURCES if p.exists()), key=lambda p: p.stat().st_mtime)
    ok = t_dxe > newest.stat().st_mtime
    fmt = lambda t: time.strftime('%m/%d %H:%M:%S', time.localtime(t))  # noqa: E731
    return ok, (f'.dxe {fmt(t_dxe)}, newest source {newest.name} {fmt(newest.stat().st_mtime)}'
                + ('' if ok else '  -> STALE, build first'))


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
                runner_cmd=subprocess.list2cmdline(cmd), xruns=flags, source=pm.git_state(), notes=a.notes,
                buttons=a.buttons, **info)
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


def mean_psd(x, a, b, skip_frozen=True, exclude=()):
    """Mean one-sided PSD (FS^2/Hz) of 0.5 s Hann segments in [a, b) s; with skip_frozen, segments that touch a frozen
    stretch are left out, and so are segments that touch an `exclude` stretch (lo, hi) in s. Returns f, psd, number
    of segments."""
    i_a = max(int(a * FS), 0)
    seg = x[i_a:int(b * FS)]
    ok = frozen_free(x, a, b) if skip_frozen else np.ones(seg.size, bool)
    w = np.hanning(SEG)
    norm = FS * np.sum(w ** 2)
    acc, n = np.zeros(SEG // 2 + 1), 0
    for i in range(0, seg.size - SEG + 1, HOP):
        s0 = (i_a + i) / FS
        if ok[i:i + SEG].all() and not any(s0 < hi and s0 + SEG / FS > lo for lo, hi in exclude):
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
WAIT_RE = re.compile(r'\[ANC #\d+\] wait\s+t=([-+]?[\d.]+) s')   # status line of the ANC_START_DELAY_S wait


def parse_log(text):
    lines = text.splitlines()
    band, trip, off = BAND_RE.search(text), TRIP_RE.search(text), OFF_RE.search(text)
    return dict(banner=[l for l in lines if l.startswith('>>>')],
                step0=next((l for l in lines if l.startswith('[Step 0]')), None),
                ready=any('SYSTEM READY' in l for l in lines),
                status=[l for l in lines if re.match(r'\[ANC #\d+\]', l)],
                waits=[float(v) for v in WAIT_RE.findall(text)],
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


def print_freezes(x, a, b, win=0.05, hop=0.01, drop=3.0):
    """Stretches in [a, b) s where a status print froze J17 L: the 50 ms rms (10 ms hop) is more than `drop` dB under
    the median of the span, for 0.15-1.5 s (a print halts the core for about 0.4 s). Returns [(start, end)] in s."""
    i0, i1 = max(int(a * FS), 0), min(int(b * FS), x.size)
    n, h = int(win * FS), int(hop * FS)
    if i1 - i0 < 4 * n:
        return []
    c = np.concatenate([[0.0], np.cumsum(x[i0:i1] ** 2)])
    s = np.arange(0, i1 - i0 - n, h)
    r = 10 * np.log10((c[s + n] - c[s]) / n + 1e-30)
    low = np.concatenate([[0], (r < np.median(r) - drop).astype(int), [0]])
    edges = np.flatnonzero(np.diff(low))
    tc = (i0 + s + n / 2) / FS   # window centres
    out = []
    for k0, k1 in zip(edges[::2], edges[1::2]):   # windows k0 .. k1 - 1 are low
        st, en = tc[k0] - hop / 2, tc[k1 - 1] + hop / 2
        if out and st - out[-1][1] < 0.1:
            out[-1] = (out[-1][0], en)
        else:
            out.append((st, en))
    return [(st, en) for st, en in out if 0.15 <= en - st <= 1.5]


def find_anc_start(x, log, t_r, t_x):
    """ANC start in the recording (s) and how it was found.

    During the ANC_START_DELAY_S wait the firmware prints a status line every 2 s of core time ("[ANC #n] wait
    t=-3.0 s ..."). Each print halts the core for about 0.4 s and freezes J17 L, so the recording shows a train of
    freezes, each starting 2 s after the previous one ended. The last line's t is the core time left until the ANC
    start when the line was made, and the core runs in real time again after the print, so ANC starts |t| after the
    last freeze of the train ends. In the 2026-11-17/18 runs f0 or 6 f0 drops within 0.1 s of that time; the fixed
    runner + 28.3 s used before was 0.2 s late to 2.8 s early, as the boot prints changed with the build. Without the
    train (fan off: Err too quiet for the freezes to show) it falls back to runner + ANC_AFTER_RUNNER."""
    fallback = (t_r + ANC_AFTER_RUNNER, f'runner + {ANC_AFTER_RUNNER:.1f} s (ANC_AFTER_RUNNER, %s)')
    waits = log['waits']
    if len(waits) < 2:
        return fallback[0], fallback[1] % 'no wait-phase status lines in the log'
    left, gap = -waits[-1], waits[-1] - waits[-2]
    fr = print_freezes(x, t_r, min(t_x, t_r + 90.0))
    chain = []   # first train of freezes `gap` apart, end to next start: the wait-phase prints
    for k in range(1, len(fr)):
        if abs(fr[k][0] - fr[k - 1][1] - gap) < 0.2:
            chain = (chain or [k - 1]) + [k]
        elif len(chain) >= min(3, len(waits)):
            break
        else:
            chain = []
    if len(chain) < min(3, len(waits)):
        return fallback[0], fallback[1] % f'no train of print freezes {gap:.0f} s apart'
    note = ''
    if len(chain) > len(waits):   # a print right after the start (an early trip) joined the train
        chain = chain[:len(waits)]
        note = '; the train was longer than the wait lines, cut after the last one'
    s, e = fr[chain[-1]]
    ta = e + left
    if not 15.0 <= ta - t_r <= 60.0:
        return fallback[0], fallback[1] % f'the print freezes give runner + {ta - t_r:.1f} s, implausible'
    return ta, (f'{left:.1f} s after the last wait-phase print freeze ({s:.2f}-{e:.2f} s); '
                f'{len(chain)} of {len(waits)} wait prints found{note}')


# ---------------------------------------------------------------- output switches (SW4, SW3) and convergence

def tone_env(x, freqs, a, b):
    """Envelopes (squared amplitude, FS^2) of the tones at `freqs` Hz over [a, b) s: complex demodulation, then an
    ENV_LP Hz zero-phase low-pass, sampled at ENV_FS. Returns t (s) and an array len(freqs) x len(t)."""
    from scipy.signal import butter, sosfiltfilt
    i0, i1 = max(int(a * FS), 0), min(int(b * FS), x.size)
    k = np.arange(i0, i1)
    sos = butter(4, ENV_LP, 'low', fs=FS, output='sos')
    dec = FS // ENV_FS
    env = []
    for fq in freqs:
        ph = 2 * np.pi * fq * k / FS
        re_ = sosfiltfilt(sos, x[i0:i1] * np.cos(ph))[::dec]
        im_ = sosfiltfilt(sos, x[i0:i1] * np.sin(ph))[::dec]
        env.append(4 * (re_ ** 2 + im_ ** 2))
    return k[::dec] / FS, np.array(env)


def edge_click(x, t0, okmask, a0):
    """Transient at an edge: the largest 2 ms rms in CLICK_BAND within +-30 ms of t0, in dB re the largest one in the
    2 s before (above 0 = it stands out of the fan noise). okmask: frozen_free mask of the samples from a0 s on;
    windows that touch a frozen stretch are left out. None near the ends of the recording."""
    from scipy.signal import butter, sosfiltfilt
    i0, i1 = int((t0 - 2.2) * FS), int((t0 + 0.1) * FS)
    j0 = i0 - int(a0 * FS)
    if j0 < 0 or i1 > x.size or j0 + (i1 - i0) > okmask.size:
        return None
    y = sosfiltfilt(butter(4, CLICK_BAND, 'bandpass', fs=FS, output='sos'), x[i0:i1])
    n = int(0.002 * FS)
    c = np.concatenate([[0.0], np.cumsum(y ** 2)])
    r = np.sqrt((c[n:] - c[:-n]) / n)                        # 2 ms windows starting at each sample
    bad = np.concatenate([[0], np.cumsum(~okmask[j0:j0 + (i1 - i0)])])
    good = (bad[n:] - bad[:-n]) == 0
    tc = (i0 + np.arange(r.size) + n / 2) / FS               # window centres
    pre = r[good & (tc >= t0 - 2.1) & (tc < t0 - 0.1)]
    at = r[good & (np.abs(tc - t0) <= 0.03)]
    if pre.size < FS // 2 or not at.size:
        return None
    return float(20 * np.log10(at.max() / max(pre.max(), 1e-30)))


def switch_analysis(x, ta, f0, band, t_end, stops, buttons=False):
    """The tones ANC reduced and how fast they came down after the ANC start. With buttons (a pushbutton test: run
    --buttons) also the output switches after it, all from the recording alone (see the module docstring). Without
    buttons nothing after the start is read as a switch: with one or two reduced tones the attenuation dips to the
    ANC-off level now and then while ANC runs, and those dips would pass for presses. stops: {label: s} from the
    log's [ANC TRIP] / [ANC OFF] lines; the firmware counts these seconds from the last (re)start. Returns a dict;
    it has only 'note' when there is nothing to follow."""
    from scipy.ndimage import median_filter
    ks = [k for k in (1, 2, 5, 6) if band is None or band[0] <= k * f0 <= band[1]]
    names = ['f0' if k == 1 else f'{k} f0' for k in ks]
    a0 = max(ta - 11.0, 0.0)
    t, env = tone_env(x, [k * f0 for k in ks], a0, t_end)
    okmask = frozen_free(x, a0, t_end)
    w = int(0.03 * ENV_FS)                                   # 30 ms margins for the envelope low-pass
    ok = np.convolve(~okmask[::FS // ENV_FS][:t.size], np.ones(2 * w + 1), 'same') == 0
    pre = (t >= ta - 10.5) & (t < ta - 0.5) & ok
    early = (t >= ta + 1.0) & (t < ta + 9.0) & ok
    if pre.sum() < ENV_FS or early.sum() < ENV_FS:
        return dict(note='less than 1 s of clean recording before the ANC start or 1-9 s after it')
    A = 10 * np.log10(env[:, pre].mean(axis=1)[:, None] / np.maximum(env, 1e-30))   # dB, above 0 = quieter
    st0 = np.median(A[:, early], axis=1)
    pick = st0 >= SW_PICK_DB
    if not pick.any():
        return dict(note=f'no tone got {SW_PICK_DB:.0f} dB quieter 1-9 s after the ANC start ('
                    + ', '.join(f'{n} {v:+.1f}' for n, v in zip(names, st0)) + ' dB)')
    names, A, st0 = [n for n, p in zip(names, pick) if p], A[pick], st0[pick]
    Am = A.mean(axis=0)
    Ams = median_filter(Am, size=int(SW_MED_S * ENV_FS) | 1, mode='nearest')
    Amax = median_filter(A.max(axis=0), size=int(SW_MED_S * ENV_FS) | 1, mode='nearest')   # off: every tone back
    S = float(np.median(Am[early]))
    thr_on = max(SW_PICK_DB, 0.5 * S)
    mid = 0.5 * (thr_on + SW_OFF_DB)
    i_a = int(np.searchsorted(t, ta))
    first_stop = min([ta + s for s in stops.values() if s is not None], default=t_end)
    trans, on = [], False          # indices where the state flips: on at even positions, off at odd ones
    for i in range(i_a, t.size):
        if not ok[i]:
            continue               # frozen J17 L: keep the state
        if not on and Ams[i] >= thr_on:
            on = True
            trans.append(i)
            if not buttons:        # only the ANC start
                break
        elif on and Amax[i] <= SW_OFF_DB:
            on = False
            trans.append(i)
    ignored, k = 0, 1              # an off must come close to the ANC-off level; a dip of the attenuation is no off
    while k < len(trans):
        sl = slice(trans[k], trans[k + 1] if k + 1 < len(trans) else t.size)
        if (Am[sl][ok[sl]].min(initial=np.inf) <= SW_FLOOR_DB
                and np.sum((Amax[sl] <= SW_OFF_DB) & ok[sl]) >= SW_MIN_OFF_S * ENV_FS):
            k += 2
        else:
            del trans[k:k + 2]
            ignored += 1

    def cross(i, rising):          # time Ams crossed `mid` on the way to index i
        if not rising and not ok[i - 1]:   # off right after a frozen stretch: the firmware prints only once the
            j = i - 1                      # output is quiet (guard trip, timed switch-off), so the edge is where
            while j > i_a and not ok[j - 1]:   # the stretch began
                j -= 1
            return float(t[j])
        idx = np.flatnonzero(Ams[i_a:i + 1] < mid if rising else Ams[i_a:i + 1] > mid)
        if not idx.size or i_a + idx[-1] >= i:
            return float(t[i])
        j = i_a + idx[-1]
        v0, v1 = Ams[j], Ams[j + 1]
        return float(t[j] + (mid - v0) / (v1 - v0) / ENV_FS) if v1 != v0 else float(t[j])

    def left(i, rising):           # last time before index i at the old level (off for a rise, on for a fall)
        idx = np.flatnonzero(Amax[i_a:i + 1] <= SW_OFF_DB if rising else Ams[i_a:i + 1] >= thr_on)
        return float(t[i_a + idx[-1]]) if idx.size else float(t[i_a])

    def converge(t_ref, i_on, t_lim):
        """Per tone (steady, threshold, seconds from t_ref until it got that much quieter): steady = median 1-6 s after
        the rise; the tone must first be above the threshold level (a restart sends it up), then get below it."""
        out = {}
        sm = (t >= t[i_on] + 1.0) & (t < min(t[i_on] + 6.0, t_lim - 0.2)) & ok
        for j, nm in enumerate(names):
            if sm.sum() < 0.3 * ENV_FS:
                out[nm] = None
                continue
            st = float(np.median(A[j, sm]))
            thr = min(CONV_DB, st - 3.0)
            dt = None
            if thr >= 3.0:                 # a tone that only gets a few dB quieter has no meaningful time
                span = (t >= t_ref) & (t < t_lim) & ok
                up = np.flatnonzero(span & (A[j] < thr))
                if up.size:
                    hit = np.flatnonzero(span & (A[j] >= thr) & (np.arange(t.size) > up[0]))
                    dt = float(t[hit[0]] - t_ref) if hit.size else None
            out[nm] = (st, thr, dt)
        return out

    edges = [(cross(i, n % 2 == 0), left(i, n % 2 == 0), i) for n, i in enumerate(trans)]   # (edge, left, index)
    nxt_left = lambda n: edges[n][1] if n < len(edges) else t_end   # noqa: E731
    events, starts, stop_times = [], [ta], {}
    n = 0
    while n < len(edges):
        te, tl, i = edges[n]
        if n % 2 == 0:                                      # a rise
            if n == 0:
                ev = dict(kind='ANC start', t=ta, t_ref=ta, ref='the ANC start')
            else:
                ev = dict(kind='output on', t=te, t_ref=tl, ref='the tones left the off level')
                starts.append(tl)                           # SW3 while off restarts the firmware clock about here
            ev['conv'] = converge(ev['t_ref'], i, nxt_left(n + 1) if buttons else min(ta + 10.0, first_stop))
            n += 1
        elif n + 1 < len(edges) and edges[n + 1][0] - te < SW_DIP_S:   # off briefly, then on: SW3 while on
            ev = dict(kind='restart (SW3)', t=te, t_ref=te, ref='the cut', off_s=edges[n + 1][0] - te)
            ev['conv'] = converge(te, edges[n + 1][2], nxt_left(n + 2))
            starts.append(te)
            n += 2
        else:                                               # off for longer: SW4, the guard or the timed switch-off
            ev = dict(kind='output off', t=te, t_ref=te, ref='the edge', fade_s=float(t[i]) - tl)
            for lab, s in stops.items():   # a log line belongs to the first fall that fits a (re)start before it
                if s is None or lab in stop_times:
                    continue
                cand = [c + s for c in starts if c < te and abs(c + s - te) < 0.6]
                if cand:
                    ev['kind'] = lab
                    stop_times[lab] = min(cand, key=lambda v: abs(v - te))
                    break
            mm = (t >= t[i] + 0.3) & (t < nxt_left(n + 1) - 0.2) & ok
            ev['off_db'] = float(np.median(Am[mm])) if mm.sum() >= 0.3 * ENV_FS else None
            n += 1
        ev['click'] = edge_click(x, ev['t'], okmask, a0)
        events.append(ev)
    segs, excl = [], []            # on / off stretches between the edges, and the off ones for the ANC-on windows
    marks = [ta] + [e[0] for e in edges[1:]]
    for n in range(len(edges)):
        last = n + 1 >= len(edges)
        if n % 2 == 0:
            segs.append(('on', marks[n] + 1.0, t_end - 0.5 if last else marks[n + 1] - 0.2))
        else:
            segs.append(('off', marks[n] + 0.3, t_end - 0.5 if last else edges[n + 1][1] - 0.2))
            excl.append((marks[n] - 0.3, np.inf if last else marks[n + 1] + 1.0))
    return dict(names=names, t=t, A=np.where(ok, A, np.nan), Am=np.where(ok, Am, np.nan), thr_on=thr_on, S=S,
                st0=st0, events=events, segments=segs, exclude=excl, stop_times=stop_times, ignored=ignored,
                buttons=buttons)


def measure(wav, f0=None, buttons=None):
    """PSDs, f0, the attenuation per window, the time course and the output switches of one run, for analyze and
    report. buttons: a pushbutton test (default: "buttons" in the .json, set by run --buttons).

    ANC on ends 0.5 s before a divergence trip (the guard muted the speaker there) or a timed switch-off. In a
    pushbutton test it also leaves out the stretches where the output was switched off (switch_analysis)."""
    base, x, meta, log = load_run(wav)
    if buttons is None:
        buttons = bool(meta.get('buttons', False))
    dur = x.size / FS
    t_r, t_x = meta['runner_start_s'], meta['runner_exit_s']
    if 'anc_start_s' in meta:   # set by hand in the .json
        ta, ta_how = meta['anc_start_s'], 'anc_start_s in the .json'
    else:
        ta, ta_how = find_anc_start(x, log, t_r, t_x)
    f, p_off, n_off = mean_psd(x, ta - 10.5, ta - 0.5)
    if f0 is None:
        f0, prom = find_f0(f, p_off, *F0_RANGE)
        f0_text = f'f0 {f0:.2f} Hz (strongest peak in {F0_RANGE[0]:.0f}-{F0_RANGE[1]:.0f} Hz, {prom:.0f} dB over its surroundings)'
    else:
        f0_text = f'f0 {f0:.2f} Hz (given)'
    c_off = components(f, p_off, f0)
    sw = switch_analysis(x, ta, f0, log['band'], dur, {'guard trip': log['trip_s'], 'timed switch-off': log['off_s']},
                         buttons)
    stops = []   # (time in the recording, marker label, window after it, text)
    for s, lab, key, after, what in ((log['trip_s'], 'guard trip', 'guard trip', 'after the trip (muted)',
                                      'The guard tripped'),
                                     (log['off_s'], 'switch-off', 'timed switch-off', 'after the switch-off',
                                      'The output was switched off')):
        if s is not None:   # the firmware counts from the last (re)start; switch_analysis knows the restarts
            t0 = sw.get('stop_times', {}).get(key, ta + s)
            since = 'the ANC start' if abs(t0 - s - ta) < 1e-6 else f'the restart at {t0 - s - ta:+.2f} s'
            stops.append((t0, lab, after, f'{what} {s:.1f} s after {since}'))
    t_trip, stop_label, after_key, stop_text = min(stops) if stops else (None, None, None, None)
    stop = t_x - 2.0 if t_trip is None else min(t_x - 2.0, t_trip - 0.5)
    win = {'first 5 s': (ta + 0.5, min(ta + 5.5, stop)), 'steady (attached)': (ta + 10.0, stop)}
    if t_trip is None:
        win['after the runner exit'] = (t_x + 2.0, dur - 1.0)
    else:
        win[after_key] = (t_trip + 1.0, dur - 1.0)   # status prints are back: skip frozen stretches
    win = {k: v for k, v in win.items() if v[1] - v[0] >= 3.0}
    excl = sw.get('exclude', [])   # output switched off (SW4, a restart's dip): not ANC on
    spec, nseg, res = {'ANC off': p_off}, {'ANC off': n_off}, {}
    for k, (a, b) in list(win.items()):
        _, p, n = mean_psd(x, a, b, skip_frozen=(k == after_key), exclude=() if k == after_key else excl)
        if n == 0:
            del win[k]
            continue
        spec[k], nseg[k] = p, n
        c = components(f, p, f0)
        res[k] = {q: 10 * np.log10(c_off[q] / max(c[q], 1e-30)) for q in COMPONENTS}
    on_key = next((k for k in ('steady (attached)', 'first 5 s') if k in win), None)
    segs, contrast = [], None      # with pushbutton edges: levels of each on / off stretch, and on vs off pooled
    if any(e['kind'] in SW_KINDS for e in sw.get('events', [])):
        pool = {'on': [0.0, 0], 'off': [0.0, 0]}
        for state, a, b in sw['segments']:
            if b - a < 2.0:
                continue
            _, p, n = mean_psd(x, a, b)
            if n:
                c = components(f, p, f0)
                segs.append((state, a, b, n, {q: 10 * np.log10(c_off[q] / max(c[q], 1e-30)) for q in COMPONENTS}))
                pool[state] = [pool[state][0] + p * n, pool[state][1] + n]
        if pool['on'][1] and pool['off'][1]:
            c_on, c_of = (components(f, pool[s][0] / pool[s][1], f0) for s in ('on', 'off'))
            contrast = {q: 10 * np.log10(c_of[q] / max(c_on[q], 1e-30)) for q in COMPONENTS}
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
    return dict(base=base, meta=meta, log=log, dur=dur, t_r=t_r, t_x=t_x, ta=ta, ta_how=ta_how, t_trip=t_trip,
                stop_label=stop_label, stop_text=stop_text, f=f, f0=f0, f0_text=f0_text, c_off=c_off, win=win,
                spec=spec, nseg=nseg, res=res, on_key=on_key, tt=np.array(tt),
                rows={q: rows[:, i] for i, q in enumerate(COMPONENTS)}, sw=sw, segs=segs, contrast=contrast)


def analyze(wav, f0=None, buttons=None):
    m = measure(wav, f0, buttons)
    base, meta, log, ta, t_r, c_off = m['base'], m['meta'], m['log'], m['ta'], m['t_r'], m['c_off']
    L = [f'{Path(wav).name}: {m["dur"]:.1f} s; runner start {t_r:.2f} s, exit {m["t_x"]:.2f} s',
         f'ANC start {ta:.2f} s (runner + {ta - t_r:.2f} s): {m["ta_how"]}', f'Notes: {meta.get("notes", "")}',
         m['f0_text'],
         f'ANC off: {ta - 10.5:.1f}-{ta - 0.5:.1f} s, {m["nseg"]["ANC off"]} segments of 0.5 s without frozen stretches',
         'Attenuation, off - on (dB, above 0 = quieter); tones = power within +-6 Hz:',
         '  window                          ' + ''.join(f'{c:>11s}' for c in COMPONENTS)]
    for k, (a, b) in m['win'].items():
        L.append(f'  {k:22s} {a:6.1f}-{b:6.1f}' + ''.join(f'{m["res"][k][q]:+11.1f}' for q in COMPONENTS))
    if m['t_trip'] is not None:
        L.append(f'  {m["stop_text"]} ({m["t_trip"]:.1f} s): ANC on ends 0.5 s before it.')
    L.append('  ANC off levels (dBFS on the US205; tones as amplitude): '
             + ', '.join(f'{q} {10 * np.log10((2 if "f0" in q else 1) * c_off[q] + 1e-30):.1f}' for q in COMPONENTS))
    L += switch_lines(m)
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


def switch_lines(m):
    """Text for analyze: the edges switch_analysis found, how fast the tones came down after each rise, and the levels
    of the on / off stretches when the pushbuttons were used."""
    sw, ta = m['sw'], m['ta']
    if 'note' in sw:
        return [f'Output switches: not analyzed, {sw["note"]}']
    L = [f'Output switches and convergence (tone envelopes, {ENV_LP:.0f} Hz low-pass; times re the ANC start):',
         '  tones ANC reduced 1-9 s after the start: '
         + ', '.join(f'{n} {v:+.1f} dB' for n, v in zip(sw['names'], sw['st0']))
         + f'; on when their mean is above {sw["thr_on"]:.1f} dB, off when each is within {SW_OFF_DB:.0f} dB of its '
         'ANC-off level']
    if sw['buttons']:
        L.append('  ("output on" is SW4, or SW3 pressed while off: that press leaves no trace, so its time counts from '
                 'the tones leaving the off level)')
        if len(sw['names']) < 3:
            L.append(f'  (only {len(sw["names"])} tone(s) followed: a short dip of the attenuation can pass for a '
                     'restart; check the "restart (SW3)" lines against the presses)')
    for ev in sw['events']:
        s = f'  {ev["t"] - ta:+8.2f} s  {ev["kind"]}'
        if 'off_s' in ev:
            s += f', off {ev["off_s"]:.2f} s'
        if 'fade_s' in ev:
            s += f': fall {ev["fade_s"]:.3f} s' + ('' if ev['off_db'] is None
                                                   else f', off level {ev["off_db"]:+.1f} dB re ANC off')
        if ev.get('conv'):
            parts = [f'{nm} -' if v is None or v[2] is None
                     else f'{nm} {v[1]:.{0 if v[1] == CONV_DB else 1}f} dB in {v[2]:.2f} s'
                     for nm, v in ev['conv'].items()]
            s += ': ' + ', '.join(parts) + f' after {ev["ref"]}'
        if ev['click'] is not None:
            s += f'; edge transient {ev["click"]:+.1f} dB'
        L.append(s)
    if not sw['buttons']:
        L.append('  (not a pushbutton test, so edges after the start were not looked for: run --buttons, analyze '
                 '--buttons)')
    if sw['ignored']:
        L.append(f'  ({sw["ignored"]} dips of the attenuation that did not reach the ANC-off level were not counted)')
    if m['segs']:
        L.append('On / off stretches (without 1 s after a rise, 0.3 s after a fall and 0.2 s before an edge), '
                 'off - on re ANC off before the start:')
        L.append('  state  from-to (s re start)' + ''.join(f'{c:>11s}' for c in COMPONENTS))
        for state, a, b, n, r in m['segs']:
            L.append(f'  {state:5s}  {a - ta:7.1f}-{b - ta:7.1f}     ' + ''.join(f'{r[q]:+11.1f}' for q in COMPONENTS))
        if m['contrast']:
            L.append('  on vs off, pooled          ' + ''.join(f'{m["contrast"][q]:+11.1f}' for q in COMPONENTS))
    return L


def plot(base, m):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    f, spec, f0, tt, t_exit = m['f'], m['spec'], m['f0'], m['tt'], m['t_x'] - m['ta']
    plt.rcParams.update({'font.size': 11, 'axes.edgecolor': '#999999', 'axes.labelcolor': INK, 'xtick.color': INK,
                         'ytick.color': INK, 'axes.titlelocation': 'left'})
    sw = m['sw'] if m['sw'].get('events') else None
    fig = plt.figure(figsize=(13, 13.5 if sw else 9.5))
    gs = fig.add_gridspec(3 if sw else 2, 2, height_ratios=[1, 1, 1.1] if sw else [1, 1.1])
    if sw:
        plot_envelopes(fig.add_subplot(gs[1, :]), m)
        plot_switches(base, m)
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
        a2 = fig.add_subplot(gs[2 if sw else 1, j])
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


def plot_envelopes(ax, m):
    """Attenuation of the tones switch_analysis follows, over the whole run, with the edges it found."""
    import warnings
    sw, ta = m['sw'], m['ta']
    nb = sw['t'].size // 100          # 0.1 s medians: the on / off states; the envelope detail is in the zoom figure
    t = sw['t'][:nb * 100].reshape(nb, 100).mean(axis=1) - ta

    def med(v):
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)   # all-NaN blocks (frozen J17 L) stay NaN
            return np.nanmedian(v[:nb * 100].reshape(nb, 100), axis=1)
    for j, nm in enumerate(sw['names']):
        ax.plot(t, med(sw['A'][j]), color=C_TONE.get(nm, '#666666'), lw=1.0, label=nm)
    if len(sw['names']) > 1:
        ax.plot(t, med(sw['Am']), color=C_TONE['mean'], lw=1.4, label='mean')
    ax.axhline(sw['thr_on'], color='#888888', lw=0.7, ls='--')
    ax.axhline(SW_OFF_DB, color='#888888', lw=0.7, ls=':')
    ax.axhline(0, color='#888888', lw=0.8)
    for ev in sw['events']:
        ax.axvline(ev['t'] - ta, color='#555555', lw=0.7, ls='--')
        ax.text(ev['t'] - ta, 0.98, ' ' + ev['kind'], transform=ax.get_xaxis_transform(), rotation=90, va='top',
                fontsize=8, color='#444444')
    ax.set_xlim(-5, t[-1])
    ax.set_ylim(-8, max(25.0, float(np.nanpercentile(sw['A'], 99.5)) + 3))
    ax.set_xlabel(f'time from the ANC start (s); 0.1 s medians of the tone envelopes ({ENV_LP:.0f} Hz low-pass)')
    ax.set_ylabel('attenuation re ANC off (dB)')
    ax.set_title(f'Tones ANC reduced: output on above {sw["thr_on"]:.1f} dB (dashed), off below {SW_OFF_DB:.0f} dB '
                 '(dotted); zoom in "... switches.png"')
    ax.grid(True, lw=0.5, color=GRID)
    ax.legend(frameon=False, ncol=5, loc='lower left', fontsize=9)


def plot_switches(base, m):
    """<base> switches.png: each edge switch_analysis found, -0.3 to +1.2 s around it, with the time each tone took
    to come down (dots) after a rise."""
    import matplotlib.pyplot as plt
    sw, ta = m['sw'], m['ta']
    evs = sw['events'][:12]
    cols = min(len(evs), 4)
    rows = (len(evs) + cols - 1) // cols
    fig, axs = plt.subplots(rows, cols, figsize=(3.3 * cols + 0.7, 3.0 * rows + 0.5), squeeze=False, sharey=True)
    t = sw['t']
    for ax, ev in zip(axs.flat, evs):
        t0 = ev['t_ref']
        sel = (t >= t0 - 0.3) & (t <= t0 + 1.2)
        for j, nm in enumerate(sw['names']):
            ax.plot(t[sel] - t0, sw['A'][j][sel], color=C_TONE.get(nm, '#666666'), lw=1.2, label=nm)
            v = (ev.get('conv') or {}).get(nm)
            if v is not None and v[2] is not None:
                ax.plot(v[2], v[1], 'o', ms=5, color=C_TONE.get(nm, '#666666'))
        if len(sw['names']) > 1:
            ax.plot(t[sel] - t0, sw['Am'][sel], color=C_TONE['mean'], lw=1.0, ls='--', label='mean')
        ax.axvline(0, color='#555555', lw=0.8)
        ax.axhline(0, color='#888888', lw=0.7)
        ax.set_title(f'{ev["kind"]}, {ev["t"] - ta:+.2f} s' + ('' if ev['click'] is None
                                                              else f'\nedge transient {ev["click"]:+.1f} dB'),
                     fontsize=10)
        ax.set_xlabel(f's re {ev["ref"]}', fontsize=9)
        ax.grid(True, lw=0.5, color=GRID)
    for ax in list(axs.flat)[len(evs):]:
        ax.axis('off')
    axs[0, 0].set_ylim(-8, max(25.0, float(np.nanpercentile(sw['A'], 99.5)) + 3))
    for r in range(rows):
        axs[r, 0].set_ylabel('attenuation re ANC off (dB)')
    axs[0, 0].legend(frameon=False, fontsize=8, loc='lower right')
    fig.tight_layout()
    fig.savefig(str(base) + ' switches.png', dpi=90)
    plt.close(fig)


def track(wav, step=1.0):
    """Second by second (1 s Hann windows every `step` s, from 5 s before the ANC start), for when the attenuation does
    something odd. Answers "did the fan change, or the recording?". Prints and writes <name> track.txt with:
    - the strongest peak within +-15% of f0 and of 6 f0 (fan speed) and their levels;
    - the 2 f0 line, the 7-12 kHz band and the total;
    - how much of the window J17 L was frozen: a status print or the diagnostics dump after a trip or the timed
      switch-off. Those seconds are not the Err mic: on 2026-11-19 a 14 s dump looked like a recording 7 dB quieter."""
    base, x, meta, log = load_run(wav)
    t_r, t_x = meta['runner_start_s'], meta['runner_exit_s']
    ta = meta['anc_start_s'] if 'anc_start_s' in meta else find_anc_start(x, log, t_r, t_x)[0]
    f, p_off, _ = mean_psd(x, ta - 10.5, ta - 0.5)
    f0, _ = find_f0(f, p_off, *F0_RANGE)
    a0, n = ta - 5.0, FS
    okm = frozen_free(x, a0, x.size / FS)
    w = np.hanning(n)
    fr = np.fft.rfftfreq(n, 1 / FS)
    L = [f'{Path(wav).name}: ANC start {ta:.2f} s, f0 {f0:.2f} Hz (ANC off); levels in dBFS (power), tones = 5 bins',
         '  t re start   f0 peak Hz  f0 dB  6f0 peak Hz  6f0 dB  2f0 dB  7-12k dB  total dB  frozen']
    for a in np.arange(a0, x.size / FS - 1.0, step):
        i0 = int(a * FS)
        p = np.abs(np.fft.rfft(x[i0:i0 + n] * w)) ** 2 * 2 / (np.sum(w ** 2) * n)

        def peak(fc):
            idx = np.flatnonzero((fr >= 0.85 * fc) & (fr <= 1.15 * fc))
            i = idx[np.argmax(p[idx])]
            return fr[i], 10 * np.log10(p[i - 2:i + 3].sum() + 1e-30)
        band = lambda lo, hi: 10 * np.log10(p[(fr >= lo) & (fr < hi)].sum() + 1e-30)   # noqa: E731
        (f1, l1), (f6, l6) = peak(f0), peak(6 * f0)
        j0 = i0 - int(a0 * FS)
        frozen = 100 * (1 - okm[j0:j0 + n].mean())
        L.append(f'  {a - ta:+10.1f}   {f1:10.1f} {l1:6.1f}  {f6:11.1f} {l6:7.1f} {band(2 * f0 - 6, 2 * f0 + 7):7.1f} '
                 f'{band(7000, 12000):9.1f} {band(60, 20000):9.1f}  {frozen:4.0f}%')
    txt = '\n'.join(L)
    print(txt)
    Path(str(base) + ' track.txt').write_text(txt + '\n', encoding='utf-8')


PICKUP_SEG_S = 0.2           # pickup: Hann segment, short enough to fit inside a wait-phase print freeze (~0.34 s)
PICKUP_MARGIN_S = 0.03       # ... taken this far inside the freeze edges that print_freezes finds
PICKUP_LINES = (1, 2, 3, 4, 5, 6, 12, 18)   # fan lines k f0


def pickup(wav, f0=None):
    """Which lines come through the core and which enter after the DAC, from the wait phase before the ANC start
    (speaker silent). Each status print halts the core for about 0.34 s while the SPORT DMA replays its last buffers,
    so J17 L then carries nothing new from the ADC side (Err mic, mic board, J12, ADC). A line that keeps its level
    inside those freezes enters after the DAC (J17 L, the cable, the US205); a line that drops comes through the
    core. Compares each fan line inside the freezes with the same line live in between (PICKUP_SEG_S Hann segments),
    each against its local floor (median within +-50 Hz). Made for the 2 f0 check (2026-11-23): on 2026-11-19 the
    2 f0 line kept its level through a 13 s diagnostics freeze. Prints and writes <name> pickup.txt."""
    base, x, meta, log = load_run(wav)
    t_r, t_x = meta['runner_start_s'], meta['runner_exit_s']
    if 'anc_start_s' in meta:
        ta, ta_how = meta['anc_start_s'], 'anc_start_s in the .json'
    else:
        ta, ta_how = find_anc_start(x, log, t_r, t_x)
    a, b = t_r + PROG_AFTER_RUNNER + 1.0, ta - 0.2      # program running (after the codec setup), ANC not started
    n = int(PICKUP_SEG_S * FS)
    w = np.hanning(n)
    f = np.fft.rfftfreq(n, 1 / FS)
    df = f[1] - f[0]

    def psd(spans):   # mean one-sided PSD (FS^2/Hz) of half-overlapping segments inside the spans
        acc, k = np.zeros(f.size), 0
        for s, e in spans:
            for i in range(int(s * FS), int(e * FS) - n + 1, n // 2):
                acc += 2 * np.abs(np.fft.rfft(x[i:i + n] * w)) ** 2 / (FS * np.sum(w ** 2))
                k += 1
        return acc / max(k, 1), k

    def rms(spans):
        v = np.concatenate([x[int(s * FS):int(e * FS)] for s, e in spans]) if spans else np.zeros(1)
        return 10 * np.log10(np.mean(v ** 2) + 1e-30)

    fz = print_freezes(x, a, b)
    frozen = [(s + PICKUP_MARGIN_S, e - PICKUP_MARGIN_S) for s, e in fz]
    frozen = [(s, e) for s, e in frozen if e - s >= PICKUP_SEG_S]
    edges = [a] + [v for s, e in fz for v in (s - 0.1, e + 0.1)] + [b]
    live = [(s, e) for s, e in zip(edges[::2], edges[1::2]) if e - s >= PICKUP_SEG_S]
    p_live, n_live = psd(live)
    p_frz, n_frz = psd(frozen)
    if f0 is not None:
        f0_text = f'f0 {f0:.2f} Hz (given)'
    else:
        fl, pl, _ = mean_psd(x, a, b)
        f0, prom = find_f0(fl, pl, *F0_RANGE)
        f0_text = (f'f0 {f0:.2f} Hz (strongest peak in {F0_RANGE[0]:.0f}-{F0_RANGE[1]:.0f} Hz, '
                   f'{prom:.0f} dB over its surroundings)')
        if prom < 10.0:   # no acoustic f0 (Err mic unplugged): half the 2 f0 line
            f2, prom2 = find_f0(fl, pl, 2 * F0_RANGE[0], 2 * F0_RANGE[1])
            if prom2 >= 10.0:
                f0, f0_text = f2 / 2, (f'f0 {f2 / 2:.2f} Hz = half the 2 f0 line at {f2:.1f} Hz ({prom2:.0f} dB over '
                                       f'its surroundings; the f0 peak stood only {prom:.0f} dB out)')

    def level(p, i):   # (amplitude dBFS, dB over the floor) of the line in bins i-2 .. i+2
        tone = p[i - 2:i + 3].sum() * df
        ring = np.r_[p[max(i - int(50 / df), 0):i - 3], p[i + 4:i + int(50 / df) + 1]]
        return 10 * np.log10(2 * tone + 1e-30), 10 * np.log10(tone / max(np.median(ring) * 5 * df, 1e-30))

    L = [f'{Path(wav).name}: wait phase {a:.1f}-{b:.1f} s (speaker silent); ANC start {ta:.2f} s: {ta_how}',
         f'Notes: {meta.get("notes", "")}', f0_text,
         f'{len(fz)} print freezes in the wait phase, {len(frozen)} long enough: {n_frz} frozen and {n_live} live '
         f'segments of {PICKUP_SEG_S:g} s. J17 L rms live {rms(live):.1f}, frozen {rms(frozen):.1f} dBFS']
    if not frozen:
        L.append('No usable freezes (J17 L too quiet to show them?): live levels only.')
    L += ['Fan lines, dBFS on the US205 as amplitude (power within +-2 bins); "floor" = dB over the median '
          'within +-50 Hz:', '  line         Hz     live  floor   frozen  floor  live-frozen']
    for k in PICKUP_LINES:
        i0 = int(round(k * f0 / df))
        i = i0 - 3 + int(np.argmax((p_live + p_frz)[i0 - 3:i0 + 4]))   # the line's bin, live or frozen
        lv, lv_snr = level(p_live, i)
        row = f'  {k:2d} f0 {f[i]:9.1f}  {lv:7.1f} {lv_snr:+6.1f}'
        if n_frz:
            fv, fv_snr = level(p_frz, i)
            d = lv - fv
            what = ('not above the floor' if lv_snr < 6 else 'through the core' if d >= 10
                    else 'after the DAC' if abs(d) <= 3 and fv_snr >= 10 else 'both, or unclear')
            row += f'  {fv:7.1f} {fv_snr:+6.1f}  {d:+8.1f}    {what}'
        L.append(row)
    L.append('Reading: live - frozen >= 10 dB: through the core (Err mic, mic board, J12, ADC); within 3 dB with the '
             'frozen line >= 10 dB over its floor: enters after the DAC (J17 L, cable, US205).')
    L += ['Core status lines in the wait (levels in the Ref band, as the core sees them):']
    L += ['  ' + l for l in log['status'] if 'wait' in l][-2:]
    txt = '\n'.join(L)
    print(txt)
    Path(str(base) + ' pickup.txt').write_text(txt + '\n', encoding='utf-8')


def listen(a):
    """Record J17 L for a.seconds without loading anything (the board keeps running whatever it runs) and print the
    fan lines, for quick A/B checks of the setup, e.g. which cable or supply brings in the 2 f0 pickup (2026-11-23).
    Saves "<name>".wav/.json/.txt in measurements/anc_test/. --wav analyzes an earlier recording again (its .txt is
    rewritten). --windows "a-b,c-d,..." gives each line per stretch of the recording and its change against the first
    stretch, e.g. output off, then on after an SW3 press; --course prints the lines second by second to find the press."""
    if a.wav:
        base = Path(str(a.wav)[:-4])
        fs, x = wavfile.read(str(a.wav))
        assert fs == FS, fs
        meta = json.loads(Path(str(base) + '.json').read_text(encoding='utf-8'))
        started, notes = meta.get('started', '?'), meta.get('notes', '')
    else:
        if not a.name:
            sys.exit('Give --name for a new recording, or --wav for an earlier one.')
        sd = pm.import_sounddevice()
        dev_in, _ = pm.wasapi_devices(sd, pm.DEVICE)
        vol = pm.windows_volume(dev_in['name'], capture=True)
        if 'scalar' in vol and abs(vol['scalar'] - 0.5) > 0.01:
            print('WARNING: the US205 mic volume should stay 0.500')
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        base = OUT_DIR / a.name
        for ext in ('.wav', '.json', '.txt'):
            if Path(str(base) + ext).exists():
                sys.exit(f'{base.name}{ext} exists already; pick another --name.')
        started, notes = time.strftime('%Y-%m-%d %H:%M:%S'), a.notes
        try:
            x = sd.rec(int(a.seconds * FS), samplerate=FS, channels=1, dtype='float32', device=dev_in['index'],
                       extra_settings=sd.WasapiSettings(exclusive=True), blocking=True)[:, 0]
        except Exception as e:  # noqa: BLE001
            sys.exit(f'Audio error: {e}. Close REW or anything else that uses the US205.')
        wavfile.write(str(base) + '.wav', FS, x)
        meta = dict(fs=FS, started=started, kind='listen', seconds=a.seconds, device=dev_in['name'],
                    windows_volume=vol, notes=notes)
        Path(str(base) + '.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')
    x = x.astype(np.float64)
    dur = x.size / FS
    wins = [(0.5, dur)]   # the first 0.5 s: stream start
    if a.windows:
        wins = [tuple(float(v) for v in w.split('-')) for w in a.windows.split(',')]
    specs = [mean_psd(x, lo, hi, skip_frozen=False) for lo, hi in wins]
    f, p = specs[0][0], specs[0][1]
    df = f[1] - f[0]
    if a.f0:
        f0, f0_text = a.f0, f'f0 {a.f0:.2f} Hz (given)'
    else:
        f0, prom = find_f0(f, p, *F0_RANGE)
        f0_text = f'f0 {f0:.2f} Hz (strongest peak in {F0_RANGE[0]:.0f}-{F0_RANGE[1]:.0f} Hz, {prom:.0f} dB out)'
        if prom < 10.0:
            f2, prom2 = find_f0(f, p, 2 * F0_RANGE[0], 2 * F0_RANGE[1])
            f0_text += (f'; taken as half the 2 f0 line at {f2:.1f} Hz ({prom2:.0f} dB out)' if prom2 >= 10.0
                        else '; no clear f0 or 2 f0 line: give --f0')
            if prom2 >= 10.0:
                f0 = f2 / 2
    L = [f'{base.name}: {dur:.1f} s from {started}; J17 L rms {db(np.sqrt(np.mean(x ** 2))):.1f}, '
         f'peak {db(np.max(np.abs(x))):.1f} dBFS', f'Notes: {notes}', f0_text,
         'Fan lines, dBFS on the US205 as amplitude (power within +-3 bins of the peak near k f0, 0.5 s segments); '
         '"floor" = dB over the median within +-50 Hz in the first stretch'
         + ('; "change" = the first stretch minus this one (dB, above 0 = quieter)' if len(wins) > 1 else '') + ':',
         '  line         Hz' + ''.join(f'{f"{lo:g}-{hi:g} s":>10s}' + ('  floor' if j == 0 else '')
                                     for j, (lo, hi) in enumerate(wins))
         + ''.join(f'{"change " + str(j + 1):>10s}' for j in range(1, len(wins)))]
    p_all = sum(s[1] for s in specs)
    for k in PICKUP_LINES:
        i0 = int(round(k * f0 / df))
        i = i0 - 4 + int(np.argmax(p_all[i0 - 4:i0 + 5]))
        row, levs = f'  {k:2d} f0 {f[i]:9.1f}', []
        for j, (_, pj, _) in enumerate(specs):
            tone = pj[i - 3:i + 4].sum() * df
            levs.append(10 * np.log10(2 * tone + 1e-30))
            row += f'{levs[-1]:10.1f}'
            if j == 0:
                ring = np.r_[pj[max(i - int(50 / df), 0):i - 5], pj[i + 6:i + int(50 / df) + 1]]
                row += f' {10 * np.log10(tone / max(np.median(ring) * 7 * df, 1e-30)):+6.1f}'
        L.append(row + ''.join(f'{levs[0] - v:+10.1f}' for v in levs[1:]))
    tot = [10 * np.log10(np.sum(pj[(f >= 60) & (f < 20000)]) * df + 1e-30) for _, pj, _ in specs]
    L.append(f'  {"total 60-20k":15s}' + ''.join(f'{v:10.1f}' + ('       ' if j == 0 else '') for j, v in enumerate(tot))
             + ''.join(f'{tot[0] - v:+10.1f}' for v in tot[1:]))
    if a.course:
        n = FS   # 1 s windows: 1 Hz bins
        w = np.hanning(n)
        fr = np.fft.rfftfreq(n, 1 / FS)
        L += ['Second by second (1 s Hann windows; tones = power within +-3 Hz of k f0, as amplitude):',
              '        t s     f0   2 f0   6 f0  total']
        for t0 in np.arange(0.0, dur - 1.0 + 1e-9, 1.0):
            i0 = int(t0 * FS)
            ps = 2 * np.abs(np.fft.rfft(x[i0:i0 + n] * w)) ** 2 / (FS * np.sum(w ** 2))
            lv = [10 * np.log10(2 * ps[int(round(k * f0)) - 3:int(round(k * f0)) + 4].sum() + 1e-30) for k in (1, 2, 6)]
            L.append(f'  {t0:5.0f}-{t0 + 1:<4.0f}' + ''.join(f'{v:7.1f}' for v in lv)
                     + f'{10 * np.log10(ps[(fr >= 60) & (fr < 20000)].sum() + 1e-30):7.1f}')
    txt = '\n'.join(L)
    print(txt)
    Path(str(base) + '.txt').write_text(txt + '\n', encoding='utf-8')


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


FLOOR_BANDS = ((250, 400), (400, 560), (560, 1000), (1000, 1500), (1500, 5000), (5000, 20000))


def floor_changes(m):
    """Numbers for the spectrum slide, off - on in dB (above 0 = quieter):
    - the Err floor between the fan tones per FLOOR_BANDS band (bins within TONE_HW of any k f0 left out);
    - the 12 f0 and 18 f0 tones (power within TONE_HW);
    - the largest rise of the 1/24-octave off - on curve of the spectrum figure, and where it is.
    Returns ({(lo, hi): dB}, {k: dB}, (dB, Hz))."""
    f, p_off, p_on, f0 = m['f'], m['spec']['ANC off'], m['spec'][m['on_key']], m['f0']
    far = np.abs(f - np.round(f / f0) * f0) > TONE_HW

    def ratio(sel):
        return float(10 * np.log10(p_off[sel].sum() / max(p_on[sel].sum(), 1e-30)))
    bands = {(lo, hi): ratio((f >= lo) & (f < hi) & far) for lo, hi in FLOOR_BANDS}
    tones = {k: ratio(np.abs(f - k * f0) <= TONE_HW) for k in (12, 18)}
    fc, a_off = octave_avg(f, p_off)
    _, a_on = octave_avg(f, p_on)
    d = 10 * np.log10(a_off / np.maximum(a_on, 1e-30))
    i = int(np.argmin(d))
    return bands, tones, (float(d[i]), float(fc[i]))


def floor_lines(lab, m):
    """report text: floor_changes of one run."""
    bands, tones, (dmin, fmin) = floor_changes(m)
    span = lambda lo, hi: f'{lo / 1000:g}-{hi / 1000:g} kHz' if lo >= 1500 else f'{lo}-{hi} Hz'   # noqa: E731
    return [f'  {lab}: floor between the tones ' + ', '.join(f'{span(lo, hi)} {v:+.1f}' for (lo, hi), v in
                                                              bands.items()),
            f'    12 f0 {tones[12]:+.1f}, 18 f0 {tones[18]:+.1f} dB; largest rise on the 1/{SMOOTH_OCT}-octave curve '
            f'{dmin:+.1f} dB at {fmin:.0f} Hz']


THIRD_OCT = (500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000)   # nominal 1/3-octave centres (Hz)


def a_weight(f):
    """A-weighting (IEC 61672) as power gains at f Hz."""
    fa = np.maximum(f, 1.0)
    ra = (12194 ** 2 * fa ** 4) / ((fa ** 2 + 20.6 ** 2) * np.sqrt((fa ** 2 + 107.7 ** 2) * (fa ** 2 + 737.9 ** 2))
                                  * (fa ** 2 + 12194 ** 2))
    return (ra * 10 ** (2.0 / 20)) ** 2


def critical_band(fc):
    """Critical bandwidth (Hz) around fc (Zwicker's formula, as ECMA-74 uses for the tone-to-noise ratio)."""
    return 25 + 75 * (1 + 1.4 * (fc / 1000) ** 2) ** 0.69


def prominence_limit(fc):
    """ECMA-74: a tone is prominent when its tone-to-noise ratio reaches this many dB."""
    return 8.0 + 8.33 * np.log10(1000 / fc) if fc < 1000 else 8.0


def tone_to_noise(f, p, fc, f0):
    """Tone-to-noise ratio (dB) of the tone at fc, ECMA-74 style: the tone's power (within TONE_HW) over the noise in
    the critical band around it (the band's mean PSD without the fan tones, times the bandwidth)."""
    cb = critical_band(fc)
    far = np.abs(f - np.round(f / f0) * f0) > TONE_HW
    noise = float(np.mean(p[(f >= fc - cb / 2) & (f < fc + cb / 2) & far])) * cb
    tone = float(np.sum(p[np.abs(f - fc) <= TONE_HW + 1e-9])) * (f[1] - f[0])
    return 10 * np.log10(tone / noise)


def perceived_lines(lab, m, names):
    """report text: what the ear goes by rather than the total. ANC only takes the tones out, and those are a small
    part of the fan's power (37% for f0 + 5 f0 + 6 f0 on 2026-11-19), so the total barely moves; but a tone that
    stands out of the noise in its critical band is what one hears, so the tone-to-noise ratio tells more."""
    f, p_off, p_on, f0 = m['f'], m['spec']['ANC off'], m['spec'][m['on_key']], m['f0']
    df = f[1] - f[0]
    band = lambda p, lo, hi: float(np.sum(p[(f >= lo) & (f < hi)]) * df)   # noqa: E731
    wa = a_weight(f)
    a_db = 10 * np.log10(band(p_off * wa, 20, 20000) / band(p_on * wa, 20, 20000))
    ks = (1, 5, 6)
    share = sum(band(p_off, k * f0 - TONE_HW, k * f0 + TONE_HW + 1e-9) for k in ks) / band(p_off, 60, 20000)
    oct3 = []
    for j in sorted({int(round(3 * np.log2(k * f0 / 1000))) for k in ks}):   # base-2 bands, so each holds its tone
        c = 1000 * 2 ** (j / 3)
        lo, hi = c * 2 ** (-1 / 6), c * 2 ** (1 / 6)
        nom = min(THIRD_OCT, key=lambda v: abs(np.log2(v / c)))
        oct3.append(f'{nom / 1000:g} kHz' if nom >= 1000 else f'{nom} Hz')
        oct3[-1] += f' {10 * np.log10(band(p_off, lo, hi) / band(p_on, lo, hi)):+.1f}'
    tnr = [f'{names[q].split(" (")[0]} {tone_to_noise(f, p_off, k * f0, f0):+.1f} -> '
           f'{tone_to_noise(f, p_on, k * f0, f0):+.1f} dB (limit {prominence_limit(k * f0):.1f})'
           for k, q in zip(ks, ('f0', '5 f0', '6 f0'))]
    return [f'  {lab}: A-weighted total {a_db:+.1f} dB(A); f0, 5 f0 and 6 f0 are {100 * share:.0f}% of the ANC-off '
            f'power (taking them out completely lowers the total by {-10 * np.log10(1 - share):.1f} dB)',
            f'    1/3-octave bands of the tones: ' + ', '.join(oct3) + ' dB',
            '    tone-to-noise ratio, ANC off -> on: ' + ', '.join(tnr)]


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
              f'  ANC start {m["ta"]:.2f} s: {m["ta_how"]}',
              f'  ANC off {m["ta"] - 10.5:.1f}-{m["ta"] - 0.5:.1f} s ({m["nseg"]["ANC off"]} segments of 0.5 s); '
              f'ANC on = {m["on_key"]} {a:.1f}-{b:.1f} s ({m["nseg"][m["on_key"]]} segments)']
        if m['t_trip'] is not None:
            L.append(f'  {m["stop_text"]}; ANC on ends 0.5 s before it.')
    w = max(len(l) for l in labels)
    L += ['Attenuation, off - on (dB, above 0 = quieter); tones = power within +-6 Hz:',
          '  ' + ' ' * w + ''.join(f'{q:>11s}' for q in COMPONENTS)]
    L += [f'  {lab:{w}s}' + ''.join(f'{m["res"][m["on_key"]][q]:+11.1f}' for q in COMPONENTS)
          for lab, m in zip(labels, runs)]
    L.append(f'Err floor between the fan tones (bins within +-{TONE_HW:.0f} Hz of every k f0 left out), off - on (dB):')
    for lab, m in zip(labels, runs):
        L += floor_lines(lab, m)
    L.append('What the ear goes by, off - on (dB): A-weighted total, the 1/3-octave bands of the tones, and the '
             'tone-to-noise ratio in the critical band (ECMA-74 style; a tone is prominent at or above the limit):')
    for lab, m in zip(labels, runs):
        L += perceived_lines(lab, m, names)
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

SYNTH_SW = dict(off1=20.0, on1=28.0, r1=40.0, off2=55.0, r2=62.0, off3=70.0)   # synth's presses, s after the start


def synth(path):
    """Fan-like noise with f0 and 6 f0 tones. At the ANC start f0 drops 22 dB with a 0.1 s time constant; 6 f0 stays.
    As on the board, the seven wait-phase status prints (t = -13 ... -1 s) freeze the output for 0.4 s each, each 2 s
    after the previous one ended, the last one ending 1 s before the ANC start. The analysis must find the start from
    them (it is 1.3 s off the ANC_AFTER_RUNNER fallback) and leave them out of ANC off.
    Then the output is switched as in a pushbutton test (SYNTH_SW):
    - SW4 off, then on: 50 ms fades, W kept.
    - SW3 while on: y cut at once, with a click; W = 0 during the 50 ms fade-in, then it converges as at the start.
    - SW4 off, then SW3 while off.
    - SW4 off for good."""
    from scipy.signal import butter, lfilter
    rng = np.random.default_rng(1)
    t_r, f0 = PRE_S, 602.37
    t_x = t_r + TIMEOUT_S + 7.6
    n = int((t_x + AFTER_S) * FS)
    t = np.arange(n) / FS
    b, a_ = butter(2, [80, 8000], 'bandpass', fs=FS)
    x = lfilter(b, a_, rng.standard_normal(n)) * 10 ** (-58 / 20)
    t_anc = t_r + ANC_AFTER_RUNNER + 1.3
    tau, fl, fade = 0.1, 10 ** (-22 / 20), 0.05
    T = {k: t_anc + v for k, v in SYNTH_SW.items()}
    ramp = lambda t0, up: np.clip((t - t0) / fade, 0, 1) if up else 1 - np.clip((t - t0) / fade, 0, 1)  # noqa: E731
    sw = np.ones(n)                   # output switch gain (sw_gain)
    for k0, k1, up in (('off1', 'on1', False), ('on1', 'r1', True), ('r1', 'off2', True), ('off2', 'r2', False),
                       ('r2', 'off3', True), ('off3', None, False)):
        mm = (t >= T[k0]) & (t < (T[k1] if k1 else np.inf))
        sw[mm] = ramp(T[k0], up)[mm]
    c = np.zeros(n)                   # converged part of W: from 0 at the start and after each restart's fade-in
    for t0, t1 in ((t_anc, T['r1']), (T['r1'] + fade, T['r2']), (T['r2'] + fade, np.inf)):
        mm = (t >= t0) & (t < t1)
        c[mm] = 1 - np.exp(-(t[mm] - t0) / tau)
    g = 1 - sw * c * (1 - fl)
    x += 10 ** (-60 / 20) * g * np.sin(2 * np.pi * f0 * t) + 10 ** (-57 / 20) * np.sin(2 * np.pi * 6 * f0 * t + 1)
    i, m_ = int(T['r1'] * FS), int(0.003 * FS)      # the click of the cut
    x[i:i + m_] += 1.2e-2 * rng.standard_normal(m_) * np.exp(-np.arange(m_) / (0.0004 * FS))
    waits = np.arange(-13.0, 0.0, 2.0)
    end = t_anc - 1.0
    for _ in waits:   # backwards from the last print; the held output decays through the US205 AC coupling
        i0, i1 = int((end - 0.4) * FS), int(end * FS)
        x[i0:i1] = x[i0] * np.exp(-np.arange(i1 - i0) / (0.0015 * FS))
        end -= 0.4 + 2.0
    p = Path(path)
    wavfile.write(str(p), FS, x.astype(np.float32))
    base = Path(str(p)[:-4])
    Path(str(base) + '.json').write_text(json.dumps(dict(fs=FS, runner_start_s=t_r, runner_exit_s=t_x, runner_rc=0,
                                                         notes='synthetic: f0 -22 dB, 6 f0 unchanged; pushbutton '
                                                               'sequence SYNTH_SW', buttons=True)),
                                         encoding='utf-8')
    status = ''.join(f'[ANC #{int(FS * (15 + w)) + 24}] wait t={w:+.1f} s  ref=-44.6 err=-66.0 dB  Px\'=1.00e-08  '
                     f'y_pk=0.000 clip=0/96000  W2=0.000e+00\n' for w in waits)
    Path(str(base) + '.log').write_text('[Step 0] SPORT4/DMA10-11 idle at boot\n>>> SYSTEM READY\n'
                                        '>>> Ref band 400-1500 Hz, mu 0.01, eps 1e-11, clamp 0.9\n' + status,
                                        encoding='utf-8')
    print('wrote', p, f'(expect the ANC start at {t_anc:.2f} s from the print freezes, f0 about +20 dB, limited by '
          'the noise within +-6 Hz; 6 f0 and the bands 0 dB; total about +1 dB)')
    print('  expected edges (s after the start): output off +20.0 and +55.0 and +70.0 (no transient); output on +28.0, '
          'f0 10 dB about 0.02 s after the tones leave the off level; restart (SW3) +40.0 with a transient, f0 10 dB '
          'about 0.20 s after the cut; output on +62.1 (SW3 while off), about 0.11 s after the tones leave the off '
          'level; f0 10 dB about 0.15 s after the start')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('check')
    r = sub.add_parser('run')
    r.add_argument('--name', required=True, help='"MMDD ANC ..." file name in measurements/anc_test/')
    r.add_argument('--notes', default='', help='conditions: fan duty, band, who is in front of the outlet, ...')
    r.add_argument('--timeout', type=float, default=TIMEOUT_S, help='runner timeout, s (the board runs on after it)')
    r.add_argument('--cces', help='CrossCore Embedded Studio folder (default: CCES_ROOT)')
    r.add_argument('--buttons', action='store_true', help='pushbutton test: look for SW4 / SW3 edges after the start')
    z = sub.add_parser('analyze')
    z.add_argument('wav')
    z.add_argument('--f0', type=float, help='fan f0 in Hz (default: the strongest peak in 560-640 Hz)')
    z.add_argument('--buttons', action='store_true', default=None,
                   help='look for SW4 / SW3 edges even if the .json does not say it was a pushbutton test')
    p = sub.add_parser('report', help='slide figures: bars, Err spectrum off/on, time course')
    p.add_argument('wav', nargs='+', help='1-3 runs, compared in this order')
    p.add_argument('--labels', nargs='+', help='one per wav (default: "ref band-pass <band>" from the log)')
    p.add_argument('--f0', type=float, nargs='+', help='fan f0 in Hz, one per wav (default: found per run)')
    p.add_argument('--out', help='folder for the figures (default: the folder of the first wav)')
    p.add_argument('--name', default='anc_lab', help='file prefix: <name>_bars.png, _spectrum.png, _time.png, .txt')
    p.add_argument('--title', help='title of the bar chart')
    d = sub.add_parser('diag')
    d.add_argument('wav')
    k = sub.add_parser('track', help='second by second: fan f0 and tones, 7-12 kHz, total, frozen J17 L')
    k.add_argument('wav')
    k.add_argument('--step', type=float, default=1.0, help='seconds between the 1 s windows')
    u = sub.add_parser('pickup', help='fan lines inside vs outside the wait-phase print freezes: through the core '
                                      'or after the DAC')
    u.add_argument('wav')
    u.add_argument('--f0', type=float, help='fan f0 in Hz (default: from the live wait phase)')
    li = sub.add_parser('listen', help='record J17 L without loading anything, print the fan lines (setup A/B checks)')
    li.add_argument('--name', help='"MMDD ..." file name in measurements/anc_test/ for a new recording')
    li.add_argument('--wav', help='analyze this earlier listen recording again instead of recording')
    li.add_argument('--seconds', type=float, default=10.0)
    li.add_argument('--f0', type=float, help='fan f0 in Hz (default: the f0 peak, else half the 2 f0 line)')
    li.add_argument('--windows', help='"a-b,c-d,...": stretches in s, each compared with the first (e.g. off, then on)')
    li.add_argument('--course', action='store_true', help='also the lines second by second (find an SW3 / SW4 press)')
    li.add_argument('--notes', default='', help='what is plugged in and powered, fan duty, ...')
    s = sub.add_parser('synth')
    s.add_argument('wav')
    a = ap.parse_args()
    if a.cmd == 'check':
        ok, _ = check(pm.import_sounddevice())
        print('READY' if ok else 'NOT READY')
    elif a.cmd == 'run':
        run(a)
    elif a.cmd == 'analyze':
        analyze(a.wav, a.f0, a.buttons)
    elif a.cmd == 'report':
        report(a.wav, a.labels, a.f0, a.out, a.name, a.title)
    elif a.cmd == 'diag':
        diag(a.wav)
    elif a.cmd == 'track':
        track(a.wav, a.step)
    elif a.cmd == 'pickup':
        pickup(a.wav, a.f0)
    elif a.cmd == 'listen':
        listen(a)
    else:
        synth(a.wav)


if __name__ == '__main__':
    main()
