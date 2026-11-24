#!/usr/bin/env python3
"""The boot pop: record J18 Right while the board powers up or CCES_runner loads a program, and list the transients.

J18 Left / Right are the single-ended DAC3P / DAC4P outputs of the ADAU1962A through a 10 uF series capacitor, 475 ohm
and 49.9 kohm to ground (carrier schematic p. 7): no buffer and no mute, so every step of the DAC output's DC level
reaches the jack. The first program load after a board power-up gave two large steps on 2026-11-17 (US205 clipped);
a reload gave one blip at -35 dBFS (measurements/README, trip_check).

Setup, all steps silent: speaker amp OFF, the J18 Right leg of the split cable in the US205 mic input instead of
J17 Left. J18 Right alone: DAC full scale is +7.1 dBFS on the US205, which clips at 0.44 of DAC full scale.

  python tools/boot_pop.py power --name "1124 pop power-on" [--seconds 60]
        record while the board is switched off and on (the next load is then the first one since the power-up)
  python tools/boot_pop.py load --name "1124 pop preload" --dxe preload|release|pathmeasure|<file.dxe> [--timeout 30]
        [--order 0-3 [--settle MS]]
        record while CCES_runner loads and runs it (--reset-on-connect); "preload" runs the preload alone, which
        sets up clocks and memory and leaves the codecs alone. --order / --settle: the DAC power order and the wait
        before DAC1-12 on for a POP_PROBE 2 build (fix test, Audio_Talkthrough_I2S.c DAC_POWER_ORDER, DAC_SETTLE_MS),
        written to ORDER_FILE before the load; without them a leftover ORDER_FILE is removed, so such a build uses
        the compiled-in values
  python tools/boot_pop.py analyze <wav> [<wav> ...]
        the transients again; also for other recordings (the 11/17 trip_check ones), which only get printed

Transients: 1 ms peak envelope over 6 x its median, joined across gaps under 3 ms. Times are from the runner start
(load) or the recording start (power). Each one also gets, over 100 ms from its start, how long it stays over 0.1 of
US205 full scale and its energy: these still compare pops that clip the US205. Data: measurements/boot_check/
"<name>".wav/.json/.log/.txt.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.io import wavfile

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import anc_labtest as al     # noqa: E402  cces_paths, runner_cmd, DXE, db
import pathmeas as pm        # noqa: E402

REPO = TOOLS.parent
OUT_DIR = REPO / 'measurements' / 'boot_check'
FS = 48000
PRE_S, AFTER_S = 3.0, 5.0      # recording before the runner starts and after it exits
DAC_FS_DB = 7.1                # DAC full scale on the US205, J18 Right alone (2026-11-17)
EVENT_X = 6.0                  # an event: 1 ms peak over this many times the median
SPAN_MS, SPAN_LEVEL = 100.0, 0.1   # per event: how long it stays over 0.1 of US205 full scale, energy, in 100 ms
DXES = {'release': al.DXE, 'pathmeasure': pm.PATHMEAS_DXE, 'preload': None}
ORDER_FILE = OUT_DIR / 'pop_order.txt'   # POP_ORDER_FILE in Audio_Talkthrough_I2S.c


def record(sd, seconds, during=None):
    """Record the US205 mic input (WASAPI exclusive) for `seconds`, or around during() (PRE_S before, AFTER_S after);
    returns the samples and (start, end) of during() in recording seconds."""
    dev_in, _ = pm.wasapi_devices(sd, pm.DEVICE)
    buf = np.zeros(int(900 * FS), np.float32)
    pos, marks = [0], []

    def cb(indata, frames, t, status):
        k = min(frames, buf.size - pos[0])
        buf[pos[0]:pos[0] + k] = indata[:k, 0]
        pos[0] += k
        marks.append((pos[0], time.perf_counter()))

    span = None
    with sd.InputStream(device=dev_in['index'], channels=1, samplerate=FS, dtype='float32', latency=pm.LATENCY_S,
                        extra_settings=sd.WasapiSettings(exclusive=True), callback=cb):
        if during is None:
            time.sleep(seconds)
        else:
            time.sleep(PRE_S)
            t0 = time.perf_counter()
            during()
            t1 = time.perf_counter()
            time.sleep(AFTER_S)
            ends, perf = np.array([m[0] for m in marks], float), np.array([m[1] for m in marks], float)
            span = tuple(round(float(np.interp(t, perf, ends)) / FS, 3) for t in (t0, t1))
    return buf[:pos[0]].copy(), span


def events(x, t_ref=0.0):
    """[(start s re t_ref, length ms, signed peak, ms over SPAN_LEVEL, energy dB)] of the transients in x. The last
    two cover SPAN_MS from the start: the span from the start to the last sample over SPAN_LEVEL, and sum(x^2) / FS in
    dB re 1 (full scale squared x 1 s). A clipped pop is still compared by these two."""
    n = FS // 1000
    env = np.abs(x[:x.size // n * n]).reshape(-1, n).max(axis=1)
    hot = np.flatnonzero(env > EVENT_X * max(np.median(env), 1e-6))
    out = []
    for g in (np.split(hot, np.flatnonzero(np.diff(hot) > 3) + 1) if hot.size else []):
        a, b = g[0] * n, (g[-1] + 1) * n
        i = a + int(np.argmax(np.abs(x[a:b])))
        w = x[a:a + int(SPAN_MS * FS / 1000)]
        over = np.flatnonzero(np.abs(w) > SPAN_LEVEL)
        out.append((a / FS - t_ref, len(g), float(x[i]), (over[-1] + 1) / FS * 1e3 if over.size else 0.0,
                    10 * np.log10(max(float(np.sum(w ** 2)) / FS, 1e-20))))
    return out


def report(wav):
    wav = Path(wav)
    fs, x = wavfile.read(wav)
    x = x.astype(float)
    meta_p = wav.with_suffix('.json')
    meta = json.loads(meta_p.read_text(encoding='utf-8')) if meta_p.exists() else {}
    t_ref = meta.get('runner_start_s', 0.0)
    ref = 'runner start' if 'runner_start_s' in meta else 'recording start'
    n = FS // 1000
    floor = np.median(np.abs(x[:x.size // n * n]).reshape(-1, n).max(axis=1))
    L = [f'{wav.name}: {x.size / fs:.1f} s; {meta.get("notes", "")}',
         f'1 ms peak floor {al.db(floor):.1f} dBFS; transients over {EVENT_X:g} x that, times re the {ref}'
         + (f" (runner {meta['runner_start_s']:.2f}-{meta['runner_exit_s']:.2f} s in the recording)"
            if 'runner_exit_s' in meta else '') + ':']
    ev = events(x, t_ref)
    for t, ms, pk, span, e in ev:
        L.append(f'  {t:+8.3f} s  {ms:4d} ms  peak {pk:+.3f} = {al.db(pk):6.1f} dBFS on the US205, '
                 f'{abs(pk) / 10 ** (DAC_FS_DB / 20):.3f} of DAC full scale; over {SPAN_LEVEL:g} for {span:5.1f} ms, '
                 f'energy {e:6.1f} dB' + ('  CLIPPED' if abs(pk) > 0.99 else ''))
    if not ev:
        L.append('  none')
    txt = '\n'.join(L)
    print(txt)
    if meta.get('kind') == 'boot_pop':      # never over another tool's .txt (the 11/17 trip_check ones)
        wav.with_suffix('.txt').write_text(txt + '\n', encoding='utf-8')


def save(name, x, meta, log=None):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    base = OUT_DIR / name
    wavfile.write(str(base) + '.wav', FS, x)
    Path(str(base) + '.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')
    if log is not None:
        Path(str(base) + '.log').write_text(log, encoding='utf-8')
    report(str(base) + '.wav')


def free_name(name):
    if any(Path(str(OUT_DIR / name) + ext).exists() for ext in ('.wav', '.json')):
        sys.exit(f'{name} exists already in {OUT_DIR.name}; pick another --name.')


def cmd_power(a):
    free_name(a.name)
    sd = pm.import_sounddevice()
    print(f'Recording {a.seconds:.0f} s from {time.strftime("%H:%M:%S")}: switch the board off, wait 3 s, switch it '
          f'on (speaker amp OFF).', flush=True)
    x, _ = record(sd, a.seconds)
    save(a.name, x, dict(kind='boot_pop', mode='power', started=time.strftime('%Y-%m-%d %H:%M:%S'), notes=a.notes,
                         source=pm.git_state()))


def cmd_load(a):
    free_name(a.name)
    runner, preload = al.cces_paths(a.cces)
    dxe = DXES.get(a.dxe, Path(a.dxe))
    if dxe is not None and not Path(dxe).exists():
        sys.exit(f'Not found: {dxe}')
    cmd = al.runner_cmd(runner, preload, a.timeout, dxe)
    sd = pm.import_sounddevice()
    if a.settle is not None and a.order is None:
        sys.exit('--settle needs --order')
    if a.order is not None:
        ORDER_FILE.write_text(f'{a.order}' + (f' {a.settle}' if a.settle is not None else '') + '\n', encoding='ascii')
    elif ORDER_FILE.exists():
        ORDER_FILE.unlink()
        print(f'Removed the leftover {ORDER_FILE.name}: a POP_PROBE 2 build uses DAC_POWER_ORDER')
    out = {}

    def run():
        try:
            p = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True, text=True, encoding='utf-8',
                               errors='replace', timeout=a.timeout + 120)
            out.update(log=p.stdout + p.stderr, rc=p.returncode)
        except subprocess.TimeoutExpired:
            out.update(log='[boot_pop] runner timed out', rc=None)

    print(f'Recording from {time.strftime("%H:%M:%S")}; runner at +{PRE_S:.0f} s: {subprocess.list2cmdline(cmd)}',
          flush=True)
    x, (t0, t1) = record(sd, None, run)
    print(f"Runner exited ({out['rc']}) after {t1 - t0:.1f} s")
    for line in out['log'].splitlines():
        if any(k in line for k in ('[POP]', 'ADAU_1962', '[ERROR]', 'SYSTEM READY')):
            print('  log: ' + line.strip())
    save(a.name, x, dict(kind='boot_pop', mode='load', dxe=a.dxe, order=a.order, settle_ms=a.settle,
                         started=time.strftime('%Y-%m-%d %H:%M:%S'),
                         runner_start_s=t0, runner_exit_s=t1, runner_rc=out['rc'],
                         runner_cmd=subprocess.list2cmdline(cmd), notes=a.notes, source=pm.git_state()), out['log'])


def main():
    sys.stdout.reconfigure(errors='replace')
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('power', help='record while the board is switched off and on')
    p.add_argument('--name', required=True)
    p.add_argument('--seconds', type=float, default=60.0)
    p.add_argument('--notes', default='')
    q = sub.add_parser('load', help='record while CCES_runner loads and runs a program')
    q.add_argument('--name', required=True)
    q.add_argument('--dxe', default='release', help='preload, release, pathmeasure or a .dxe path')
    q.add_argument('--timeout', type=float, default=30.0, help='runner timeout, s (the board runs on after it)')
    q.add_argument('--cces', help='CrossCore Embedded Studio folder (default: CCES_ROOT)')
    q.add_argument('--order', type=int, choices=range(4), help='DAC power order for a POP_PROBE 2 build')
    q.add_argument('--settle', type=int, help='with --order: ms from the set-up to DAC1-12 on (orders 2, 3; '
                                              '0-10000, default DAC_SETTLE_MS)')
    q.add_argument('--notes', default='')
    z = sub.add_parser('analyze')
    z.add_argument('wav', nargs='+')
    a = ap.parse_args()
    if a.cmd == 'power':
        cmd_power(a)
    elif a.cmd == 'load':
        cmd_load(a)
    else:
        for w in a.wav:
            report(w)
            print()


if __name__ == '__main__':
    main()
