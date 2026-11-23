#!/usr/bin/env python3
"""Demo set-up on site: measure S and F with the PathMeasure build, check them, hand them to the ANC build and start
ANC, then leave the US205 to REW for the live Err spectrum.

  python tools/demo.py check                silent: CCES, both builds, the US205, Windows volumes, the S/F in use
  python tools/demo.py all [--quick] [--skip-f] [--notes "..."] [--cces DIR]
                                            steps 1-8 below, waiting for Enter wherever a hand is needed
  python tools/demo.py paths [--quick] [--skip-f] [--notes "..."]
                                            steps 1-6: new S/F files; the board is left running the silent bridge
  python tools/demo.py anc [--notes "..."]  steps 7-8 with the S/F files there are now
  python tools/demo.py review <npz> [...]   the step 5 checks for earlier pathmeas.py results (silent)

Setup as in the lab: J17 -> ground-loop isolator -> L/R split cable, one leg in the US205 mic input; US205 output ->
J16; J18 Right -> speaker amp; EZ-KIT debug USB to the laptop. CrossCore comes from --cces or CCES_ROOT. REW must be
closed until step 8 (the scripts open the US205 in WASAPI exclusive mode), and no CCES debug session may be active.

1. Check (silent). It stops on a missing or stale build, a busy US205 or no CCES_runner, and mutes the US205 output
   if needed: it stays muted except while pathmeas.py plays its sweeps (us205_mute.py run).
2. Load the PathMeasure build with CCES_runner --reset-on-connect, which also stops a running ANC build. The first
   load after a board power-up pops loudly while the codecs start, so the speaker amp stays off for that one. The
   bridge passes only the PC sweeps to the speaker, so it is silent while the US205 output is muted.
3. S: amp on, fan off, nobody in front of the outlet (people there change S above 500 Hz: after Enter there are 10 s
   to step beside or behind the chassis), J17 Left in the mic input. The default settings are those of the compiled-in S-hat / F-hat (lab,
   2026-11-17): S -3 dBFS 4M x 4 (6.2 min), F -12 dBFS 2M x 2 (1.6 min). --quick: 2M x 2 for both, 1.6 min each,
   6 dB less SNR, for rehearsals.
4. F: J17 Right in the mic input instead. --skip-f keeps the F there is: the Ref mic sits inside the box, and moving
   the chassis from the desk to the lab changed F by less than 0.7 dB / 5 deg in 400-1500 Hz (2026-11-17).
5. Each new path against the compiled-in one (second_path.h / feedback_path.h).
   Errors offer the measurement again and are never exported:
   - wrong jack or no signal: over 200 Hz-8 kHz (mean of the 1/3-octave levels) F is 18.3 dB louder than S, while
     S measured on other days, at the desk, in the lab and with the user seated stayed within 0.6 dB, and F within
     0.1 dB (2026-11-05..17); the shape (rms about that mean) is 0.3-1.6 dB from S to S and 3.1-3.5 dB from S to
     F. A path more than 9 dB off its own level, within 9 dB of the other and closer to its shape came from the
     other jack; more than 9 dB under both: no mic signal or no speaker sound; otherwise the level is off.
   Warnings offer the measurement again, and go on if declined:
   - level 3-9 dB off: speaker knob, amp gain or Windows output volume;
   - run to run more than 1 dB or 10 deg in an octave of the ANC band (2026-11-05..17: at most 0.2 dB / 3.4 deg):
     someone moved in front of the outlet during the sweeps;
   - SNR under 15 dB at a fan tone in the ANC band (2026-11-17: 25-41 dB): the fan was on, or noise.
   For information, the phase against the compiled-in path over the ANC band. Where the S phase is more than
   90 deg off, the compiled-in S-hat would make FxNLMS push the Err up at this site.
   The report goes to "<MMDD> <tag> check.txt" in the sweep folder.
6. export_paths.py --runtime writes measurements/anc_paths/second_path.bin / feedback_path.bin, which the ANC build
   reads over JTAG at start-up instead of the compiled-in headers (ANC_PATHS_FROM_HOST).
7. ANC: J17 Left in the mic input, fan on, amp on. anc_labtest.py run loads the Release build and records the Err mic
   for about 95 s (ANC starts about 32 s after the load starts), then prints the attenuation. The boot banner must
   name the files, and in "all" the ones just measured. Who sits or stands where goes into the notes, since it
   changes the fan tones at the Err mic (2026-11-23: 6 f0 -9 dB with the user seated next to the chassis).
8. Hand-over: REW RTA on the US205 for the audience. SW4 = speaker output off/on (W kept), SW3 = restart (W = 0),
   LED10 = divergence guard trip (speaker muted: press SW3). ANC runs until the board is switched off: SW4 off, then
   the amp, then the board.

Data: the sweeps in measurements/sweep/YYYY-MM-DD/ with the tag "demoHHMM" (a measurement again: "demoHHMM-2"), plus
"<MMDD> <tag> PathMeasure load.log" and "<MMDD> <tag> check.txt"; the ANC run as "<MMDD> ANC <tag>" in
measurements/anc_test/.
"""
import argparse
import datetime
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import anc_labtest as al     # noqa: E402  cces_paths, runner_cmd, build_state, parse_log, measure, OUT_DIR, DXE
import compare_paths as cp   # noqa: E402  smooth_c, stretches
import export_paths as ep    # noqa: E402  header_taps, HEADER, RUNTIME, TAPS, repo_rel
import pathmeas as pm        # noqa: E402
import us205_mute            # noqa: E402  set_mute

REPO = TOOLS.parent
CORE_FS = 48000
MEASURE = {'S': dict(level=-3.0, length='4M', repeats=4),    # the compiled-in S-hat / F-hat (lab, 2026-11-17)
           'F': dict(level=-12.0, length='2M', repeats=2)}
QUICK = dict(length='2M', repeats=2)
BRIDGE_TIMEOUT_S = 30.0    # runner timeout for the PathMeasure load: the program starts about 8 s after the runner,
                           # and its banner (about 25 JTAG prints of about 0.4 s) is out some 12 s later
ANC_TIMEOUT_S = 60.0       # anc_labtest run --timeout. ANC starts about 32 s after the runner start (31.9 s with the
                           # path files, 2026-11-23), so 25 s of steady ANC with the runner attached and 17 s after it
STEP_AWAY_S = 10.0         # after Enter for S, before the sweeps: time to step away from the outlet
MIC_BAND = (200.0, 8000.0)  # 1/3-octave levels for the wrong-jack check
WRONG_MIC_DB = 9.0         # half of the 18.3 dB between F and S
LEVEL_DB = 3.0
SPREAD_DB, SPREAD_DEG = 1.0, 10.0
SNR_DB = 15.0
MIC = {'S': 'Err mic', 'F': 'Ref mic'}


def ask(text):
    """Show what to do by hand and wait for Enter; Ctrl+C stops the script."""
    print('\n' + text)
    try:
        input('Press Enter when ready (Ctrl+C stops): ')
    except (EOFError, KeyboardInterrupt):
        sys.exit('\nStopped.')


def yes(question, default):
    try:
        r = input(f"{question} [{'Y/n' if default else 'y/N'}] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        sys.exit('\nStopped.')
    return default if not r else r.startswith('y')


def ref_band():
    """The ANC reference band (Hz) from anc_fxnlms.h; the build check makes sure the Release build matches it."""
    src = (REPO / 'anc_fxnlms.h').read_text(encoding='utf-8', errors='replace')
    m = re.search(r'#define\s+ANC_REF_BAND\s+ANC_REF_BAND_(\d+)_(\d+)', src)
    return (float(m[1]), float(m[2])) if m else (100.0, 6000.0)


def run_minutes(length, repeats):
    n = pm.parse_length(length)
    one = (int(pm.LEAD_S * pm.FS) + pm.N_REFS * int(pm.DELAY_S * pm.FS) + n + int(pm.TAIL_S * pm.FS)) / pm.FS
    return repeats * (one + pm.PREROLL_S + pm.MARGIN_S) / 60


def new_tag(folder):
    """'demoHHMM', with a letter added when that is taken (sweeps, logs or ANC runs)."""
    base = f'demo{datetime.datetime.now():%H%M}'
    names = [p.name for d in (folder, al.OUT_DIR) if d.is_dir() for p in d.iterdir()]
    for s in ('',) + tuple('bcdefghjk'):
        if not any(re.search(rf'(^| ){re.escape(base + s)}([ .-]|$)', n) for n in names):
            return base + s
    sys.exit('No free tag for this minute: try again in a minute.')


# ---------------------------------------------------------------- the S-hat / F-hat in use

def runtime_taps(path):
    """(taps, note) of measurements/anc_paths/*.bin, checked as the firmware's Load_Path_File checks it, or
    (None, why the firmware would use the compiled-in path instead)."""
    rel, code = ep.RUNTIME[path]
    try:
        b = (REPO / rel).read_bytes()
    except OSError:
        return None, 'no file'
    if (len(b) < 128 + 4 * ep.TAPS or b[:4] != b'ANCP'
            or np.frombuffer(b[4:20], '<u4').tolist() != [1, code, CORE_FS, ep.TAPS]):
        return None, 'not a version 1 file for this path'
    taps = np.frombuffer(b[128:128 + 4 * ep.TAPS], '<f4').astype(float)
    if not np.all(np.abs(taps) < 1.0):        # also false for a NaN
        return None, 'a tap out of range'
    return taps, b[20:128].split(b'\0')[0].decode('ascii', 'replace')


def in_use(path):
    """What the ANC build loads at start-up now: the file with its note, or the compiled-in header."""
    rel = ep.RUNTIME[path][0]
    taps, note = runtime_taps(path)
    if taps is not None:
        return f'{rel}: {note}'
    name = ep.HEADER[path][0]
    src = re.search(r'\* Source:\s+(.+)', (REPO / name).read_text(encoding='utf-8'))
    return f"compiled-in {name}{f' ({Path(src[1].strip()).stem})' if src else ''}; {rel}: {note}"


def header_response(path, n):
    """The compiled-in S-hat / F-hat on the rfft grid of n points at the core rate, as pathmeas.py measures the path:
    F-hat tap 0 multiplies y(n-1) (export_paths.py), so F gets its sample back."""
    H = np.fft.rfft(ep.header_taps(path).astype(float), n)
    if path == 'F':
        H = H * np.exp(-2j * np.pi * np.arange(H.size) / n)
    return H


def third_octaves(f, H):
    """Power levels (dB) of H in the 1/3-octave bands of MIC_BAND."""
    fc = 1000 * 2.0 ** (np.arange(-15, 16) / 3)
    fc = fc[(fc >= MIC_BAND[0]) & (fc <= MIC_BAND[1])]
    p = np.abs(H) ** 2
    return np.array([10 * np.log10(np.mean(p[(f >= c * 2 ** (-1 / 6)) & (f < c * 2 ** (1 / 6))]) + 1e-30) for c in fc])


# ---------------------------------------------------------------- step 5: checks of a new path

def review(npz, band):
    """The checks of one pathmeas.py result (.npz) against the compiled-in paths: (errors, warnings, report lines)."""
    z = np.load(npz, allow_pickle=True)
    f, M, Nm, Xs = z['f'], z['X'], z['N'], z['X_runs']
    meta = json.loads(str(z['meta']))
    path = meta['path']
    other = 'F' if path == 'S' else 'S'
    n = 2 * (f.size - 1)
    if meta.get('fs') != CORE_FS or not np.allclose(f, np.fft.rfftfreq(n, 1 / CORE_FS)):
        sys.exit(f'{Path(npz).name}: not a {CORE_FS} Hz pathmeas.py result')
    hdr = {p: header_response(p, n) for p in 'SF'}
    errors, warnings = [], []
    L = [f"== {Path(npz).stem}: {Xs.shape[0]} run(s), started {meta.get('started', '?')}; notes: "
         f"{meta.get('notes') or 'none'}"]

    lv = third_octaves(f, M)
    d = {p: lv - third_octaves(f, hdr[p]) for p in 'SF'}
    own, oth = float(d[path].mean()), float(d[other].mean())
    shape = {p: float(np.std(d[p])) for p in 'SF'}    # S vs S 0.3-1.6 dB rms, S vs F 3.1-3.5 (2026-11-05..17)
    L.append(f'Level re the compiled-in {path} / {other} (1/3 octaves {MIC_BAND[0]:g} Hz-{MIC_BAND[1] / 1e3:g} kHz): '
             f'mean {own:+.1f} / {oth:+.1f} dB, shape {shape[path]:.1f} / {shape[other]:.1f} dB rms')
    if abs(own) > WRONG_MIC_DB:
        if own < -WRONG_MIC_DB and oth < -WRONG_MIC_DB:
            errors.append(f'{path} is far under both paths: no mic signal or no speaker sound. Check the '
                          f'{MIC[path]}, the ADAU1787 board power, the speaker amp and J18')
        elif abs(oth) <= WRONG_MIC_DB and shape[other] < shape[path]:
            errors.append(f'this {path} looks like {other}: {pm.JACK[other]} was in the US205 mic input instead of '
                          f'{pm.JACK[path]}')
        else:
            errors.append(f'{path} is {own:+.1f} dB off: check the speaker knob, the amp, J18 and the Windows output '
                          f'volume')
    elif abs(own) > LEVEL_DB:
        warnings.append(f'{path} is {own:+.1f} dB off the compiled-in one (other days and seats: within 0.6 dB): '
                        f'speaker knob, amp gain or Windows output volume?')

    octs = [i for i, fc in enumerate(pm.OCTAVES) if band[0] <= fc <= band[1]]
    if Xs.shape[0] > 1 and octs:
        w = pm.inv_var((M, Nm))
        valid = np.abs(M) > 0
        dev = np.array([[pm.band_ratio(f, X, M, w, valid)[i] for i in octs] for X in Xs])
        sd, sp = float(np.max(np.abs(dev[..., 0]))), float(np.max(np.abs(dev[..., 1])))
        L.append(f"Run to run: largest run - mean in the octaves {', '.join(f'{pm.OCTAVES[i]:g}' for i in octs)} Hz: "
                 f'{sd:.2f} dB, {sp:.1f} deg')
        if sd > SPREAD_DB or sp > SPREAD_DEG:
            warnings.append(f'the runs disagree by up to {sd:.1f} dB / {sp:.0f} deg (lab: 0.2 dB / 3.4 deg): did '
                            f'someone move in front of the outlet during the sweeps?')
    else:
        L.append('Run to run: one run, nothing to compare')

    tones = [(ft, s) for ft, (_, s) in zip(pm.TONES, pm.tone_values(f, M, Nm)) if band[0] <= ft <= band[1]]
    L.append('SNR at the fan tones in the ANC band: ' + ', '.join(f'{ft} Hz {s:.0f} dB' for ft, s in tones))
    low = [ft for ft, s in tones if s < SNR_DB]
    if low:
        warnings.append(f"SNR under {SNR_DB:g} dB at {', '.join(map(str, low))} Hz (lab: 25-41 dB): was the fan "
                        f'off? Noise in the room?')

    m = (f >= band[0] / 2) & (f <= band[1] * 2)
    fm = f[m]
    r = cp.smooth_c(fm, M[m]) / cp.smooth_c(fm, hdr[path][m])
    ph, mag = np.degrees(np.angle(r)), 20 * np.log10(np.abs(r))
    at = [(ft, int(np.argmin(np.abs(fm - ft)))) for ft, _ in tones]
    L.append(f'New / compiled-in {path} (1/12 octave): '
             + ', '.join(f'{ft} Hz {mag[i]:+.1f} dB {ph[i]:+.0f} deg' for ft, i in at))
    over = cp.stretches(fm, (fm >= band[0]) & (fm <= band[1]) & (np.abs(ph) > 90))
    L.append(f'  phase more than 90 deg off in the band {band[0]:g}-{band[1]:g} Hz: '
             + (', '.join(f'{a:.0f}-{b:.0f} Hz' for a, b in over) or 'nowhere')
             + (': the compiled-in S-hat would make FxNLMS push the Err up there' if over and path == 'S' else ''))
    return errors, warnings, L


# ---------------------------------------------------------------- step 1: checks before anything runs

def check(a, need):
    """Silent. Prints the state; returns (problems, US205 output name, Windows volumes). need: 'release' and/or
    'pathmeasure', the builds the step needs (the other one is still shown)."""
    problems, warns, L = [], [], []
    try:
        runner, _ = al.cces_paths(a.cces)
        L.append(f'CCES_runner: {runner}')
    except SystemExit as e:
        problems.append(str(e))
    ok, text = al.build_state()
    L.append(f'Release build (ANC): {text}')
    if 'release' in need:
        if not ok:
            problems.append(f'Release build: {text}')
        elif b'S-hat: %s' not in al.DXE.read_bytes():
            problems.append('the Release build does not read S/F files: build it with ANC_PATHS_FROM_HOST 1')
    rec, w = pm.pathmeas_build()
    L.append(f"PathMeasure build: {rec.get('build', 'missing')}" + ('' if rec.get('app_mode') else ', not the bridge'))
    if 'pathmeasure' in need:
        (warns if rec.get('app_mode') else problems).extend(w)   # stale sources only warn, as in pathmeas.py
    L.append(f'Code: {pm.source_text(pm.git_state())}')
    for p in 'SF':
        L.append(f'{p}-hat the ANC build loads now: {in_use(p)}')

    sd = pm.import_sounddevice()
    out_name, vol = None, {}
    try:
        dev_in, dev_out = pm.wasapi_devices(sd, pm.DEVICE)
    except SystemExit as e:
        problems.append(str(e))
    else:
        out_name = dev_out['name']
        for chk, dev, ch in ((sd.check_output_settings, dev_out, 2), (sd.check_input_settings, dev_in, 1)):
            try:
                chk(device=dev['index'], channels=ch, dtype='float32', samplerate=pm.FS,
                    extra_settings=sd.WasapiSettings(exclusive=True))
            except sd.PortAudioError as e:
                problems.append(f"{dev['name']} does not take {pm.FS} Hz in exclusive mode: {e}")
        try:
            x = sd.rec(int(0.5 * pm.FS), samplerate=pm.FS, channels=1, dtype='float32', device=dev_in['index'],
                       extra_settings=sd.WasapiSettings(exclusive=True), blocking=True)[:, 0]
            L.append(f"US205 input free, peak now {al.db(np.max(np.abs(x))):.1f} dBFS")
        except sd.PortAudioError as e:
            problems.append(f'the US205 input cannot be opened exclusively ({e}): close REW or whatever uses it')
        vol = pm.read_volumes(dev_out, dev_in)
        L.append(f'Windows volume: {pm.volume_text(vol)}')
        if vol['in'].get('mute'):
            problems.append('the US205 mic input is muted in Windows')
        warns += [v for v in pm.volume_warnings(vol) if 'muted' not in v]
        if abs(vol['in'].get('scalar', 0.5) - 0.5) > 0.01:
            warns.append('the US205 mic volume should stay 0.500')
        if abs(vol['out'].get('scalar', 1.0) - 1.0) > 0.01:
            warns.append('the US205 output volume was 1.000 for the compiled-in S/F; another one shows as a level '
                         'change of the new paths')
    print('\n'.join(L))
    for v in warns:
        print(f'WARNING: {v}')
    for v in problems:
        print(f'PROBLEM: {v}')
    return problems, out_name, vol


def ensure_muted(out_name, vol):
    if not vol['out'].get('mute'):
        if not us205_mute.set_mute(out_name, True):
            sys.exit('Could not mute the US205 output: mute it in Windows, then start again.')
        print('Muted the US205 output in Windows (it stays muted except during the sweeps).')


# ---------------------------------------------------------------- steps 2-6: S and F

def load_bridge(cces, log):
    runner, preload = al.cces_paths(cces)
    cmd = al.runner_cmd(runner, preload, BRIDGE_TIMEOUT_S, pm.PATHMEAS_DXE)
    print(f'Loading the PathMeasure build, about {BRIDGE_TIMEOUT_S + 10:.0f} s: {subprocess.list2cmdline(cmd)}')
    t0 = time.perf_counter()
    try:
        p = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=BRIDGE_TIMEOUT_S + 120)
        out, rc = p.stdout + p.stderr, p.returncode
    except subprocess.TimeoutExpired as e:
        out = ''.join(x.decode('utf-8', 'replace') if isinstance(x, bytes) else x
                      for x in (e.stdout, e.stderr) if x) + '\n[demo] runner timed out'
        rc = None
    log.write_text(out, encoding='utf-8')
    print(f'Runner exited ({rc}) after {time.perf_counter() - t0:.0f} s; log {ep.repo_rel(log)}')
    if 'SYSTEM READY' not in out or pm.BRIDGE_BANNER.decode() not in out:
        print('\n'.join('  ' + line for line in out.splitlines()[-12:]))
        sys.exit('The board did not report SYSTEM READY with the Path Measurement Bridge: see the log.')
    print('The board runs the Path Measurement Bridge.')


def measure(path, tag, notes, quick):
    """pathmeas.py measure through us205_mute.py run (the US205 output is unmuted for it only); the result .npz."""
    s = dict(MEASURE[path], **(QUICK if quick else {}))
    day = datetime.date.today()
    args = ['--path', path, '--levels', f"{s['level']:g}", '--length', s['length'], '--repeats', str(s['repeats']),
            '--tag', tag, '--notes', notes]
    rc = subprocess.call([sys.executable, str(TOOLS / 'us205_mute.py'), 'run', '--', *args], stdin=subprocess.DEVNULL)
    npz = pm.DATA_ROOT / day.isoformat() / (pm.group_name(dict(mmdd=day.strftime('%m%d'), path=path, tag=tag,
                                                               level_dbfs=s['level'], length=s['length'])) + '.npz')
    if rc or not npz.exists():
        sys.exit(f'The {path} measurement failed ({rc}): see above. The board still runs the bridge.')
    return npz


def prompt(path, quick, again):
    s = dict(MEASURE[path], **(QUICK if quick else {}))
    head = (f"{path} ({'secondary' if path == 'S' else 'feedback'} path, speaker -> {MIC[path]}), "
            f"{s['repeats']} x {s['length']} sweeps at {s['level']:g} dBFS through the speaker, about "
            f"{run_minutes(s['length'], s['repeats']):.1f} min{', again' if again else ''}:")
    if path == 'S':
        todo = ['speaker amp on (knob untouched)', 'fan OFF', 'J17 Left (Err mic) in the US205 mic input',
                f'nobody in front of the outlet: after Enter there are {STEP_AWAY_S:.0f} s to step beside or behind '
                f'the chassis; keep people away until the last of the {s["repeats"]} speaker sweeps has ended']
    else:
        todo = ['J17 Right (Ref mic) in the US205 mic input instead of J17 Left', 'amp on and fan off as for S']
    return head + ''.join(f'\n  - {t}' for t in todo)


def paths_step(a, need):
    """Steps 1-6; returns the session tag and {path: exported npz}."""
    problems, out_name, vol = check(a, need)
    if problems:
        sys.exit('Not starting: fix the PROBLEM lines first.')
    band = ref_band()
    day = datetime.date.today()
    folder = pm.DATA_ROOT / day.isoformat()
    folder.mkdir(parents=True, exist_ok=True)
    tag, mmdd = new_tag(folder), day.strftime('%m%d')
    print(f'Session tag {tag}; ANC reference band {band[0]:g}-{band[1]:g} Hz (anc_fxnlms.h)')
    ensure_muted(out_name, vol)
    ask('Load the PathMeasure build (this stops whatever the board runs now):\n'
        '  - the first load since the board was powered up pops loudly while the codecs start: keep the speaker amp\n'
        '    OFF for that one (a reload over a running program gives only a small click)')
    load_bridge(a.cces, folder / f'{mmdd} {tag} PathMeasure load.log')
    report, chosen = [], {}
    for path in ('S' if a.skip_f else 'SF'):
        k = 1
        while True:
            t = tag if k == 1 else f'{tag}-{k}'
            ask(prompt(path, a.quick, k > 1))
            notes = (f'demo.py {t}' + (f': {a.notes}' if a.notes else '') + '. Confirmed at the prompt: '
                     + ('speaker amp on, fan off, nobody in front of the outlet, ' if path == 'S' else
                        'speaker amp on, fan off, ') + f'{pm.JACK[path]} in the US205 mic input')
            if path == 'S':
                print(f'Step away from the outlet now: the sweeps start in {STEP_AWAY_S:.0f} s.', flush=True)
                time.sleep(STEP_AWAY_S)
            npz = measure(path, t, notes, a.quick)
            errors, warnings, L = review(npz, band)
            L += [f'ERROR: {e}' for e in errors] + [f'WARNING: {w}' for w in warnings]
            print('\n' + '\n'.join(L))
            report += L
            if errors or warnings:
                if yes(f'Measure {path} again?', bool(errors)):
                    k += 1
                    continue
                if errors:
                    report.append('Stopped: nothing exported')
                    (folder / f'{mmdd} {tag} check.txt').write_text('\n'.join(report) + '\n', encoding='utf-8')
                    sys.exit('Stopped: nothing exported. The board still runs the bridge.')
            chosen[path] = npz
            report.append(f'To export: {npz.stem}')
            break
    for path, npz in chosen.items():
        rc = subprocess.call([sys.executable, str(TOOLS / 'export_paths.py'), path, str(npz), '--runtime'],
                             stdin=subprocess.DEVNULL)
        taps, note = runtime_taps(path)
        if rc or taps is None or not note.startswith(npz.stem):
            sys.exit(f'{ep.RUNTIME[path][0]} does not hold {npz.stem} ({note}): see above.')
    report += [f'{p}-hat the ANC build loads now: {in_use(p)}' for p in 'SF']
    out = folder / f'{mmdd} {tag} check.txt'
    out.write_text('\n'.join(report) + '\n', encoding='utf-8')
    print('\n'.join(report[-2:]) + f'\nReport: {ep.repo_rel(out)}')
    return tag, chosen


# ---------------------------------------------------------------- steps 7-8: ANC and the hand-over

def anc_step(a, tag, expect):
    """Steps 7-8. expect: {path: npz stem} that the boot banner must name (the paths just measured, so the board runs
    the bridge and this load is a reload)."""
    day = datetime.date.today()
    print('\n' + '\n'.join(f'{p}-hat the ANC build will load: {in_use(p)}' for p in 'SF'))
    ask('ANC: the Release build is loaded and the Err mic recorded for about 95 s; ANC starts about 32 s after the\n'
        'load starts.\n'
        '  - J17 Left (Err mic) in the US205 mic input\n'
        '  - fan ON\n'
        + ('  - speaker amp on' if expect else
           '  - speaker amp on; only if this is the first load since the board was powered up: amp OFF now, and on\n'
           '    once LED9 blinks (the 15 s wait before ANC starts)'))
    try:
        who = input('Who sits or stands near the chassis or in front of the outlet? (goes into the notes) ').strip()
    except (EOFError, KeyboardInterrupt):
        sys.exit('\nStopped.')
    name = f"{day.strftime('%m%d')} ANC {tag}"
    notes = '; '.join([f'demo.py {tag}' + (f': {a.notes}' if a.notes else ''), f"people: {who or 'not given'}",
                       'fan on, amp on, J17 Left in the US205 mic input (confirmed at the prompt)',
                       f"S-hat {in_use('S')}", f"F-hat {in_use('F')}"])
    cmd = [sys.executable, str(TOOLS / 'anc_labtest.py'), 'run', '--name', name, '--timeout', f'{ANC_TIMEOUT_S:g}',
           '--notes', notes] + (['--cces', a.cces] if a.cces else [])
    rc = subprocess.call(cmd, stdin=subprocess.DEVNULL)
    if not (al.OUT_DIR / f'{name}.wav').exists():
        sys.exit(f'anc_labtest.py run failed ({rc}): see above. Press SW4 (output off) if ANC started.')
    anc_report(name, expect)
    print('\nHand-over:\n'
          '  - REW can have the US205 now: RTA on its mic input (J17 Left = Err mic). With SW4 off (LED9 off),\n'
          '    "Save current" keeps the ANC-off spectrum as the reference trace; SW4 on shows the reduction.\n'
          '  - SW4 = speaker output off/on (W kept), SW3 = restart (W = 0); LED10 = guard trip (speaker muted): SW3.\n'
          '  - ANC runs until it is switched off: SW4 off first, then the amp, then the board.')


def anc_report(name, expect):
    """After anc_labtest.py run: the S-hat / F-hat the boot banner names, a guard trip, the attenuation."""
    wav = al.OUT_DIR / f'{name}.wav'
    log = al.parse_log((al.OUT_DIR / f'{name}.log').read_text(encoding='utf-8', errors='replace'))
    print('\n== demo.py: ANC start ==')
    trouble = []
    for p in 'SF':
        line = next((l[4:] for l in log['banner'] if l.startswith(f'>>> {p}-hat:')), None)
        if line is None:
            trouble.append(f'no {p}-hat line in the boot log')
        elif ep.RUNTIME[p][0] not in line:
            trouble.append(f'the build used the compiled-in path, not the file: {line}')
        elif expect and p in expect and expect[p] not in line:
            trouble.append(f'{line}: not the one just measured ({expect[p]})')
        else:
            print(f'OK  {line}')
    if log['trip']:
        trouble.append(f"{log['trip'][0]} -> LED10 is on and the speaker muted: press SW3 to restart")
    m = al.measure(wav)
    if m['on_key']:
        r = m['res'][m['on_key']]
        print(f"ANC on ({m['on_key']}) vs the 10 s before the start: f0 {r['f0']:+.1f}, 6 f0 {r['6 f0']:+.1f}, "
              f"total {r['total']:+.1f} dB (above 0 = quieter); fan f0 {m['f0']:.1f} Hz")
        if max(r['f0'], r['6 f0']) < 6.0:
            trouble.append('neither f0 nor 6 f0 came down 6 dB: is the fan on? Were S/F measured as prompted?')
    for t in trouble:
        print(f'WARNING: {t}')


# ---------------------------------------------------------------- commands

def cmd_check(a):
    problems, _, _ = check(a, ('release', 'pathmeasure'))
    print('READY' if not problems else 'NOT READY')
    sys.exit(1 if problems else 0)


def cmd_paths(a):
    paths_step(a, ('pathmeasure',))
    print('\nThe board runs the bridge (silent while the US205 output is muted). Next: python tools/demo.py anc')


def cmd_anc(a):
    problems, _, _ = check(a, ('release',))
    if problems:
        sys.exit('Not starting: fix the PROBLEM lines first.')
    anc_step(a, new_tag(pm.DATA_ROOT / datetime.date.today().isoformat()), None)


def cmd_all(a):
    tag, chosen = paths_step(a, ('pathmeasure', 'release'))
    anc_step(a, tag, {p: npz.stem for p, npz in chosen.items()})


def cmd_review(a):
    band = ref_band()
    bad = False
    for npz in a.npz:
        errors, warnings, L = review(Path(npz), band)
        print('\n'.join(L + [f'ERROR: {e}' for e in errors] + [f'WARNING: {w}' for w in warnings]) + '\n')
        bad |= bool(errors)
    sys.exit(1 if bad else 0)


def main():
    sys.stdout.reconfigure(errors='replace')
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--cces', help='CrossCore Embedded Studio folder (default: CCES_ROOT)')
    common.add_argument('--notes', default='', help='the site and anything unusual, stored with every run')
    sweeps = argparse.ArgumentParser(add_help=False)
    sweeps.add_argument('--quick', action='store_true', help='2M x 2 sweeps for S and F (rehearsals)')
    sweeps.add_argument('--skip-f', action='store_true', help='measure S only; F stays as it is')
    sub.add_parser('check', parents=[common], help='silent: CCES, builds, US205, Windows volumes, S/F in use')
    sub.add_parser('all', parents=[common, sweeps], help='S/F, checks, export, then ANC')
    sub.add_parser('paths', parents=[common, sweeps], help='S/F, checks, export')
    sub.add_parser('anc', parents=[common], help='load ANC with the S/F files, record its start, hand over')
    r = sub.add_parser('review', help='the S/F checks for earlier pathmeas.py results')
    r.add_argument('npz', nargs='+')
    a = ap.parse_args()
    try:
        {'check': cmd_check, 'all': cmd_all, 'paths': cmd_paths, 'anc': cmd_anc, 'review': cmd_review}[a.cmd](a)
    except KeyboardInterrupt:      # us205_mute.py mutes the US205 output again however pathmeas.py ends
        sys.exit('\nStopped. If ANC was started, SW4 switches its output off.')


if __name__ == '__main__':
    main()
