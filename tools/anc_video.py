#!/usr/bin/env python3
"""Demo video of a lab ANC run: the Err spectrum over time, with the Err mic recording as the sound track.

Why: at a demo the audience cannot hear the ANC, because the Err mic sits at the fan outlet, far from their ears.
A video made from a recording (anc_labtest.py run, best with --buttons and SW4 / SW3 presses) shows two things:
- The spectrum. Gray is ANC off: the 10 s before the ANC start, without the status-print freezes. Blue is the
  0.75 s around each frame.
- The two target tones, marked, with their reduction in dB.
The sound track is what the Err mic picked up, so viewers with headphones hear the tones go away.

  python tools/anc_video.py <wav> [--from S] [--to S] [--fps 15] [--level DBFS] [--out FILE] [--stills S ...]

--from and --to are seconds from the ANC start. Default: 3 s before the first pushbutton edge to 5 s after the
last one, or 2-40 s when there are no edges. Keep the clip out of stretches where J17 L was frozen: the wait-phase
prints before the ANC start, and the diagnostics dump after a trip or the timed switch-off. There the recording is
not the Err mic, and a frozen stretch clicks. Check with "anc_labtest.py track".

The sound keeps one gain for the whole clip, so on and off keep their real difference. The ANC-off rms is brought
to --level (default -24 dBFS) and peaks are soft-limited.

Output: "<name> video.mp4" next to the wav (measurements/ is gitignored). --stills writes PNG frames at those times
(s from the ANC start) for a quick look, without the video.

Needs ffmpeg from the imageio-ffmpeg package (pip install imageio-ffmpeg).
"""
import argparse
import subprocess
import sys
import tempfile
import warnings
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import welch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import anc_labtest as lt  # noqa: E402

FS = lt.FS
WIN_S = 0.75          # spectrum of each frame: this many seconds around it
NPER = 8192           # Welch segment (5.9 Hz bins), 50% overlap
F_LO, F_HI = 100.0, 12000.0
TONES = ((1, '600 Hz', '轉速音'), (6, '3.6 kHz', '葉片通過音'))   # (multiple of f0, label, what it is)
C_OFF, C_ON, INK, GRID = '#7a7a7a', '#2a6fdb', '#222222', '#e3e3e3'
STATE_AFTER = {'ANC start': True, 'output on': True, 'restart (SW3)': True,
               'output off': False, 'guard trip': False, 'timed switch-off': False}
CAPTION = {'ANC start': 'ANC 啟動', 'output on': 'ANC 輸出打開', 'restart (SW3)': 'SW3：重新開始（W 歸零）',
           'output off': 'SW4：關閉 ANC 輸出', 'guard trip': '發散保護：靜音', 'timed switch-off': '定時關閉'}


def masked_welch(x, a, b):
    """Welch PSD (NPER, 50% overlap) over [a, b) s, without segments that touch a frozen stretch of J17 L."""
    i0 = max(int(a * FS), 0)
    seg = x[i0:int(b * FS)]
    ok = lt.frozen_free(x, a, b)
    w = np.hanning(NPER)
    norm = FS * np.sum(w ** 2)
    acc, n = np.zeros(NPER // 2 + 1), 0
    for i in range(0, seg.size - NPER + 1, NPER // 2):
        if ok[i:i + NPER].all():
            acc += 2 * np.abs(np.fft.rfft(seg[i:i + NPER] * w)) ** 2 / norm
            n += 1
    if n == 0:
        sys.exit(f'No clean {NPER}-sample segment in {a:.1f}-{b:.1f} s.')
    return np.fft.rfftfreq(NPER, 1 / FS), acc / n


def tone_power(f, p, fc):
    return float(np.sum(p[np.abs(f - fc) <= lt.TONE_HW + 1e-9]) * (f[1] - f[0]))


def timeline(m):
    """[(time in the recording, output on?, event kind)] from the ANC start and the pushbutton edges."""
    evs = m['sw'].get('events') or [dict(kind='ANC start', t=m['ta'])]
    return [(ev['t'], STATE_AFTER.get(ev['kind'], True), ev['kind']) for ev in evs]


def state_at(tl, t):
    on, kind, t_ev = False, None, None
    for te, st, k in tl:
        if te <= t:
            on, kind, t_ev = st, k, te
    return on, kind, t_ev


def make_audio(x, m, a, b, level_db, path):
    """The clip a-b s at one gain: the ANC-off rms (frozen stretches left out) to level_db, soft-limited, 20 ms fades."""
    ta = m['ta']
    ref = x[int((ta - 10.5) * FS):int((ta - 0.5) * FS)][lt.frozen_free(x, ta - 10.5, ta - 0.5)]
    gain = 10 ** (level_db / 20) / max(float(np.sqrt(np.mean(ref ** 2))), 1e-12)
    y = x[int(a * FS):int(b * FS)] * gain
    y = 0.95 * np.tanh(y / 0.95)
    n = int(0.02 * FS)
    y[:n] *= np.linspace(0, 1, n)
    y[-n:] *= np.linspace(1, 0, n)
    wavfile.write(str(path), FS, (y * 32767).astype(np.int16))
    return 20 * np.log10(gain)


def build_figure(m, f_ref, p_ref):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': ['Microsoft JhengHei', 'DejaVu Sans'], 'axes.unicode_minus': False,
                         'font.size': 15, 'axes.edgecolor': '#999999', 'axes.labelcolor': INK,
                         'xtick.color': INK, 'ytick.color': INK})
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    gs = fig.add_gridspec(3, 1, height_ratios=[1.15, 5.0, 0.42], hspace=0.42, left=0.08, right=0.97, top=0.98,
                          bottom=0.1)
    head, ax, bar = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])
    head.axis('off')
    sel = (f_ref >= F_LO) & (f_ref <= F_HI)
    ref_db = 10 * np.log10(p_ref[sel] + 1e-30)
    ax.plot(f_ref[sel], ref_db, color=C_OFF, lw=1.2, label='ANC 關（啟動前 10 秒平均）')
    (live,) = ax.plot(f_ref[sel], ref_db, color=C_ON, lw=1.0, label='現在')
    for k, lab, what in TONES:
        ax.axvline(k * m['f0'], color='#555555', lw=0.9, ls=':', zorder=0)
        ax.text(k * m['f0'], 0.98, f' {lab}\n {what}', transform=ax.get_xaxis_transform(), va='top', ha='left',
                fontsize=13, color='#444444')
    ax.set_xscale('log')
    ax.set_xlim(F_LO, F_HI)
    ticks = (100, 200, 500, 1000, 2000, 5000, 10000)
    ax.set_xticks(ticks)
    ax.set_xticklabels([f'{t // 1000}k' if t >= 1000 else str(t) for t in ticks])
    ax.minorticks_off()
    ax.set_ylim(float(np.percentile(ref_db, 1)) - 8, float(ref_db.max()) + 6)
    ax.set_xlabel('頻率 (Hz)')
    ax.set_ylabel('誤差麥克風 (dB)')
    ax.grid(True, lw=0.6, color=GRID)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.legend(loc='lower left', frameon=False, fontsize=14)
    state = head.text(0.0, 0.5, '', fontsize=32, fontweight='bold', va='center', ha='left')
    nums = head.text(0.24, 0.78, '', fontsize=24, va='center', ha='left', color=INK)
    cap = head.text(0.24, 0.18, '', fontsize=19, va='center', ha='left', color='#b03a2e')
    return fig, ax, bar, live, sel, state, nums, cap


def draw_bar(bar, tl, a, b, ta):
    """Timeline under the spectrum: blue while the output is on, gray while off; returns the cursor line."""
    edges = [a] + [te for te, _, _ in tl if a < te < b] + [b]
    for t0, t1 in zip(edges[:-1], edges[1:]):
        on = state_at(tl, 0.5 * (t0 + t1))[0]
        bar.axvspan(t0 - ta, t1 - ta, color=C_ON if on else '#c9c9c9', lw=0)
    bar.set_xlim(a - ta, b - ta)
    bar.set_yticks([])
    bar.set_xlabel('ANC 啟動後的時間 (s)　藍 = ANC 開、灰 = 關', fontsize=12)
    for s in ('top', 'right', 'left'):
        bar.spines[s].set_visible(False)
    return bar.axvline(a - ta, color=INK, lw=2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('wav')
    ap.add_argument('--from', dest='t_from', type=float, help='clip start, s from the ANC start')
    ap.add_argument('--to', dest='t_to', type=float, help='clip end, s from the ANC start')
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--level', type=float, default=-24.0, help='ANC-off rms of the sound track, dBFS')
    ap.add_argument('--out', help='output .mp4 (default: "<name> video.mp4" next to the wav)')
    ap.add_argument('--stills', type=float, nargs='+', help='only write PNG frames at these times (s from the start)')
    a = ap.parse_args()

    m = lt.measure(a.wav)
    base, ta = m['base'], m['ta']
    _, x, _, _ = lt.load_run(a.wav)
    tl = timeline(m)
    sw_times = [te - ta for te, _, k in tl if k != 'ANC start']
    t_from = a.t_from if a.t_from is not None else (sw_times[0] - 3.0 if sw_times else 2.0)
    t_to = a.t_to if a.t_to is not None else (sw_times[-1] + 5.0 if sw_times else 40.0)
    t_to = min(t_to, x.size / FS - ta - WIN_S)
    ca, cb = ta + t_from, ta + t_to
    f_ref, p_ref = masked_welch(x, ta - 10.5, ta - 0.5)
    ref_tone = {k: tone_power(f_ref, p_ref, k * m['f0']) for k, _, _ in TONES}

    fig, ax, bar, live, sel, state, nums, cap = build_figure(m, f_ref, p_ref)
    cursor = draw_bar(bar, tl, ca, cb, ta)

    def update(t):
        """Draw the frame of time t (s in the recording)."""
        seg = x[int((t - WIN_S / 2) * FS):int((t + WIN_S / 2) * FS)]
        f, p = welch(seg, FS, window='hann', nperseg=NPER, noverlap=NPER // 2)
        live.set_ydata(10 * np.log10(p[sel] + 1e-30))
        on, kind, t_ev = state_at(tl, t)
        state.set_text('ANC 開' if on else 'ANC 關')
        state.set_color(C_ON if on else '#666666')
        parts = []
        for k, lab, _ in TONES:
            d = 10 * np.log10(ref_tone[k] / max(tone_power(f, p, k * m['f0']), 1e-30))
            parts.append(f'{lab} {"降" if d >= 0 else "升"} {abs(d):4.1f} dB')
        nums.set_text('　　'.join(parts))
        cap.set_text(CAPTION.get(kind, '') if (t_ev is not None and t - t_ev < 2.5 and t >= ca) else '')
        cursor.set_xdata([t - ta, t - ta])

    if a.stills:
        for s in a.stills:
            update(ta + s)
            p = Path(f'{base} still {s:+.1f}s.png')
            fig.savefig(p, dpi=100)
            print('wrote', p.name)
        return

    import imageio_ffmpeg
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    plt.rcParams['animation.ffmpeg_path'] = ffmpeg
    out = Path(a.out) if a.out else Path(f'{base} video.mp4')
    n = int((cb - ca) * a.fps)
    with tempfile.TemporaryDirectory() as tmp:
        silent, audio = Path(tmp) / 'video.mp4', Path(tmp) / 'audio.wav'
        gain_db = make_audio(x, m, ca, ca + n / a.fps, a.level, audio)
        writer = FFMpegWriter(fps=a.fps, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '20'])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            with writer.saving(fig, str(silent), dpi=100):
                for i in range(n):
                    update(ca + i / a.fps)
                    writer.grab_frame()
        subprocess.run([ffmpeg, '-y', '-loglevel', 'error', '-i', str(silent), '-i', str(audio), '-c:v', 'copy',
                        '-c:a', 'aac', '-b:a', '192k', '-shortest', str(out)], check=True)
    print(f'wrote {out} ({n / a.fps:.1f} s, {t_from:+.1f} to {t_from + n / a.fps:+.1f} s from the ANC start, '
          f'{a.fps:g} fps; sound gain {gain_db:+.1f} dB, ANC off at {a.level:g} dBFS rms)')


if __name__ == '__main__':
    main()
