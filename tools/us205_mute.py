#!/usr/bin/env python3
"""Mute or unmute the US205 output in Windows, or unmute it just for one pathmeas measurement.

The US205 output stays muted in Windows between measurements (since 2026-11-06), so a stray sound from the PC cannot
reach the speaker. pathmeas.py refuses a muted endpoint. "run" unmutes the output, runs pathmeas.py measure with the
given arguments (adding --yes) and mutes the output again however the measurement ends.

  python tools/us205_mute.py status
  python tools/us205_mute.py mute | unmute
  python tools/us205_mute.py run -- --path S --levels -3 --length 4M --repeats 4 --tag lab --notes "..."

Uses IAudioEndpointVolume::SetMute (vtable index 14) through ctypes, with the endpoint lookup of
pathmeas.windows_volume.
"""
import ctypes
import subprocess
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import pathmeas as pm  # noqa: E402


def set_mute(name, mute, capture=False):
    """Mute (True) or unmute the active endpoint whose friendly name is name. Returns False if there is none."""
    ole32 = ctypes.windll.ole32
    init = ole32.CoInitialize(None)
    held = []

    def ptr():
        held.append(ctypes.c_void_p())
        return held[-1]

    def get(p, i, ctype, *args):
        v = ctype()
        pm.com(p, i, *([ctypes.c_uint] * len(args)), ctypes.c_void_p)(*args, ctypes.byref(v))
        return v.value

    try:
        enum, devs = ptr(), ptr()
        ctypes.oledll.ole32.CoCreateInstance(pm.guid('bcde0395-e52f-467c-8e3d-c4579291692e'), None, 1,
                                             pm.guid('a95664d2-9614-4f35-a746-de8db63617e6'), ctypes.byref(enum))
        pm.com(enum, 3, ctypes.c_int, ctypes.c_uint, ctypes.c_void_p)(int(capture), 1, ctypes.byref(devs))
        key = pm.PropertyKey(pm.guid('a45c254e-df1c-4efd-8020-67d146a850e0'), 14)          # friendly name
        for i in range(get(devs, 3, ctypes.c_uint)):
            dev, store, vol = ptr(), ptr(), ptr()
            pm.com(devs, 4, ctypes.c_uint, ctypes.c_void_p)(i, ctypes.byref(dev))
            pm.com(dev, 4, ctypes.c_uint, ctypes.c_void_p)(0, ctypes.byref(store))
            pv = pm.PropVariant()
            pm.com(store, 5, ctypes.c_void_p, ctypes.c_void_p)(ctypes.byref(key), ctypes.byref(pv))
            friendly = ctypes.wstring_at(pv.ptr) if pv.vt == 31 else None
            ole32.PropVariantClear(ctypes.byref(pv))
            if friendly != name:
                continue
            pm.com(dev, 3, ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p)(
                pm.guid('5cdf2c82-841e-4546-9722-0cf74078229a'), 23, None, ctypes.byref(vol))  # IAudioEndpointVolume
            pm.com(vol, 14, ctypes.c_int, ctypes.c_void_p)(int(bool(mute)), None)            # SetMute
            return True
        return False
    finally:
        for p in reversed(held):
            if p.value:
                pm.com(p, 2)()
        if init in (0, 1):
            ole32.CoUninitialize()


def output_name():
    _, dev_out = pm.wasapi_devices(pm.import_sounddevice(), pm.DEVICE)
    return dev_out['name']


def status(name):
    v = pm.windows_volume(name, capture=False)
    print('US205 output:', 'muted' if v.get('mute') else 'NOT muted', v)
    return v


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('status', 'mute', 'unmute', 'run'):
        sys.exit(__doc__)
    cmd = sys.argv[1]
    name = output_name()
    if cmd == 'status':
        status(name)
    elif cmd in ('mute', 'unmute'):
        if not set_mute(name, cmd == 'mute'):
            sys.exit(f'No active output endpoint named {name}')
        status(name)
    else:
        args = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
        rc = 1
        try:
            if not set_mute(name, False):
                sys.exit(f'No active output endpoint named {name}')
            if status(name).get('mute'):
                sys.exit('The output is still muted; not measuring.')
            rc = subprocess.call([sys.executable, str(TOOLS / 'pathmeas.py'), 'measure', *args, '--yes'])
        finally:
            set_mute(name, True)
            if not status(name).get('mute'):
                print('WARNING: could not mute the US205 output again; mute it in Windows')
        sys.exit(rc)


if __name__ == '__main__':
    main()
