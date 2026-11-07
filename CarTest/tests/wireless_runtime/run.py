"""Exercise production runtime with a host-only flash adapter; no board access."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--zig', required=True)
    args = p.parse_args()
    def run(argv):
        r = subprocess.run([str(x) for x in argv], cwd=ROOT, capture_output=True, text=True)
        if r.returncode: raise RuntimeError(r.stdout + r.stderr)
        return r.stdout
    with tempfile.TemporaryDirectory(prefix='wireless-runtime-') as d:
        d = Path(d)
        flash = d / 'flash.o'
        run([args.zig,'cc','-std=c11','-I',ROOT/'Inc','-c',ROOT/'App/wireless_flash_stm32.c','-o',flash])
        exe=d/'runtime.exe'
        run([args.zig,'cc','-std=c11','-Wall','-Wextra','-Werror','-DCARTEST_WIRELESS_ENABLED=1','-DCARTEST_GYRO350_ENABLED=1',
             '-I',ROOT/'Inc',*[ROOT/x for x in ['tests/wireless_runtime/test_runtime.c',
             'App/wireless_runtime.c','App/wireless_control.c','App/wireless_settings.c','App/wireless_protocol.c']],flash,'-o',exe])
        result=run([exe]);print(result)
    out=ROOT.parent/'validation/WIRELESS_RUNTIME.json'
    sources=['App/wireless_runtime.c','App/wireless_control.c','App/wireless_settings.c','App/wireless_protocol.c',
             'Inc/wireless_runtime.h','tests/wireless_runtime/test_runtime.c']
    out.write_text(json.dumps({'passed':True,'summary':result.strip(),'hardware_actions':'none',
        'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in sources}},indent=2),encoding='utf-8')

if __name__=='__main__': main()
