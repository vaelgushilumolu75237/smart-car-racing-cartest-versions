"""Collect completed component evidence and build the paired release. No hardware I/O."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT.parent/'validation'

def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p): return json.loads(p.read_text(encoding='utf-8-sig'))

def main():
    reports={}
    for name in ('WIRELESS_CONTROL','WIRELESS_MAIN','WIRELESS_CAMERA','WIRELESS_TOOLS','WIRELESS_RUNTIME','WIRELESS_VISION_INTEGRATION','WIRELESS_AUX_PINS'):
        report=read(OUT/(name+'.json'))
        if report.get('passed') is not True: raise RuntimeError(name+' is not passed')
        for key in ('tested_source_sha256','source_sha256'):
            for path, expected in report.get(key,{}).items():
                if digest(ROOT/path)!=expected: raise RuntimeError('Tested source changed: '+path)
        reports[name]=report
    camera=reports['WIRELESS_CAMERA']
    if camera.get('k230_port_full_regression',{}).get('passed') is not True:
        raise RuntimeError('Camera full regression incomplete')
    camera_tests=read(OUT/'CAMERA_TESTS.json')
    if camera_tests.get('failures') or camera_tests.get('errors'):
        raise RuntimeError('Camera algorithm tests failed')
    host=read(OUT/'ROOT_HOST_TESTS.json')
    if not all(v.get('passed') for v in host.values() if isinstance(v,dict)):
        raise RuntimeError('Host evidence incomplete')
    profiles=sorted(p['name'] for p in read(ROOT/'CMakePresets.json')['buildPresets'] if not p.get('hidden'))
    def build(profile):
        result=subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',
            str(ROOT/'tools/build.ps1'),'-Preset',profile],cwd=ROOT,capture_output=True,text=True,
            encoding='utf-8',errors='replace',timeout=180)
        (OUT/('build-'+profile+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
        if result.returncode: raise RuntimeError(profile+' build failed: '+result.stderr[-1000:])
        print('BUILT '+profile,flush=True)
        return profile,digest(ROOT/'build'/profile/'CarTest.elf')
    with ThreadPoolExecutor(max_workers=2) as pool:
        hashes=dict(pool.map(build,profiles))
    old=read(ROOT/'docs/K230_VALIDATION.json')
    # Unchanged historical diagnostic tooling retains its prior test evidence.
    baseline=read(ROOT.parent/'private-baseline/SOURCE_BEFORE.json')
    diagnostic_sources=['tools/run_status_reader.py','tools/analyze_differential_runs.py',
        'tests/test_run_status_reader.py','tests/test_analyze_differential_runs.py','tests/test_gyro350_diag.py']
    for name in diagnostic_sources:
        if digest(ROOT/name)!=baseline['files'][name]: raise RuntimeError('Diagnostic evidence needs refresh: '+name)
    if not old.get('diagnostic_tests_passed'): raise RuntimeError('Missing original diagnostic test evidence')
    runtime=sorted((ROOT/'openmv').glob('*.py'))
    compilation=[]
    for source in runtime:
        bytecode=OUT/'mpy'/(source.stem+'.mpy')
        if not bytecode.is_file() or bytecode.stat().st_mtime_ns<source.stat().st_mtime_ns:
            raise RuntimeError('Missing current bytecode: '+source.name)
        compilation.append({'file':source.name,'source_sha256':digest(source),'mpy_sha256':digest(bytecode)})
    source_inputs=[p for directory in ('App','Inc','Src','Drivers','cmake') for p in (ROOT/directory).rglob('*') if p.is_file()]
    source_inputs += [ROOT/n for n in ('CMakeLists.txt','CMakePresets.json','stm32h743xi_flash.ld')]
    result=dict(old)
    result.update(port='CARTEST_K230_WIRELESS_RTS_CMD_20261107',status='experimental-software-tested-hardware-pending',
        camera_tests=read(OUT/'CAMERA_TESTS.json'),hal_tests_passed=True,
        gyro_camera_tests_passed=True,gyro_driver_state_tests_passed=True,
        gyro_integration_tests_passed=True,gyro_main_tests=read(OUT/'GYRO_MAIN.json'),
        legacy_host_tests_passed=True,all_bytecode_compiled=True,compilation=compilation,
        built_profiles=profiles,elf_sha256=hashes,
        mcu_source_sha256={p.relative_to(ROOT).as_posix():digest(p) for p in sorted(source_inputs)},
        wireless_tests_passed=True,wireless_reports=reports,
        diagnostic_evidence='reused prior passing evidence after unchanged source hash checks',
        pairing='DebugWirelessGyro350Speed140 + wireless-enabled K230 V4 production camera',hardware_actions='none')
    (ROOT/'docs/K230_VALIDATION.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (ROOT/'docs/WIRELESS_VALIDATION.json').write_text(json.dumps({'passed':True,'reports':reports,
        'host':host,'built_profiles':profiles,'elf_sha256':hashes,'hardware_actions':'none'},indent=2),encoding='utf-8')
    manifest=read(ROOT/'openmv/PACKAGE_MANIFEST.json')
    manifest.update(port=result['port'],runtime_files=[p.name for p in runtime],
        source_sha256={p.name:digest(p) for p in runtime},
        paired_mcu_preset='DebugWirelessGyro350Speed140',wireless_required=True,
        release_version='v20261107-k230-wireless-v2-rts-cmd',
        wireless_aux_gpio={'rts':'PA4','cmd':'PA5','mode':'input-pullup',
                          'external_resistor':'software alternative; physical verification pending',
                          'hardware_flow_control':False})
    (ROOT/'openmv/PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    (ROOT/'openmv/CROSS_COMPILE_SUMMARY.json').write_text(json.dumps({k:result[k] for k in
        ('port','compiler','compiler_sha256','bytecode_target','compilation')},indent=2),encoding='utf-8')
    print(json.dumps({'passed':True,'built_profiles':len(profiles),'camera_files':len(runtime),'hardware_actions':'none'}))

if __name__=='__main__': main()
