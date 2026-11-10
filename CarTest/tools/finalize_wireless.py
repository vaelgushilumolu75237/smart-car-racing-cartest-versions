"""Collect completed component evidence and build the paired release. No hardware I/O."""
from concurrent.futures import ThreadPoolExecutor
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT.parent/'validation'

def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p): return json.loads(p.read_text(encoding='utf-8-sig'))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--release-version', required=True)
    parser.add_argument('--camera-ram-evidence', type=Path,
                        help='Sanitized RAM benchmark summary, with exact runtime hashes; no images/logs')
    parser.add_argument("--reuse-verified-mcu", action="store_true")
    parser.add_argument("--rebuild-wireless-only", action="store_true")
    parser.add_argument("--completed-mcu-evidence", type=Path,
                        help="Reuse separately recorded successful builds after exact source/ELF hash checks")
    parser.add_argument('--mcu-only', action='store_true',
                        help='Require camera runtime to match prior evidence; record a host-built MCU-only release')
    args=parser.parse_args()
    reports={}
    for name in ('WIRELESS_CONTROL','WIRELESS_MAIN','WIRELESS_CAMERA','WIRELESS_TOOLS','WIRELESS_RUNTIME','WIRELESS_VISION_INTEGRATION','WIRELESS_AUX_PINS','IMU_GPIO_SPI'):
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
    old=read(ROOT/'docs/K230_VALIDATION.json')
    if args.mcu_only:
        prior_manifest=read(ROOT/'openmv/PACKAGE_MANIFEST.json')
        for name,expected in prior_manifest['source_sha256'].items():
            if digest(ROOT/'openmv'/name)!=expected:
                raise RuntimeError('MCU-only release changed camera runtime: '+name)
    source_inputs=[p for directory in ('App','Inc','Src','Drivers','cmake') for p in (ROOT/directory).rglob('*') if p.is_file()]
    source_inputs += [ROOT/n for n in ('CMakeLists.txt','CMakePresets.json','stm32h743xi_flash.ld')]
    current_sources={p.relative_to(ROOT).as_posix():digest(p) for p in sorted(source_inputs)}
    completed=None
    if sum(bool(v) for v in (args.rebuild_wireless_only,args.reuse_verified_mcu,args.completed_mcu_evidence)) > 1:
        raise RuntimeError('Choose only one MCU evidence mode')
    if args.completed_mcu_evidence:
        completed=read(args.completed_mcu_evidence)
        if completed.get('passed') is not True or set(completed.get('profiles',{})) != set(profiles):
            raise RuntimeError('Completed build evidence is incomplete')
        if completed.get('mcu_source_sha256') != current_sources:
            raise RuntimeError('Completed build source set or hashes changed')
        hashes={profile:digest(ROOT/'build'/profile/'CarTest.elf') for profile in profiles}
        for profile in profiles:
            row=completed['profiles'][profile]
            if row.get('passed') is not True or row.get('elf_sha256') != hashes[profile]:
                raise RuntimeError('Completed ELF evidence mismatch: '+profile)
        print('REUSED 20 completed actual builds after exact source and ELF hash checks',flush=True)
    elif args.rebuild_wireless_only:
        if args.reuse_verified_mcu:
            raise RuntimeError('Choose only one MCU evidence mode')
        changed=[name for name, expected in old['mcu_source_sha256'].items()
                 if digest(ROOT/name)!=expected]
        if set(changed)-{'App/wireless_runtime.c'}:
            raise RuntimeError('Wireless-only rebuild scope exceeded: '+repr(changed))
        if set(profiles)!=set(old['built_profiles']):
            raise RuntimeError('MCU profile set changed')
        hashes={profile:digest(ROOT/'build'/profile/'CarTest.elf') for profile in profiles}
        if hashes!=old['elf_sha256']:
            raise RuntimeError('Pre-build ELF evidence changed')
        # CMake includes wireless_runtime.c only for this wireless preset.
        profile, elf_hash=build('DebugWirelessGyro350Speed140')
        hashes[profile]=elf_hash
        print('REUSED 19 unaffected non-wireless builds after source/ELF hash checks',flush=True)
    elif args.reuse_verified_mcu:
        if set(profiles) != set(old['built_profiles']):
            raise RuntimeError('MCU profile set changed')
        for name, expected in old['mcu_source_sha256'].items():
            if digest(ROOT/name) != expected:
                raise RuntimeError('MCU source changed: '+name)
        hashes={profile:digest(ROOT/'build'/profile/'CarTest.elf') for profile in profiles}
        if hashes != old['elf_sha256']:
            raise RuntimeError('MCU ELF evidence changed')
        print('REUSED verified unchanged MCU source and 20 paired ELFs',flush=True)
    else:
        with ThreadPoolExecutor(max_workers=2) as pool:
            hashes=dict(pool.map(build,profiles))
    # Unchanged historical diagnostic tooling retains its prior test evidence.
    baseline=read(ROOT.parent/'private-baseline/SOURCE_BEFORE.json')
    diagnostic_sources=['tools/run_status_reader.py','tools/analyze_differential_runs.py',
        'tests/test_run_status_reader.py','tests/test_analyze_differential_runs.py','tests/test_gyro350_diag.py']
    for name in diagnostic_sources:
        if digest(ROOT/name)!=baseline['files'][name]: raise RuntimeError('Diagnostic evidence needs refresh: '+name)
    if not old.get('diagnostic_tests_passed'): raise RuntimeError('Missing original diagnostic test evidence')
    runtime=sorted((ROOT/'openmv').glob('*.py'))
    ram=None
    if args.camera_ram_evidence:
        ram=read(args.camera_ram_evidence)
        if ram.get('source_sha256') != {p.name:digest(p) for p in runtime}:
            raise RuntimeError('RAM camera benchmark source changed')
        if ram.get('actual_camera') is not True or ram.get('actual_h743') is not False:
            raise RuntimeError('RAM evidence must describe the actual camera with simulated H743 input')
        if ram.get('deployed') is not False:
            raise RuntimeError('This finalizer only records an undeployed RAM candidate')
    compilation=[]
    for source in runtime:
        bytecode=OUT/'mpy'/(source.stem+'.mpy')
        if not bytecode.is_file() or bytecode.stat().st_mtime_ns<source.stat().st_mtime_ns:
            raise RuntimeError('Missing current bytecode: '+source.name)
        compilation.append({'file':source.name,'source_sha256':digest(source),'mpy_sha256':digest(bytecode)})
    result=dict(old)
    result.pop('completed_mcu_builds',None)
    result.pop('camera_ram_benchmark',None)
    result.pop('camera_evidence_origin',None)
    result['release_version']=args.release_version
    result.update(port='CARTEST_K230_FRESH_ROUTE_V24_20261110',status='experimental-software-tested-hardware-pending',
        camera_tests=read(OUT/'CAMERA_TESTS.json'),hal_tests_passed=True,
        gyro_camera_tests_passed=True,gyro_driver_state_tests_passed=True,
        gyro_integration_tests_passed=True,gyro_main_tests=read(OUT/'GYRO_MAIN.json'),
        legacy_host_tests_passed=True,all_bytecode_compiled=True,compilation=compilation,
        built_profiles=profiles,elf_sha256=hashes,
        mcu_build_evidence="completed actual builds reused after exact source/ELF hash checks" if completed else "prior actual builds reused after source and ELF hash verification" if args.reuse_verified_mcu else "wireless actual rebuild; 19 unaffected non-wireless ELFs verified unchanged" if args.rebuild_wireless_only else "actual builds this run",
        mcu_source_sha256=current_sources,
        wireless_tests_passed=True,wireless_reports=reports,
        imu_gpio_spi_tests_passed=True,
        diagnostic_evidence='reused prior passing evidence after unchanged source hash checks',
        pairing='DebugWirelessGyro350Speed140 + wireless-enabled K230 V4 production camera',
        hardware_actions='finite camera USB RAM diagnostics; no production camera installation, H743 flash/reset or motor start',
        hardware_diagnostics={'current_version_hardware_verified':False,
            'camera_runtime_unchanged':False,'production_camera_installed':False,
            'h743_camera_link_verified':False,'road_behavior_verified':False,
            'raw_vehicle_logs_published':False,
            'radio_start_tested_on_vehicle':False})
    if ram:
        result['camera_ram_benchmark']=ram
        result['board_runtime_fps']='RAM-only camera benchmark; see camera_ram_benchmark; paired H743 link pending'
    else:
        result['board_runtime_fps']='final runtime not benchmarked; hardware verification pending'
    if args.mcu_only:
        result['release_version']=args.release_version
        result['camera_evidence_origin']=old.get('release_version','v20261110-k230-fresh-route-v24')
        result['hardware_actions']='read-only SWD diagnostics; no camera upload, H743 flash/reset or motor start'
        result['hardware_diagnostics'].update(camera_runtime_unchanged=True)
        result['board_runtime_fps']='Camera runtime unchanged; prior V24 RAM benchmark retained as historical evidence only; paired current H743 receive rate pending'
    if completed:
        result['completed_mcu_builds']=completed
    (ROOT/'docs/K230_VALIDATION.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (ROOT/'docs/WIRELESS_VALIDATION.json').write_text(json.dumps({'passed':True,'reports':reports,
        'host':host,'built_profiles':profiles,'elf_sha256':hashes,
        'hardware_actions':result['hardware_actions'],
        'hardware_diagnostics':result['hardware_diagnostics']},indent=2),encoding='utf-8')
    manifest=read(ROOT/'openmv/PACKAGE_MANIFEST.json')
    manifest.update(port=result['port'],runtime_files=[p.name for p in runtime],
        source_sha256={p.name:digest(p) for p in runtime},
        paired_mcu_preset='DebugWirelessGyro350Speed140',wireless_required=True,
        release_version=args.release_version,
        dedicated_camera_uart_baudrate=2000000,wireless_pc_uart_baudrate=115200,
        wireless_aux_gpio={'rts':'PA4','cmd':'PA5','mode':'input-pullup',
                          'external_resistor':'software alternative; physical verification pending',
                          'hardware_flow_control':False})
    (ROOT/'openmv/PACKAGE_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    (ROOT/'openmv/CROSS_COMPILE_SUMMARY.json').write_text(json.dumps({k:result[k] for k in
        ('port','compiler','compiler_sha256','bytecode_target','compilation')},indent=2),encoding='utf-8')
    print(json.dumps({'passed':True,'built_profiles':len(profiles),'camera_files':len(runtime),
        'deployment_actions':'none','hardware_diagnostics':result['hardware_diagnostics']}))

if __name__=='__main__': main()
