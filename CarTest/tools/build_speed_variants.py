"""Build the explicit A/B/C speed variants; never flash or start hardware."""
from pathlib import Path
import argparse,json,os,subprocess

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--percent',type=int,choices=(100,120,130,140),required=True)
    parser.add_argument('--profile',choices=('A','B','C'),required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    base={'A':'DebugFastNoRing','B':'DebugRingBase','C':'DebugRingFast'}[args.profile]
    preset=base+('_Plus'+str(args.percent-100) if args.percent!=100 else '')
    bundles=Path(os.environ['LOCALAPPDATA'])/'stm32cube/bundles'
    def find(bundle,name):return sorted((bundles/bundle).glob('*/bin/'+name))[-1]
    cmake=find('cmake','cmake.exe');ninja=find('ninja','ninja.exe');gcc=find('gnu-tools-for-stm32','arm-none-eabi-gcc.exe')
    env=dict(os.environ);env['PATH']=str(gcc.parent)+os.pathsep+str(ninja.parent)+os.pathsep+env['PATH']
    subprocess.run([str(cmake),'--preset',preset,'-DCMAKE_MAKE_PROGRAM='+str(ninja)],cwd=root,env=env,check=True)
    commands=json.loads((root/'build'/preset/'compile_commands.json').read_text())
    command=next(x['command'] for x in commands if x['file'].replace('\\','/').endswith('/Src/main.c'))
    expected={'CARTEST_SPEED_PERCENT':args.percent,'CARTEST_TRACK_PROFILE':{'A':1,'B':2,'C':3}[args.profile],
              'CARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS':4800 if args.profile=='B' else 5600,
              'CARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS':4000 if args.profile=='B' else 4400}
    for key,value in expected.items():
        if '-D'+key+'='+str(value) not in command:raise RuntimeError('Wrong effective define: '+key)
    subprocess.run([str(cmake),'--build','--preset',preset,'--parallel','8'],cwd=root,env=env,check=True)
    print(root/'build'/preset/'CarTest.elf')

if __name__=='__main__':main()
