#!/usr/bin/env python3
"""Compare actual pinned C++ functions to Rust; tolerate <=5e-6 per scalar before running."""
import os,pathlib,random,subprocess,tempfile,json
ROOT=pathlib.Path(__file__).resolve().parents[1]
LEGACY=pathlib.Path(os.environ.get('FLIX_LEGACY',str(ROOT.parent/'flix')))
REVISION='1f5b268271536e45a9377d1c5317d3eb6f3c3bba'
def run(args,**kwargs):return subprocess.run(args,check=True,text=True,capture_output=True,**kwargs).stdout
def main():
    if run(['git','-C',str(LEGACY),'rev-parse','HEAD']).strip()!=REVISION:raise RuntimeError('legacy revision differs')
    randomizer=random.Random(0x5eed)
    lines=[]
    for _ in range(100):lines.append('mix,'+','.join(map(str,[randomizer.uniform(.1,1),*[randomizer.uniform(-2,2) for _ in range(3)]])))
    for _ in range(100):lines.append('quaternion,'+','.join(str(randomizer.uniform(-1,1)) for _ in range(3)))
    for mode in [1,2]:
        for _ in range(100):lines.append('control,'+','.join(map(str,[*[randomizer.uniform(-1,1) for _ in range(3)],randomizer.uniform(.1,1),*[randomizer.uniform(-.5,.5) for _ in range(3)],*[randomizer.uniform(-1,1) for _ in range(3)],mode])))
    cases='\n'.join(lines)+'\n'
    with tempfile.TemporaryDirectory(prefix='flix-parity-') as tmp:
        cxx=os.environ.get('CXX','clang++');flags=['-std=c++17','-I'+str(LEGACY/'gazebo'),'-I'+str(LEGACY/'flix')]
        for name in ['defects','oracle']:run([cxx,*flags,str(ROOT/'fixtures/legacy'/f'{name}.cpp'),'-o',str(pathlib.Path(tmp)/name)])
        defects=run([str(pathlib.Path(tmp)/'defects')]);cpp=run([str(pathlib.Path(tmp)/'oracle')],input=cases)
    rust=run([str(ROOT/'target/release/flight-replay'),'--oracle'],input=cases)
    expected=[list(map(float,row.split(','))) for row in cpp.splitlines()];actual=[list(map(float,row.split(','))) for row in rust.splitlines()]
    assert len(expected)==len(actual)==400
    maximum=0
    for i,(a,b) in enumerate(zip(expected,actual)):
        assert len(a)==len(b)
        for x,y in zip(a,b):
            error=abs(x-y);maximum=max(maximum,error)
            if error>5e-6:raise AssertionError((i,lines[i],a,b,error))
    print(defects.strip());print(json.dumps({'legacy':REVISION,'cases':400,'tolerance':5e-6,'max_absolute_error':maximum,'seed':0x5eed}))
if __name__=='__main__':main()
