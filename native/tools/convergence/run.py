import argparse, concurrent.futures, hashlib, json, subprocess
from pathlib import Path

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--matrix',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--binary',type=Path,required=True);p.add_argument('--core',type=Path,required=True);args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    summary=json.loads((args.matrix/'summary.json').read_text());manifest=json.loads((args.matrix/'manifest.json').read_text());expected=summary['accepted'];items=[]
    for case in manifest['cases']:
        rec=json.loads((args.matrix/'cases'/(case['id']+'.json')).read_text())
        if rec['category']!='accepted': continue
        assert rec['accepted'] is True and rec['note']=='validate-confirmed' and rec['cliSha256']==summary['config']['cliSha256']
        save=(args.matrix/'outs'/(case['id']+'.coaster')).resolve();digest=sha(save);assert digest==rec['outSha256'],str(save)
        replay=(args.matrix/'replays'/(case['id']+'.replay.json')).resolve();assert sha(replay)==rec['replayJsonSha256']
        items.append({'source':str(save),'sha256':digest,'seed':case['seed'],'terrain':case['terrain'],'replay':str(replay),'replaySha256':rec['replayJsonSha256']})
    assert len(items)==expected and len({x['source'] for x in items})==expected and expected>0
    source_hashes={str(f.resolve()):sha(f) for f in args.core.rglob('*') if f.is_file() and f.suffix in ('.cpp','.hpp','.h')}
    provenance={'runtime':summary['config']['generatorVersion'],'matrixCliSha256':summary['config']['cliSha256'],'auditBinarySha256':sha(args.binary),'auditSourceSha256':sha(Path(__file__).with_name('audit.cpp')),'coreSourceHashes':source_hashes,'matrixManifestSha256':sha(args.matrix/'manifest.json'),'inputs':items}
    (args.out/'manifest.json').write_text(json.dumps(provenance,indent=2))
    lists=[]
    for i in range(2):
        target=args.out/f'worker-{i}.txt';target.write_text(''.join(x['source']+'\n' for x in items[i::2]),encoding='utf-8');lists.append(target)
    def run(i):
        result=subprocess.run([str(args.binary.resolve()),str(lists[i].resolve()),str((args.out/f'worker-{i}.jsonl').resolve())],capture_output=True,text=True,timeout=7200)
        (args.out/f'worker-{i}.stdout.txt').write_text(result.stdout);(args.out/f'worker-{i}.stderr.txt').write_text(result.stderr)
        assert result.returncode in (0,2),(i,result.returncode,result.stderr);return result.stdout.strip()
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        for line in pool.map(run,range(2)): print(line,flush=True)
    results=[]
    for i in range(2): results.extend(json.loads(line) for line in (args.out/f'worker-{i}.jsonl').read_text().splitlines())
    assert len(results)==expected and {x['source'] for x in results}=={x['source'] for x in items}
    assert all(x['runtime']==provenance['runtime'] for x in results)
    assert all(sha(x['source'])==x['sha256'] for x in items),'Input save changed'
    assert all(sha(f)==v for f,v in source_hashes.items()),'Core source changed during audit'
    assert sha(args.binary)==provenance['auditBinarySha256'],'Audit binary changed'
    by_source={x['source']:x for x in items}
    for result in results:
        original=by_source[result['source']];assert sha(original['replay'])==original['replaySha256']
        replay=json.loads(Path(original['replay']).read_text());coarse=result.get('coarseReport',{})
        result['coarseMatchesFrozenReplay']={k:v for k,v in coarse.items() if k!='generationSeconds'}=={k:v for k,v in replay.items() if k!='generationSeconds'} and isinstance(coarse.get('metrics'),dict)
        result['passed']=result['passed'] and result['coarseMatchesFrozenReplay'] and result.get('hasSeatStatistics') is True
    results.sort(key=lambda x:x['source'])
    (args.out/'results.json').write_text(json.dumps(results,indent=2))
    failures=[{'source':x['source'],'seed':x.get('seed'),'terrain':x.get('terrain'),'error':x.get('error'),'fineTargetErrors':x.get('fineTargetErrors',[]),'coarseMatchesFrozenReplay':x.get('coarseMatchesFrozenReplay'),'hasSeatStatistics':x.get('hasSeatStatistics'),'metrics':[k for k,v in x.get('metrics',{}).items() if not v['passed']]} for x in results if not x['passed']]
    metrics=sorted({k for x in results for k in x.get('metrics',{})})
    maxima={k:max((v for x in results if (v:=x.get('metrics',{}).get(k,{}).get('normalizedError')) is not None),default=None) for k in metrics}
    report={'count':len(results),'passed':len(results)-len(failures),'failed':len(failures),'maxNormalizedErrors':maxima,'failures':failures,'runtime':provenance['runtime'],'matrixCliSha256':provenance['matrixCliSha256'],'auditBinarySha256':provenance['auditBinarySha256']}
    (args.out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));return 0 if not failures else 2
if __name__=='__main__': raise SystemExit(main())
