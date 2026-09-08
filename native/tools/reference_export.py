"""Export one explicitly selected eligible group to the native comparison contract.
No shipping data, inferred benchmark, or claim of source authenticity. Raw files are re-read.
"""
from __future__ import annotations
import argparse, hashlib, json, math, os, pathlib, tempfile
import reference_forces as rf

METHOD="piecewise-linear-positive10s-v1"

def raw_path(analysis, analysis_path):
    raw=pathlib.Path(analysis["input_csv"])
    if not raw.is_absolute() and not raw.exists():raw=analysis_path.parent/raw
    return raw.resolve()

def export_group(paths, key):
    if not isinstance(key,list) or len(key)!=5 or not all(isinstance(x,str) and x.strip() for x in key):
        raise rf.ForceValidationError("Select exactly [ride, configuration, seat, device, calibration_id]")
    analyses=[];sources={}
    for path in sorted(set(pathlib.Path(p).resolve() for p in paths)):
        old=json.loads(path.read_text(encoding="utf-8")); raw=raw_path(old,path)
        if not raw.is_file() or rf.sha256_file(raw)!=old.get("input_sha256"):
            raise rf.ForceValidationError("Original raw input unavailable or changed: "+str(path))
        gap=old.get("validation",{}).get("max_gap_allowed_s",rf.DEFAULT_MAX_GAP_S)
        if not isinstance(gap,(int,float)) or not math.isfinite(gap) or gap<=0 or gap>rf.DEFAULT_MAX_GAP_S:
            raise rf.ForceValidationError("Game export requires a finite gap rule no looser than processor default")
        fresh=rf.analyze_recording(raw,old["manifest"],gap)
        for field in ("input_sha256","canonical_samples_sha256","strongest10s","eligibility","flags","validation"):
            if old.get(field)!=fresh.get(field):raise rf.ForceValidationError("Cached analysis differs from raw replay: "+field)
        analyses.append(fresh)
        rid=fresh["manifest"]["recording_id"]
        sources.setdefault(rid,[]).append((rf.sha256_file(path),fresh))
    agg=rf.aggregate_recordings(analyses)
    selected=[g for g in agg["groups"] if [g[x] for x in ("ride","configuration","seat","device","calibration_id")]==key]
    if len(selected)!=1 or selected[0]["status"]!="ok":raise rf.ForceValidationError("Selected group has insufficient, conflicting, or incompatible eligible recordings")
    group=selected[0];records=[]
    if not math.isfinite(group["median_S_g_s"]) or group["median_S_g_s"]<=0:
        raise rf.ForceValidationError("A native intensity target needs finite positive exposure")
    for rid in group["eligible_recording_ids"]:
        sha,a=min(sources[rid],key=lambda x:x[0]);m=a["manifest"];cal=m["calibration"];v=a["validation"]
        notes=json.dumps({"calibration":cal,"quality":m.get("quality",{}),"stationarity":a["flags"]["stationarity"],
                          "spike_review":a["flags"]["spike_review"]},sort_keys=True,ensure_ascii=False,separators=(",",":"))
        records.append({"recording_id":rid,"raw_sha256":a["input_sha256"],"canonical_sha256":a["canonical_samples_sha256"],
                        "analysis_sha256":sha,"source":m["source"],"notes":notes,"sample_rate_hz":1/v["dt_median"],
                        "sample_rate_min_hz":1/v["dt_max"],"sample_rate_max_hz":1/v["dt_min"],"S_g_s":a["strongest10s"]["S_g_s"]})
    result={**group,"schema":1,"method":METHOD,"group_id":"rf-v1:"+hashlib.sha256(json.dumps(key,ensure_ascii=False,separators=(",",":")).encode()).hexdigest(),
            "provenance_status":"processed-eligible; authenticity-not-independently-verified", "measurement_uncertainty":"not-quantified",
            "force_transition_calibration":"unassessed","recordings":records}
    interchange(result) # Apply shared size/string constraints before returning.
    return result

def _q(value):
    if not isinstance(value,str) or len(value.encode("utf-8"))>16384 or any(ord(c)<32 or ord(c)==127 for c in value):
        raise rf.ForceValidationError("Reference strings must be bounded UTF-8 without controls")
    return '"'+value.replace('\\','\\\\').replace('"','\\"')+'"'

def interchange(obj):
    records=obj["recordings"];q=obj.get("quartiles_g_s",[])
    if not 3<=len(records)<=256:raise rf.ForceValidationError("Native reference supports 3 to 256 independent recordings")
    line=lambda xs:' '.join(_q(x) for x in xs)+'\n'
    text="COASTER_REFERENCE 1\n"+line([obj["method"],obj["group_id"]])+line([obj[k] for k in ("ride","configuration","seat","device","calibration_id")])
    text+=' '.join(format(float(x),'.17g') for x in (len(records),obj['median_S_g_s'],obj['min_S_g_s'],obj['max_S_g_s'],bool(q),q[0] if q else 0,q[2] if q else 0))+'\n'
    for r in records:
        text+=line([r[k] for k in ("recording_id","raw_sha256","canonical_sha256","analysis_sha256","source","notes")]).rstrip('\n')+' '
        text+=' '.join(format(r[k],'.17g') for k in ("sample_rate_hz","sample_rate_min_hz","sample_rate_max_hz","S_g_s"))+'\n'
    if len(text.encode())>1024*1024:raise rf.ForceValidationError("Reference export exceeds 1 MiB")
    return text

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inputs',nargs='+',required=True);p.add_argument('--group',required=True,help='JSON [ride, configuration, seat, device, calibration_id]')
    p.add_argument('--out',required=True);p.add_argument('--json',help='Optional readable metadata output');a=p.parse_args(argv)
    inputs=[pathlib.Path(x) for x in a.inputs];out=pathlib.Path(a.out)
    # Prevent replacement of an analysis, raw recording or manifest-shaped source.
    raw=[raw_path(json.loads(x.read_text(encoding='utf-8')),x.resolve()) for x in inputs]
    protected=inputs+raw
    rf.ensure_out_not_input(out,protected)
    if a.json:rf.ensure_out_not_input(pathlib.Path(a.json),protected+[out])
    obj=export_group(inputs,json.loads(a.group));data=interchange(obj)
    out.parent.mkdir(parents=True,exist_ok=True);fd,tmp=tempfile.mkstemp(dir=out.parent,prefix='.reference-')
    try:
        with os.fdopen(fd,'w',encoding='utf-8',newline='\n') as f:f.write(data)
        os.replace(tmp,out)
    finally:
        if os.path.exists(tmp):os.unlink(tmp)
    if a.json:rf.atomic_write_json(pathlib.Path(a.json),obj)
    print(json.dumps({'status':obj['provenance_status'],'group_id':obj['group_id'],'n':obj['n_eligible'],'measurement_uncertainty':'not-quantified'}))
    return 0

if __name__=='__main__':raise SystemExit(main())
