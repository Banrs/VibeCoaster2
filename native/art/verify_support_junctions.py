"""Check fitted structural meshes and the retained native track samples."""
import json
import math
from pathlib import Path
from support_detail_geometry import support_fabrication_plan, fitted_support_members
from verify_support_details import check_closed
root = Path(__file__).resolve().parents[2]
profile = json.loads((root/'native/art/track_study_profile.json').read_text())
results = []
for path in sorted((root/'out/support-study/data').glob('*-?.json')):
    data = json.loads(path.read_text())
    old = json.loads((root/'out/support-study/rejected-20260929/data'/path.name).read_text())
    previous = {q[3]:q[:3] for q in old['track']}
    shared = [q for q in data['track'] if q[3] in previous]
    assert len(shared) >= len(old['track'])-len(old['supports'])
    assert all(q[:3] == previous[q[3]] for q in shared), 'Track changed'
    fitted = fitted_support_members(support_fabrication_plan(profile,data))
    assert fitted['mitres'] > 0 and fitted['copedEnds'] > 0
    for key,(vertices,faces) in fitted['meshes'].items():
        check_closed({'name':str((path.name,key)),'vertices':vertices,'faces':faces})
    for endpoint,ring in fitted['rings'].items():
        key,end = endpoint
        actual = fitted['meshes'][key][0][end*32:(end+1)*32]
        assert max(min(math.dist(v,w) for w in actual) for v in ring) < 1e-8
    results.append({'case':path.stem,'fittedMembers':len(fitted['meshes']),
                    'mitres':fitted['mitres'],'copedEnds':fitted['copedEnds'],
                    'unchangedTrackSamples':len(shared)})
assert len(results) == 9
report = {'passed':True,'cases':results,'railRadiusMetres':profile['rail_radius']}
(root/'out/support-study/junction-checks.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
