"""Check fabricated support meshes against the native attachment frames.

This checks the art buffers and their placement, not structural engineering or
adoption of their dimensions by the runtime clearance system.
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path

from support_detail_geometry import (support_fabrication_plan, fit_support_fabrication, member_splice_details,
    footing_fixing_details, foundation_plate_thickness, sd_seated_tube, sd_part,
    sd_add, sd_sub, sd_mul, sd_dot, sd_cross)


def check_closed(part):
    vertices, faces = part['vertices'], part['faces']
    assert vertices and faces, part['name']
    assert all(math.isfinite(x) for v in vertices for x in v), part['name']
    edges = Counter(tuple(sorted((face[j],face[(j+1)%len(face)])))
                    for face in faces for j in range(len(face)))
    assert set(edges.values()) == {2}, ('open/nonmanifold',part['name'])
    for face in faces:
        assert len(face) == len(set(face)), ('repeated index',part['name'])
        area = (0,0,0)
        # Shift to a nearby origin to avoid cancellation at large coordinates.
        origin = vertices[face[0]]
        for j,i in enumerate(face):
            area = sd_add(area,sd_cross(sd_sub(vertices[i],origin),sd_sub(vertices[face[(j+1)%len(face)]],origin)))
        assert sd_dot(area,area) > 1e-16, ('degenerate face',part['name'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', default='out/support-study/data')
    parser.add_argument('--compare', help='Optional earlier exports: verify native members and foundations are unchanged')
    parser.add_argument('--report', default='out/support-study/fabrication-checks.json')
    args = parser.parse_args()
    profile = json.loads(Path(__file__).with_name('track_study_profile.json').read_text())
    cases, checked_parts, inverted, changed_frames, inclined = [], 0, 0, 0, 0
    for path in sorted(Path(args.data).glob('*.json')):
        data = json.loads(path.read_text())
        if data.get('format') != 2 or not data.get('regions'):
            continue
        tracks = {q[3]:q[:3] for q in data['track']}
        stations = [q[3] for q in data['track']]
        assert all(b > a for a,b in zip(stations,stations[1:]))
        assert max(b-a for a,b in zip(stations,stations[1:])) <= .350001
        fabrication = support_fabrication_plan(profile,data)
        fitted = fit_support_fabrication(fabrication)
        for key,(vertices,faces) in fitted['meshes'].items():
            check_closed({'name':str((path.name,key)),'vertices':vertices,'faces':faces})
        details = fabrication['details']
        art_members = [m for group in fabrication['members'] for m in group]
        splice_count, footing_count, min_anchors, widened_tops = 0,0,1000,0
        for support_index,(support,detail) in enumerate(zip(data['supports'],details)):
            assert tracks[support['distance']] == support['frame'], 'Attachment frame missing from track sweep'
            r,u,f = detail['right'],detail['up'],detail['forward']
            assert abs(sd_dot(r,u)) < 1e-9
            assert abs(sd_dot(sd_cross(f,r),u)-1) < 1e-9, 'Reflected joint frame'
            inverted += u[2] < -.5
            inclined += not detail['inline_stem']
            contact = next(m for m in support['members'] if m[5])
            assert math.dist(contact[1],support['attachment']) < 1e-8
            assert math.dist(sd_sub(contact[1],contact[0]),sd_mul(u,detail['available'])) < 1e-8
            native_underside = sd_sub(support['frame'][0],sd_mul(u,data['nativeSpineDepth']+data['nativeSpineRadius']))
            assert math.dist(native_underside,support['attachment']) < 1e-8
            def local(v):
                delta = sd_sub(v,detail['origin'])
                return (sd_dot(delta,f),sd_dot(delta,r),sd_dot(delta,u))
            head = next(p for p in detail['parts'] if p['name'] == 'Column / constant diameter head')
            coords = [local(v) for v in head['vertices']]
            assert min(z for x,y,z in coords) < max(z for x,y,z in coords)-.05
            assert all(abs(math.hypot(x,y)-detail['radius']) < 1e-8 for x,y,z in coords), 'Contracted head'
            assert detail['radius'] >= .375
            weld_parts = [p for p in detail['parts'] if p.get('weld_vertices')]
            assert len(weld_parts) == 4, 'Missing transverse or longitudinal load path'
            nearby = [q for q in data['track'] if abs(q[3]-support['distance']) < 2.]
            offset = profile['spine_radius']-data['nativeSpineDepth']-data['nativeSpineRadius']
            centres = [sd_add(q[0],sd_mul(q[2],offset)) for q in nearby]
            def axis_distance(v,a,b):
                ab = sd_sub(b,a)
                t = max(0.,min(1.,sd_dot(sd_sub(v,a),ab)/sd_dot(ab,ab)))
                return math.dist(v,sd_add(a,sd_mul(ab,t)))
            for part in weld_parts:
                for i in part['weld_vertices']:
                    v = part['vertices'][i]
                    distance = min(axis_distance(v,a,b) for a,b in zip(centres,centres[1:]))
                    penetration = profile['spine_radius']-distance
                    assert .002 < penetration < .012, ('Saddle misses curved spine or enters too deeply',path.name,support_index,penetration)
            for j,m in detail['replacement_members'].items():
                assert m[:2] == support['members'][j][:2], 'Incoming axis moved'
                assert m[3] == detail['radius'] and m[2] >= m[3], 'Contracted incoming member'
            parts = list(detail['parts'])
            for j,original in enumerate(support['members']):
                m = fabrication['members'][support_index][j]
                assert m[:2] == original[:2], 'Native member axis moved'
                if m[4]:
                    assert m[2] == original[2] and original[3] <= m[3] < m[2], 'Foundation ground footprint changed'
                    widened_tops += m[3] > original[3]+1e-8
                    adjoining = [a for a in art_members if not a[4] and math.dist(a[0],m[1]) < 1e-6]
                    fixings = footing_fixing_details(m,adjoining)
                    anchors = sum(p['name'] == 'Foundation anchor / shank' for p in fixings)
                    assert anchors >= 4, ('Foundation has insufficient visible anchor locations',path.name,j,anchors)
                    min_anchors = min(min_anchors,anchors)
                    parts += fixings
                    footing_count += 1
                else:
                    foundation = fabrication['foundations'].get(tuple(m[0]))
                    if foundation is not None:
                        height = foundation[1][2]+foundation_plate_thickness(foundation)-.002
                        mesh = sd_seated_tube(m[0],m[1],m[2],m[3],height)
                        assert all(abs(v[2]-height) < 1e-8 for v in mesh[0][:32]), 'Gap beneath inclined pipe end'
                        radius = max(math.hypot(v[0]-foundation[1][0],v[1]-foundation[1][1]) for v in mesh[0][:32])
                        assert radius < foundation[3]*.94-.13, 'Pipe overhangs its bearing plate'
                        parts.append(sd_part('Foundation / seated pipe',mesh))
                    splices = member_splice_details(m)
                    splice_count += sum(p['name'] == 'Pipe splice / flange' for p in splices)//2
                    parts += splices
            for part in parts:
                check_closed(part)
            checked_parts += len(parts)
        compared = False
        prior_path = Path(args.compare)/path.name if args.compare else None
        if prior_path and prior_path.is_file():
            prior = json.loads(prior_path.read_text())
            assert prior['counts'] == data['counts'] and prior['terrain'] == data['terrain']
            assert len(prior['supports']) == len(data['supports'])
            for old,new in zip(prior['supports'],data['supports']):
                for key in ('distance','attachment','members'):
                    assert old[key] == new[key], ('Native graph changed',path.name,key)
            compared = True
            changed_frames += 1
        cases.append({'case':path.stem,'trackMounts':len(details),'pipeSplices':splice_count,
                      'foundations':footing_count,'minimumVisibleAnchors':min_anchors,
                      'pedestalTopsWidenedInsideOriginalFootprint':widened_tops,
                      'nativeGraphComparedUnchanged':compared})
    assert len(cases) >= 9 and inverted > 0, 'Missing terrain or inversion fixtures'
    report = {'passed':True,'cases':cases,'closedPartsChecked':checked_parts,
              'invertedMountsChecked':inverted,'inclinedHeadsChecked':inclined,'nativeGraphsComparedUnchanged':changed_frames,
              'railRadiusMetres':profile['rail_radius'],'gaugeMetres':profile['gauge'],
              'runtimeEnvelopeAdopted':False}
    Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
