"""Low-cost native graph overview of the separately regenerated retained ride.

Execute through Blender MCP. This scene shows native clearance members, not
the fabricated fittings reviewed in the four detailed support scenes.
"""
import bpy
import json
import math
from mathutils import Vector

# TRACK_STUDY_GEOMETRY
# SUPPORT_DETAIL_GEOMETRY

DATA = json.loads(__SUPPORT_CONTEXT__)
PROFILE = json.loads(__TRACK_PROFILE__)
EXPORT_ROOT = '__EXPORT_ROOT__'
scene = bpy.data.scenes.get('Native Layout Review / Riftwake') or bpy.data.scenes.new('Native Layout Review / Riftwake')
bpy.context.window.scene = scene
for obj in list(scene.objects):
    bpy.data.objects.remove(obj,do_unlink=True)
scene['source_data'] = DATA['sourceReviewFile']
scene['source_sha256'] = DATA['sourceReviewSha256']
scene['representation'] = 'Exact native support solids; simplified track context; no fabrication fittings'
scene['counts'] = json.dumps(DATA['counts'])

def mesh(name,vertices,faces,color):
    data = bpy.data.meshes.new(name)
    data.from_pydata(vertices,[],faces)
    data.update()
    obj = bpy.data.objects.new(name,data)
    scene.collection.objects.link(obj)
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED').inputs['Base Color'].default_value = (*color,1)
    mat.diffuse_color = (*color,1)
    data.materials.append(mat)
    return obj

for concrete,color in ((False,(.19,.25,.28)),(True,(.52,.51,.47))):
    vertices,faces = [],[]
    for support in DATA['supports']:
        for a,b,r0,r1,kind,contact in support['members']:
            if bool(kind) != concrete:
                continue
            vv,ff = sd_tube(a,b,r0,r1,12)
            base = len(vertices)
            vertices.extend(vv)
            faces.extend(tuple(base+i for i in face) for face in ff)
    mesh('Native foundations' if concrete else 'Native steel graph',vertices,faces,color)

# A lightweight centreline provides layout context. Full-profile accuracy is
# checked in the four detailed scenes, not inferred from this overview tube.
curve = bpy.data.curves.new('Retained track context','CURVE')
curve.dimensions = '3D'
curve.bevel_depth = .45
curve.bevel_resolution = 1
spline = curve.splines.new('POLY')
spline.points.add(len(DATA['track'])-1)
for point,q in zip(spline.points,DATA['track']):
    point.co = (*q[0],1)
obj = bpy.data.objects.new('Retained track context',curve)
scene.collection.objects.link(obj)
mat = bpy.data.materials.new('Native overview track')
mat.use_nodes = True
next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED').inputs['Base Color'].default_value = (.78,.64,.34,1)
curve.materials.append(mat)
terrain = DATA['terrain']
columns,rows = terrain['columns'],terrain['rows']
faces = []
for y in range(rows-1):
    for x in range(columns-1):
        i = y*columns+x
        faces.extend(((i,i+1,i+columns+1),(i,i+columns+1,i+columns)))
mesh('Retained native terrain',terrain['points'],faces,(.29,.34,.29))
lo,hi = map(Vector,DATA['bounds'])
centre,extent = (lo+hi)*.5,max(hi-lo)
data = bpy.data.cameras.new('Retained native graph camera')
camera = bpy.data.objects.new('Retained native graph camera',data)
scene.collection.objects.link(camera)
camera.location = centre+Vector((-.4*extent,-1.5*extent,1.1*extent))
camera.rotation_euler = (centre-camera.location).to_track_quat('-Z','Y').to_euler()
data.type,data.ortho_scale,data.clip_end = 'ORTHO',extent*1.45,20000
scene.camera = camera
scene.world = bpy.data.worlds.new('Native graph sky')
scene.world.use_nodes = True
background = next(n for n in scene.world.node_tree.nodes if n.type == 'BACKGROUND')
background.inputs[0].default_value = (.32,.51,.76,1)
background.inputs[1].default_value = .7
data = bpy.data.lights.new('Native graph sun','SUN')
data.energy = 3
sun = bpy.data.objects.new('Native graph sun',data)
scene.collection.objects.link(sun)
sun.rotation_euler = (.45,-.5,-.6)
scene.render.engine = 'CYCLES'
scene.cycles.device = 'GPU'
scene.render.threads_mode = 'FIXED'
scene.render.threads = 4
scene.cycles.samples = 8
scene.cycles.use_denoising = True
scene.render.resolution_x,scene.render.resolution_y = 1600,1000
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.filepath = EXPORT_ROOT+'/retained-layout-v3.png'
print('NATIVE_LAYOUT_OVERVIEW_READY '+scene['counts'])
