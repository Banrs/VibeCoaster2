"""Blender review of the functional station at saved canonical StationBox poses.

This is a Blender preview. The station context is injected by run_blender_mcp.py
from a dump of buildStation on an accepted design.
"""
import bpy
import json
import math
from mathutils import Vector

EXPORT_ROOT = "__EXPORT_ROOT__"
context = json.loads(__STATION_CONTEXT__)
boxes = context["boxes"]
assert context["cars"] == 7
assert len([b for b in boxes if b["role"] == "HoldingLane"]) == context["cars"]
assert len([b for b in boxes if b["role"] == "BoardingGate"]) == context["cars"]
scene = bpy.context.scene
for obj in scene.objects:
    obj.hide_render = True
scene.render.engine = "BLENDER_EEVEE"
scene.render.resolution_x = 1600
scene.render.resolution_y = 1000
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.world.use_nodes = True
scene.world.node_tree.nodes["Background"].inputs[0].default_value = (.30, .38, .44, 1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value = .7
scene.view_settings.view_transform = "AgX"
scene.view_settings.exposure = .4

concrete = bpy.data.materials["VC2_Concrete"]
petrol = bpy.data.materials["VC2_Petrol"]
steel = bpy.data.materials["VC2_Steel"]
light = bpy.data.materials["VC2_Light"]
copper = bpy.data.materials["VC2_Copper"]

def instance(name, position, scale=(1, 1, 1), yaw=0, role="context"):
    source = bpy.data.objects[name]
    obj = source.copy()
    scene.collection.objects.link(obj)
    obj.name = "Review " + name
    obj.hide_render = False
    obj.hide_set(False)
    obj.location = position
    obj.scale = scale
    obj.rotation_euler = (0, 0, yaw)
    obj["review_role"] = role
    instances.append((role, obj))
    return obj

def block(name, position, size, material, role="context"):
    bpy.ops.mesh.primitive_cube_add(size=1, location=position)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    obj.data.materials.append(material)
    obj["review_role"] = role
    instances.append((role, obj))
    return obj

def tile_platform(box):
    x, y, z = box["center"]
    hx, hy, hz = box["half"]
    end = x + hx
    at = x - hx
    yaw = math.pi if y > 0 else 0
    while end-at >= 3-1e-8:
        instance("SM_StationPlatformPanel", (at+1.5, y, z+hz),
                 yaw=yaw, role="Platform")
        at += 3
    while end-at >= 1-1e-8:
        instance("SM_StationPlatformEndPanel", (at+.5, y, z+hz),
                 yaw=yaw, role="Platform")
        at += 1
    if end-at > 1e-7:
        block("Canonical platform remainder", ((at+end)*.5, y, z),
              (end-at, 2*hy, 2*hz), concrete, "Platform")

def tile_canopy(box):
    x, y, z = box["center"]
    hx, hy, hz = box["half"]
    end = x+hx
    at = x-hx
    while end-at >= 3-1e-8:
        instance("SM_StationRoofPanel", (at+1.5, y, z),
                 role="Canopy")
        at += 3
    if end-at > 1e-7:
        block("Canonical canopy remainder", ((at+end)*.5, y, z),
              (end-at, 2*hy, 2*hz), petrol, "Canopy")

mesh_for_role = {
    "QueueDeck": "SM_StationQueueDeck",
    "RouteRoof": "SM_StationRouteRoof",
    "MergeDeck": "SM_StationMergeDeck",
    "HoldingLane": "SM_StationHoldingLane",
    "BoardingGate": "SM_StationBoardingGate",
    "DispatchCabin": "SM_StationDispatchCabin",
    "UnloadDeck": "SM_StationUnloadDeck",
    "ExitWalkway": "SM_StationExitWalkway",
    "Lift": "SM_StationLift",
    "Stair": "SM_StationStair",
    "Underpass": "SM_StationUnderpass",
    "QueueRail": "SM_StationQueueRail",
}
instances = []
for box in boxes:
    role = box["role"]
    center, half = box["center"], box["half"]
    if role == "Platform":
        tile_platform(box)
    elif role == "Canopy":
        tile_canopy(box)
    elif role == "Post":
        # SM_StationPost has a base pivot and a 5.22 m canonical height.
        instance("SM_StationPost",
                 (center[0], center[1], center[2]-half[2]),
                 (half[0]/.2, half[1]/.2, 2*half[2]/5.22),
                 role=role)
    elif role in ("Pier", "Footing"):
        block("Canonical " + role, center, tuple(2*h for h in half),
              concrete if role == "Footing" else steel, role)
    else:
        instance(mesh_for_role[role], center, half,
                 math.pi if role == "Stair" and center[1] < 0 else 0,
                 role=role)

# Review context follows the certified level station segment. The full
# coaster beyond it is not represented in this architectural render.
station_min = min(b["center"][0]-b["half"][0] for b in boxes)
station_max = max(b["center"][0]+b["half"][0] for b in boxes)
queue = next(b for b in boxes if b["role"] == "QueueDeck")
ground_z = queue["center"][2]+queue["half"][2]-.30
block("Review ground", ((station_min+station_max)*.5, 0, ground_z-.12),
      (station_max-station_min+28, 75, .24), concrete)
for y, z, radius in ((-.65, 0, .085), (.65, 0, .085), (0, -.55, .16)):
    bpy.ops.mesh.primitive_cylinder_add(vertices=12, radius=radius,
        depth=station_max-station_min+14,
        location=((station_min+station_max)*.5, y, z))
    rail = bpy.context.object
    rail.name = "Review straight station rail"
    rail.rotation_euler.y = math.pi*.5
    rail.data.materials.append(steel if y else petrol)
    instances.append(("track", rail))
for x in range(math.floor(station_min/3)*3, math.ceil(station_max/3)*3+1, 3):
    instance("SM_TrackTieWeb", (x, 0, -.19), role="track")

def row_x(box):
    return box["center"][0]


rows = sorted((b for b in boxes if b["role"] == "HoldingLane"), key=row_x)
for i, row in enumerate(rows):
    name = "SM_LeadCar" if i == len(rows)-1 else "SM_TrainCar"
    instance(name, (row["center"][0], 0, 0), role="train")
assert all(abs(rows[i]["center"][0]-(30+i*context["spacing"])) < 1e-7
           for i in range(context["cars"]))

stamp = bpy.data.curves.new("Blender preview stamp", "FONT")
stamp.body = "BLENDER PREVIEW / FUNCTIONAL STATION"
stamp.size = 1.18
stamp.align_x = "CENTER"
stamp_obj = bpy.data.objects.new("Blender preview stamp", stamp)
scene.collection.objects.link(stamp_obj)
stamp_obj.location = ((station_min+station_max)*.5, 32, ground_z+.035)
stamp_obj.rotation_euler.z = math.pi
stamp.materials.append(light)

bpy.ops.object.light_add(type="SUN", location=(-35, -45, 90))
sun = bpy.context.object
sun.data.energy = 2.1
sun.data.angle = math.radians(8)
sun.rotation_euler = (math.radians(25), math.radians(-22), math.radians(-28))
for x in (station_min+15, (station_min+station_max)*.5, station_max-15):
    bpy.ops.object.light_add(type="AREA", location=(x, 0, 15))
    lamp = bpy.context.object
    lamp.data.energy = 2500
    lamp.data.shape = "DISK"
    lamp.data.size = 20

bpy.ops.object.camera_add()
camera = bpy.context.object
scene.camera = camera
focus = Vector(((station_min+station_max)*.5, 2, 0))
camera.data.type = "ORTHO"
camera.data.ortho_scale = 112
camera.location = focus + Vector((-36, 76, 44))
camera.rotation_euler = (focus-camera.location).to_track_quat("-Z", "Y").to_euler()
scene.render.filepath = EXPORT_ROOT + "/station-blender-exterior.png"
bpy.ops.render.render(write_still=True)

# Architectural cutaway: remove authored roof instances and section the
# underpass roof, exposing the walking surface at the same canonical box.
for role, obj in instances:
    if role in ("Canopy", "RouteRoof", "Underpass"):
        obj.hide_render = True
underpass = next(b for b in boxes if b["role"] == "Underpass")
cx, cy, cz = underpass["center"]
hx, hy, hz = underpass["half"]
block("Cutaway underpass floor", (cx, cy, cz-hz+.08),
      (2*hx, 2*hy, .16), concrete, "cutaway")

def label(body, role, height, material=light):
    candidates = [b for b in boxes if b["role"] == role]
    if not candidates:
        return
    center = candidates[0]["center"]
    curve = bpy.data.curves.new("Review label " + body, "FONT")
    curve.body = body
    curve.size = .90
    curve.align_x = "CENTER"
    obj = bpy.data.objects.new("Review label " + body, curve)
    scene.collection.objects.link(obj)
    obj.location = (center[0], center[1], height)
    obj.rotation_euler.z = math.pi
    curve.materials.append(material)

label("COVERED QUEUE", "QueueDeck", ground_z+.36)
label("MERGE / ROW HOLD", "MergeDeck", .06)
label("DISPATCH", "DispatchCabin", 3.38)
label("UNLOAD / EXIT", "UnloadDeck", .11, copper)
label("STEP-FREE LIFTS", "Lift", 2.8)
label("PROTECTED EXIT", "Underpass", ground_z+.25, copper)
camera.data.ortho_scale = 105
camera.location = focus + Vector((-16, 50, 86))
camera.rotation_euler = (focus-camera.location).to_track_quat("-Z", "Y").to_euler()
scene.render.filepath = EXPORT_ROOT + "/station-blender-cutaway.png"
bpy.ops.render.render(write_still=True)
print("Rendered Blender-only functional station exterior and cutaway from",
      len(boxes), "canonical StationBoxes.")
