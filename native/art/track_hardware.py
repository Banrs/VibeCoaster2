"""Blender source appended to author_models.py before its final manifest write.

The tie pivot is 0.19 m below the rail datum. Operation meshes use the existing
unit-box pivot, so the verified operation boxes continue to set every dimension.
"""


def author_track_hardware(box, tube, collect, materials):
    import bpy
    import bmesh
    from mathutils import Vector

    shell = materials["shell"]
    edge = materials["edge"]
    dark = materials["dark"]
    conductor = materials["conductor"]
    steel = materials["steel"]

    def solid(name, vertices, faces, finish):
        mesh = bpy.data.meshes.new(name)
        mesh.from_pydata(vertices, [], faces)
        mesh.update()
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.scene.collection.objects.link(obj)
        obj.data.materials.append(finish)
        bm = bmesh.new()
        bm.from_mesh(mesh)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bm.to_mesh(mesh)
        bm.free()
        return obj

    def diagonal_plate(name, a, direction, normal, begin, end, x_half, n_half, finish):
        # This plate is inscribed in the canonical diagonal track-web prism.
        vertices = []
        for t in (begin, end):
            p = a + direction * t
            for x, n in ((-x_half, -n_half), (x_half, -n_half),
                         (x_half, n_half), (-x_half, n_half)):
                q = p + Vector((x, 0, 0)) + normal * n
                vertices.append(tuple(q))
        faces = [(3, 2, 1, 0), (4, 5, 6, 7)]
        faces += [(j, (j + 1) % 4, (j + 1) % 4 + 4, j + 4) for j in range(4)]
        solid(name, vertices, faces, finish)

    start = set(bpy.context.scene.objects)
    for side in (-1, 1):
        a = Vector((0, side * .09, -.24))
        b = Vector((0, side * .65, 0))
        direction = b - a
        normal = Vector((0, -direction.z, direction.y)).normalized()
        # Twin slim chords, with daylight through their middle, replace the
        # former solid crosshead. All main pieces fit the validated web OBB.
        for x in (-.044, .044):
            offset = Vector((x, 0, 0))
            tube("Track web edge chord", a + offset, b + offset,
                 .011, edge, 8)
        diagonal_plate("Web root socket", a, direction, normal,
                       0, .105, .052, .028, shell)
        diagonal_plate("Rail-side web socket", a, direction, normal,
                       .895, 1, .052, .028, shell)
        for t in (.28, .50, .72):
            diagonal_plate("Open web crosspiece", a, direction, normal,
                           t - .025, t + .025, .051, .015,
                           conductor if t == .50 else steel)
        # A small insulated service conduit visually follows the web inside
        # its protected cross-section; it stops short of both sockets.
        inset = normal * -.023
        tube("Web service conduit", a + direction * .12 + inset,
             a + direction * .88 + inset, .006, dark, 8)
        # The certified saddle reaches the underside of the 85 mm running rail.
        box("Rail inboard saddle", (0, side * .605, .0485),
            (.104, .120, .113), dark, .006)
        box("Rail saddle wear pad", (0, side * .635, .098),
            (.094, .060, .014), steel, .004)
    collect("SM_TrackTieWeb", start)

    start = set(bpy.context.scene.objects)
    # Normalized unit-box motor cassette. The runtime scales this same module
    # into the side stators and their lower spine mounting boxes.
    box("LSM sealed bed", (0, 0, -.315), (.94, .86, .37), dark, .024)
    for side in (-1, 1):
        box("LSM side rail", (0, side * .445, .055),
            (.96, .075, .89), shell, .018)
        box("LSM copper bus", (0, side * .385, .225),
            (.88, .025, .06), conductor, .009)
    for index in range(7):
        x = (index - 3) * .116
        box("LSM laminated pole", (x, 0, .09),
            (.082, .71, .34), steel, .012)
        box("LSM exposed winding", (x, 0, .27),
            (.061, .66, .039), conductor, .007)
    box("LSM centre isolation seam", (0, 0, .301),
        (.84, .025, .012), dark, .003)
    collect("SM_LSMStator", start)

    start = set(bpy.context.scene.objects)
    # A tapered conductive fin with a broad root and thin relief channels.
    # It also reads as a compact caliper when the canonical brake boxes scale it
    # wider. Trim-fin animation keeps exactly the same normalized pivot.
    contour = [(-.48, -.47), (.48, -.47), (.44, -.31),
               (.34, .41), (.28, .47), (-.28, .47),
               (-.34, .41), (-.44, -.31)]
    vertices = [(x, y, z) for y in (-.405, .405) for x, z in contour]
    size = len(contour)
    faces = [tuple(reversed(range(size))), tuple(range(size, size * 2))]
    faces += [(i, (i + 1) % size, (i + 1) % size + size, i + size)
              for i in range(size)]
    solid("Tapered conductive brake fin", vertices, faces, steel)
    for side in (-1, 1):
        y = side * .413
        box("Brake cooling relief", (0, y, .085),
            (.44, .010, .095), dark, .008)
        box("Brake wear indicator", (0, side * .421, -.14),
            (.69, .008, .017), conductor, .004)
    box("Brake root cartridge", (0, 0, -.421),
        (.91, .91, .105), shell, .017)
    collect("SM_BrakeFin", start)
