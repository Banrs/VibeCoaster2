"""Join the exported terrain boundary to a distant review backdrop without gaps."""
import bpy


def add_context_apron(scene, terrain, material):
    for obj in list(scene.objects):
        if obj.name.startswith(('Terrain / outside context apron', 'Native context apron')):
            mesh = obj.data
            scene.collection.objects.unlink(obj)
            if obj.users == 0:
                bpy.data.objects.remove(obj)
                if mesh.users == 0:
                    bpy.data.meshes.remove(mesh)
    columns, rows = terrain['columns'], terrain['rows']
    boundary = (list(range(columns))
                + [y*columns+columns-1 for y in range(1, rows)]
                + [(rows-1)*columns+x for x in range(columns-2, -1, -1)]
                + [y*columns for y in range(rows-2, 0, -1)])
    inner = [terrain['points'][i] for i in boundary]
    cx = (inner[0][0]+inner[columns-1][0])*.5
    cy = (terrain['points'][0][1]+terrain['points'][-1][1])*.5
    extent = max(columns, rows)*terrain['step']*.5
    factor = max(4., 6000/extent)
    level = min(0., min(p[2] for p in inner))-.25
    outer = [(cx+(x-cx)*factor, cy+(y-cy)*factor, level) for x, y, z in inner]
    n = len(inner)
    faces = [(i, i+n, (i+1)%n+n, (i+1)%n) for i in range(n)]
    mesh = bpy.data.meshes.new('Native context apron')
    mesh.from_pydata(inner+outer, [], faces)
    mesh.update()
    obj = bpy.data.objects.new('Native context apron', mesh)
    scene.collection.objects.link(obj)
    mesh.materials.append(material)
