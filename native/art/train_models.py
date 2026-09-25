"""Riftwake Vector: a compact high-speed speedster train, authored in metres.

The continuous bonnet/sidepod silhouette is original. Each asset remains one
two-seat row; runtime supplies seven 3.4 m-spaced rows.
The rail pivot, +X direction, eye (0, 0, 1.2) and clearance remain unchanged.
"""
import bpy
import bmesh
import math
from mathutils import Vector


def author_train_models(box, tube, collect, materials):
    shell, ceramic, carbon = materials["shell"], materials["ceramic"], materials["carbon"]
    metal, accent, cushion = materials["metal"], materials["accent"], materials["cushion"]
    glass, light = materials["glass"], materials["light"]
    scene = bpy.context.scene

    def mesh(name, vertices, faces, material, smooth=True):
        data = bpy.data.meshes.new(name)
        data.from_pydata(vertices, [], faces)
        data.update()
        obj = bpy.data.objects.new(name, data)
        scene.collection.objects.link(obj)
        data.materials.append(material)
        bm = bmesh.new()
        bm.from_mesh(data)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bm.to_mesh(data)
        bm.free()
        for face in data.polygons:
            face.use_smooth = smooth
        return obj

    def sections(keys, steps=5):
        result = []
        for i in range(len(keys)-1):
            a,b,c,d = keys[max(0,i-1)],keys[i],keys[i+1],keys[min(len(keys)-1,i+2)]
            for j in range(steps):
                t = j/steps
                row = [b[0]+(c[0]-b[0])*t]
                for k in range(1,len(b)):
                    value = .5*(2*b[k]+(-a[k]+c[k])*t+(2*a[k]-5*b[k]+4*c[k]-d[k])*t*t+(-a[k]+3*b[k]-3*c[k]+d[k])*t*t*t)
                    row.append(max(min(b[k],c[k]),min(max(b[k],c[k]),value)))
                result.append(row)
        return result+[keys[-1]]

    def loft(name, rings, material, smooth=True):
        n = len(rings[0])
        vertices = [v for ring in rings for v in ring]
        faces = [tuple(reversed(range(n)))]
        for i in range(len(rings)-1):
            for j in range(n):
                faces.append((i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j))
        faces.append(tuple((len(rings)-1)*n+j for j in range(n)))
        return mesh(name,vertices,faces,material,smooth)

    def sheet(name, rows, material, thickness=.01, axis=(0,0,1)):
        n = len(rows[0])
        vertices = [v for row in rows for v in row]
        count = len(vertices)
        vertices += [tuple(v[k]-thickness*axis[k] for k in range(3)) for v in vertices]
        faces = []
        for side in range(2):
            for i in range(len(rows)-1):
                for j in range(n-1):
                    face=(side*count+i*n+j,side*count+i*n+j+1,side*count+(i+1)*n+j+1,side*count+(i+1)*n+j)
                    faces.append(face if side==0 else tuple(reversed(face)))
        rim=list(range(n))+[i*n+n-1 for i in range(1,len(rows))]
        rim+=list(range(count-2,count-n-1,-1))+[i*n for i in range(len(rows)-2,0,-1)]
        faces += [(a,b,b+count,a+count) for a,b in zip(rim,rim[1:]+rim[:1])]
        return mesh(name,vertices,faces,material)

    def plate(name, polygon, y, thickness, material, bevel=.008):
        rings=[[(x,y-thickness*.5,z) for x,z in polygon],[(x,y+thickness*.5,z) for x,z in polygon]]
        obj=loft(name,rings,material,False)
        if bevel:
            bpy.context.view_layer.objects.active=obj
            mod=obj.modifiers.new("Machined plate edges","BEVEL")
            mod.width=bevel
            mod.segments=2
            bpy.ops.object.modifier_apply(modifier=mod.name)
        return obj

    def annulus(name, centre, axis, profile, material, sides=28):
        centre,axis=Vector(centre),Vector(axis).normalized()
        u=Vector((1,0,0))
        v=axis.cross(u).normalized()
        rings=[]
        for offset,radius in profile:
            rings.append([tuple(centre+axis*offset+(u*math.cos(2*math.pi*j/sides)+v*math.sin(2*math.pi*j/sides))*radius) for j in range(sides)])
        vertices=[p for ring in rings for p in ring]
        faces=[]
        for i in range(len(rings)):
            for j in range(sides):
                faces.append((i*sides+j,i*sides+(j+1)%sides,((i+1)%len(rings))*sides+(j+1)%sides,((i+1)%len(rings))*sides+j))
        return mesh(name,vertices,faces,material)

    def bogies():
        # A machined spine joins both trucks. The wheel-clamping plates have
        # generous axle bosses, rather than dangling rods or road-car tyres.
        box("Machined chassis spine",(-.05,0,.23),(2.24,.34,.23),metal,.028)
        for x in (-.91,.83):
            box("Bogie bolster",(x,0,.26),(.29,1.31,.19),metal,.025)
            for side in (-1,1):
                y=side*.65
                centre=(x,y,.295)
                annulus("High-speed polyurethane tread",centre,(0,1,0),
                    [(-.065,.152),(-.074,.194),(-.052,.21),(.052,.21),(.074,.194),(.065,.152)],cushion)
                annulus("Ventilated wheel rim",centre,(0,1,0),
                    [(-.06,.132),(-.065,.158),(.065,.158),(.06,.132)],metal)
                tube("Wheel bearing",(x,y-.079,.295),(x,y+.079,.295),.055,metal,16)
                for angle in range(0,360,45):
                    a=math.radians(angle)
                    spoke=box("Ventilated spoke",(x+.095*math.cos(a),y,.295+.095*math.sin(a)),(.099,.071,.025),metal,.005)
                    spoke.rotation_euler.y=-a
                # Guide rollers occupy the inboard rail face; angled upstops
                # retain the outboard underside without occupying tie attachment.
                tube("Side roller",(x,side*.47,-.040),(x,side*.47,.040),.095,cushion,20)
                tube("Side roller bearing",(x,side*.47,-.043),(x,side*.47,.043),.045,metal,14)
                c=Vector((x,side*.745,-.164545))
                axle=Vector((0,side*.866025,.5))
                tube("Upstop roller",c-axle*.031,c+axle*.031,.105,cushion,20)
                tube("Upstop bearing",c-axle*.035,c+axle*.035,.052,metal,16)
                cheek=[(x-.16,.34),(x+.15,.34),(x+.15,.18),(x+.09,-.10),(x+.09,-.15),(x-.07,-.15),(x-.14,.12)]
                plate("Forged bogie cheek",cheek,side*.812,.038,metal,.012)
                tube("Upper axle cap",(x,side*.73,.295),(x,side*.833,.295),.062,metal,16)
                tube("Retaining pin",(x,side*.817,.295),(x,side*.84,.295),.024,carbon,8)
                tube("Lower clevis pin",c+axle*.031,(x,side*.831,-.116),.028,metal,12)
                plate("Guide axle carrier",[(x-.12,.28),(x+.12,.28),(x+.08,.04),(x-.07,.04)],side*.47,.07,metal)

    def body(lead):
        # The bonnet and both sidepods share section widths and edge heights.
        # This keeps the upper body a continuous volume, including in profile.
        hood_keys = ([(.34,.835,.955,.91),(.51,.805,1.075,.975),
                      (.70,.745,1.115,1.035),(.91,.61,.975,.92),
                      (1.10,.425,.795,.755),(1.265,.19,.60,.565)] if lead else
                     [(.34,.835,.865,.82),(.51,.805,.935,.855),
                      (.70,.745,.94,.86),(.91,.61,.825,.77),
                      (1.10,.425,.665,.625),(1.265,.19,.49,.455)])
        flank_keys = [(-1.265,.55,.65),(-1.13,.76,.74),(-.80,.855,.785),
                      (-.42,.88,.70),(-.04,.865,.655),(.18,.85,.75)]
        flank_keys += [(x,w,edge) for x,w,_,edge in hood_keys]
        for side in (-1,1):
            rings=[]
            for x,w,rim in sections(flank_keys,6):
                # Broad concave sidepod with an actual upper return into the
                # cockpit; its lower crease gives the wheel pods breathing room.
                yz=[(w-.12,.365),(w-.145,rim-.065),(w-.09,rim-.005),
                    (w-.035,rim),(w,rim-.04),(w-.008,.47),
                    (w-.07,.335),(w-.14,.305)]
                rings.append([(x,side*y,z) for y,z in yz])
            loft("Continuous sculpted sidepod",rings,shell)
            blade=[]
            for x,w,rim in sections(flank_keys,6):
                if x < hood_keys[0][0]:
                    blade.append([(x,side*(w-.034),rim+.002),
                                  (x,side*(w-.081),rim+.003)])
            for x,w,crown,edge in sections(hood_keys,7):
                blade.append([(x,side*(w-offset),
                               edge+(crown-edge)*(1-((w-offset)/w)**1.65)+.003)
                              for offset in (.034,.081)])
            sheet("Flush titanium shoulder",blade,ceramic,.004)
            # A single recessed cooling channel follows the flank; no applique
            # rods, cage, faux road wheels or disconnected decorative fins.
            rows=[]
            for x,w,z in [(-1.10,.763,.46),(-.76,.86,.465),(-.30,.877,.435),
                          (.18,.847,.46),(.48,.813,.57),(.74,.724,.63)]:
                rows.append([(x,side*(w+.002),z),
                             (x,side*(w+.002),z+.055)])
            sheet("Sidepod cooling channel",rows,carbon,.006,axis=(0,side,0))
            trace=[]
            for x,w,z in [(-1.10,.763,.521),(-.76,.86,.526),(-.30,.877,.496),
                          (.18,.847,.521),(.48,.813,.631),(.74,.724,.691)]:
                trace.append([(x,side*(w+.003),z),
                              (x,side*(w+.003),z+.012)])
            sheet("Copper coachline",trace,accent,.003,axis=(0,side,0))
        rings=[]
        for x,w,crown,edge in sections(hood_keys,7):
            roof=[(x,w*u,edge+(crown-edge)*(1-abs(u)**1.65))
                  for u in [j/8 for j in range(-8,9)]]
            rings.append(roof+[(x,w-.07,.355),(x,-w+.07,.355)])
        loft("Continuous high bonnet",rings,shell)
        # A narrow centre spine is a flush surface on the bonnet itself.
        rows=[]
        for x,w,crown,edge in sections(hood_keys,7):
            rows.append([(x,-.025,crown+.001),(x,.025,crown+.001)])
        sheet("Bonnet centre seam",rows,carbon,.003)
        if lead:
            for side in (-1,1):
                # Small lamps sit on the outer bevel, leaving the bonnet as one
                # uninterrupted surface instead of an animal or racing-car face.
                rows=[]
                for x,w,crown,edge in sections(hood_keys[2:5],7):
                    z=edge+(crown-edge)*(1-.80**1.65)
                    rows.append([(x,side*w*.80,z+.005),
                                 (x,side*(w*.80-.018),z+.010)])
                sheet("Recessed running light",rows,light,.006)
        box("Carbon cockpit floor",(-.40,0,.392),(1.69,1.41,.075),carbon,.025)
        box("Seat mount crossmember",(-.49,0,.445),(.30,1.26,.16),metal,.018)
        box("Rear structural bulkhead",(-1.13,0,.48),(.22,1.14,.40),shell,.055)
        for side in (-1,1):
            box("Rear service panel",(-1.248,side*.34,.49),(.024,.40,.16),carbon,.017)
            box("Rear marker",(-1.264,side*.42,.63),(.009,.21,.022),light,.006)
        box("Coupler mounting tongue",(-1.175,0,.28),(.20,.25,.17),metal,.018)

    def seat(y):
        # Two distinct bucket seats per row, with a .75 m pan matching the
        # clearance model's eye-minus-.45 m seating assumption.
        rows=[]
        for z,w,x in sections([(.58,.20,-.66),(.76,.255,-.56),(.99,.268,-.65),
                              (1.28,.245,-.765),(1.51,.185,-.805),
                              (1.625,.14,-.79)],5):
            rows.append([(x+.12*u*u,y+w*u,z-.012*abs(u))
                         for u in [j/6 for j in range(-6,7)]])
        sheet("Streamlined carbon seat shell",rows,carbon,.075,axis=(1,0,0))
        rows=[]
        for z,w,x in sections([(.77,.205,-.485),(.98,.205,-.57),
                              (1.24,.20,-.674),(1.48,.148,-.715),
                              (1.585,.105,-.72)],5):
            rows.append([(x+.082*u*u,y+w*u,z) for u in [j/6 for j in range(-6,7)]])
        sheet("Sculpted back cushion",rows,cushion,.085,axis=(1,0,0))
        rows=[]
        for x,w,z in sections([(-.50,.195,.79),(-.30,.23,.75),
                              (-.02,.224,.75),(.15,.20,.795)],5):
            rows.append([(x,y+w*u,z+.041*u*u) for u in [j/6 for j in range(-6,7)]])
        sheet("Bucket seat pan",rows,cushion,.055)
        box("Seat pedestal",(-.39,y,.595),(.46,.37,.29),carbon,.04)
        for side in (-1,1):
            # The hinge housing grows from the floor behind each seat and the
            # closed arm carries the lap pad. It is not an overhead safety cage.
            arm_y=y+side*.267
            plate("Restraint hinge housing",[(-.89,.415),(-.69,.415),
                  (-.66,.88),(-.73,1.35),(-.82,1.415),(-.95,1.34)],
                  arm_y,.075,carbon,.018)
            tube("Restraint pivot",(-.82,arm_y-.056,1.335),
                 (-.82,arm_y+.056,1.335),.066,metal,20)
            arm=[(-.87,1.355),(-.80,1.414),(-.70,1.398),(-.47,1.25),
                 (-.18,1.05),(.18,.965),(.215,.92),(.16,.88),
                 (-.23,.98),(-.54,1.19),(-.76,1.333)]
            plate("Tapered carbon restraint arm",arm,arm_y,.038,carbon,.012)
            # A slim shell edge continues the body's titanium line up the seat.
            trim=[]
            for z,w,x in [(.89,.258,-.61),(1.15,.257,-.715),
                           (1.40,.218,-.788),(1.565,.164,-.80)]:
                trim.append([(x+.12,y+side*w,z),(x+.14,y+side*(w-.018),z)])
            sheet("Seat shell edge",trim,ceramic,.008,axis=(1,0,0))
        # A low transverse pad, directly attached to both arms. Every bar in the
        # forward half of the vehicle remains below the canonical eye.
        rings=[]
        for x,w,low,high in [( .095,.229,.872,.922),(.14,.242,.865,.969),
                              (.24,.228,.89,.972),(.275,.205,.91,.95)]:
            rings.append([(x,y-w,low),(x,y+w,low),(x,y+w,high),(x,y-w,high)])
        loft("Contoured lap restraint",rings,cushion)
        tube("Lap pad structural axle",(.165,y-.287,.925),
             (.165,y+.287,.925),.024,metal,16)
        for side in (-1,1):
            tube("Restraint handgrip",(.235,y+side*.15,.963),
                 (.275,y+side*.15,1.019),.018,carbon,12)

    def aeroscreen(lead):
        # Compact transparent screen rises into outboard wind-deflecting cheeks.
        # The central top edge remains below the 1.2 m eye, so clear visibility
        # does not depend on correct transparency in the runtime importer.
        rows=[]
        base=1.065 if lead else .925
        for j in range(9):
            v=j/8
            row=[]
            for i in range(33):
                u=(i-16)/16
                x=.535-.19*v-.19*abs(u)**2
                z=base-.09*abs(u)**2+v*((1.16-base)+.24*abs(u)**5)
                row.append((x,(.79-.025*v)*u,z))
            rows.append(row)
        sheet("Low wraparound aeroscreen",rows,glass,.006,axis=(1,0,0))
        # Opaque mounting shoes are below the rim and at the outer shoulders.
        for side in (-1,1):
            plate("Aeroscreen shoulder shoe",[(.115,.705),(.33,.785),
                  (.37,base-.095),(.26,base-.07),(.115,.82)],
                  side*.765,.052,shell,.013)
        row=rows[0]
        sheet("Aeroscreen base gasket",[[p,(p[0]-.012,p[1],p[2]-.012)]
              for p in row],carbon,.008)

    result={}
    for name,lead in (("SM_LeadCar",True),("SM_TrainCar",False)):
        start=set(scene.objects)
        bogies()
        body(lead)
        for y in (-.43,.43):
            seat(y)
        aeroscreen(lead)
        result["lead" if lead else "passenger"]=collect(name,start)
    start=set(scene.objects)
    box("Articulated drawbar",(0,0,0),(1,.11,.11),metal,.015)
    for x in (-.40,.40):
        box("Coupler socket",(x,0,0),(.20,.20,.18),carbon,.032)
        tube("Coupler pin",(x,0,-.089),(x,0,.089),.050,metal,16)
    collect("SM_TrainCoupler",start)
    return result
