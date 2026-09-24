"""Functional station kit. All semantic meshes use a centre pivot and unit half bounds.

The canonical StationBox supplies the exact envelope and runtime scale. Local
+Y is the loading side and +X follows train travel. The existing platform
panels keep their top-centre pivot and native dimensions.
"""


def author_station_models(box, tube, collect, materials):
    import bpy
    from mathutils import Vector

    shell = materials["shell"]
    ceramic = materials["ceramic"]
    dark = materials["dark"]
    metal = materials["metal"]
    concrete = materials["stone"]
    glass = materials["glass"]
    light = materials["light"]
    result = {}

    def start():
        return set(bpy.context.scene.objects)

    def block(name, centre, size, material, bevel=0):
        return box(name, centre, size, material, bevel)

    def rod(name, a, b, radius, material):
        return tube(name, a, b, radius, material, 8)

    def finish(name, before, low=(-1, -1, -1), high=(1, 1, 1)):
        for obj in bpy.context.scene.objects:
            if obj.type != "MESH" or obj in before:
                continue
            for vertex in obj.data.vertices:
                p = obj.matrix_world @ vertex.co
                if any(p[i] < low[i] - 1e-5 or p[i] > high[i] + 1e-5
                       for i in range(3)):
                    raise RuntimeError(name + " exceeds its canonical prototype: " + obj.name + " " + str(tuple(p)))
        result[name] = collect(name, before)

    # The native platform panel contract predates semantic station roles.
    # It remains a 3 m / 1 m tile, with the top at local Z=0.
    for name, length in (("SM_StationPlatformPanel", 2.992),
                         ("SM_StationPlatformEndPanel", .992)):
        before = start()
        block("Boarding structural slab", (0, 0, -.43),
              (length, 3.85, .74), concrete, .018)
        block("Boarding paving", (0, 0, -.035),
              (length, 3.81, .07), ceramic, .008)
        block("Tactile boarding edge", (0, 1.82, -.012),
              (length, .14, .024), metal)
        block("Outer wayfinding edge", (0, -1.88, -.011),
              (length, .05, .022), shell)
        finish(name, before, (-length / 2, -1.925, -.8),
               (length / 2, 1.925, 0))

    before = start()
    block("Queue reinforced floor", (0, 0, .72), (2, 2, .56), concrete, .018)
    block("Queue route inlay", (0, 0, .999), (1.92, .055, .002), shell)
    finish("SM_StationQueueDeck", before)

    before = start()
    block("Canopy insulated plane", (0, 0, 0), (2, 2, 1.55), shell, .03)
    for y in (-.92, .92):
        block("Canopy edge fascia", (0, y, -.76), (2, .13, .24), metal)
    for x in (-.85, -.30, .30, .85):
        block("Canopy linear light", (x, 0, -.80), (.045, 1.7, .045), light)
    finish("SM_StationRouteRoof", before)

    before = start()
    block("Clear feeder paving", (0, 0, .72), (2, 2, .56), concrete)
    block("Feeder direction band", (0, 0, .999), (1.9, .09, .002), shell)
    finish("SM_StationMergeDeck", before)

    before = start()
    block("Numbered holding lane floor", (0, 0, -.92),
          (2, 2, .16), ceramic)
    for x in (-.94, .94):
        for y in (-.87, 0, .87):
            block("Holding rail stanchion", (x, y, -.21),
                  (.07, .07, 1.48), dark)
        rod("Continuous holding rail", (x, -.9, .58),
            (x, .9, .58), .028, metal)
    block("Entry lane threshold", (0, .93, -.82), (1.78, .07, .08), shell)
    finish("SM_StationHoldingLane", before)

    before = start()
    for x in (-.91, .91):
        block("Gate controlled post", (x, 0, -.05),
              (.14, 1.5, 1.9), dark)
    rod("Closed safety arm", (-.84, 0, .48), (.84, 0, .48), .045, metal)
    block("Gate reader", (.87, -.55, .45), (.16, .24, .32), shell)
    finish("SM_StationBoardingGate", before)

    before = start()
    block("Dispatch cabin floor", (0, 0, -.94), (2, 2, .12), concrete)
    block("Dispatch cabin roof", (0, 0, .91), (2, 2, .18), shell, .04)
    for x in (-.93, .93):
        block("Cabin side mullion", (x, 0, 0), (.12, 2, 1.78), dark)
    block("Train-facing control glass", (0, -.94, 0),
          (1.74, .08, 1.72), glass)
    for x in (-.74, .74):
        block("Feeder-side glazed leaf", (x, .94, 0),
              (.45, .08, 1.72), glass)
    block("Cabin dispatch console", (0, -.61, -.56),
          (1.22, .33, .22), metal)
    finish("SM_StationDispatchCabin", before)

    before = start()
    block("Unload tactile edge", (0, 0, 0), (2, 2, 2), ceramic)
    block("Amber exit direction inset", (0, .65, .999),
          (1.90, .14, .002), metal)
    finish("SM_StationUnloadDeck", before)

    before = start()
    block("Exit passage walking slab", (0, 0, .72),
          (2, 2, .56), concrete)
    for x in (-.92, .92):
        block("Exit deck edge light", (x, 0, .999),
              (.035, 1.9, .002), light)
    finish("SM_StationExitWalkway", before)

    before = start()
    # The shaft spans -4.5..+2.65 m. Its lower cab floor meets the
    # -4.2 m queue/exit deck exactly while retaining the -1 bottom bound.
    lift_floor_top = -1 + 2 * .30 / 7.15
    block("Lift lower floor", (0, 0, (lift_floor_top-1)/2),
          (1.9, 1.9, lift_floor_top+1), concrete)
    block("Lift upper cap", (0, 0, .91), (1.9, 1.9, .18), shell)
    for x in (-.94, .94):
        block("Lift glazed side", (x, 0, -.01),
              (.09, 1.9, 1.80), glass)
        block("Lift vertical frame", (x, -.90, -.01),
              (.10, .10, 1.80), dark)
        block("Lift vertical frame", (x, .90, -.01),
              (.10, .10, 1.80), dark)
    finish("SM_StationLift", before)

    before = start()
    # The canonical stair box spans ground=-4.5 m to +1 m. Its upper tread
    # lands at platform Z=0; the lower tread is 175 mm above the queue deck.
    # Twenty-four equal 175 mm rises connect the -4.2 m queue floor to Z=0.
    tread_y = [-1 + (i+.5)*2/24 for i in range(24)]
    tread_top = [(.0 - (-1.75)) / 2.75 - i * (.175 / 2.75)
                 for i in range(24)]
    for i, (y, top) in enumerate(zip(tread_y, tread_top)):
        block("Accessible stair tread", (0, y, top-.0175),
              (1.82, 2/24, .035), concrete)
        for x in (-.92, .92):
            block("Stair handrail upright", (x, y, top+.16),
                  (.045, .045, .32), metal)
    for x in (-.92, .92):
        for i in range(23):
            rod("Stair sloping handrail",
                (x, tread_y[i], tread_top[i]+.32),
                (x, tread_y[i+1], tread_top[i+1]+.32),
                .018, metal)
    finish("SM_StationStair", before)

    before = start()
    # This box spans -4.65..-1.15 m; the walking face meets both
    # lower ExitWalkway slabs at -4.2 m.
    passage_floor_top = -1 + 2 * .45 / 3.5
    block("Passage floor", (0, 0, (passage_floor_top-1)/2),
          (2, 2, passage_floor_top+1), concrete)
    block("Protected overhead", (0, 0, .91), (2, 2, .18), shell)
    block("East side wall", (.94, 0, 0), (.12, 2, 1.80), glass)
    for low, high in ((-1, -.59), (-.32, 1)):
        block("West wall beside transfer", (-.94, (low+high)*.5, 0),
              (.12, high-low, 1.80), glass)
    for y in (-.9, .9):
        block("Passage floor guide", (0, y, passage_floor_top-.002),
              (1.75, .04, .004), metal)
    finish("SM_StationUnderpass", before)

    before = start()
    for x in (-.94, 0, .94):
        block("Queue guide stanchion", (x, 0, -.1),
              (.07, 1.50, 1.8), dark)
    rod("Queue top rail", (-.97, 0, .90),
        (.97, 0, .90), .03, metal)
    block("Queue lower guard", (0, 0, -.67),
          (1.96, 1.5, .12), shell)
    finish("SM_StationQueueRail", before)
    return result
