#pragma once
#include "CoreMinimal.h"
#include "coaster/persistence.hpp"
struct FVibeMesh {
    TArray<FVector> Vertices, Normals;
    TArray<int32> Indices;
    TArray<FVector2D> UV;
    TArray<FLinearColor> Colors;
};
struct FVibeMeshes {
    FVibeMesh Rails, Ties, Terrain, Supports, Station;
};
FVector VibePosition(coaster::Vec3 V);
FVibeMeshes BuildVibeMeshes(const coaster::Design &Design, const coaster::Cancel &Cancel);
FVibeMesh BuildVibeCar();
