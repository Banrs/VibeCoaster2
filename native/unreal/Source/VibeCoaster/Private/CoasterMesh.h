#pragma once
#include "CoreMinimal.h"
#include "Math/RotationMatrix.h"
#include "coaster/coaster.hpp"
#include "CoordinateContract.h"
#include <atomic>
#include <memory>

namespace VibeMesh
{
inline FVector Position(coaster::Vec3 P) { auto V = VibeCoordinates::Position(P); return FVector(V.X, V.Y, V.Z); }
inline FVector Direction(coaster::Vec3 P) { auto V = VibeCoordinates::Direction(P); return FVector(V.X, V.Y, V.Z); }
inline FQuat Rotation(const coaster::TrackSample& P) { return FRotationMatrix::MakeFromXZ(Direction(P.tangent), Direction(P.up)).ToQuat(); }

struct FChunk
{
    TArray<FVector> Vertices, Normals;
    TArray<int32> Indices;
    TArray<FVector2D> UV;
    bool Ground = false, Structure = false, Footing = false;
};
enum class EStationInstanceKind : uint8 { CubeSteel, CubeConcrete, Platform, PlatformEnd, Roof, Post };
struct FStationInstance
{
    FTransform Transform;
    EStationInstanceKind Kind = EStationInstanceKind::CubeSteel;
    int32 SourceBoxIndex = INDEX_NONE;
};
struct FPreparedRide
{
    std::shared_ptr<const coaster::Design> Design;
    TArray<FChunk> Chunks;
    TArray<FTransform> Ties, Supports;
    TArray<FStationInstance> Station;
    FBox Bounds{ForceInit};
    FString Error;
    uint64 Revision = 0;
};
// Imported assets are already centimetre-sized: authored modules use unit scale.
// This value-only helper tiles contained details inside a canonical station box;
// unsupported cross-sections/poses retain the original cube representation.
bool AppendStationBoxInstances(FPreparedRide& Out, const coaster::StationBox& Box,
    int32 SourceBoxIndex, coaster::Vec3 StationMidline, const coaster::Cancel& Cancel);
// Plain value buffers only. No UObject creation, lookup or mutation on workers.
bool AppendGround(FPreparedRide& Out, const coaster::Cancel& Cancel);
bool Prepare(FPreparedRide& Out, const coaster::Cancel& Cancel);
}

