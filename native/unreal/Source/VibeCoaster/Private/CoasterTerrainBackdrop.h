#pragma once
#include "CoasterMesh.h"

// Additive distant visual LOD only. Near-track terrain/physics are unchanged.
namespace VibeMesh
{
namespace TerrainBackdrop
{
struct FRing
{
    double MinX, MinY, MaxX, MaxY;
    int32 SegmentsX, SegmentsY;

    int32 Segments(int32 Side) const { return (Side & 1) ? SegmentsY : SegmentsX; }
    FVector2D Corner(int32 Index) const
    {
        switch (Index & 3)
        {
        case 0: return FVector2D(MinX, MinY);
        case 1: return FVector2D(MaxX, MinY);
        case 2: return FVector2D(MaxX, MaxY);
        default: return FVector2D(MinX, MaxY);
        }
    }
    FVector2D Point(int32 Side, int32 Index) const
    {
        const int32 Count = Segments(Side);
        // Explicit endpoints give adjacent side strips identical corner values.
        if (Index == 0) return Corner(Side);
        if (Index == Count) return Corner(Side + 1);
        const FVector2D A = Corner(Side), B = Corner(Side + 1);
        return A + ((B - A) / double(Count)) * double(Index);
    }
};
constexpr int32 DenseRings = 48, CoarseRings = 12;
// Each chunk becomes one procedural component/section. Keep cliff geometry
// intact while avoiding hundreds of tiny draw submissions in the dense halo.
constexpr int32 CliffChunkVertices = 6144, DefaultChunkVertices = 288;
constexpr double DenseBandWidth = 20, DenseEdgeLength = 20;
// One explicit plan drives both preflight budgets and actual emission. Keeping
// a 960 m halo at <=20 m side spacing preserves steep walls beside the ride.
inline TArray<FRing> BuildRingPlan(double X0, double Y0, int32 NX, int32 NY, bool DetailedCliffs)
{
    const int32 DenseBands = DetailedCliffs ? DenseRings : 0;
    TArray<FRing> Plan; Plan.Reserve(1 + DenseBands + CoarseRings);
    Plan.Add({X0, Y0, X0 + NX * 320., Y0 + NY * 320., NX * 16, NY * 16});
    for (int32 Ring = 0; Ring < DenseBands + CoarseRings; ++Ring)
    {
        const FRing Inner = Plan.Last();
        const double Width = Ring < DenseBands ? DenseBandWidth : 20. * double(uint64(1) << (Ring - DenseBands));
        const double MinX = Inner.MinX - Width, MinY = Inner.MinY - Width;
        const double MaxX = Inner.MaxX + Width, MaxY = Inner.MaxY + Width;
        const int32 SX = Ring < DenseBands ? FMath::CeilToInt((MaxX - MinX) / DenseEdgeLength) : FMath::Max(8, (Inner.SegmentsX + 1) / 2);
        const int32 SY = Ring < DenseBands ? FMath::CeilToInt((MaxY - MinY) / DenseEdgeLength) : FMath::Max(8, (Inner.SegmentsY + 1) / 2);
        Plan.Add({MinX, MinY, MaxX, MaxY, SX, SY});
    }
    return Plan;
}
}

// Call after the existing NX*NY near-terrain tiles, before Prepare returns.
// X0/Y0 and NX/NY must describe those exact 320m tiles (16 cells of 20m).
// Out.Bounds intentionally remains ride-only: overview framing must not grow.
// Like Prepare, false on cancellation means discard the unfinished candidate;
// already appended staging chunks are never committed by this helper.
inline bool AppendTerrainBackdrop(FPreparedRide& Out, const coaster::Terrain& Terrain,
    double X0, double Y0, int32 NX, int32 NY, const coaster::Cancel& Cancel)
{
    constexpr int32 MaxChunks = 4096;
    constexpr int64 MaxVertices = 2000000;
    const auto Cancelled = [&] { return Cancel && Cancel(); };
    if (Cancelled()) return false;
    if (!FMath::IsFinite(X0) || !FMath::IsFinite(Y0) || NX < 1 || NY < 1 || NX > MaxChunks || NY > MaxChunks)
    { Out.Error = TEXT("Invalid near-terrain boundary for distant backdrop."); return false; }

    // Exact zipper triangle count: inner+outer segments on each of four sides.
    // Three independent vertices per triangle avoid chunk-boundary index issues;
    // identical edge positions/normals still join continuously without T-junctions.
    const bool DetailedCliffs = Terrain.kind == coaster::TerrainKind::Canyon && Terrain.cliffHeight > 0;
    const int32 ChunkVertices = DetailedCliffs ? TerrainBackdrop::CliffChunkVertices : TerrainBackdrop::DefaultChunkVertices;
    const auto RingPlan = TerrainBackdrop::BuildRingPlan(X0, Y0, NX, NY, DetailedCliffs);
    int64 TriangleCount = 0, ExistingVertices = 0;
    for (int32 Ring = 1; Ring < RingPlan.Num(); ++Ring)
    {
        const auto& Inner = RingPlan[Ring - 1]; const auto& Outer = RingPlan[Ring];
        TriangleCount += 2 * int64(Inner.SegmentsX + Inner.SegmentsY + Outer.SegmentsX + Outer.SegmentsY);
    }
    for (const auto& Existing : Out.Chunks)
    {
        if (Cancelled()) return false;
        ExistingVertices += Existing.Vertices.Num();
    }
    const int64 AddedVertices = TriangleCount * 3;
    const int64 AddedChunks = (AddedVertices + ChunkVertices - 1) / ChunkVertices;
    if (ExistingVertices + AddedVertices > MaxVertices || int64(Out.Chunks.Num()) + AddedChunks > MaxChunks)
    { Out.Error = TEXT("Distant terrain exceeds aggregate render budget; active ride retained."); return false; }

    FChunk Chunk; Chunk.Terrain = true;
    const auto Flush = [&]
    {
        if (!Chunk.Vertices.IsEmpty())
        { Out.Chunks.Add(MoveTemp(Chunk)); Chunk = FChunk{}; Chunk.Terrain = true; }
    };
    const auto Vertex = [&](const FVector2D& XY)
    {
        const double X = XY.X, Y = XY.Y;
        const double Height = Terrain.height(X, Y);
        const double DX = (Terrain.height(X + 1, Y) - Terrain.height(X - 1, Y)) * .5;
        const double DY = (Terrain.height(X, Y + 1) - Terrain.height(X, Y - 1)) * .5;
        // A 1 m derivative cannot shade multi-kilometre distant triangles
        // consistently. Fade cliff normal detail inside the last 160 m of
        // the dense halo; coarse rings then share upward normals. This XY
        // field is identical on both sides of every ring/chunk seam.
        const double Outside = FMath::Max(0., FMath::Max(FMath::Max(X0 - X, X - (X0 + NX * 320.)), FMath::Max(Y0 - Y, Y - (Y0 + NY * 320.))));
        const double NormalDetail = DetailedCliffs ? 1. - coaster::smooth((Outside - 800.) / 160.) : 1.;
        Chunk.Vertices.Add(Position({X, Y, Height}));
        Chunk.Normals.Add(Direction(coaster::unit({-DX * NormalDetail, -DY * NormalDetail, 1})));
        Chunk.UV.Add(FVector2D(X / 80, Y / 80));
    };
    int64 EmittedTriangles = 0;
    const auto Triangle = [&](const FVector2D& A, const FVector2D& B, const FVector2D& C)
    {
        if ((EmittedTriangles & 63) == 0 && Cancelled()) return false;
        if (Chunk.Vertices.Num() + 3 > ChunkVertices) Flush();
        const int32 Base = Chunk.Vertices.Num();
        Vertex(A); Vertex(B); Vertex(C);
        // Core XY triangles are upward CCW. Position reflects Y; UE's clockwise
        // front-face convention therefore needs these original indices unchanged.
        Chunk.Indices.Append({Base, Base + 1, Base + 2});
        ++EmittedTriangles;
        return true;
    };

    for (int32 Ring = 1; Ring < RingPlan.Num(); ++Ring)
    {
        if (Cancelled()) return false;
        const auto& Inner = RingPlan[Ring - 1]; const auto& Outer = RingPlan[Ring];
        for (int32 Side = 0; Side < 4; ++Side)
        {
            const int32 NI = Inner.Segments(Side), NO = Outer.Segments(Side);
            int32 I = 0, O = 0;
            while (I < NI || O < NO)
            {
                // Integer cross-products merge normalized side parameters exactly.
                // Ties advance inner first; the following outer triangle closes
                // that quad. Every fine and coarse boundary edge is used once.
                if (I < NI && (O == NO || int64(I + 1) * NO <= int64(O + 1) * NI))
                {
                    if (!Triangle(Inner.Point(Side, I), Outer.Point(Side, O), Inner.Point(Side, I + 1))) return false;
                    ++I;
                }
                else
                {
                    if (!Triangle(Inner.Point(Side, I), Outer.Point(Side, O), Outer.Point(Side, O + 1))) return false;
                    ++O;
                }
            }
        }
    }
    Flush();
    if (EmittedTriangles != TriangleCount)
    { Out.Error = TEXT("Distant terrain triangle coverage did not match its boundary plan."); return false; }
    return !Cancelled();
}
}
