#include "CoasterMesh.h"
#include "coaster/support_mesh.hpp"
#include "Math/RotationMatrix.h"

namespace VibeMesh
{
namespace
{
constexpr int32 RingSides = 8;
constexpr double ChunkMetres = 80, RailStep = 2;
constexpr int32 MaxChunks = 4096, MaxVertices = 2000000, MaxInstances = 100000;
void EngineTriangle(FChunk& Chunk, int32 A, int32 B, int32 C)
{
    // UE's GenerateBoxMesh uses clockwise front faces: Cross(B-A, C-A) has
    // negative dot with the outward shading normal. Reflecting core Y already
    // converts its outward-CCW triangles to that convention. Keep their indices;
    // an additional swap exposes backfaces and flips two-sided shading normals.
    Chunk.Indices.Append({A, B, C});
}
void Tube(FChunk& Chunk, TArrayView<const coaster::TrackSample> Samples, double Begin, double End, double Side, double Height, double Radius)
{
    const int32 Rings = Samples.Num();
    const int32 Base = Chunk.Vertices.Num();
    for (int32 I = 0; I < Rings; ++I)
    {
        const double S = FMath::Lerp(Begin, End, double(I) / (Rings - 1));
        const auto& P = Samples[I];
        const auto Centre = P.position + P.right * Side + P.up * Height;
        for (int32 J = 0; J < RingSides; ++J)
        {
            const double Angle = 2 * coaster::pi * J / RingSides;
            const auto Normal = P.right * std::cos(Angle) + P.up * std::sin(Angle);
            Chunk.Vertices.Add(Position(Centre + Normal * Radius));
            Chunk.Normals.Add(Direction(Normal));
            Chunk.UV.Add(FVector2D(S / 4, double(J) / RingSides));
            if (I + 1 < Rings)
            {
                const int32 A = Base + I * RingSides + J;
                const int32 B = A + RingSides;
                const int32 C = Base + I * RingSides + (J + 1) % RingSides;
                const int32 D = C + RingSides;
                EngineTriangle(Chunk, A, B, C);
                EngineTriangle(Chunk, C, B, D);
            }
        }
    }
}
}
bool AppendStationBoxInstances(FPreparedRide& Out, const coaster::StationBox& Box,
    int32 SourceBoxIndex, coaster::Vec3 StationMidline, const coaster::Cancel& Cancel)
{
    if (Cancel && Cancel()) return false;
    const auto B = VibeCoordinates::StationBox(Box);
    const FVector F(B.Forward.X, B.Forward.Y, B.Forward.Z), U(B.Up.X, B.Up.Y, B.Up.Z);
    const FQuat Frame = FRotationMatrix::MakeFromXZ(F, U).ToQuat();
    const bool Concrete = Box.role == coaster::StationRole::Platform || Box.role == coaster::StationRole::Footing;
    const auto CubeKind = Concrete ? EStationInstanceKind::CubeConcrete : EStationInstanceKind::CubeSteel;
    // A quaternion cannot represent skew or scale in the saved axes. Details are
    // enabled only for the canonical orthonormal station frame and exact sizes.
    const bool Rigid = std::abs(coaster::norm(Box.forward) - 1) < 1e-9 &&
        std::abs(coaster::norm(Box.up) - 1) < 1e-9 &&
        coaster::norm(coaster::cross(Box.forward, Box.up) - Box.right) < 1e-9 &&
        std::abs(coaster::dot(Box.forward, Box.up)) < 1e-9;
    const double Side = coaster::dot(Box.center - StationMidline, Box.right);
    const bool Platform = Rigid && Box.role == coaster::StationRole::Platform &&
        std::abs(Box.half.y - 1.925) < 1e-9 && std::abs(Box.half.z - .4) < 1e-9 &&
        std::abs(Side) > Box.half.y;
    const bool Roof = Rigid && Box.role == coaster::StationRole::Canopy &&
        std::abs(Box.half.y - 5.65) < 1e-9 && std::abs(Box.half.z - .18) < 1e-9;
    const bool Post = Rigid && Box.role == coaster::StationRole::Post &&
        std::abs(Box.half.x - .2) < 1e-9 && std::abs(Box.half.y - .2) < 1e-9 &&
        std::abs(Box.half.z - 2.61) < 1e-9;
    const auto Add = [&](EStationInstanceKind Kind, coaster::Vec3 Centre, FVector Scale, FQuat Rotation)
    {
        if (Cancel && Cancel()) return false;
        if (Out.Station.Num() >= MaxInstances)
        { Out.Error = TEXT("Station detail exceeds the instance budget; active ride retained."); return false; }
        Out.Station.Add({FTransform(Rotation, Position(Centre), Scale), Kind, SourceBoxIndex});
        return true;
    };
    if (Platform || Roof)
    {
        double Begin = -Box.half.x, Remaining = 2 * Box.half.x;
        // Source platform stripe is at -Y, hence imported local +Y. On the
        // positive rider-right side, rotate the symmetric panel by 180 degrees
        // so that stripe always faces the actual track midline. No negative scale.
        const FQuat ModuleFrame = Platform && Side > 0 ? Frame * FQuat(0, 0, 1, 0) : Frame;
        const auto Tile = [&](double Length, EStationInstanceKind Kind)
        {
            auto Centre = Box.center + Box.forward * (Begin + Length * .5);
            if (Platform) Centre = Centre + Box.up * Box.half.z; // top-centre pivot
            if (!Add(Kind, Centre, FVector::OneVector, ModuleFrame)) return false;
            Begin += Length; Remaining -= Length;
            return true;
        };
        while (Remaining >= 3.)
            if (!Tile(3., Platform ? EStationInstanceKind::Platform : EStationInstanceKind::Roof)) return false;
        if (Platform)
            while (Remaining >= 1.)
                if (!Tile(1., EStationInstanceKind::PlatformEnd)) return false;
        // A nonstandard remainder is an exact conservative cube segment, not a
        // stretched slab/rib/fastener. A sub-nanometre remainder is only a seam.
        if (Remaining > 1e-9 && !Add(CubeKind, Box.center + Box.forward * (Begin + Remaining * .5),
            FVector(Remaining, 2 * Box.half.y, 2 * Box.half.z), Frame)) return false;
    }
    else if (Post)
    {
        if (!Add(EStationInstanceKind::Post, Box.center - Box.up * Box.half.z, FVector::OneVector, Frame)) return false;
    }
    else if (!Add(CubeKind, Box.center, FVector(B.Scale.X, B.Scale.Y, B.Scale.Z), Frame)) return false;
    // Bounds are the original certified box; decorative holes do not enlarge or
    // shrink the structure/terrain domain used elsewhere in preparation.
    for (double X : {-1., 1.}) for (double Y : {-1., 1.}) for (double Z : {-1., 1.})
        Out.Bounds += Position(Box.center + Box.forward * (X * Box.half.x) + Box.right * (Y * Box.half.y) + Box.up * (Z * Box.half.z));
    return true;
}
bool Prepare(FPreparedRide& Out, const coaster::Cancel& Cancel)
{
    if (!Out.Design || !Out.Design->accepted()) { Out.Error = TEXT("Rejected design cannot be rendered as a ride."); return false; }
    const auto& D = *Out.Design;
    if (!std::isfinite(D.track.length) || D.track.length <= 0 || D.track.length > 150000)
    { Out.Error = TEXT("Track exceeds the prototype rendering budget."); return false; }
    int64 Vertices = 0;
    for (double Begin = 0; Begin < D.track.length; Begin += ChunkMetres)
    {
        if (Cancel()) return false;
        FChunk Chunk;
        const double End = FMath::Min(Begin + ChunkMetres, D.track.length);
        const int32 Rings = FMath::CeilToInt((End - Begin) / RailStep) + 1;
        TArray<coaster::TrackSample, TInlineAllocator<41>> Samples;
        Samples.Reserve(Rings);
        for (int32 I = 0; I < Rings; ++I)
            Samples.Add(D.track.sample(FMath::Lerp(Begin, End, double(I) / (Rings - 1))));
        // All three tubes share the exact same sampled centreline/frame. Keep
        // tube/vertex/index order unchanged while avoiding three geometry queries.
        Chunk.Vertices.Reserve(3 * Rings * RingSides);
        Chunk.Normals.Reserve(3 * Rings * RingSides);
        Chunk.UV.Reserve(3 * Rings * RingSides);
        Chunk.Indices.Reserve(3 * (Rings - 1) * RingSides * 6);
        Tube(Chunk, MakeArrayView(Samples), Begin, End, -.65, 0, .085);
        Tube(Chunk, MakeArrayView(Samples), Begin, End, .65, 0, .085);
        Tube(Chunk, MakeArrayView(Samples), Begin, End, 0, -coaster::spineDepth, coaster::spineRadius);
        for (const FVector& P : Chunk.Vertices) Out.Bounds += P;
        Vertices += Chunk.Vertices.Num();
        Out.Chunks.Add(MoveTemp(Chunk));
    }
    for (double S = 0; S < D.track.length; S += 3)
    {
        if (Cancel()) return false;
        const auto P = D.track.sample(S);
        Out.Ties.Add(FTransform(Rotation(P), Position(P.position - P.up * .19), FVector::OneVector));
    }
    const auto AddMember = [&Out](coaster::Vec3 From, coaster::Vec3 To)
    {
        const auto Member = VibeCoordinates::SteelMember(From, To);
        const FVector Axis(Member.Axis.X, Member.Axis.Y, Member.Axis.Z);
        const FVector Centre(Member.Centre.X, Member.Centre.Y, Member.Centre.Z);
        // Engine cylinder: 100 cm tall, 100 cm diameter. Canonical member radius
        // is 0.18 m, matching the core's member clearance envelope.
        Out.Supports.Add(FTransform(FRotationMatrix::MakeFromZ(Axis).ToQuat(), Centre, FVector(2 * coaster::supportRadius, 2 * coaster::supportRadius, Member.LengthCm / 100)));
    };
    FChunk Steel, Footings; Steel.Structure = true; Footings.Footing = true;
    const auto Flush = [&Out](FChunk& Chunk)
    {
        if (Chunk.Vertices.Num()) { const bool Structure = Chunk.Structure, Footing = Chunk.Footing;
            Out.Chunks.Add(MoveTemp(Chunk)); Chunk = FChunk{}; Chunk.Structure = Structure; Chunk.Footing = Footing; }
    };
    int64 MemberCount = 0;
    for (const auto& Support : D.supports)
    {
        if (Cancel()) return false;
        if (Support.members.empty())
        {
            AddMember(Support.base, Support.top);
            if (Support.hasAttachment) AddMember(Support.top, Support.attachment);
            continue;
        }
        for (const auto& Member : Support.members)
        {
            if (Cancel()) return false;
            if (++MemberCount > int64(coaster::maxTotalSupportMembers))
            { Out.Error = TEXT("Canonical support member budget exceeded; active ride retained."); return false; }
            const auto Mesh = coaster::supportMemberMesh(Member);
            FChunk& Chunk = Member.kind == coaster::SupportMemberKind::Footing ? Footings : Steel;
            if (Chunk.Vertices.Num() + int32(Mesh.positions.size()) > 984) Flush(Chunk);
            const int32 Base = Chunk.Vertices.Num();
            for (size_t I = 0; I < Mesh.positions.size(); ++I)
            {
                const FVector P = Position(Mesh.positions[I]);
                Chunk.Vertices.Add(P); Chunk.Normals.Add(Direction(Mesh.normals[I]));
                Chunk.UV.Add(FVector2D(0, 0)); Out.Bounds += P;
            }
            for (size_t I = 0; I < Mesh.indices.size(); I += 3)
                EngineTriangle(Chunk, Base + Mesh.indices[I], Base + Mesh.indices[I+1], Base + Mesh.indices[I+2]);
            Vertices += Mesh.positions.size();
            if (Vertices > MaxVertices || Out.Chunks.Num() > MaxChunks)
            { Out.Error = TEXT("Canonical support mesh exceeds the prototype budget; active ride retained."); return false; }
        }
    }
    Flush(Steel); Flush(Footings);
    const auto StationMidline = D.track.sample(0).position;
    for (size_t I = 0; I < D.station.boxes.size(); ++I)
        if (!AppendStationBoxInstances(Out, D.station.boxes[I], int32(I), StationMidline, Cancel)) return false;

    return AppendGround(Out, Cancel);
}
bool AppendGround(FPreparedRide& Out, const coaster::Cancel& Cancel)
{
    if (Cancel && Cancel()) return false;
    if (!Out.Bounds.IsValid || Out.Bounds.Min.ContainsNaN() || Out.Bounds.Max.ContainsNaN())
    { Out.Error = TEXT("Ground requires finite ride bounds."); return false; }
    int64 Vertices = 4;
    for (const auto& Chunk : Out.Chunks) Vertices += Chunk.Vertices.Num();
    if (Out.Chunks.Num() + 1 > MaxChunks || Vertices > MaxVertices ||
        Out.Ties.Num() + Out.Supports.Num() + Out.Station.Num() > MaxInstances)
    { Out.Error = TEXT("Accepted geometry exceeds the mesh budget; active ride retained."); return false; }
    // A single plane covers the ride and horizon; overview bounds stay ride-only.
    constexpr double MarginCm = 5000000.;
    const double X0 = Out.Bounds.Min.X - MarginCm, X1 = Out.Bounds.Max.X + MarginCm;
    const double Y0 = Out.Bounds.Min.Y - MarginCm, Y1 = Out.Bounds.Max.Y + MarginCm;
    FChunk Ground; Ground.Ground = true;
    Ground.Vertices = {{X0,Y0,0},{X0,Y1,0},{X1,Y1,0},{X1,Y0,0}};
    for (const FVector& Vertex : Ground.Vertices)
    {
        Ground.Normals.Add(FVector::UpVector);
        Ground.UV.Add(FVector2D(Vertex.X / 8000., -Vertex.Y / 8000.));
    }
    EngineTriangle(Ground, 0, 1, 2); EngineTriangle(Ground, 0, 2, 3);
    Out.Chunks.Add(MoveTemp(Ground));
    return true;
}
}
