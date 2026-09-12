#include "CoasterMesh.h"
#include "CoasterTerrainBackdrop.h"
#include "coaster/support_mesh.hpp"
#include "Math/RotationMatrix.h"
#include <array>

namespace VibeMesh
{
namespace
{
constexpr int32 RingSides = 8;
constexpr double ChunkMetres = 80, RailStep = 2;
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
    struct FCircleDirection { double Cos, Sin; };
    static const auto Circle = []
    {
        std::array<FCircleDirection, RingSides> Directions{};
        for (int32 J = 0; J < RingSides; ++J)
        {
            const double Angle = 2 * coaster::pi * J / RingSides;
            Directions[J] = {std::cos(Angle), std::sin(Angle)};
        }
        return Directions;
    }();
    const int32 Rings = Samples.Num();
    const int32 Base = Chunk.Vertices.Num();
    for (int32 I = 0; I < Rings; ++I)
    {
        const double S = FMath::Lerp(Begin, End, double(I) / (Rings - 1));
        const auto& P = Samples[I];
        const auto Centre = P.position + P.right * Side + P.up * Height;
        for (int32 J = 0; J < RingSides; ++J)
        {
            const auto Normal = P.right * Circle[J].Cos + P.up * Circle[J].Sin;
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
TArray<FDriveHardwareRun> OperationHardwareRuns(const coaster::Track& Track, const std::vector<coaster::Operation>& Operations)
{
    using Kind = EDriveHardwareKind;
    const double TrackLength = Track.length;
    double StationApproach = TrackLength;
    // The generator reserves at most the final 100 m for the level station
    // return. Extend friction equipment only across contiguous spans that are
    // actually horizontal and upright. Polynomial coefficient bounds cover the
    // interiors too; a flat pair of endpoints is not sufficient.
    for (auto It = Track.spans.rbegin(); It != Track.spans.rend(); ++It)
    {
        const auto& Span = *It;
        double SpeedLower = coaster::norm(Span.c[1]), VerticalDerivative = std::abs(Span.c[1].z);
        for (size_t I = 2; I < Span.c.size(); ++I)
        { SpeedLower -= I * coaster::norm(Span.c[I]); VerticalDerivative += I * std::abs(Span.c[I].z); }
        double FrameError = coaster::norm(Span.referenceUp[0] - coaster::Vec3{0,0,1});
        for (size_t I = 1; I < Span.referenceUp.size(); ++I) FrameError += coaster::norm(Span.referenceUp[I]);
        for (double Coefficient : Span.bank) FrameError += std::abs(Coefficient);
        // Microradian tolerance accommodates canonical roundoff, not a banked
        // turn or sloping canyon transfer masquerading as the station approach.
        if (SpeedLower <= 0 || VerticalDerivative > 1e-6 * SpeedLower || FrameError > 1e-6) break;
        StationApproach = FMath::Max(Span.start, TrackLength - 100.);
        if (StationApproach <= TrackLength - 100.) break;
    }
    const auto Powered = [](const coaster::Operation& O) { return O.maxForce > 0 && O.maxPower > 0; };
    const auto Motor = [](const coaster::Operation& O) { return O.kind == coaster::DriveKind::Launch || O.kind == coaster::DriveKind::Boost; };
    std::vector<double> Cuts{0, StationApproach, TrackLength};
    for (const auto& O : Operations) if (Powered(O)) { Cuts.push_back(O.start); Cuts.push_back(O.end); }
    std::sort(Cuts.begin(), Cuts.end()); Cuts.erase(std::unique(Cuts.begin(), Cuts.end()), Cuts.end());
    std::vector<bool> Paired(Operations.size(), false);
    for (size_t I = 0; I < Operations.size(); ++I)
    {
        const auto& O = Operations[I];
        if (O.kind == coaster::DriveKind::Brake)
            Paired[I] = std::any_of(Operations.begin(), Operations.end(), [&](const coaster::Operation& Other)
                { return Powered(Other) && Motor(Other) && Other.start == O.start && Other.end == O.end && Other.targetSpeed == O.targetSpeed; });
    }
    TArray<FDriveHardwareRun> Runs;
    // Split at every physical boundary, including the station's wrapping seam.
    // Opposing controllers with the same zone/target describe one stator bank;
    // unlike overlapping targets (terminal transfer + stop) retain both lanes.
    for (Kind K : {Kind::Stator, Kind::Brake, Kind::Station})
        for (size_t I = 1; I < Cuts.size(); ++I)
        {
            const double Begin = Cuts[I - 1], End = Cuts[I], S = (Begin + End) * .5;
            bool HasMotor = false, HasBrake = false, HasStation = false;
            for (size_t J = 0; J < Operations.size(); ++J)
            {
                const auto& O = Operations[J];
                if (!Powered(O) || !(O.start <= O.end ? S >= O.start && S < O.end : S >= O.start || S < O.end)) continue;
                HasMotor |= Motor(O);
                if (O.kind == coaster::DriveKind::Station)
                {
                    // Retain magnetic fins through the high-speed approach;
                    // level pre-seam return and wrapped boarding track get hold equipment.
                    if (O.start > O.end && S >= O.start && S < StationApproach) HasBrake = true;
                    else HasStation = true;
                }
                if (O.kind == coaster::DriveKind::Brake)
                    HasBrake |= !Paired[J];
            }
            const bool Present = K == Kind::Stator ? HasMotor : K == Kind::Station ? HasStation : HasBrake && !HasStation;
            if (!Present) continue;
            if (!Runs.IsEmpty() && Runs.Last().Kind == K && Runs.Last().End == Begin) Runs.Last().End = End;
            else Runs.Add({Begin, End, K});
        }
    return Runs;
}
bool AppendOperationHardware(FPreparedRide& Out, const coaster::Track& Track,
    const std::vector<coaster::Operation>& Operations, const coaster::Cancel& Cancel)
{
    // Original schematic dimensions, not manufacturer CAD. Stators occupy the
    // left in-rail lane, brake/hold equipment the right. Both stay above the tie
    // top (-.11 m), below the runtime chassis (.10 m), and inside the running rails.
    // Motor/fin appearance follows Intamin LSM and InTraSys construction types;
    // the persisted bounded controller remains the sole force authority.
    int64 Vertices = 0;
    for (const auto& Chunk : Out.Chunks) Vertices += Chunk.Vertices.Num();
    for (const auto& Run : OperationHardwareRuns(Track, Operations))
    {
        FChunk Dark, Metal; Dark.Structure = true;
        Dark.Hardware = Metal.Hardware = Run.Kind;
        const auto Flush = [&](FChunk& Chunk)
        {
            if (Chunk.Vertices.IsEmpty()) return;
            const bool Structure = Chunk.Structure;
            Out.Chunks.Add(MoveTemp(Chunk)); Chunk = FChunk{};
            Chunk.Structure = Structure; Chunk.Hardware = Run.Kind;
        };
        const auto Box = [&](FChunk& Chunk, double Begin, double End, double Y0, double Y1, double Z0, double Z1)
        {
            if (Chunk.Vertices.Num() + 24 > 960) Flush(Chunk);
            const coaster::TrackSample Q[]{Track.sample(Begin), Track.sample(End)};
            constexpr int Faces[6][4]{{0,4,6,2},{1,3,7,5},{0,1,5,4},{2,6,7,3},{0,2,3,1},{4,5,7,6}};
            for (int Face = 0; Face < 6; ++Face)
            {
                const int32 Base = Chunk.Vertices.Num();
                for (int J = 0; J < 4; ++J)
                {
                    const int Corner = Faces[Face][J]; const auto& P = Q[Corner & 1];
                    const FVector V = Position(P.position + P.right * (Corner & 2 ? Y1 : Y0) + P.up * (Corner & 4 ? Z1 : Z0));
                    const auto N = Face < 2 ? P.tangent : Face < 4 ? P.right : P.up;
                    Chunk.Vertices.Add(V); Chunk.Normals.Add(Direction(N * (Face % 2 ? 1. : -1.)));
                    Chunk.UV.Add(FVector2D(J == 1 || J == 2, J >= 2)); Out.Bounds += V;
                }
                // Core rider-right is tangent x up: this local frame is LH.
                // Reflecting it into UE makes the RH box table counterclockwise;
                // reverse only these locally-authored faces, not world-space solids.
                EngineTriangle(Chunk, Base, Base + 2, Base + 1); EngineTriangle(Chunk, Base, Base + 3, Base + 2);
            }
            Vertices += 24;
        };
        for (double Begin = Run.Begin; Begin < Run.End; Begin += 3.)
        {
            if (Cancel && Cancel()) return false;
            const double End = FMath::Min(Begin + 3., Run.End), Gap = FMath::Min(.045, (End - Begin) * .1);
            const double A = Begin + Gap, B = End - Gap;
            if (Run.Kind == EDriveHardwareKind::Stator)
            {
                Box(Dark, A, B, -.45, -.11, -.09, -.025); // mounting bed
                Box(Metal, A, B, -.41, -.15, -.025, .035); // laminated stator
                for (double S = A; S < B; S += .6)
                    Box(Dark, S, FMath::Min(S + .045, B), -.41, -.15, .035, .045); // pole segmentation
            }
            else if (Run.Kind == EDriveHardwareKind::Brake)
            {
                Box(Dark, A, B, .12, .44, -.09, -.045);
                Box(Metal, A, B, .266, .294, -.045, .075); // continuous upright braking fin
            }
            else
            {
                Box(Dark, A, B, .12, .44, -.09, -.06);
                const double Mid = (A + B) * .5;
                Box(Metal, A, Mid, .16, .24, -.06, .055); // opposing friction calipers
                Box(Metal, A, Mid, .32, .40, -.06, .055);
                const auto P = Track.sample((Mid + B) * .5);
                const double Radius = FMath::Min(.065, (B - Mid) * .45);
                const auto Wheel = coaster::supportMemberMesh({P.position + P.right * .21,
                    P.position + P.right * .35, Radius, Radius, coaster::SupportMemberKind::Steel, false});
                if (Dark.Vertices.Num() + int32(Wheel.positions.size()) > 960) Flush(Dark);
                const int32 Base = Dark.Vertices.Num();
                for (size_t I = 0; I < Wheel.positions.size(); ++I)
                {
                    const FVector V = Position(Wheel.positions[I]);
                    Dark.Vertices.Add(V); Dark.Normals.Add(Direction(Wheel.normals[I]));
                    Dark.UV.Add(FVector2D::ZeroVector); Out.Bounds += V;
                }
                for (size_t I = 0; I < Wheel.indices.size(); I += 3)
                    EngineTriangle(Dark, Base + Wheel.indices[I], Base + Wheel.indices[I + 1], Base + Wheel.indices[I + 2]);
                Vertices += Wheel.positions.size();
            }
            if (Vertices > MaxVertices || Out.Chunks.Num() + 2 > MaxChunks)
            { Out.Error = TEXT("Operation hardware exceeds the render budget; active ride retained."); return false; }
        }
        Flush(Dark); Flush(Metal);
    }
    return true;
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
    if (!AppendOperationHardware(Out, D.track, D.operations, Cancel)) return false;
    Vertices = 0;
    for (const auto& Chunk : Out.Chunks) Vertices += Chunk.Vertices.Num();

    // Fixed terrain query in SI. Sampling controls visual resolution only; it never
    // changes terrain heights or clearance validation to accommodate track.
    const auto Min = VibeCoordinates::CorePosition({Out.Bounds.Min.X, Out.Bounds.Max.Y, Out.Bounds.Min.Z});
    const auto Max = VibeCoordinates::CorePosition({Out.Bounds.Max.X, Out.Bounds.Min.Y, Out.Bounds.Max.Z});
    // Coarse cliff triangles can self-shadow against their smooth normals.
    // Refine the near surface, preserving the actual heightfield and lighting.
    const bool DetailedCliffs = D.request.terrain.kind == coaster::TerrainKind::Canyon && D.request.terrain.cliffHeight > 0;
    const int32 Cells = DetailedCliffs ? TerrainBackdrop::CliffNearCells : TerrainBackdrop::DefaultNearCells;
    constexpr double TileSize = 320;
    const double Step = TileSize / Cells;
    const double X0 = std::floor((Min.x - 250) / TileSize) * TileSize;
    const double Y0 = std::floor((Min.y - 250) / TileSize) * TileSize;
    const int32 NX = FMath::CeilToInt((Max.x + 250 - X0) / TileSize);
    const int32 NY = FMath::CeilToInt((Max.y + 250 - Y0) / TileSize);
    if (int64(NX) * NY + Out.Chunks.Num() > MaxChunks ||
        Vertices + int64(NX) * NY * (Cells + 1) * (Cells + 1) > MaxVertices ||
        Out.Ties.Num() + Out.Supports.Num() + Out.Station.Num() > MaxInstances)
    { Out.Error = TEXT("Accepted geometry exceeds the prototype mesh budget; active ride retained."); return false; }
    for (int32 TY = 0; TY < NY; ++TY) for (int32 TX = 0; TX < NX; ++TX)
    {
        if (Cancel()) return false;
        FChunk Chunk; Chunk.Terrain = true;
        for (int32 Y = 0; Y <= Cells; ++Y)
        {
            if (Cancel()) return false;
            for (int32 X = 0; X <= Cells; ++X)
            {
                const double PX = X0 + TX * TileSize + X * Step, PY = Y0 + TY * TileSize + Y * Step;
                const double Height = D.request.terrain.height(PX, PY);
                const double DX = (D.request.terrain.height(PX + 1, PY) - D.request.terrain.height(PX - 1, PY)) * .5;
                const double DY = (D.request.terrain.height(PX, PY + 1) - D.request.terrain.height(PX, PY - 1)) * .5;
                Chunk.Vertices.Add(Position({PX, PY, Height}));
                Chunk.Normals.Add(Direction(coaster::unit({-DX, -DY, 1})));
                Chunk.UV.Add(FVector2D(PX / 80, PY / 80));
                if (X < Cells && Y < Cells)
                {
                    const int32 A = Y * (Cells + 1) + X, B = A + 1, C = A + Cells + 1, E = C + 1;
                    EngineTriangle(Chunk, A, B, C);
                    EngineTriangle(Chunk, C, B, E);
                }
            }
        }
        Out.Chunks.Add(MoveTemp(Chunk));
    }
    return AppendTerrainBackdrop(Out, D.request.terrain, X0, Y0, NX, NY, Cancel);
}
}


