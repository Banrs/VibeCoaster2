#include "coaster/operation_hardware.hpp"
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
void Tube(FChunk& Chunk, TArrayView<const coaster::TrackSample> Samples, double Begin, double End, double Side, double Height, double Radius, bool Keel = false)
{
    // The centre spine is a shallow, faceted aero keel. Its profile stays
    // inside the original 0.16 m circular envelope, with the same top web
    // attachment and bottom support-contact points as the canonical tube.
    constexpr double KeelY[RingSides] = {.84, .69, 0, -.69, -.84, -.56, 0, .56};
    constexpr double KeelZ[RingSides] = {0, .69, 1, .69, 0, -.69, -1, -.69};
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
            const auto Offset = P.right * (Keel ? KeelY[J] : std::cos(Angle)) +
                P.up * (Keel ? KeelZ[J] : std::sin(Angle));
            Chunk.Vertices.Add(Position(Centre + Offset * Radius));
            Chunk.Normals.Add(Direction(coaster::unit(Offset)));
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
    const bool Concrete = Box.role == coaster::StationRole::Platform || Box.role == coaster::StationRole::Footing ||
        Box.role == coaster::StationRole::QueueDeck || Box.role == coaster::StationRole::MergeDeck ||
        Box.role == coaster::StationRole::HoldingLane || Box.role == coaster::StationRole::UnloadDeck ||
        Box.role == coaster::StationRole::ExitWalkway || Box.role == coaster::StationRole::Stair ||
        Box.role == coaster::StationRole::Underpass;
    const auto CubeKind = Concrete ? EStationInstanceKind::CubeConcrete : EStationInstanceKind::CubeSteel;
    // A quaternion cannot represent skew or scale in the saved axes. Details are
    // enabled only for the canonical orthonormal station frame and exact sizes.
    const bool Rigid = std::abs(coaster::norm(Box.forward) - 1) < 1e-9 &&
        std::abs(coaster::norm(Box.up) - 1) < 1e-9 &&
        coaster::norm(coaster::cross(Box.forward, Box.up) - Box.right) < 1e-9 &&
        std::abs(coaster::dot(Box.forward, Box.up)) < 1e-9;
    const double Side = coaster::dot(Box.center - StationMidline, Box.right);
    EStationInstanceKind FunctionalKind = EStationInstanceKind::CubeSteel;
    bool Functional = true;
    switch (Box.role)
    {
    case coaster::StationRole::QueueDeck: FunctionalKind = EStationInstanceKind::QueueDeck; break;
    case coaster::StationRole::RouteRoof: FunctionalKind = EStationInstanceKind::RouteRoof; break;
    case coaster::StationRole::MergeDeck: FunctionalKind = EStationInstanceKind::MergeDeck; break;
    case coaster::StationRole::HoldingLane: FunctionalKind = EStationInstanceKind::HoldingLane; break;
    case coaster::StationRole::BoardingGate: FunctionalKind = EStationInstanceKind::BoardingGate; break;
    case coaster::StationRole::DispatchCabin: FunctionalKind = EStationInstanceKind::DispatchCabin; break;
    case coaster::StationRole::UnloadDeck: FunctionalKind = EStationInstanceKind::UnloadDeck; break;
    case coaster::StationRole::ExitWalkway: FunctionalKind = EStationInstanceKind::ExitWalkway; break;
    case coaster::StationRole::Lift: FunctionalKind = EStationInstanceKind::Lift; break;
    case coaster::StationRole::Stair: FunctionalKind = EStationInstanceKind::Stair; break;
    case coaster::StationRole::Underpass: FunctionalKind = EStationInstanceKind::Underpass; break;
    case coaster::StationRole::QueueRail: FunctionalKind = EStationInstanceKind::QueueRail; break;
    default: Functional = false; break;
    }
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
    if (Functional && Rigid)
    {
        // Functional assets are authored in a centred [-1,1] metre envelope.
        // Scaling by the canonical half-extents keeps every piece in its
        // clearance-validated source box, including variable station lengths.
        // The exit stair climbs toward the rail on the negative station side.
        const FQuat DetailFrame = Box.role == coaster::StationRole::Stair && Side < 0
            ? Frame * FQuat(0, 0, 1, 0) : Frame;
        if (!Add(FunctionalKind, Box.center,
            FVector(Box.half.x, Box.half.y, Box.half.z), DetailFrame)) return false;
    }
    else if (Platform || Roof)
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
        Tube(Chunk, MakeArrayView(Samples), Begin, End, 0, -coaster::spineDepth, coaster::spineRadius, true);
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
    for (const auto& Hardware : coaster::buildOperationHardware(D.track, D.operations))
    {
        if (Cancel()) return false;
        const auto& Box = Hardware.box;
        const auto Frame = FRotationMatrix::MakeFromXZ(Direction(Box.forward), Direction(Box.up)).ToQuat();
        auto& Instances = Hardware.powered ? Out.LSMHardware : Out.BrakeHardware;
        FTransform Transform(Frame, Position(Box.center), FVector(2*Box.half.x, 2*Box.half.y, 2*Box.half.z));
        if (Hardware.movingFin)
        {
            const auto Travel = Position(Box.up*.26);
            Transform.AddToTranslation(-Travel);
            Out.TrimFins.Add({Hardware.operation, Instances.Num(), Transform, Travel});
        }
        Instances.Add(Transform);
        for (double X : {-1.,1.}) for (double Y : {-1.,1.}) for (double Z : {-1.,1.})
            Out.Bounds += Position(Box.center + Box.forward*(X*Box.half.x) + Box.right*(Y*Box.half.y) + Box.up*(Z*Box.half.z));
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
    const coaster::Terrain Flat;
    const auto& Terrain = Out.Design ? Out.Design->request.terrain : Flat;
    if (!Terrain.valid()) { Out.Error = TEXT("Ground requires a valid resolved terrain profile."); return false; }
    const bool Highlands = Terrain.kind == coaster::TerrainKind::Highlands;
    int64 ExistingVertices = 0;
    for (const auto& Chunk : Out.Chunks) ExistingVertices += Chunk.Vertices.Num();
    if (Out.Chunks.Num() + 1 > MaxChunks || ExistingVertices + 4 > MaxVertices ||
        Out.Ties.Num() + Out.Supports.Num() + Out.Station.Num() + Out.LSMHardware.Num() + Out.BrakeHardware.Num() > MaxInstances)
    { Out.Error = TEXT("Accepted geometry exceeds the mesh budget; active ride retained."); return false; }
    if (Highlands)
    {
        std::array<double,4> Patch{double(Out.Bounds.Min.X)/100,-double(Out.Bounds.Max.Y)/100,
            double(Out.Bounds.Max.X)/100,-double(Out.Bounds.Min.Y)/100};
        if (Out.Design)
        {
            const auto Enclose=[&](coaster::Vec3 P,double Radius)
            {
                Patch[0]=std::min(Patch[0],P.x-Radius);Patch[1]=std::min(Patch[1],P.y-Radius);
                Patch[2]=std::max(Patch[2],P.x+Radius);Patch[3]=std::max(Patch[3],P.y+Radius);
            };
            // Bernstein controls enclose the complete position polynomials,
            // not merely the sampled rail mesh. The existing body radius is a
            // rotationally invariant enclosure of every occupied cross-section.
            const double Radius = coaster::occupiedRadius(Out.Design->request.train);
            for (const auto& Span : Out.Design->track.spans)
            {
                if (Cancel && Cancel()) return false;
                for (int32 K=0; K<10; ++K)
                {
                    coaster::Vec3 P{}; double Weight=1;
                    for (int32 J=0; J<=K; ++J)
                    {
                        if (J>0) Weight *= double(K-J+1)/double(10-J);
                        P=P+Span.c[J]*Weight;
                    }
                    Enclose(P,Radius);
                }
            }
            // Source solids also cover legacy instanced supports and the
            // radial extrema that an inscribed polygon mesh need not contain.
            for(const auto& Support:Out.Design->supports)
            {
                if(Cancel&&Cancel())return false;
                if(Support.members.empty())
                {
                    Enclose(Support.base,coaster::supportRadius);Enclose(Support.top,coaster::supportRadius);
                    if(Support.hasAttachment)Enclose(Support.attachment,coaster::supportRadius);
                }
                for(const auto& Member:Support.members){Enclose(Member.base,Member.radiusBase);Enclose(Member.top,Member.radiusTop);}
            }
            for(const auto& Box:Out.Design->station.boxes)
            {
                if(Cancel&&Cancel())return false;
                for(double X:{-1.,1.})for(double Y:{-1.,1.})for(double Z:{-1.,1.})
                    Enclose(Box.center+Box.forward*(X*Box.half.x)+Box.right*(Y*Box.half.y)+Box.up*(Z*Box.half.z),0);
            }
        }
        // One surrounding source cell is mesh coverage, not extra clearance.
        for (int32 Axis=0; Axis<2; ++Axis)
        {
            Patch[Axis]=std::floor(Patch[Axis]/coaster::Terrain::gridStep)*coaster::Terrain::gridStep-coaster::Terrain::gridStep;
            Patch[Axis+2]=std::ceil(Patch[Axis+2]/coaster::Terrain::gridStep)*coaster::Terrain::gridStep+coaster::Terrain::gridStep;
        }
        const double CellsX=(Patch[2]-Patch[0])/coaster::Terrain::gridStep, CellsY=(Patch[3]-Patch[1])/coaster::Terrain::gridStep;
        if (!std::isfinite(CellsX)||!std::isfinite(CellsY)||CellsX<1||CellsY<1||
            CellsX>MaxVertices||CellsY>MaxVertices||(CellsX+1)*(CellsY+1)+ExistingVertices>MaxVertices)
        { Out.Error=TEXT("Exact ride terrain exceeds the mesh budget; active ride retained."); return false; }
        const int32 NX=int32(std::llround(CellsX)), NY=int32(std::llround(CellsY));
        int64 Vertices=ExistingVertices+int64(NX+1)*(NY+1);
        struct FRing {std::array<double,4> Bounds;double Step;int32 NX,NY;};
        std::vector<FRing> Rings;
        const auto Landform=Terrain.gridBounds();
        constexpr double Horizon=50000;
        const std::array<double,4> Extent{std::min(Patch[0],Landform[0])-Horizon,std::min(Patch[1],Landform[1])-Horizon,
            std::max(Patch[2],Landform[2])+Horizon,std::max(Patch[3],Landform[3])+Horizon};
        auto Outer=Patch;
        for (double Step=2*coaster::Terrain::gridStep;
             Outer[0]>Extent[0]||Outer[1]>Extent[1]||Outer[2]<Extent[2]||Outer[3]<Extent[3];Step*=2)
        {
            for (int32 Axis=0;Axis<2;++Axis)
            {
                Outer[Axis]=std::floor((Outer[Axis]-4*Step)/Step)*Step;
                Outer[Axis+2]=std::ceil((Outer[Axis+2]+4*Step)/Step)*Step;
            }
            const int32 RX=int32(std::llround((Outer[2]-Outer[0])/Step)), RY=int32(std::llround((Outer[3]-Outer[1])/Step));
            Vertices+=2*int64(RX+RY);
            if (Vertices>MaxVertices) {Out.Error=TEXT("Surrounding terrain exceeds the mesh budget; active ride retained.");return false;}
            Rings.push_back({Outer,Step,RX,RY});
        }
        FChunk Ground; Ground.Ground = true;
        Ground.Vertices.Reserve(int32(Vertices-ExistingVertices));
        Ground.Normals.Reserve(int32(Vertices-ExistingVertices));
        Ground.UV.Reserve(int32(Vertices-ExistingVertices));
        auto Vertex = [&](double X, double Y)
        {
            // Every ring node is aligned to the same source grid. Coarser
            // distant faces approximate only between these shared-field nodes.
            const FVector P = Position({X,Y,Terrain.vertexHeight(X,Y)});
            const int32 Index=Ground.Vertices.Add(P);
            const double H=coaster::Terrain::gridStep;
            const double DX=(Terrain.vertexHeight(X+H,Y)-Terrain.vertexHeight(X-H,Y))/(2*H);
            const double DY=(Terrain.vertexHeight(X,Y+H)-Terrain.vertexHeight(X,Y-H))/(2*H);
            Ground.Normals.Add(Direction(coaster::unit({-DX,-DY,1})));
            Ground.UV.Add(FVector2D(P.X/8000.,-P.Y/8000.));
            return Index;
        };
        for (int32 Y=0; Y<=NY; ++Y)
        {
            if (Cancel && Cancel()) return false;
            for (int32 X=0; X<=NX; ++X) Vertex(Patch[0]+X*coaster::Terrain::gridStep,Patch[1]+Y*coaster::Terrain::gridStep);
        }
        for (int32 Y=0; Y<NY; ++Y) for (int32 X=0; X<NX; ++X)
        {
            const int32 A=Y*(NX+1)+X, B=A+1, D=A+NX+1, C=D+1;
            EngineTriangle(Ground,A,B,C); EngineTriangle(Ground,A,C,D);
        }
        std::array<TArray<int32>,4> Inner;
        for(int32 X=0;X<=NX;++X)Inner[0].Add(X);
        for(int32 Y=0;Y<=NY;++Y)Inner[1].Add(Y*(NX+1)+NX);
        for(int32 X=NX;X>=0;--X)Inner[2].Add(NY*(NX+1)+X);
        for(int32 Y=NY;Y>=0;--Y)Inner[3].Add(Y*(NX+1));
        for(const auto& Ring:Rings)
        {
            if(Cancel&&Cancel())return false;
            const auto& B=Ring.Bounds;
            const std::array<int32,4> Corners{Vertex(B[0],B[1]),Vertex(B[2],B[1]),Vertex(B[2],B[3]),Vertex(B[0],B[3])};
            std::array<TArray<int32>,4> Next;
            for(int32 Side=0;Side<4;++Side)
            {
                const int32 Segments=Side%2==0?Ring.NX:Ring.NY;
                Next[Side].Add(Corners[Side]);
                for(int32 I=1;I<Segments;++I)
                {
                    if(Side==0)Next[Side].Add(Vertex(B[0]+I*Ring.Step,B[1]));
                    else if(Side==1)Next[Side].Add(Vertex(B[2],B[1]+I*Ring.Step));
                    else if(Side==2)Next[Side].Add(Vertex(B[2]-I*Ring.Step,B[3]));
                    else Next[Side].Add(Vertex(B[0],B[3]-I*Ring.Step));
                }
                Next[Side].Add(Corners[(Side+1)%4]);
                // Zip both edge partitions. Every fine and coarse edge is
                // shared by indices, including corners; there are no T-junctions.
                int32 I=0,J=0;
                const int32 NI=Inner[Side].Num()-1,NJ=Next[Side].Num()-1;
                while(I<NI||J<NJ)
                {
                    if(I<NI&&(J==NJ||int64(I+1)*NJ<int64(J+1)*NI))
                    {EngineTriangle(Ground,Inner[Side][I],Next[Side][J],Inner[Side][I+1]);++I;}
                    else if(J<NJ&&(I==NI||int64(J+1)*NI<int64(I+1)*NJ))
                    {EngineTriangle(Ground,Inner[Side][I],Next[Side][J],Next[Side][J+1]);++J;}
                    else
                    {
                        EngineTriangle(Ground,Inner[Side][I],Next[Side][J],Next[Side][J+1]);
                        EngineTriangle(Ground,Inner[Side][I],Next[Side][J+1],Inner[Side][I+1]);++I;++J;
                    }
                }
            }
            Inner=MoveTemp(Next);
        }
        Out.GroundExactBounds=Patch;
        Out.Chunks.Add(MoveTemp(Ground));
        return true;
    }
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
    Out.GroundExactBounds={X0/100,-Y1/100,X1/100,-Y0/100};
    Out.Chunks.Add(MoveTemp(Ground));
    return true;
}
}
