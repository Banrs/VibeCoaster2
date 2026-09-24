#include "CoasterMesh.h"
#include <map>
#include <set>
#include <array>
#include <algorithm>
#include "coaster/support_mesh.hpp"
#include "coaster/track_hardware.hpp"
#include "Misc/AutomationTest.h"
#include "KismetProceduralMeshLibrary.h"
#include "ProceduralMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
// Independent rail/spine reference samples the track without production
// tube, shared-sample or coordinate helpers.
bool RailChunkMatchesUnbatchedReference(const VibeMesh::FChunk& Chunk,
    const coaster::Track& Track, int32 RailChunkIndex)
{
    const double Begin = 80. * RailChunkIndex, End = FMath::Min(Begin + 80., Track.length);
    const int32 Rings = FMath::CeilToInt((End - Begin) / 2.) + 1;
    if (Rings < 2 || Chunk.Vertices.Num() != 3 * Rings * 8 ||
        Chunk.Normals.Num() != Chunk.Vertices.Num() || Chunk.UV.Num() != Chunk.Vertices.Num()) return false;
    for (int32 TubeIndex = 0; TubeIndex < 3; ++TubeIndex)
    {
        const double Side = TubeIndex == 0 ? -.65 : TubeIndex == 1 ? .65 : 0.;
        const double Height = TubeIndex == 2 ? -coaster::spineDepth : 0.;
        const double Radius = TubeIndex == 2 ? coaster::spineRadius : .085;
        for (int32 Ring = 0; Ring < Rings; ++Ring)
        {
            const double S = FMath::Lerp(Begin, End, double(Ring) / (Rings - 1));
            const auto P = Track.sample(S);
            for (int32 SideIndex = 0; SideIndex < 8; ++SideIndex)
            {
                constexpr double KeelY[8] = {.84, .69, 0, -.69, -.84, -.56, 0, .56};
                constexpr double KeelZ[8] = {0, .69, 1, .69, 0, -.69, -1, -.69};
                const double Angle = 2 * coaster::pi * SideIndex / 8;
                const auto Offset = TubeIndex == 2
                    ? P.right * KeelY[SideIndex] + P.up * KeelZ[SideIndex]
                    : P.right * std::cos(Angle) + P.up * std::sin(Angle);
                const auto N = coaster::unit(Offset);
                const auto V = P.position + P.right * Side + P.up * Height + Offset * Radius;
                const int32 Index = (TubeIndex * Rings + Ring) * 8 + SideIndex;
                if (!Chunk.Vertices[Index].Equals(FVector(V.x * 100., -V.y * 100., V.z * 100.), 1e-8) ||
                    !Chunk.Normals[Index].Equals(FVector(N.x, -N.y, N.z), 1e-12) ||
                    !Chunk.UV[Index].Equals(FVector2D(S / 4., double(SideIndex) / 8.), 1e-12)) return false;
            }
        }
    }
    return true;
}

// Independent geometric check, calibrated below against Epic's box generator.
// Tiny/collinear triangles have no facing; every other triangle must face the
// same way as its supplied shading normals under UE's clockwise convention.
bool HasEngineFrontFaces(const TArray<FVector>& Vertices, const TArray<FVector>& Normals,
    const TArray<int32>& Indices, int64& Checked)
{
    if (Normals.Num() != Vertices.Num() || Indices.Num() % 3 != 0) return false;
    bool Valid = true;
    for (int32 I = 0; I < Indices.Num(); I += 3)
    {
        const int32 A = Indices[I], B = Indices[I + 1], C = Indices[I + 2];
        if (!Vertices.IsValidIndex(A) || !Vertices.IsValidIndex(B) || !Vertices.IsValidIndex(C)) return false;
        const FVector AB = Vertices[B] - Vertices[A], AC = Vertices[C] - Vertices[A];
        const FVector Geometric = FVector::CrossProduct(AB, AC);
        const FVector Shading = Normals[A] + Normals[B] + Normals[C];
        if (Geometric.ContainsNaN() || Shading.ContainsNaN()) { Valid = false; continue; }
        const double EdgeScale = AB.SizeSquared() * AC.SizeSquared();
        if (EdgeScale == 0 || Geometric.SizeSquared() <= 1e-20 * EdgeScale) continue;
        ++Checked;
        const double Scale = Geometric.Size() * Shading.Size();
        Valid &= Scale > 0 && FVector::DotProduct(Geometric, Shading) < -1e-8 * Scale;
    }
    return Valid;
}
// Independent declared asset envelopes, in imported centimetres. This checks the
// adapter's partitions/pivots; actual imported vertex bounds are checked in UE Python.
bool StationRepresentationMatches(const VibeMesh::FPreparedRide& Prepared,
    const std::vector<coaster::StationBox>& Boxes, coaster::Vec3 Midline)
{
    using Kind = VibeMesh::EStationInstanceKind;
    std::vector<std::vector<std::pair<double, double>>> Intervals(Boxes.size());
    for (const auto& Instance : Prepared.Station)
    {
        if (Instance.SourceBoxIndex < 0 || size_t(Instance.SourceBoxIndex) >= Boxes.size()) return false;
        const auto& Box = Boxes[Instance.SourceBoxIndex];
        FVector Min(-50), Max(50);
        const bool Cube = Instance.Kind == Kind::CubeSteel || Instance.Kind == Kind::CubeConcrete;
        const bool ConcreteRole = Box.role == coaster::StationRole::Platform || Box.role == coaster::StationRole::Footing ||
            Box.role == coaster::StationRole::QueueDeck || Box.role == coaster::StationRole::MergeDeck ||
            Box.role == coaster::StationRole::HoldingLane || Box.role == coaster::StationRole::UnloadDeck ||
            Box.role == coaster::StationRole::ExitWalkway || Box.role == coaster::StationRole::Stair ||
            Box.role == coaster::StationRole::Underpass;
        if (Cube)
        {
            if ((Instance.Kind == Kind::CubeConcrete) != ConcreteRole) return false;
        }
        else
        {
            Kind Expected = Kind::CubeSteel;
            bool Functional = true;
            switch (Box.role)
            {
            case coaster::StationRole::QueueDeck: Expected = Kind::QueueDeck; break;
            case coaster::StationRole::RouteRoof: Expected = Kind::RouteRoof; break;
            case coaster::StationRole::MergeDeck: Expected = Kind::MergeDeck; break;
            case coaster::StationRole::HoldingLane: Expected = Kind::HoldingLane; break;
            case coaster::StationRole::BoardingGate: Expected = Kind::BoardingGate; break;
            case coaster::StationRole::DispatchCabin: Expected = Kind::DispatchCabin; break;
            case coaster::StationRole::UnloadDeck: Expected = Kind::UnloadDeck; break;
            case coaster::StationRole::ExitWalkway: Expected = Kind::ExitWalkway; break;
            case coaster::StationRole::Lift: Expected = Kind::Lift; break;
            case coaster::StationRole::Stair: Expected = Kind::Stair; break;
            case coaster::StationRole::Underpass: Expected = Kind::Underpass; break;
            case coaster::StationRole::QueueRail: Expected = Kind::QueueRail; break;
            default: Functional = false; break;
            }
            if (Functional)
            {
                if (Instance.Kind != Expected ||
                    !Instance.Transform.GetScale3D().Equals(FVector(Box.half.x, Box.half.y, Box.half.z), 1e-12))
                    return false;
                Min = FVector(-100); Max = FVector(100);
                const double Side = coaster::dot(Box.center - Midline, Box.right);
                const bool ReverseStair = Box.role == coaster::StationRole::Stair && Side < 0;
                const auto Forward = VibeMesh::Direction(Box.forward) * (ReverseStair ? -1. : 1.);
                if (!Instance.Transform.GetRotation().GetAxisX().Equals(Forward, 1e-8)) return false;
            }
            else
            {
                if (!Instance.Transform.GetScale3D().Equals(FVector::OneVector, 1e-12)) return false;
                switch (Instance.Kind)
                {
                case Kind::Platform: Min = FVector(-150, -192.5, -80); Max = FVector(150, 192.5, 0); break;
                case Kind::PlatformEnd: Min = FVector(-50, -192.5, -80); Max = FVector(50, 192.5, 0); break;
                case Kind::Roof: Min = FVector(-150, -565, -18); Max = FVector(150, 565, 18); break;
                case Kind::Post: Min = FVector(-20, -20, 0); Max = FVector(20, 20, 522); break;
                default: return false;
                }
                const bool Platform = Instance.Kind == Kind::Platform || Instance.Kind == Kind::PlatformEnd;
                if (Platform && Box.role != coaster::StationRole::Platform) return false;
                if (Instance.Kind == Kind::Roof && Box.role != coaster::StationRole::Canopy) return false;
                if (Instance.Kind == Kind::Post && Box.role != coaster::StationRole::Post) return false;
                if (Platform)
                {
                    // Imported stripe is local +Y. Its direction must point toward
                    // the actual midline on both platform sides, without mirroring scale.
                    const auto Inward = VibeMesh::Position(Midline) - Instance.Transform.GetLocation();
                    if (FVector::DotProduct(Instance.Transform.GetRotation().GetAxisY(), Inward) <= 0) return false;
                }
                else if (!Instance.Transform.GetRotation().GetAxisX().Equals(VibeMesh::Direction(Box.forward), 1e-8)) return false;
            }
            if (!Instance.Transform.GetRotation().GetAxisZ().Equals(VibeMesh::Direction(Box.up), 1e-8)) return false;
        }
        coaster::Vec3 Low{1e30, 1e30, 1e30}, High{-1e30, -1e30, -1e30};
        for (double X : {Min.X, Max.X}) for (double Y : {Min.Y, Max.Y}) for (double Z : {Min.Z, Max.Z})
        {
            const auto P = Instance.Transform.TransformPosition(FVector(X, Y, Z));
            const auto Delta = VibeCoordinates::CorePosition({P.X, P.Y, P.Z}) - Box.center;
            const coaster::Vec3 Local{coaster::dot(Delta, Box.forward), coaster::dot(Delta, Box.right), coaster::dot(Delta, Box.up)};
            Low = {std::min(Low.x, Local.x), std::min(Low.y, Local.y), std::min(Low.z, Local.z)};
            High = {std::max(High.x, Local.x), std::max(High.y, Local.y), std::max(High.z, Local.z)};
            if (std::abs(Local.x) > Box.half.x + 1e-7 || std::abs(Local.y) > Box.half.y + 1e-7 || std::abs(Local.z) > Box.half.z + 1e-7) return false;
        }
        // Each longitudinal partition fills the original cross-section. Together
        // they cover the full box without duplicated full-length geometry or gaps.
        if (std::abs(Low.y + Box.half.y) > 1e-7 || std::abs(High.y - Box.half.y) > 1e-7 ||
            std::abs(Low.z + Box.half.z) > 1e-7 || std::abs(High.z - Box.half.z) > 1e-7) return false;
        Intervals[Instance.SourceBoxIndex].push_back({Low.x, High.x});
    }
    for (size_t I = 0; I < Boxes.size(); ++I)
    {
        auto& Parts = Intervals[I];
        if (Parts.empty()) return false;
        std::sort(Parts.begin(), Parts.end());
        double End = -Boxes[I].half.x;
        for (const auto& Part : Parts)
        {
            if (std::abs(Part.first - End) > 1e-7 || Part.second <= Part.first) return false;
            End = Part.second;
        }
        if (std::abs(End - Boxes[I].half.x) > 1e-7) return false;
    }
    return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterCoordinateTest, "VibeCoaster.CoordinateContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter)
bool FCoasterCoordinateTest::RunTest(const FString& Parameters)
{
    const FVector P = VibeMesh::Position({1, 2, 3});
    TestTrue(TEXT("SI metres become UE centimetres with one Y reflection"), P.Equals(FVector(100, -200, 300), 1e-10));
    const auto Core = VibeCoordinates::CorePosition({P.X, P.Y, P.Z});
    TestTrue(TEXT("Coordinate round trip"), coaster::norm(Core - coaster::Vec3{1, 2, 3}) < 1e-10);
    const coaster::TrackSample Flat{{}, {1, 0, 0}, {}, {0, 0, 1}, {0, -1, 0}, coaster::Element::Station};
    const FQuat Rotation = VibeMesh::Rotation(Flat);
    TestTrue(TEXT("Forward axis follows canonical tangent"), Rotation.GetAxisX().Equals(VibeMesh::Direction(Flat.tangent), 1e-9));
    TestTrue(TEXT("Right axis preserves rider side"), Rotation.GetAxisY().Equals(VibeMesh::Direction(Flat.right), 1e-9));
    TestTrue(TEXT("Up axis follows canonical up"), Rotation.GetAxisZ().Equals(VibeMesh::Direction(Flat.up), 1e-9));
    auto Banked = Flat;
    Banked.up = coaster::rotate(Flat.up, Flat.tangent, .73);
    Banked.right = coaster::cross(Banked.tangent, Banked.up);
    TestTrue(TEXT("Banked rider right remains consistent"), VibeMesh::Rotation(Banked).GetAxisY().Equals(VibeMesh::Direction(Banked.right), 1e-9));
    const coaster::TrainConfig Train;
    TestTrue(TEXT("Seven-row middle camera is on the physical middle car"), Train.cars == 7 && FMath::IsNearlyEqual(coaster::seatDistanceOffset(Train, 1), 0., 1e-9));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterMeshTest, "VibeCoaster.MeshContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter)
bool FCoasterMeshTest::RunTest(const FString& Parameters)
{
    // Engine-produced indices/normals are the reference, not our permutation.
    TArray<FVector> BoxVertices, BoxNormals;
    TArray<int32> BoxIndices;
    TArray<FVector2D> BoxUV;
    TArray<FProcMeshTangent> BoxTangents;
    UKismetProceduralMeshLibrary::GenerateBoxMesh(FVector(50, 50, 50), BoxVertices, BoxIndices, BoxNormals, BoxUV, BoxTangents);
    int64 BoxTriangles = 0;
    if (!TestTrue(TEXT("Epic's generated box establishes negative geometric-normal dot for UE front faces"),
        HasEngineFrontFaces(BoxVertices, BoxNormals, BoxIndices, BoxTriangles) && BoxTriangles == 12)) return false;
    Swap(BoxIndices[1], BoxIndices[2]);
    int64 ReversedBoxTriangles = 0;
    TestFalse(TEXT("Independent facing check rejects a single reversed engine triangle"),
        HasEngineFrontFaces(BoxVertices, BoxNormals, BoxIndices, ReversedBoxTriangles));
    // Exercise exactly the accepted-design boundary used by the runtime. No
    // synthetic completion/convergence flags can stand in for this fixture.
    coaster::GenerationRequest Request;
    Request.seed = 42;
    Request.terrain.kind = coaster::TerrainKind::Flat;
    Request.targets.requireIntensity = false; // Explicit PHYSICS-PROOF fixture.
    Request.simulationStep = 1. / 960;
    auto D = std::make_shared<coaster::Design>(coaster::generate(Request));
    if (!TestTrue(TEXT("Deterministic physics-proof fixture passes full acceptance"), D->accepted()))
    {
        for (const auto* Report : {&D->report, &D->simulation.report})
            for (const auto& Error : Report->errors)
                AddError(FString(UTF8_TO_TCHAR((Error.code + ": " + Error.message).c_str())));
        return false;
    }
    TestTrue(TEXT("Fixture independently passed 960/1920 Hz convergence"),
        D->convergence.performed && D->convergence.passed &&
        FMath::IsNearlyEqual(D->convergence.coarseStep, 1. / 960, 1e-12) &&
        FMath::IsNearlyEqual(D->convergence.fineStep, 1. / 1920, 1e-12));
    TestTrue(TEXT("Fixture has a canonical station and support members"),
        D->station.enabled && !D->station.boxes.empty() && !D->supports.empty());

    VibeMesh::FPreparedRide Prepared; Prepared.Design = D;
    if (!TestTrue(TEXT("Accepted canonical ride prepares real render buffers"), VibeMesh::Prepare(Prepared, [] { return false; })))
    { AddError(Prepared.Error); return false; }

    TestTrue(TEXT("Physical drive and brake assemblies are prepared"), Prepared.LSMHardware.Num()>0 && Prepared.BrakeHardware.Num()>0 && Prepared.TrimFins.Num()>0);
    for(const auto& Fin: Prepared.TrimFins) TestTrue(TEXT("Moving trim fins retain valid operation and instance mappings"), Fin.Operation<D->operations.size() && Prepared.BrakeHardware.IsValidIndex(Fin.Instance) && D->operations[Fin.Operation].kind==coaster::DriveKind::Trim);
    TArray<FVector> SteelVertices, FootingVertices;
    TArray<int32> SteelIndices, FootingIndices;
    bool ValidIndices = true, ValidGround = true, ValidAttributes = true, ChunkBudget = true;
    bool RailAttributesMatch = true;
    const VibeMesh::FChunk* FirstRailChunk = nullptr;
    int64 TotalVertices = 0; int32 GroundChunks = 0, RailChunks = 0;
    bool FacingValid[4] = {true, true, true, true};
    int64 FacingTriangles[4] = {0, 0, 0, 0}; // Terrain, rail/spine, steel, footing.
    for (const auto& Chunk : Prepared.Chunks)
    {
        TotalVertices += Chunk.Vertices.Num();
        const int32 FacingKind = Chunk.Ground ? 0 : Chunk.Footing ? 3 : Chunk.Structure ? 2 : 1;
        FacingValid[FacingKind] &= HasEngineFrontFaces(Chunk.Vertices, Chunk.Normals, Chunk.Indices, FacingTriangles[FacingKind]);
        ValidIndices &= Chunk.Indices.Num() % 3 == 0;
        for (int32 Index : Chunk.Indices) ValidIndices &= Chunk.Vertices.IsValidIndex(Index);
        ValidAttributes &= Chunk.Normals.Num() == Chunk.Vertices.Num() && Chunk.UV.Num() == Chunk.Vertices.Num();
        ChunkBudget &= Chunk.Vertices.Num() <= (Chunk.Ground ? 4 : 984);
        if (Chunk.Ground)
        {
            ++GroundChunks;
            for (const FVector& P : Chunk.Vertices)
            {
                const auto Q = VibeCoordinates::CorePosition({P.X, P.Y, P.Z});
                ValidGround &= std::abs(Q.z - D->request.terrain.height(Q.x, Q.y)) < 1e-8;
            }
        }
        else if (Chunk.Structure || Chunk.Footing)
        {
            auto& Vertices = Chunk.Footing ? FootingVertices : SteelVertices;
            auto& Indices = Chunk.Footing ? FootingIndices : SteelIndices;
            const int32 Base = Vertices.Num(); Vertices.Append(Chunk.Vertices);
            for (int32 Index : Chunk.Indices) Indices.Add(Base + Index);
        }
        else
        {
            RailAttributesMatch &= RailChunkMatchesUnbatchedReference(Chunk, D->track, RailChunks);
            if (!FirstRailChunk) FirstRailChunk = &Chunk;
            ++RailChunks;
        }
    }
    TestTrue(TEXT("Every shared-sample rail/spine vertex, normal and UV matches the independent unbatched reference"),
        RailAttributesMatch && RailChunks == FMath::CeilToInt(D->track.length / 80.));
    if (FirstRailChunk && !FirstRailChunk->Vertices.IsEmpty())
    {
        auto Altered = *FirstRailChunk;
        Altered.Vertices[0].X += .001;
        TestFalse(TEXT("Rail reference rejects a displaced vertex"), RailChunkMatchesUnbatchedReference(Altered, D->track, 0));
        Altered.Vertices[0] = FirstRailChunk->Vertices[0];
        Altered.Normals[0] *= -1.;
        TestFalse(TEXT("Rail reference rejects a reversed normal"), RailChunkMatchesUnbatchedReference(Altered, D->track, 0));
        Altered.Normals[0] = FirstRailChunk->Normals[0];
        Altered.UV[0].X += .001;
        TestFalse(TEXT("Rail reference rejects a changed UV"), RailChunkMatchesUnbatchedReference(Altered, D->track, 0));
    }
    TestTrue(TEXT("Every ground triangle follows Epic's front-face convention"), FacingValid[0] && FacingTriangles[0] > 0);
    TestTrue(TEXT("Every rail/spine triangle follows Epic's front-face convention"), FacingValid[1] && FacingTriangles[1] > 0);
    TestTrue(TEXT("Every tapered steel side/cap follows Epic's front-face convention"), FacingValid[2] && FacingTriangles[2] > 0);
    TestTrue(TEXT("Every footing side/cap follows Epic's front-face convention"), FacingValid[3] && FacingTriangles[3] > 0);
    TestTrue(TEXT("Every triangle references a valid vertex"), ValidIndices);
    TestTrue(TEXT("Every vertex has its normal and UV"), ValidAttributes);
    TestTrue(TEXT("Ground vertices lie on the physical flat plane"), ValidGround && GroundChunks > 0);
    TestTrue(TEXT("Rail, ground and support sections obey per-section payload budgets"), ChunkBudget && RailChunks > 0);
    TestTrue(TEXT("Complete rendering fits aggregate budgets"), Prepared.Chunks.Num() <= 4096 && TotalVertices <= 2000000 && Prepared.Ties.Num() + Prepared.Supports.Num() + Prepared.Station.Num() <= 100000);

    // Compare the engine boundary against the portable canonical member mesh,
    // including coordinate reflection across chunk boundaries and both materials.
    TArray<FVector> ExpectedSteel, ExpectedFootings;
    TArray<int32> ExpectedSteelIndices, ExpectedFootingIndices;
    bool AllCanonical = true;
    for (const auto& Support : D->supports)
    {
        AllCanonical &= !Support.members.empty();
        for (const auto& Member : Support.members)
        {
            const auto Mesh = coaster::supportMemberMesh(Member);
            const bool Footing = Member.kind == coaster::SupportMemberKind::Footing;
            auto& Vertices = Footing ? ExpectedFootings : ExpectedSteel;
            auto& Indices = Footing ? ExpectedFootingIndices : ExpectedSteelIndices;
            const int32 Base = Vertices.Num();
            for (const auto& P : Mesh.positions) Vertices.Add(VibeMesh::Position(P));
            for (size_t I = 0; I < Mesh.indices.size(); I += 3)
                Indices.Append({Base + int32(Mesh.indices[I]), Base + int32(Mesh.indices[I + 1]), Base + int32(Mesh.indices[I + 2])});
        }
    }
    TestTrue(TEXT("Current accepted design uses canonical members, not legacy cylinders"), AllCanonical && Prepared.Supports.IsEmpty());
    TestTrue(TEXT("Steel vertices match canonical tapered solids"), !SteelVertices.IsEmpty() && SteelVertices == ExpectedSteel);
    TestTrue(TEXT("Footing vertices match canonical full-radius solids"), !FootingVertices.IsEmpty() && FootingVertices == ExpectedFootings);
    TestTrue(TEXT("Steel and footing indices preserve canonical order after coordinate reflection"), SteelIndices == ExpectedSteelIndices && FootingIndices == ExpectedFootingIndices);

    TestTrue(TEXT("Actual accepted station uses contained repeated detail modules"), Prepared.Station.Num() > int32(D->station.boxes.size()));
    TestTrue(TEXT("Station modules preserve every canonical box, material role, pivot and inward platform edge"),
        StationRepresentationMatches(Prepared, D->station.boxes, D->track.sample(0).position));
    bool TieContract = Prepared.Ties.Num() == FMath::CeilToInt(D->track.length / 3.);
    for (int32 I = 0; I < Prepared.Ties.Num(); ++I)
    {
        const auto P = D->track.sample(3. * I); const auto& Transform = Prepared.Ties[I];
        TieContract &= Transform.GetScale3D().Equals(FVector::OneVector, 1e-12);
        for (double X : {-.07, .07}) for (double Y : {-.825, .825}) for (double Z : {-.08, .08})
            TieContract &= Transform.TransformPosition(FVector(X * 100, Y * 100, Z * 100)).Equals(
                VibeMesh::Position(P.position + P.tangent * X + P.right * Y + P.up * (Z - .19)), 1e-6);
    }
    TestTrue(TEXT("Full-size imported ties follow actual banked rail frame and unchanged -0.19 m datum"), TieContract);

    VibeMesh::FPreparedRide Cancelled; Cancelled.Design = D;
    int32 CancelChecks = 0;
    TestFalse(TEXT("Cancellation stops an in-progress preparation"), VibeMesh::Prepare(Cancelled, [&] { return ++CancelChecks > 8; }));
    TestTrue(TEXT("Cancellation was reached after work began"), CancelChecks > 8 && !Cancelled.Chunks.IsEmpty());
    TestTrue(TEXT("Cancelling preparation does not mutate accepted source geometry"), D->accepted());
    auto Rejected = std::make_shared<coaster::Design>(*D);
    Rejected->report.fail("TEST", "intentional rejection of otherwise valid fixture");
    VibeMesh::FPreparedRide Bad; Bad.Design = Rejected;
    TestFalse(TEXT("Rejected designs cannot become ride meshes"), VibeMesh::Prepare(Bad, [] { return false; }));
    TestTrue(TEXT("Rejection creates no render buffers"), Bad.Chunks.IsEmpty() && Bad.Station.IsEmpty() && Bad.Ties.IsEmpty());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterStationArtTest, "VibeCoaster.StationArtContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::EngineFilter)
bool FCoasterStationArtTest::RunTest(const FString& Parameters)
{
    // Adapter-only fixtures deliberately exercise dimensions not present in the
    // default accepted ride; they are not presented as accepted station physics.
    using Kind = VibeMesh::EStationInstanceKind;
    const coaster::Vec3 Midline{73, -28, 12};
    const auto F = coaster::unit(coaster::Vec3{.8, .6, .1});
    const auto U = coaster::rotate(coaster::unit(coaster::Vec3{0, 0, 1} - F * F.z), F, .17);
    const auto R = coaster::cross(F, U);
    std::vector<coaster::StationBox> Boxes;
    for (double Side : {-1., 1.})
        Boxes.push_back({Midline + F * 10 + R * (Side * 3.275) - U * .4, F, R, U, {2.375, 1.925, .4}, coaster::StationRole::Platform});
    Boxes.push_back({Midline + F * 10 + U * 5.4, F, R, U, {3.625, 5.65, .18}, coaster::StationRole::Canopy});
    Boxes.push_back({Midline + R * 4.8 + U * 2.61, F, R, U, {.2, .2, 2.61}, coaster::StationRole::Post});
    // Unsupported width/height preserves the original full cube, including its role.
    Boxes.push_back({Midline + R * 8 - U * .4, F, R, U, {2.1, 2.1, .4}, coaster::StationRole::Platform});
    Boxes.push_back({Midline + R * 6 + U * 2.65, F, R, U, {.2, .2, 2.65}, coaster::StationRole::Post});
    VibeMesh::FPreparedRide Prepared;
    for (size_t I = 0; I < Boxes.size(); ++I)
        if (!TestTrue(TEXT("Station art helper accepts bounded partition fixture"),
            VibeMesh::AppendStationBoxInstances(Prepared, Boxes[I], int32(I), Midline, [] { return false; }))) return false;
    TestTrue(TEXT("Rotated partitions contain all declared asset corners and cover each source box exactly"),
        StationRepresentationMatches(Prepared, Boxes, Midline));
    int32 Platforms = 0, Ends = 0, Roofs = 0, Posts = 0, Steel = 0, Concrete = 0;
    for (const auto& Instance : Prepared.Station)
    {
        Platforms += Instance.Kind == Kind::Platform; Ends += Instance.Kind == Kind::PlatformEnd;
        Roofs += Instance.Kind == Kind::Roof; Posts += Instance.Kind == Kind::Post;
        Steel += Instance.Kind == Kind::CubeSteel; Concrete += Instance.Kind == Kind::CubeConcrete;
    }
    TestTrue(TEXT("Fractional platform/roof lengths use unit modules plus exact cube remainders"),
        Platforms == 2 && Ends == 2 && Roofs == 2 && Posts == 1 && Steel == 2 && Concrete == 3);
    const auto Unsupported = [&](int32 Index, Kind Expected)
    {
        int32 Count = 0;
        for (const auto& Instance : Prepared.Station) if (Instance.SourceBoxIndex == Index)
        { ++Count; if (Instance.Kind != Expected) return false; }
        return Count == 1;
    };
    TestTrue(TEXT("Unsupported detailed cross-sections retain one original material-correct box"), Unsupported(4, Kind::CubeConcrete) && Unsupported(5, Kind::CubeSteel));
    VibeMesh::FPreparedRide Cancelled;
    int32 Calls = 0;
    TestFalse(TEXT("Station tiling checks cancellation between modules"),
        VibeMesh::AppendStationBoxInstances(Cancelled, Boxes[0], 0, Midline, [&] { return ++Calls > 2; }));
    TestTrue(TEXT("Cancellation leaves a bounded partial buffer without source mutation"), Calls > 2 && Cancelled.Station.Num() == 1 && Boxes[0].half.x == 2.375);
    const std::array<coaster::StationRole, 12> FunctionalRoles = {
        coaster::StationRole::QueueDeck, coaster::StationRole::RouteRoof,
        coaster::StationRole::MergeDeck, coaster::StationRole::HoldingLane,
        coaster::StationRole::BoardingGate, coaster::StationRole::DispatchCabin,
        coaster::StationRole::UnloadDeck, coaster::StationRole::ExitWalkway,
        coaster::StationRole::Lift, coaster::StationRole::Stair,
        coaster::StationRole::Underpass, coaster::StationRole::QueueRail
    };
    std::vector<coaster::StationBox> FunctionalBoxes;
    VibeMesh::FPreparedRide FunctionalPrepared;
    for (size_t I = 0; I < FunctionalRoles.size(); ++I)
    {
        const double Side = FunctionalRoles[I] == coaster::StationRole::Stair ? -7. : 7.;
        FunctionalBoxes.push_back({Midline + F * (50. + 10. * I) + R * Side + U * 1.3,
            F, R, U, {1.2 + .1 * I, .7 + .03 * I, .35 + .02 * I}, FunctionalRoles[I]});
        if (!TestTrue(TEXT("Functional station role has a bounded imported-asset placement"),
            VibeMesh::AppendStationBoxInstances(FunctionalPrepared, FunctionalBoxes.back(),
                int32(I), Midline, [] { return false; }))) return false;
    }
    TestTrue(TEXT("All functional station roles preserve centred normalized bounds and exit-stair orientation"),
        FunctionalPrepared.Station.Num() == int32(FunctionalRoles.size()) &&
        StationRepresentationMatches(FunctionalPrepared, FunctionalBoxes, Midline));
    return true;
}

#if WITH_EDITOR
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterImportedArtTest, "VibeCoaster.ImportedArtContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCoasterImportedArtTest::RunTest(const FString& Parameters)
{
    auto* ReliefGround = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Ground_Highlands.M_Ground_Highlands"));
    TestNotNull(TEXT("Current texture-free relief ground material is available to the packaged runtime"), ReliefGround);
    TestNotNull(TEXT("LSM hardware material is available"),LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Materials/M_LSM.M_LSM")));
    TestNotNull(TEXT("Brake hardware material is available"),LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Materials/M_Brake.M_Brake")));
    if (ReliefGround) TestTrue(TEXT("Relief ground is an opaque surface"), ReliefGround->GetBlendMode() == BLEND_Opaque);

    struct FAssetContract
    {
        const TCHAR* Name;
        FVector Min, Max;
        TArray<FName> MaterialNames;
        bool NormalizedStation = false;
    };
    // Evaluated Blender MCP source bounds at canonical runtime pivots.
    // These are measured source dimensions, independently checked against
    // imported buffers; train and hardware containment remain guarded below.
    const TArray<FAssetContract> Contracts = {
        {TEXT("SM_LeadCar"), FVector(-127.5, -88, -27.097765), FVector(126.5, 88, 162.5),
            {TEXT("VC2_Padding"), TEXT("VC2_Steel"), TEXT("VC2_Graphite"), TEXT("VC2_Petrol"), TEXT("VC2_Light"), TEXT("VC2_Pearl"), TEXT("VC2_Copper"), TEXT("VC2_Glass")}},
        {TEXT("SM_TrainCar"), FVector(-127.5, -88, -27.097765), FVector(126.5, 88, 162.5),
            {TEXT("VC2_Padding"), TEXT("VC2_Steel"), TEXT("VC2_Graphite"), TEXT("VC2_Petrol"), TEXT("VC2_Light"), TEXT("VC2_Pearl"), TEXT("VC2_Copper"), TEXT("VC2_Glass")}},
        {TEXT("SM_TrackTieWeb"), FVector(-5.500003, -66.500002, -26.573604), FVector(5.500003, 66.500002, 10.5),
            {TEXT("VC2_Petrol"), TEXT("VC2_Pearl"), TEXT("VC2_Graphite"), TEXT("VC2_Steel"), TEXT("VC2_Copper")}},
        {TEXT("SM_LSMStator"), FVector(-47.999999, -48.249999, -50), FVector(47.999999, 48.249999, 50),
            {TEXT("VC2_Graphite"), TEXT("VC2_Petrol"), TEXT("VC2_Copper"), TEXT("VC2_Steel")}},
        {TEXT("SM_BrakeFin"), FVector(-47.999999, -45.500001, -47.350001), FVector(47.999999, 45.500001, 47),
            {TEXT("VC2_Steel"), TEXT("VC2_Graphite"), TEXT("VC2_Copper"), TEXT("VC2_Petrol")}},
        {TEXT("SM_StationRoofPanel"), FVector(-149.600005, -565.00001, -18.000001), FVector(149.600005, 565.00001, 18.000001),
            {TEXT("VC2_Pearl"), TEXT("VC2_Petrol"), TEXT("VC2_Light"), TEXT("VC2_Graphite")}},
        {TEXT("SM_StationPost"), FVector(-20, -20, -0), FVector(20, 20, 522.000027),
            {TEXT("VC2_Petrol"), TEXT("VC2_Steel"), TEXT("VC2_Graphite"), TEXT("VC2_Copper")}},
        {TEXT("SM_StationPlatformPanel"), FVector(-149.600005, -192.499995, -79.999995), FVector(149.600005, 192.500007, 0),
            {TEXT("VC2_Concrete"), TEXT("VC2_Pearl"), TEXT("VC2_Steel"), TEXT("VC2_Petrol")}},
        {TEXT("SM_StationPlatformEndPanel"), FVector(-49.599999, -192.499995, -79.999995), FVector(49.599999, 192.500007, 0),
            {TEXT("VC2_Concrete"), TEXT("VC2_Pearl"), TEXT("VC2_Steel"), TEXT("VC2_Petrol")}},
        {TEXT("SM_StationQueueDeck"), FVector(-100), FVector(100), {TEXT("VC2_Concrete"), TEXT("VC2_Petrol")}, true},
        {TEXT("SM_StationRouteRoof"), FVector(-100), FVector(100), {TEXT("VC2_Petrol"), TEXT("VC2_Steel"), TEXT("VC2_Light")}, true},
        {TEXT("SM_StationMergeDeck"), FVector(-100), FVector(100), {TEXT("VC2_Concrete"), TEXT("VC2_Petrol")}, true},
        {TEXT("SM_StationHoldingLane"), FVector(-100), FVector(100), {TEXT("VC2_Pearl"), TEXT("VC2_Graphite"), TEXT("VC2_Steel"), TEXT("VC2_Petrol")}, true},
        {TEXT("SM_StationBoardingGate"), FVector(-100), FVector(100), {TEXT("VC2_Graphite"), TEXT("VC2_Steel"), TEXT("VC2_Petrol")}, true},
        {TEXT("SM_StationDispatchCabin"), FVector(-100), FVector(100), {TEXT("VC2_Concrete"), TEXT("VC2_Petrol"), TEXT("VC2_Graphite"), TEXT("VC2_Glass"), TEXT("VC2_Steel")}, true},
        {TEXT("SM_StationUnloadDeck"), FVector(-100), FVector(100), {TEXT("VC2_Pearl"), TEXT("VC2_Steel")}, true},
        {TEXT("SM_StationExitWalkway"), FVector(-100), FVector(100), {TEXT("VC2_Concrete"), TEXT("VC2_Light")}, true},
        {TEXT("SM_StationLift"), FVector(-100), FVector(100), {TEXT("VC2_Concrete"), TEXT("VC2_Petrol"), TEXT("VC2_Glass"), TEXT("VC2_Graphite")}, true},
        {TEXT("SM_StationStair"), FVector(-100), FVector(100), {TEXT("VC2_Concrete"), TEXT("VC2_Steel")}, true},
        {TEXT("SM_StationUnderpass"), FVector(-100), FVector(100), {TEXT("VC2_Concrete"), TEXT("VC2_Petrol"), TEXT("VC2_Glass"), TEXT("VC2_Steel")}, true},
        {TEXT("SM_StationQueueRail"), FVector(-100), FVector(100), {TEXT("VC2_Graphite"), TEXT("VC2_Steel"), TEXT("VC2_Petrol")}, true}
    };
    constexpr double ToleranceCm = .02;
    for (const auto& Contract : Contracts)
    {
        const FString Label(Contract.Name);
        const FString AssetRoot = TEXT("/Game/Art/V3");
        const FString Path = FString::Printf(TEXT("%s/%s.%s"), *AssetRoot, Contract.Name, Contract.Name);
        UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *Path);
        if (!TestNotNull(Label + TEXT(" loads the actual current imported asset"), Mesh)) continue;
        const auto AssetBounds = Mesh->GetBoundingBox(); // waits for pending mesh build data
        const bool BoundsContract = Contract.NormalizedStation
            ? AssetBounds.Min.X >= Contract.Min.X - ToleranceCm &&
                AssetBounds.Min.Y >= Contract.Min.Y - ToleranceCm &&
                AssetBounds.Min.Z >= Contract.Min.Z - ToleranceCm &&
                AssetBounds.Max.X <= Contract.Max.X + ToleranceCm &&
                AssetBounds.Max.Y <= Contract.Max.Y + ToleranceCm &&
                AssetBounds.Max.Z <= Contract.Max.Z + ToleranceCm
            : AssetBounds.Min.Equals(Contract.Min, ToleranceCm) &&
                AssetBounds.Max.Equals(Contract.Max, ToleranceCm);
        TestTrue(Label + TEXT(" stays inside its authored centimetre envelope"), BoundsContract);
        const auto& Materials = Mesh->GetStaticMaterials();
        TestEqual(Label + TEXT(" has the expected material-slot count"), Materials.Num(), Contract.MaterialNames.Num());
        bool MaterialContract = true;
        TSet<FName> Found;
        for (int32 I = 0; I < Materials.Num(); ++I)
        {
            const FName SourceName = Materials[I].ImportedMaterialSlotName;
            const auto* Material = Mesh->GetMaterial(I);
            MaterialContract &= Contract.MaterialNames.Contains(SourceName) && !Found.Contains(SourceName) && Material != nullptr;
            if (Material) MaterialContract &= Material->GetPathName().StartsWith(AssetRoot + TEXT("/Materials/"));
            if (Material)
                TestTrue(Label + TEXT(" materials support instanced static meshes at runtime"),
                    Material->GetUsageByFlag(MATUSAGE_InstancedStaticMeshes));
            if (Material)
                TestTrue(Label + TEXT(" glass is translucent and structural materials are opaque"),
                    Material->GetBlendMode() == (SourceName == FName(TEXT("VC2_Glass")) ? BLEND_Translucent : BLEND_Opaque));
            Found.Add(SourceName);
        }
        TestTrue(Label + TEXT(" preserves distinct authored material roles without a default-material fallback"), MaterialContract && Found.Num() == Contract.MaterialNames.Num());
        FBox VertexBounds(ForceInit);
        int64 VertexCount = 0, TriangleCount = 0;
        bool GeometryValid = true, TrainEnvelopeFit = true, TieAssemblyFit = true;
        // The temporary flag permits reading resident buffers; the asset is not saved.
        const bool PreviousCPUAccess = Mesh->bAllowCPUAccess;
        Mesh->bAllowCPUAccess = true;
        for (int32 Section = 0; Section < Mesh->GetNumSections(0); ++Section)
        {
            TArray<FVector> Vertices, Normals;
            TArray<int32> Triangles;
            TArray<FVector2D> UV;
            TArray<FProcMeshTangent> Tangents;
            UKismetProceduralMeshLibrary::GetSectionFromStaticMesh(Mesh, 0, Section, Vertices, Triangles, Normals, UV, Tangents);
            GeometryValid &= !Vertices.IsEmpty() && Triangles.Num() > 0 && Triangles.Num() % 3 == 0;
            VertexCount += Vertices.Num(); TriangleCount += Triangles.Num() / 3;
            for (int32 Index : Triangles) GeometryValid &= Vertices.IsValidIndex(Index);
            if (Label == TEXT("SM_TrackTieWeb"))
            {
                const auto Webs = coaster::trackWebsLocal();
                const auto Saddles = coaster::trackTieSaddlesLocal();
                const coaster::StationBox Tie{{0,0,-.19},{1,0,0},{0,1,0},{0,0,1},{.07,.825,.08},coaster::StationRole::Post};
                // Test whole triangles in one convex certified solid, not just
                // isolated vertices in a non-convex union. Art pivot is the tie.
                for (int32 I = 0; I + 2 < Triangles.Num(); I += 3)
                {
                    bool Contained = false;
                    for (const auto& Box : {Tie, Webs[0], Webs[1], Saddles[0], Saddles[1]})
                    {
                        bool AllCorners = true;
                        for (int32 Corner = 0; Corner < 3; ++Corner)
                        {
                            const int32 Index = Triangles[I + Corner];
                            if (!Vertices.IsValidIndex(Index)) { AllCorners = false; continue; }
                            const auto& P = Vertices[Index];
                            const coaster::Vec3 Delta = coaster::Vec3{P.X*.01,P.Y*.01,P.Z*.01-.19}-Box.center;
                            AllCorners &= std::abs(coaster::dot(Delta,Box.forward)) <= Box.half.x + 2e-5 &&
                                std::abs(coaster::dot(Delta,Box.right)) <= Box.half.y + 2e-5 &&
                                std::abs(coaster::dot(Delta,Box.up)) <= Box.half.z + 2e-5;
                        }
                        Contained |= AllCorners;
                    }
                    TieAssemblyFit &= Contained;
                }
            }
            for (const FVector& P : Vertices)
            {
                const bool Finite = FMath::IsFinite(P.X) && FMath::IsFinite(P.Y) && FMath::IsFinite(P.Z);
                GeometryValid &= Finite;
                if (Finite) VertexBounds += P;
                if (Label == TEXT("SM_TrainCar") || Label == TEXT("SM_LeadCar"))
                    TrainEnvelopeFit &= Finite && std::abs(P.X) <= 100 * coaster::trainHalfLength + ToleranceCm &&
                        std::abs(P.Y) <= 100 * coaster::patronHalfWidth + ToleranceCm &&
                        P.Z >= 100 * coaster::trainEnvelopeBottom - ToleranceCm &&
                        P.Z <= 100 * coaster::patronTopHeight(coaster::TrainConfig{}) + ToleranceCm;
            }
        }
        Mesh->bAllowCPUAccess = PreviousCPUAccess;
        TestTrue(Label + TEXT(" has actual finite indexed LOD0 geometry"), GeometryValid && VertexCount > 0 && TriangleCount > 0);
        TestTrue(Label + TEXT(" actual vertices match its centimetre bounds and pivot, not a recentered approximation"),
            VertexBounds.IsValid && VertexBounds.Min.Equals(AssetBounds.Min, ToleranceCm) &&
            VertexBounds.Max.Equals(AssetBounds.Max, ToleranceCm));
        if (Label == TEXT("SM_TrackTieWeb"))
            TestTrue(TEXT("Every imported tie/web triangle is contained in one canonical hardware solid at the unchanged tie pivot"), TieAssemblyFit);
        if (Label == TEXT("SM_TrainCar") || Label == TEXT("SM_LeadCar"))
        {
            TestTrue(TEXT("All imported train vertices fit the unchanged occupied clearance envelope"), TrainEnvelopeFit);
            TestTrue(TEXT("Train origin remains at the rail midpoint with below-rail wheel retention and raised seating"),
                VertexBounds.Min.Z < 0 && VertexBounds.Max.Z > 150 && VertexBounds.GetCenter().Z > 0);
        }
    }
    return true;
}

#endif // WITH_EDITOR: imported source slot names and resident LOD CPU data.

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterGroundTest, "VibeCoaster.GroundContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCoasterGroundTest::RunTest(const FString& Parameters)
{
    VibeMesh::FPreparedRide Prepared;
    const FBox Bounds(FVector(-10000,-20000,1000), FVector(30000,40000,50000));
    Prepared.Bounds = Bounds;
    if (!TestTrue(TEXT("Flat ground prepares"), VibeMesh::AppendGround(Prepared, {}))) return false;
    TestEqual(TEXT("One ground component"), Prepared.Chunks.Num(), 1);
    const auto& Ground = Prepared.Chunks[0];
    TestTrue(TEXT("Plane preserves overview bounds"), Prepared.Bounds.Equals(Bounds, 0));
    TestTrue(TEXT("Four vertices and two triangles"), Ground.Ground && Ground.Vertices.Num()==4 && Ground.Indices.Num()==6);
    int64 FacingCount=0;
    TestTrue(TEXT("Ground faces upward under UE winding"), HasEngineFrontFaces(Ground.Vertices,Ground.Normals,Ground.Indices,FacingCount) && FacingCount==2);
    FBox Coverage(ForceInit);
    for (int32 I=0; I<Ground.Vertices.Num(); ++I)
    {
        const auto& V=Ground.Vertices[I]; Coverage+=V;
        TestTrue(TEXT("Plane vertices, normals and UVs agree with flat SI ground"), V.Z==0 && Ground.Normals[I]==FVector::UpVector && Ground.UV[I]==FVector2D(V.X/8000.,-V.Y/8000.));
    }
    TestTrue(TEXT("Ground extends beyond the entire ride"), Coverage.Min.X<Bounds.Min.X && Coverage.Min.Y<Bounds.Min.Y && Coverage.Max.X>Bounds.Max.X && Coverage.Max.Y>Bounds.Max.Y);
    auto Landscape = std::make_shared<coaster::Design>();
    Landscape->request.terrain.kind=coaster::TerrainKind::Highlands;
    Landscape->request.terrain.centerX=17.3; Landscape->request.terrain.centerY=-51.7;
    Landscape->request.terrain.bend=-.28;
    Landscape->request.terrain.plateau=.7;
    Landscape->request.terrain.cliffX=150; Landscape->request.terrain.cliffHeading=.2;
    Landscape->request.terrain.ramps.push_back({-300,-200,100,200,0,60,.1,.2,100});
    Landscape->request.terrain.ridge.points={{-120,-80,75,.2,.1},{0,-40,105,.15,0},{90,15,85,-.15,.2},{0,40,70,0,0}};
    Landscape->request.terrain.ridge.spineCount=3;
    Landscape->request.terrain.cliffCurvature=.001;
    Landscape->request.terrain.ravines.push_back({-250,-50,100,100,30,65,60,90});
    // A broad foothill lies beyond all ride interaction. It must be sampled by
    // the distant rings without forcing its whole footprint onto the fine grid.
    Landscape->request.terrain.knolls.push_back({900,850,120,600});
    coaster::Span Bow; Bow.c[0]={0,0,50}; Bow.c[1]={1600,0,0}; Bow.c[2]={-1600,0,0};
    Landscape->track.spans.push_back(Bow);
    VibeMesh::FPreparedRide Relief; Relief.Bounds=Bounds; Relief.Design=Landscape;
    if (TestTrue(TEXT("Resolved highlands prepare"),VibeMesh::AppendGround(Relief,{})))
    {
        const auto& Surface=Relief.Chunks[0];
        TestTrue(TEXT("Highlands preserve ride-only overview bounds"),Relief.Bounds.Equals(Bounds,0));
        TestTrue(TEXT("Highlands contain real terrain relief"),Surface.Vertices.Num()>100 && Surface.Vertices.ContainsByPredicate([](const FVector& P){return P.Z>5000;}));
        int64 Faces=0;
        TestTrue(TEXT("Every highland and basin face has the correct winding"),HasEngineFrontFaces(Surface.Vertices,Surface.Normals,Surface.Indices,Faces));
        const auto& Exact=Relief.GroundExactBounds;
        const double BodyRadius=coaster::occupiedRadius(Landscape->request.train);
        bool EnclosesRide=Exact[0]<=Bounds.Min.X/100&&Exact[1]<=-Bounds.Max.Y/100&&Exact[2]>=Bounds.Max.X/100&&Exact[3]>=-Bounds.Min.Y/100;
        for(int32 I=0;I<=32;++I){const double U=I/32.,X=1600*U*(1-U);EnclosesRide&=Exact[0]<=X-BodyRadius&&Exact[2]>=X+BodyRadius&&Exact[1]<=-BodyRadius&&Exact[3]>=BodyRadius;}
        TestTrue(TEXT("Exact terrain encloses structures and the complete bowed track plus occupied radius"),EnclosesRide);
        bool ExactSurface=true,SharedField=true,Finite=true,DistantRelief=false;
        int64 NearFaces=0;
        FBox SurfaceBounds(ForceInit);
        TMap<uint64,int32> EdgeUses;
        for(int32 I=0;I<Surface.Vertices.Num();++I){
            const auto& P=Surface.Vertices[I];SurfaceBounds+=P;
            const auto Q=VibeCoordinates::CorePosition({P.X,P.Y,P.Z});
            SharedField&=std::abs(Q.z-Landscape->request.terrain.height(Q.x,Q.y))<1e-7;
            Finite&=!P.ContainsNaN()&&!Surface.Normals[I].ContainsNaN()&&FMath::IsFinite(Surface.UV[I].X)&&FMath::IsFinite(Surface.UV[I].Y);
        }
        for(int32 I=0;I<Surface.Indices.Num();I+=3){
            const auto& A=Surface.Vertices[Surface.Indices[I]];const auto& B=Surface.Vertices[Surface.Indices[I+1]];const auto& C=Surface.Vertices[Surface.Indices[I+2]];
            const FVector P=(A+B+C)/3;
            const auto Q=VibeCoordinates::CorePosition({P.X,P.Y,P.Z});
            if(Q.x>=Exact[0]&&Q.x<=Exact[2]&&Q.y>=Exact[1]&&Q.y<=Exact[3]){
                ++NearFaces;ExactSurface&=std::abs(Q.z-Landscape->request.terrain.height(Q.x,Q.y))<1e-7;
                for(const FVector* V:{&A,&B,&C}){const auto R=VibeCoordinates::CorePosition({V->X,V->Y,V->Z});ExactSurface&=R.x>=Exact[0]&&R.x<=Exact[2]&&R.y>=Exact[1]&&R.y<=Exact[3];}
            }else DistantRelief|=P.Z>100&&(A-B).SizeSquared2D()>800.*800.;
            for(int32 E=0;E<3;++E){const uint32 First=uint32(Surface.Indices[I+E]),Second=uint32(Surface.Indices[I+(E+1)%3]);const uint64 Key=(uint64(FMath::Min(First,Second))<<32)|FMath::Max(First,Second);++EdgeUses.FindOrAdd(Key);}
        }
        bool Stitched=true;
        for(const auto& Edge:EdgeUses){
            if(Edge.Value==2)continue;
            const auto& A=Surface.Vertices[int32(Edge.Key>>32)];const auto& B=Surface.Vertices[int32(uint32(Edge.Key))];
            const bool Horizon=(A.X==SurfaceBounds.Min.X&&B.X==SurfaceBounds.Min.X)||(A.X==SurfaceBounds.Max.X&&B.X==SurfaceBounds.Max.X)||
                (A.Y==SurfaceBounds.Min.Y&&B.Y==SurfaceBounds.Min.Y)||(A.Y==SurfaceBounds.Max.Y&&B.Y==SurfaceBounds.Max.Y);
            Stitched&=Edge.Value==1&&Horizon;
        }
        TestTrue(TEXT("Near-interaction triangle interiors retain the exact native clearance surface"),ExactSurface&&NearFaces>100);
        TestTrue(TEXT("Distant terrain nodes sample the same finite resolved field"),SharedField&&Finite&&DistantRelief);
        TestTrue(TEXT("Every interior terrain edge is shared, including fine/coarse ring stitches"),Stitched);
    }
    VibeMesh::FPreparedRide Cancelled; Cancelled.Bounds=Bounds;
    TestFalse(TEXT("Cancelled ground is not appended"),VibeMesh::AppendGround(Cancelled,[]{return true;}));
    TestTrue(TEXT("Cancellation preserves existing state"),Cancelled.Chunks.IsEmpty() && Cancelled.Bounds.Equals(Bounds,0));
    VibeMesh::FPreparedRide Full; Full.Bounds=Bounds; Full.Chunks.SetNum(4096);
    TestFalse(TEXT("Ground respects component budget"),VibeMesh::AppendGround(Full,{}));
    TestEqual(TEXT("Budget refusal preserves chunks"),Full.Chunks.Num(),4096);
    VibeMesh::FPreparedRide VerticesFull; VerticesFull.Bounds=Bounds; VerticesFull.Chunks.AddDefaulted(); VerticesFull.Chunks[0].Vertices.SetNumUninitialized(2000000);
    TestFalse(TEXT("Ground respects vertex budget"),VibeMesh::AppendGround(VerticesFull,{}));
    VibeMesh::FPreparedRide Empty;
    TestFalse(TEXT("Uninitialised bounds are refused"),VibeMesh::AppendGround(Empty,{}));
    return true;
}
#endif
