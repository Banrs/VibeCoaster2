#include "CoasterMesh.h"
#include "CoasterTerrainBackdrop.h"
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
// Independent pre-batching reference: each rail/spine independently samples
// the accepted track. Deliberately do not use Tube(), its shared sample array,
// or rendering coordinate helpers; test both attributes and tube/ring ordering.
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
                const double Angle = 2 * coaster::pi * SideIndex / 8;
                const auto N = P.right * std::cos(Angle) + P.up * std::sin(Angle);
                const auto V = P.position + P.right * Side + P.up * Height + N * Radius;
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
        const bool ConcreteRole = Box.role == coaster::StationRole::Platform || Box.role == coaster::StationRole::Footing;
        if (Cube)
        {
            if ((Instance.Kind == Kind::CubeConcrete) != ConcreteRole) return false;
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
            if (!Instance.Transform.GetRotation().GetAxisZ().Equals(VibeMesh::Direction(Box.up), 1e-8)) return false;
            if (Platform)
            {
                // Imported stripe is local +Y. Its direction must point toward
                // the actual midline on both platform sides, without mirroring scale.
                const auto Inward = VibeMesh::Position(Midline) - Instance.Transform.GetLocation();
                if (FVector::DotProduct(Instance.Transform.GetRotation().GetAxisY(), Inward) <= 0) return false;
            }
            else if (!Instance.Transform.GetRotation().GetAxisX().Equals(VibeMesh::Direction(Box.forward), 1e-8)) return false;
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
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterCoordinateTest, "VibeCoaster.CoordinateContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
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
    TestTrue(TEXT("Middle camera is on a physical middle car"), FMath::IsNearlyEqual(coaster::seatDistanceOffset(Train, 1), 1.7, 1e-9));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterMeshTest, "VibeCoaster.MeshContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
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

    TArray<FVector> SteelVertices, FootingVertices;
    TArray<int32> SteelIndices, FootingIndices;
    bool ValidIndices = true, ValidGround = true, ValidAttributes = true, ChunkBudget = true;
    bool RailAttributesMatch = true;
    const VibeMesh::FChunk* FirstRailChunk = nullptr;
    int64 TotalVertices = 0; int32 TerrainChunks = 0, RailChunks = 0;
    bool FacingValid[4] = {true, true, true, true};
    int64 FacingTriangles[4] = {0, 0, 0, 0}; // Terrain, rail/spine, steel, footing.
    for (const auto& Chunk : Prepared.Chunks)
    {
        TotalVertices += Chunk.Vertices.Num();
        const int32 FacingKind = Chunk.Terrain ? 0 : Chunk.Footing ? 3 : Chunk.Structure ? 2 : 1;
        FacingValid[FacingKind] &= HasEngineFrontFaces(Chunk.Vertices, Chunk.Normals, Chunk.Indices, FacingTriangles[FacingKind]);
        ValidIndices &= Chunk.Indices.Num() % 3 == 0;
        for (int32 Index : Chunk.Indices) ValidIndices &= Chunk.Vertices.IsValidIndex(Index);
        ValidAttributes &= Chunk.Normals.Num() == Chunk.Vertices.Num() && Chunk.UV.Num() == Chunk.Vertices.Num();
        ChunkBudget &= Chunk.Vertices.Num() <= (Chunk.Terrain ? VibeMesh::TerrainBackdrop::CliffChunkVertices : 984);
        if (Chunk.Terrain)
        {
            ++TerrainChunks;
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
    TestTrue(TEXT("Every terrain triangle follows Epic's front-face convention"), FacingValid[0] && FacingTriangles[0] > 0);
    TestTrue(TEXT("Every rail/spine triangle follows Epic's front-face convention"), FacingValid[1] && FacingTriangles[1] > 0);
    TestTrue(TEXT("Every tapered steel side/cap follows Epic's front-face convention"), FacingValid[2] && FacingTriangles[2] > 0);
    TestTrue(TEXT("Every footing side/cap follows Epic's front-face convention"), FacingValid[3] && FacingTriangles[3] > 0);
    TestTrue(TEXT("Every triangle references a valid vertex"), ValidIndices);
    TestTrue(TEXT("Every vertex has its normal and UV"), ValidAttributes);
    TestTrue(TEXT("Terrain vertices use the unchanged canonical height query"), ValidGround && TerrainChunks > 0);
    TestTrue(TEXT("Rail, terrain and support sections obey per-section payload budgets"), ChunkBudget && RailChunks > 0);
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

    // Check actual canyon render triangles against the continuous heightfield,
    // not just vertices (which also passed on the visibly faceted 20 m grid).
    Request.terrain = coaster::Terrain::seeded(coaster::TerrainKind::Canyon, 42);
    auto Canyon = std::make_shared<coaster::Design>(coaster::generate(Request));
    if (!TestTrue(TEXT("Canyon tessellation fixture passes full acceptance"), Canyon->accepted())) return false;
    VibeMesh::FPreparedRide Cliff; Cliff.Design = Canyon;
    if (!TestTrue(TEXT("Accepted canyon prepares within unchanged render budgets"), VibeMesh::Prepare(Cliff, [] { return false; })))
    { AddError(Cliff.Error); return false; }
    const auto& Landscape = Canyon->request.terrain;
    int64 CliffSamples = 0, CliffVertices = 0;
    double HeightError = 0, NormalErrorDegrees = 0;
    for (const auto& Chunk : Cliff.Chunks)
    {
        CliffVertices += Chunk.Vertices.Num();
        if (!Chunk.Terrain) continue;
        for (int32 I = 0; I < Chunk.Indices.Num(); I += 3)
        {
            FVector P[3], N[3]; bool NearRide = true;
            for (int32 J = 0; J < 3; ++J)
            {
                P[J] = Chunk.Vertices[Chunk.Indices[I + J]]; N[J] = Chunk.Normals[Chunk.Indices[I + J]];
                NearRide &= P[J].X >= Cliff.Bounds.Min.X && P[J].X <= Cliff.Bounds.Max.X &&
                    P[J].Y >= Cliff.Bounds.Min.Y && P[J].Y <= Cliff.Bounds.Max.Y;
            }
            if (!NearRide) continue;
            // Centroid and edge midpoints independently assess linear raster
            // interpolation; a finer derivative is the reference normal.
            for (const FVector& Weight : {FVector(1. / 3), FVector(.5, .5, 0), FVector(.5, 0, .5), FVector(0, .5, .5)})
            {
                const FVector V = P[0] * Weight.X + P[1] * Weight.Y + P[2] * Weight.Z;
                const auto Q = VibeCoordinates::CorePosition({V.X, V.Y, V.Z});
                const double DX = (Landscape.height(Q.x + .01, Q.y) - Landscape.height(Q.x - .01, Q.y)) / .02;
                const double DY = (Landscape.height(Q.x, Q.y + .01) - Landscape.height(Q.x, Q.y - .01)) / .02;
                if (std::hypot(DX, DY) < .2) continue; // Exercise the wall, not only the rims.
                const FVector Normal = (N[0] * Weight.X + N[1] * Weight.Y + N[2] * Weight.Z).GetSafeNormal();
                const FVector Reference = VibeMesh::Direction(coaster::unit({-DX, -DY, 1}));
                HeightError = FMath::Max(HeightError, std::abs(Q.z - Landscape.height(Q.x, Q.y)));
                NormalErrorDegrees = FMath::Max(NormalErrorDegrees, FMath::RadiansToDegrees(std::acos(FMath::Clamp(FVector::DotProduct(Normal, Reference), -1., 1.))));
                ++CliffSamples;
            }
        }
    }
    AddInfo(FString::Printf(TEXT("Canyon near-wall samples=%lld max height error=%.6f m normal error=%.6f degrees vertices=%lld chunks=%d"),
        CliffSamples, HeightError, NormalErrorDegrees, CliffVertices, Cliff.Chunks.Num()));
    TestTrue(TEXT("Actual near-wall triangles stay within 0.35 m of the canonical cliff"), CliffSamples > 10000 && HeightError < .35);
    TestTrue(TEXT("Interpolated near-wall normals stay within 0.6 degrees of the continuous cliff"), NormalErrorDegrees < .6);
    TestTrue(TEXT("Fine canyon rendering retains aggregate budgets"), CliffVertices <= 2000000 && Cliff.Chunks.Num() <= 4096);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterStationArtTest, "VibeCoaster.StationArtContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
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
    return true;
}

#if WITH_EDITOR
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterImportedArtTest, "VibeCoaster.ImportedArtContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCoasterImportedArtTest::RunTest(const FString& Parameters)
{
    auto* ReliefGround = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Ground_Relief.M_Ground_Relief"));
    TestNotNull(TEXT("Current texture-free relief ground material is available to the packaged runtime"), ReliefGround);
    if (ReliefGround) TestTrue(TEXT("Relief ground is an opaque surface"), ReliefGround->GetBlendMode() == BLEND_Opaque);

    struct FAssetContract
    {
        const TCHAR* Name;
        FVector Min, Max;
        TArray<FName> MaterialNames;
    };
    // Actual evaluated Blender bounds, expressed in imported centimetres. Slabs
    // deliberately have 4 mm end seams; module placement still uses 3 m / 1 m.
    const TArray<FAssetContract> Contracts = {
        {TEXT("SM_TrainCar"), FVector(-127.5, -85, 10), FVector(127.5, 85, 151),
            {TEXT("VCTrain4_Trim_IceCyan"), TEXT("VCTrain4_StructuralCarbon"), TEXT("VCTrain4_WindDeflector_ClearCyan"),
             TEXT("VCTrain4_Metal_BrushedAluminium"), TEXT("VCTrain4_Shell_PearlTitanium"), TEXT("VCTrain4_Padding_Graphite"),
             TEXT("VCTrain4_Restraint_Ceramic"), TEXT("VCTrain4_Rubber_GripAndTyre"), TEXT("VCTrain4_Shell_DeepPetrol"), TEXT("VCTrain4_Metal_DarkChassis")}},
        {TEXT("SM_TrackTieWeb"), FVector(-7, -82.5, -27.55840421), FVector(7, 82.5, 8),
            {TEXT("VC_ENVKIT_BlueSteel"), TEXT("VC_ENVKIT_Graphite"), TEXT("VC_ENVKIT_MachinedSteel")}},
        {TEXT("SM_StationPlatformPanel"), FVector(-149.6, -192.5, -80), FVector(149.6, 192.5, 0),
            {TEXT("VC_ENVKIT_Concrete"), TEXT("VC_ENVKIT_ConcreteEdge"), TEXT("VC_ENVKIT_DarkRecess"), TEXT("VC_ENVKIT_Graphite"), TEXT("VC_ENVKIT_SafetyOchre")}},
        {TEXT("SM_StationPlatformEndPanel"), FVector(-49.6, -192.5, -80), FVector(49.6, 192.5, 0),
            {TEXT("VC_ENVKIT_Concrete"), TEXT("VC_ENVKIT_ConcreteEdge"), TEXT("VC_ENVKIT_DarkRecess"), TEXT("VC_ENVKIT_Graphite"), TEXT("VC_ENVKIT_SafetyOchre")}},
        {TEXT("SM_StationRoofPanel"), FVector(-149.6, -565, -18), FVector(149.6, 565, 18),
            {TEXT("VC_ENVKIT_BlueSteel"), TEXT("VC_ENVKIT_DarkRecess"), TEXT("VC_ENVKIT_Graphite"), TEXT("VC_ENVKIT_LightDiffuser")}},
        {TEXT("SM_StationPost"), FVector(-20, -20, 0), FVector(20, 20, 522),
            {TEXT("VC_ENVKIT_BlueSteel"), TEXT("VC_ENVKIT_DarkRecess"), TEXT("VC_ENVKIT_Graphite"), TEXT("VC_ENVKIT_MachinedSteel")}}
    };
    constexpr double ToleranceCm = .02;
    for (const auto& Contract : Contracts)
    {
        const FString Label(Contract.Name);
        const FString AssetRoot = Label == TEXT("SM_TrackTieWeb") ? TEXT("/Game/Art/V072/TrackWeb1") : TEXT("/Game/Art/V072/Import1");
        const FString Path = FString::Printf(TEXT("%s/%s.%s"), *AssetRoot, Contract.Name, Contract.Name);
        UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *Path);
        if (!TestNotNull(Label + TEXT(" loads the actual current imported asset"), Mesh)) continue;
        const auto AssetBounds = Mesh->GetBoundingBox(); // waits for pending mesh build data
        TestTrue(Label + TEXT(" has centimetre bounds at its authored pivot"),
            AssetBounds.Min.Equals(Contract.Min, ToleranceCm) && AssetBounds.Max.Equals(Contract.Max, ToleranceCm));
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
                TestTrue(Label + TEXT(" clear shield uses translucent shading; structure stays opaque"),
                    Material->GetBlendMode() == (SourceName == FName(TEXT("VCTrain4_WindDeflector_ClearCyan")) ? BLEND_Translucent : BLEND_Opaque));
            Found.Add(SourceName);
        }
        TestTrue(Label + TEXT(" preserves distinct authored material roles without a default-material fallback"), MaterialContract && Found.Num() == Contract.MaterialNames.Num());
        FBox VertexBounds(ForceInit);
        int64 VertexCount = 0, TriangleCount = 0;
        bool GeometryValid = true, TrainBodyFit = true, TieAssemblyFit = true;
        // This editor-only test reads the already resident render buffers. The
        // temporary flag only suppresses a cooked-access warning; restore it
        // without dirtying/saving the asset or retaining runtime CPU buffers.
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
                const coaster::StationBox Tie{{0,0,-.19},{1,0,0},{0,1,0},{0,0,1},{.07,.825,.08},coaster::StationRole::Post};
                // Test whole triangles in one convex certified solid, not just
                // isolated vertices in a non-convex union. Art pivot is the tie.
                for (int32 I = 0; I + 2 < Triangles.Num(); I += 3)
                {
                    bool Contained = false;
                    for (const auto& Box : {Tie, Webs[0], Webs[1]})
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
                if (Label == TEXT("SM_TrainCar"))
                    TrainBodyFit &= Finite && std::abs(P.X) <= 127.5 + ToleranceCm &&
                        std::abs(P.Y) <= 85 + ToleranceCm && P.Z >= -ToleranceCm && P.Z <= 240 + ToleranceCm;
            }
        }
        Mesh->bAllowCPUAccess = PreviousCPUAccess;
        TestTrue(Label + TEXT(" has actual finite indexed LOD0 geometry"), GeometryValid && VertexCount > 0 && TriangleCount > 0);
        TestTrue(Label + TEXT(" actual vertices match its centimetre bounds and pivot, not a recentered approximation"),
            VertexBounds.IsValid && VertexBounds.Min.Equals(Contract.Min, ToleranceCm) && VertexBounds.Max.Equals(Contract.Max, ToleranceCm));
        if (Label == TEXT("SM_TrackTieWeb"))
            TestTrue(TEXT("Every imported tie/web triangle is contained in one canonical hardware solid at the unchanged tie pivot"), TieAssemblyFit);
        if (Label == TEXT("SM_TrainCar"))
        {
            TestTrue(TEXT("All imported train vertices fit the unchanged above-rail body and 2.55 x 1.70 m footprint"), TrainBodyFit);
            TestTrue(TEXT("Train origin remains at the rail midpoint: body bottom +10 cm, top +151 cm, not bounds-centred"),
                VertexBounds.Min.Z > 0 && FMath::IsNearlyEqual(VertexBounds.GetCenter().Z, 80.5, ToleranceCm));
        }
    }
    return true;
}

#endif // WITH_EDITOR: imported source slot names and resident LOD CPU data.

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterTerrainBackdropTest, "VibeCoaster.TerrainBackdropContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCoasterTerrainBackdropTest::RunTest(const FString& Parameters)
{
    // Standalone geometry-helper fixtures; these are not accepted ride fixtures.
    // Boundary incidence/area tests are independent of the zipper ordering.
    using Point = std::array<double, 2>;
    using Edge = std::array<double, 4>;
    const auto EdgeKey = [](Point A, Point B) { if (B < A) std::swap(A, B); return Edge{A[0], A[1], B[0], B[1]}; };
    constexpr double X0 = -320, Y0 = -640, X1 = X0 + 2 * 320, Y1 = Y0 + 3 * 320;
    const FBox OriginalBounds(FVector(-100, -200, -300), FVector(400, 500, 600));
    const auto RingPlan = VibeMesh::TerrainBackdrop::BuildRingPlan(X0, Y0, 2, 3, true);
    TestEqual(TEXT("Backdrop plans 48 dense halo bands before 12 distant bands"), RingPlan.Num(), 61);
    bool DenseHalo = true;
    for (int32 Ring = 1; Ring < RingPlan.Num(); ++Ring)
    {
        const auto& Inner = RingPlan[Ring - 1]; const auto& Outer = RingPlan[Ring];
        if (Ring <= 48) DenseHalo &= Inner.MinX - Outer.MinX == 20 && Inner.MinY - Outer.MinY == 20 &&
            (Outer.MaxX - Outer.MinX) / Outer.SegmentsX <= 20 && (Outer.MaxY - Outer.MinY) / Outer.SegmentsY <= 20;
    }
    TestTrue(TEXT("Steep-wall halo keeps <=20 m side spacing through 960 m"), DenseHalo && RingPlan[48].MinX == X0 - 960);

    const auto BoundsUnchanged = [&](const VibeMesh::FPreparedRide& P)
    { return P.Bounds.IsValid == OriginalBounds.IsValid && P.Bounds.Min == OriginalBounds.Min && P.Bounds.Max == OriginalBounds.Max; };
    for (const auto& Terrain : {coaster::Terrain::seeded(coaster::TerrainKind::Flat, 42),
        coaster::Terrain::seeded(coaster::TerrainKind::Hills, 42), coaster::Terrain::seeded(coaster::TerrainKind::Canyon, 42),
        coaster::Terrain{coaster::TerrainKind::Canyon}})
    {
        const bool DetailedCliffs = Terrain.kind == coaster::TerrainKind::Canyon && Terrain.cliffHeight > 0;
        std::set<Edge> ExpectedInner;
        const double NearStep = DetailedCliffs ? 5. : 20.;
        for (int32 I = 0; I < (X1 - X0) / NearStep; ++I)
        {
            ExpectedInner.insert(EdgeKey(Point{X0 + I * NearStep, Y0}, Point{X0 + (I + 1) * NearStep, Y0}));
            ExpectedInner.insert(EdgeKey(Point{X0 + I * NearStep, Y1}, Point{X0 + (I + 1) * NearStep, Y1}));
        }
        for (int32 I = 0; I < (Y1 - Y0) / NearStep; ++I)
        {
            ExpectedInner.insert(EdgeKey(Point{X0, Y0 + I * NearStep}, Point{X0, Y0 + (I + 1) * NearStep}));
            ExpectedInner.insert(EdgeKey(Point{X1, Y0 + I * NearStep}, Point{X1, Y0 + (I + 1) * NearStep}));
        }
        const auto CasePlan = VibeMesh::TerrainBackdrop::BuildRingPlan(X0, Y0, 2, 3, DetailedCliffs);
        const double Extension = DetailedCliffs ? 81900 + 48 * 20 : 81900;
        const int32 ChunkLimit = DetailedCliffs ? VibeMesh::TerrainBackdrop::CliffChunkVertices : VibeMesh::TerrainBackdrop::DefaultChunkVertices;
        int64 CaseTriangles = 0;
        for (int32 Ring = 1; Ring < CasePlan.Num(); ++Ring) CaseTriangles += 2 * int64(CasePlan[Ring - 1].SegmentsX + CasePlan[Ring - 1].SegmentsY + CasePlan[Ring].SegmentsX + CasePlan[Ring].SegmentsY);
        TestEqual(TEXT("Only actual cliff profiles add dense halo bands"), CasePlan.Num(), DetailedCliffs ? 61 : 13);
        const FString Label = FString(UTF8_TO_TCHAR(Terrain.name().c_str())) +
            (Terrain.kind == coaster::TerrainKind::Canyon && !DetailedCliffs ? TEXT(" default profile") : TEXT(""));
        VibeMesh::FPreparedRide Prepared; Prepared.Bounds = OriginalBounds;
        if (!TestTrue(Label + TEXT(": backdrop prepares"), VibeMesh::AppendTerrainBackdrop(Prepared, Terrain, X0, Y0, 2, 3, [] { return false; })))
        { AddError(Prepared.Error); return false; }
        bool Valid = true, Canonical = true, Facing = true, Nondegenerate = true, DenseEdges = true;
        double Area = 0; int64 Checked = 0, VertexCount = 0, TriangleCount = 0;
        std::map<Edge, int32> Edges;
        for (const auto& Chunk : Prepared.Chunks)
        {
            VertexCount += Chunk.Vertices.Num();
            Valid &= Chunk.Terrain && !Chunk.Structure && !Chunk.Footing && Chunk.Vertices.Num() <= ChunkLimit && Chunk.Vertices.Num() == Chunk.Normals.Num() && Chunk.Vertices.Num() == Chunk.UV.Num() && Chunk.Indices.Num() % 3 == 0;
            Facing &= HasEngineFrontFaces(Chunk.Vertices, Chunk.Normals, Chunk.Indices, Checked);
            for (int32 I = 0; I < Chunk.Vertices.Num(); ++I)
            {
                const auto& P = Chunk.Vertices[I]; const auto& Normal = Chunk.Normals[I];
                const auto C = VibeCoordinates::CorePosition({P.X, P.Y, P.Z});
                Canonical &= std::isfinite(C.x) && std::isfinite(C.y) && std::isfinite(C.z) && std::abs(C.z - Terrain.height(C.x, C.y)) < 1e-8;
                Canonical &= std::isfinite(Normal.X) && std::isfinite(Normal.Y) && std::isfinite(Normal.Z) && Normal.Z > 0 && FMath::Abs(Normal.SizeSquared() - 1) < 1e-8;
                Canonical &= Chunk.UV[I].Equals(FVector2D(C.x / 80, C.y / 80), 1e-8);
            }
            for (int32 I = 0; I + 2 < Chunk.Indices.Num(); I += 3)
            {
                Point XY[3]; bool TriangleValid = true;
                for (int32 J = 0; J < 3; ++J)
                {
                    const int32 Index = Chunk.Indices[I + J];
                    if (!Chunk.Vertices.IsValidIndex(Index)) { TriangleValid = false; break; }
                    const auto& V = Chunk.Vertices[Index]; const auto P = VibeCoordinates::CorePosition({V.X, V.Y, V.Z}); XY[J] = {P.x, P.y};
                }
                if (!TriangleValid) { Valid = false; continue; }
                const double TwiceArea = (XY[1][0] - XY[0][0]) * (XY[2][1] - XY[0][1]) - (XY[1][1] - XY[0][1]) * (XY[2][0] - XY[0][0]);
                Nondegenerate &= std::isfinite(TwiceArea) && TwiceArea > 0;
                const auto InHalo = [&](const Point& P) { return P[0] >= X0 - 960 && P[0] <= X1 + 960 && P[1] >= Y0 - 960 && P[1] <= Y1 + 960; };
                if (DetailedCliffs && InHalo(XY[0]) && InHalo(XY[1]) && InHalo(XY[2]))
                    for (int32 J = 0; J < 3; ++J) DenseEdges &= std::hypot(XY[J][0] - XY[(J + 1) % 3][0], XY[J][1] - XY[(J + 1) % 3][1]) <= 45;

                Area += std::abs(TwiceArea) * .5; ++TriangleCount;
                for (int32 J = 0; J < 3; ++J) ++Edges[EdgeKey(XY[J], XY[(J + 1) % 3])];
            }
        }
        bool EdgeCoverage = true; int32 OuterEdges = 0;
        for (const auto& E : ExpectedInner)
        { const auto Found = Edges.find(E); EdgeCoverage &= Found != Edges.end() && Found->second == 1; }
        for (const auto& Entry : Edges)
        {
            const Edge& E = Entry.first; const int32 Count = Entry.second;
            if (ExpectedInner.count(E)) { EdgeCoverage &= Count == 1; continue; }
            if (Count == 2) continue;
            const bool Outer =
                (E[0] == X0 - Extension && E[2] == X0 - Extension) ||
                (E[0] == X1 + Extension && E[2] == X1 + Extension) ||
                (E[1] == Y0 - Extension && E[3] == Y0 - Extension) ||
                (E[1] == Y1 + Extension && E[3] == Y1 + Extension);
            EdgeCoverage &= Count == 1 && Outer; if (Outer) ++OuterEdges;
        }
        const double ExpectedArea = (X1 - X0 + 2 * Extension) * (Y1 - Y0 + 2 * Extension) - (X1 - X0) * (Y1 - Y0);
        TestTrue(Label + TEXT(": fine inner seam exact, internal edges paired, only outer boundary open"), EdgeCoverage && OuterEdges == 32);
        TestTrue(Label + TEXT(": triangles cover exactly the rectangle annulus"), Nondegenerate && FMath::Abs(Area - ExpectedArea) < ExpectedArea * 1e-10);
        TestTrue(Label + TEXT(": emitted dense-halo edges stay bounded and match planned triangle budget"), DenseEdges && TriangleCount == CaseTriangles);
        TestTrue(Label + TEXT(": all faces follow Epic clockwise convention"), Facing && Checked == TriangleCount && Checked > 0);
        TestTrue(Label + TEXT(": canonical heights, finite unit normals and world UV"), Canonical);
        TestTrue(Label + TEXT(": valid attributes and bounded chunk/aggregate payload"), Valid && VertexCount <= 2000000 && Prepared.Chunks.Num() <= 4096);
        TestEqual(Label + TEXT(": exact coalesced component count preserves all planned triangles"), int64(Prepared.Chunks.Num()), (CaseTriangles * 3 + ChunkLimit - 1) / ChunkLimit);
        TestTrue(Label + TEXT(": ride overview bounds remain unchanged"), BoundsUnchanged(Prepared));
    }
    coaster::Terrain Terrain;
    VibeMesh::FPreparedRide Cancelled; Cancelled.Bounds = OriginalBounds;
    int32 CancelCalls = 0;
    TestFalse(TEXT("Backdrop cancellation stops during staged triangle emission"), VibeMesh::AppendTerrainBackdrop(Cancelled, Terrain, X0, Y0, 2, 3, [&] { return ++CancelCalls > 6; }));
    TestTrue(TEXT("Cancellation reached partial work without changing ride bounds"), CancelCalls > 6 && !Cancelled.Chunks.IsEmpty() && BoundsUnchanged(Cancelled));
    VibeMesh::FPreparedRide CliffCancelled; CliffCancelled.Bounds = OriginalBounds;
    const auto CliffTerrain = coaster::Terrain::seeded(coaster::TerrainKind::Canyon, 42);
    TestFalse(TEXT("Cliff cancellation remains responsive after a coalesced chunk is staged"),
        VibeMesh::AppendTerrainBackdrop(CliffCancelled, CliffTerrain, X0, Y0, 2, 3, [&] { return !CliffCancelled.Chunks.IsEmpty(); }));
    TestTrue(TEXT("Cliff cancellation preserves one bounded completed staging chunk and ride bounds"),
        CliffCancelled.Chunks.Num() == 1 && CliffCancelled.Chunks[0].Vertices.Num() == VibeMesh::TerrainBackdrop::CliffChunkVertices && BoundsUnchanged(CliffCancelled));
    VibeMesh::FPreparedRide ChunkFull; ChunkFull.Bounds = OriginalBounds; ChunkFull.Chunks.SetNum(4096);
    TestFalse(TEXT("Full aggregate chunk budget rejects before appending"), VibeMesh::AppendTerrainBackdrop(ChunkFull, Terrain, X0, Y0, 2, 3, [] { return false; }));
    TestTrue(TEXT("Chunk-budget failure preserves prior buffers and bounds"), ChunkFull.Chunks.Num() == 4096 && BoundsUnchanged(ChunkFull));
    VibeMesh::FPreparedRide VertexFull; VertexFull.Bounds = OriginalBounds; VertexFull.Chunks.AddDefaulted(); VertexFull.Chunks[0].Vertices.SetNumUninitialized(2000000);
    TestFalse(TEXT("Full aggregate vertex budget rejects before appending"), VibeMesh::AppendTerrainBackdrop(VertexFull, Terrain, X0, Y0, 2, 3, [] { return false; }));
    TestTrue(TEXT("Vertex-budget failure preserves prior buffers and bounds"), VertexFull.Chunks.Num() == 1 && VertexFull.Chunks[0].Vertices.Num() == 2000000 && BoundsUnchanged(VertexFull));
    return true;
}
#endif



