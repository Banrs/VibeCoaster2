#include "VibeMesh.h"
#include "coaster/vehicle.hpp"
using coaster::Vec3;
FVector VibePosition(Vec3 V) {
    return FVector(V.x, V.y, V.z) * 100;
}
namespace {
void Triangle(FVibeMesh &M, Vec3 A, Vec3 B, Vec3 C, FLinearColor Color) {
    const int32 N = M.Vertices.Num();
    const FVector Normal = FVector::CrossProduct(VibePosition(B - A), VibePosition(C - A)).GetSafeNormal();
    for (auto P : {A, B, C}) {
        M.Vertices.Add(VibePosition(P));
        M.Normals.Add(Normal);
        M.Colors.Add(Color);
        M.UV.Add(FVector2D(P.x * .1, P.y * .1));
    }
    M.Indices.Append({N, N + 2, N + 1});
}
void Quad(FVibeMesh &M, Vec3 A, Vec3 B, Vec3 C, Vec3 D, FLinearColor Color) {
    Triangle(M, A, B, C, Color);
    Triangle(M, A, C, D, Color);
}
void Beam(FVibeMesh &M, Vec3 A, Vec3 B, double Radius, FLinearColor Color, int Sides = 6) {
    const Vec3 T = coaster::unit(B - A),
               R = coaster::unit(coaster::cross(T, std::abs(T.z) < .9 ? Vec3{0, 0, 1} : Vec3{0, 1, 0})),
               U = coaster::cross(R, T);
    for (int I = 0; I < Sides; ++I) {
        const double P = 2 * coaster::pi * I / Sides, Q = 2 * coaster::pi * (I + 1) / Sides;
        const Vec3 X = R * (Radius * std::cos(P)) + U * (Radius * std::sin(P)),
                   Y = R * (Radius * std::cos(Q)) + U * (Radius * std::sin(Q));
        Quad(M, A + X, B + X, B + Y, A + Y, Color);
    }
}
void Box(FVibeMesh &M, Vec3 C, Vec3 T, Vec3 R, Vec3 U, Vec3 H, FLinearColor Color) {
    std::array<Vec3, 8> P;
    for (int I = 0; I < 8; ++I)
        P[I] = C + T * ((I & 1) ? H.x : -H.x) + R * ((I & 2) ? H.y : -H.y) + U * ((I & 4) ? H.z : -H.z);
    for (auto Q : std::array<std::array<int, 4>, 6>{
             {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}}})
        Quad(M, P[Q[0]], P[Q[1]], P[Q[2]], P[Q[3]], Color);
}
void Solid(FVibeMesh &mesh, const coaster::ClearanceObstacle &shape, bool cylinder, FLinearColor color) {
    if (cylinder)
        Beam(mesh, shape.center - shape.axes[0] * shape.half[0], shape.center + shape.axes[0] * shape.half[0],
             shape.half[1], color);
    else
        Box(mesh, shape.center, shape.axes[0], shape.axes[1], shape.axes[2],
            {shape.half[0], shape.half[1], shape.half[2]}, color);
}
} // namespace
FVibeMeshes BuildVibeMeshes(const coaster::Design &D, const coaster::Cancel &Cancel) {
    FVibeMeshes M;
    const FLinearColor Rail(.92f, .23f, .075f), Tie(.2f, .22f, .24f), Support(.72f, .73f, .7f);
    const auto &Track = D.track;
    const int Count = int(std::ceil(Track.length / 1.5));
    auto Prior = Track.at(0);
    for (int I = 1; I <= Count; ++I) {
        coaster::poll(Cancel);
        const auto F = Track.at(Track.length * I / Count);
        for (double Side : {-.65, .65})
            Beam(M.Rails, Prior.p + Prior.r * Side, F.p + F.r * Side, .105, Rail);
        Beam(M.Rails, Prior.p - Prior.u * .58, F.p - F.u * .58, .29, Rail);
        if (I % 2 == 0) {
            Beam(M.Ties, F.p - F.r * .7 - F.u * .15, F.p + F.r * .7 - F.u * .15, .065, Tie);
            Beam(M.Ties, F.p - F.u * .55, F.p - F.r * .58 - F.u * .1, .065, Tie);
            Beam(M.Ties, F.p - F.u * .55, F.p + F.r * .58 - F.u * .1, .065, Tie);
        }
        Prior = F;
    }
    if (!D.validation)
        throw std::runtime_error("Scene geometry requires fresh validation");
    for (const auto &part : D.validation->scene.parts) {
        coaster::poll(Cancel);
        const bool structural = part.material == coaster::SceneMaterial::Support;
        const FLinearColor color =
            structural                                          ? Support
            : part.material == coaster::SceneMaterial::Platform ? FLinearColor(.48f, .47f, .43f)
            : part.material == coaster::SceneMaterial::Roof     ? FLinearColor(.16f, .2f, .22f)
                                                                : Tie;
        Solid(structural ? M.Supports : M.Station, part.shape, part.cylinder, color);
    }
    // Shared indexed terrain: the clearance proof uses these exact vertices
    // and triangle diagonals; analytic normals only smooth its shading.
    const auto &Xs = coaster::terrainXCoordinates();
    const auto &Ys = coaster::terrainYCoordinates();
    const int NX = int(Xs.size()), NY = int(Ys.size()), Vertices = NX * NY;
    M.Terrain.Vertices.Reserve(Vertices);
    M.Terrain.Normals.Reserve(Vertices);
    M.Terrain.Colors.Reserve(Vertices);
    M.Terrain.UV.Reserve(Vertices);
    M.Terrain.Indices.Reserve((NX - 1) * (NY - 1) * 6);
    for (double X : Xs)
        for (double Y : Ys) {
            coaster::poll(Cancel);
            M.Terrain.Vertices.Add(VibePosition({X, Y, coaster::siteElevation(X, Y, D.recipe.plateau)}));
            const Vec3 N = coaster::terrainShadingNormal(X, Y, D.recipe.plateau);
            M.Terrain.Normals.Add(FVector(N.x, N.y, N.z));
            M.Terrain.UV.Add(FVector2D(X * .1, Y * .1));
            const float Shade = float(.92 + .05 * std::sin(X * .035) * std::cos(Y * .023));
            M.Terrain.Colors.Add(FLinearColor(.39f * Shade, .3f * Shade, .19f * Shade));
        }
    for (int X = 0; X < NX - 1; ++X)
        for (int Y = 0; Y < NY - 1; ++Y) {
            const int A = X * NY + Y, B = (X + 1) * NY + Y, C = B + 1, D0 = A + 1;
            M.Terrain.Indices.Append({A, C, B, A, D0, C});
        }
    return M;
}
FVibeMesh BuildVibeCar() {
    FVibeMesh mesh;
    for (const auto &part : coaster::vehicleGeometry().parts) {
        const auto &shape = part.shape;
        const FLinearColor color =
            part.material == coaster::VehicleMaterial::Shell  ? FLinearColor(.10f, .18f, .22f)
            : part.material == coaster::VehicleMaterial::Seat ? FLinearColor(.055f, .075f, .09f)
                                                              : FLinearColor(.67f, .68f, .65f);
        Solid(mesh, shape, part.cylinder, color);
    }
    return mesh;
}
