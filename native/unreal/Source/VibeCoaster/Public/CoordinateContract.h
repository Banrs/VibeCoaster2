#pragma once
#include "coaster/coaster.hpp"

// The only coordinate boundary. Core: metres, RH (X/Y ground, Z up).
// Unreal: centimetres, LH (X forward, Y right, Z up). Reflect Y once.
namespace VibeCoordinates
{
struct Triple { double X, Y, Z; };
inline Triple Position(coaster::Vec3 P) { return {100 * P.x, -100 * P.y, 100 * P.z}; }
inline Triple Direction(coaster::Vec3 V) { return {V.x, -V.y, V.z}; }
inline coaster::Vec3 CorePosition(Triple P) { return {P.X / 100, -P.Y / 100, P.Z / 100}; }
struct Member { Triple Centre, Axis; double LengthCm; };
inline Member SteelMember(coaster::Vec3 From, coaster::Vec3 To)
{
    const auto Delta = To - From;
    return {Position((From + To) * .5), Direction(coaster::unit(Delta)), coaster::norm(Delta) * 100};
}
struct Box { Triple Forward, Up, Scale; };
inline Box StationBox(const coaster::StationBox& B)
{
    return {Direction(B.forward), Direction(B.up), {2*B.half.x, 2*B.half.y, 2*B.half.z}};
}
inline constexpr bool ReversesWinding = true;
}

