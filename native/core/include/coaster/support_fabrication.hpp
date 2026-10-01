#pragma once
#include "coaster/coaster.hpp"
#include "coaster/support_mesh.hpp"

namespace coaster {
enum class FabricationMaterial { Steel, Concrete, Fastener };
struct FabricationPart {
    std::vector<Vec3> vertices;
    std::vector<std::vector<uint32_t>> faces;
    FabricationMaterial material{FabricationMaterial::Steel};
    bool smooth{},spineSeat{},buried{};
    size_t owner{};int member{-1};
    std::string name;
    Vec3 axis{};
};
// Derived from the persisted graph and the saved section profile, never trusted
// from a mesh file. Used by native validation, Blender export and Unreal alike.
std::vector<FabricationPart> buildSupportFabrication(const Track&,const std::vector<Support>&,Cancel cancel={});
SupportMeshBuffer fabricationMesh(const FabricationPart&);
ValidationReport validateSupportFabrication(const Track&,const std::vector<Support>&,const Terrain&,
    const StationGeometry&,const ClearanceSweep&,Cancel cancel={});
FabricationPart exaCrosshead(const TrackSample&);
}
