#include "VibeTrace.h"
#include "coaster/track.hpp"
#include "PrimitiveSceneProxy.h"
#include "PrimitiveViewRelevance.h"
#include "MeshElementCollector.h"
#include "SceneManagement.h"
#include "SceneView.h"
namespace {
class FVibeTraceProxy final : public FPrimitiveSceneProxy {
    TArray<FVibeTraceSegment> Segments;

  public:
    explicit FVibeTraceProxy(const UVibeTrace *Component)
        : FPrimitiveSceneProxy(Component), Segments(Component->Segments) {
        bWillEverBeLit = false;
    }
    virtual SIZE_T GetTypeHash() const override {
        static int Type;
        return reinterpret_cast<SIZE_T>(&Type);
    }
    virtual uint32 GetMemoryFootprint() const override {
        return uint32(sizeof(*this) + Segments.GetAllocatedSize());
    }
    virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView *View) const override {
        FPrimitiveViewRelevance Result;
        Result.bDrawRelevance = IsShown(View);
        Result.bDynamicRelevance = true;
        Result.bNormalTranslucency = true;
        Result.bSeparateTranslucency = true;
        return Result;
    }
    virtual void GetDynamicMeshElements(const TArray<const FSceneView *> &Views, const FSceneViewFamily &,
                                        uint32 Visible, FMeshElementCollector &Collector) const override {
        for (int32 Index = 0; Index < Views.Num(); ++Index)
            if (Visible & (1u << Index)) {
                auto *Draw = Collector.GetPDI(Index);
                for (const auto &Segment : Segments)
                    Draw->DrawLine(GetLocalToWorld().TransformPosition(Segment.Start),
                                   GetLocalToWorld().TransformPosition(Segment.End), Segment.Color,
                                   SDPG_World, 2.f, 0.f, true);
            }
    }
};
} // namespace
UVibeTrace::UVibeTrace() {
    SetCollisionEnabled(ECollisionEnabled::NoCollision);
    bUseEditorCompositing = false;
    PrimaryComponentTick.bCanEverTick = false;
}
void UVibeTrace::SetTrack(const coaster::Track &Track) {
    Segments.Reset();
    const int Count = int(std::ceil(Track.length / 4));
    Segments.Reserve(Count);
    auto Prior = Track.at(0);
    auto Position = [](coaster::Vec3 P) { return FVector(P.x, P.y, P.z) * 100; };
    for (int I = 1; I <= Count; ++I) {
        const auto F = Track.at(Track.length * I / Count);
        Segments.Add({Position(Prior.p), Position(F.p),
                      Track.source[F.element].geometry.empty() ? FLinearColor(1.f, .43f, .08f)
                                                               : FLinearColor(.08f, .65f, 1.f)});
        Prior = F;
    }
    UpdateBounds();
    MarkRenderStateDirty();
}
void UVibeTrace::Clear() {
    Segments.Empty();
    MarkRenderStateDirty();
}
FPrimitiveSceneProxy *UVibeTrace::CreateSceneProxy() {
    return Segments.IsEmpty() ? nullptr : new FVibeTraceProxy(this);
}
FBoxSphereBounds UVibeTrace::CalcBounds(const FTransform &Transform) const {
    FBox Box(ForceInit);
    for (const auto &S : Segments) {
        Box += S.Start;
        Box += S.End;
    }
    if (!Box.IsValid)
        Box = FBox(FVector(-1), FVector(1));
    return FBoxSphereBounds(Box.ExpandBy(100)).TransformBy(Transform);
}
