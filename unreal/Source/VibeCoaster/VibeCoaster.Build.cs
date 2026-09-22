using UnrealBuildTool;
using System.IO;
public class VibeCoaster : ModuleRules {
    public VibeCoaster(ReadOnlyTargetRules Target) : base(Target) {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp20; bEnableExceptions = true; bUseUnity = false; FPSemantics = FPSemanticsMode.Precise;
        PublicDependencyModuleNames.AddRange(new[] {"Core", "CoreUObject", "Engine", "InputCore", "Slate", "SlateCore", "RenderCore", "RHI", "ProceduralMeshComponent", "Projects"});
        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../core/include")));
    }
}
