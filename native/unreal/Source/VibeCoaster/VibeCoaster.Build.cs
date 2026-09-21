using UnrealBuildTool;
using System.IO;
using System.Diagnostics;
using System.Text.RegularExpressions;
public class VibeCoaster : ModuleRules
{
    public VibeCoaster(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.NoPCHs;
        bUseUnity = false; // Keep portable core .cpp anonymous/static symbols isolated.
        bEnableExceptions = true; // Core validation/serialization throws; this is module-local.
        CppStandard = CppStandardVersion.Cpp20;
        FPSemantics = FPSemanticsMode.Precise; // Preserve finite/NaN checks; no fast-math or fused Clang contractions.
        PublicDependencyModuleNames.AddRange(new[] {"Core", "CoreUObject", "Engine", "InputCore", "ProceduralMeshComponent"});
        string CoreRoot = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../core"));
        using (var Git = Process.Start(new ProcessStartInfo("git", "rev-parse HEAD") {
            WorkingDirectory = Path.GetFullPath(Path.Combine(CoreRoot, "../..")),
            RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false, CreateNoWindow = true
        })) {
            string Commit = Git.StandardOutput.ReadToEnd().Trim(); Git.WaitForExit();
            if (Git.ExitCode == 0 && Regex.IsMatch(Commit, "^[0-9a-f]{40}$"))
                PublicDefinitions.Add("COASTER_BUILD_COMMIT=\"" + Commit + "\"");
        }
        PublicIncludePaths.Add(Path.Combine(CoreRoot, "include"));
        PrivateIncludePaths.Add(Path.Combine(CoreRoot, "src"));
        foreach (string File in Directory.GetFiles(CoreRoot, "*.cpp", SearchOption.AllDirectories))
            ExternalDependencies.Add(File);
        foreach (string File in Directory.GetFiles(CoreRoot, "*.hpp", SearchOption.AllDirectories))
            ExternalDependencies.Add(File);
    }
}
