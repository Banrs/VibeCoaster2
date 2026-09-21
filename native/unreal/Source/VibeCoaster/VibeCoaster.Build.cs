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
        string ReadGit(params string[] Arguments) {
            var Info = new ProcessStartInfo("git") {
                WorkingDirectory = Path.GetFullPath(Path.Combine(CoreRoot, "../..")),
                RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false, CreateNoWindow = true
            };
            foreach (string Argument in Arguments) Info.ArgumentList.Add(Argument);
            using (var Git = Process.Start(Info)) {
                string Result = Git.StandardOutput.ReadToEnd().Trim(); Git.WaitForExit();
                return Git.ExitCode == 0 ? Result : "";
            }
        }
        string Commit = ReadGit("rev-parse", "HEAD");
        if (Regex.IsMatch(Commit, "^[0-9a-f]{40}$")) {
            PublicDefinitions.Add("COASTER_BUILD_COMMIT=\"" + Commit + "\"");
            // UBT caches these definitions in its makefile. Source-only changes
            // must not retain an earlier commit, including in Git worktrees.
            string Reference = ReadGit("symbolic-ref", "-q", "HEAD");
            foreach (string Identity in new[] { "HEAD", "logs/HEAD", Reference, "packed-refs" }) {
                if (string.IsNullOrEmpty(Identity)) continue;
                string Metadata = ReadGit("rev-parse", "--path-format=absolute", "--git-path", Identity);
                if (System.IO.File.Exists(Metadata)) ExternalDependencies.Add(Metadata);
            }
        }
        PublicIncludePaths.Add(Path.Combine(CoreRoot, "include"));
        PrivateIncludePaths.Add(Path.Combine(CoreRoot, "src"));
        foreach (string File in Directory.GetFiles(CoreRoot, "*.cpp", SearchOption.AllDirectories))
            ExternalDependencies.Add(File);
        foreach (string File in Directory.GetFiles(CoreRoot, "*.hpp", SearchOption.AllDirectories))
            ExternalDependencies.Add(File);
    }
}
