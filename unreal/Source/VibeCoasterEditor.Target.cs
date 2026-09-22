using UnrealBuildTool;
public class VibeCoasterEditorTarget : TargetRules {
    public VibeCoasterEditorTarget(TargetInfo Target) : base(Target) {
        Type = TargetType.Editor; DefaultBuildSettings = BuildSettingsVersion.V7;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_8;
        ExtraModuleNames.Add("VibeCoaster");
    }
}
