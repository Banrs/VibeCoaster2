#include "CoreMinimal.h"
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#endif
// Canonical persistence includes Windows.h for atomic Unicode save replacement.
// Load it through Unreal's platform wrapper before that guarded standard include.
#include "persistence.cpp"
