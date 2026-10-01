"""Capture the saved Exa review map from its real Unreal cameras."""
import json
import time
from pathlib import Path
import unreal

repo = Path(__file__).resolve().parents[3]
output = repo/"out/track-study"
level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
if not level.load_level("/Game/Art/TrackStudy/Maps/ExaTrackStudy"):
    raise RuntimeError("Exa study map missing; run import_track_study.py first")
all_actors = actors.get_all_level_actors()
cameras = {actor.get_actor_label().rsplit(" / ", 1)[-1]: actor
           for actor in all_actors if isinstance(actor, unreal.CameraActor)}
queue = ["Hero", "Banked", "Inverted", "Rider", "Overview"]
world = unreal.EditorLevelLibrary.get_editor_world()
unreal.SystemLibrary.execute_console_command(world, "r.MotionBlurQuality 0")
unreal.SystemLibrary.execute_console_command(world, "r.EyeAdaptationQuality 0")
state = {"ready": time.monotonic()+12, "deadline": time.monotonic()+180,
         "task": None, "name": None, "captures": [], "busy": False}


def capture_tick():
    now = time.monotonic()
    if now > state["deadline"]:
        unreal.log_error("EXA_CAPTURE_TIMEOUT")
        unreal.unregister_slate_post_tick_callback(handle)
        unreal.SystemLibrary.quit_editor()
        return
    if now < state["ready"]:
        return
    task = state["task"]
    if task is not None:
        if not task.is_task_done():
            return
        path = output/("unreal-"+state["name"].lower()+".png")
        if not path.is_file():
            raise RuntimeError("Screenshot task ended without its output: "+str(path))
        state["captures"].append(str(path))
        state["task"] = None
        state["ready"] = now+2
        return
    if not queue:
        if len(state["captures"]) != 5:
            raise RuntimeError("Incomplete camera capture sequence")
        (output/"unreal-captures.json").write_text(json.dumps({"captures": state["captures"]}, indent=2)+"\n")
        unreal.log("EXA_TRACK_STUDY_CAPTURE_OK")
        unreal.unregister_slate_post_tick_callback(handle)
        unreal.SystemLibrary.quit_editor()
        return
    name = queue.pop(0)
    # Keep other examples out of each close inspection view.
    for actor in all_actors:
        label = actor.get_actor_label()
        if label in ("Exa / straight", "Exa / banked", "Exa / inverted"):
            visible = name == "Overview" or label == "Exa / "+{
                "Hero": "straight", "Banked": "banked", "Inverted": "inverted", "Rider": "banked"}.get(name, "")
            actor.set_is_temporarily_hidden_in_editor(not visible)
        elif label.startswith(("Exa / removable wheel fit", "Exa / short review column", "Exa / review stand", "Exa / support head")):
            actor.set_is_temporarily_hidden_in_editor(name not in ("Hero", "Overview"))
    state["name"] = name
    task = unreal.AutomationLibrary.take_high_res_screenshot(1600, 1000,
        str(output/("unreal-"+name.lower()+".png")), camera=cameras[name], delay=2.0)
    if not task.is_valid_task():
        raise RuntimeError("Editor screenshot task could not be initialized")
    state["task"] = task


def tick(delta):
    # Screenshot preparation can pump Slate while this callback is running.
    # Hold a guard until the returned task has been assigned to the state.
    if state["busy"]:
        return
    state["busy"] = True
    try:
        capture_tick()
    except Exception as error:
        unreal.log_error("EXA_CAPTURE_FAILED: "+str(error))
        unreal.unregister_slate_post_tick_callback(handle)
        unreal.SystemLibrary.quit_editor()
    finally:
        state["busy"] = False


unreal.EditorPythonScripting.set_keep_python_script_alive(True)
handle = unreal.register_slate_post_tick_callback(tick)
