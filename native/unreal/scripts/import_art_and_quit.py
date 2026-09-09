"""Full-editor startup import; retain the importer's receipt and all strict checks."""
from pathlib import Path
import runpy
import unreal

try:
    runpy.run_path(str(Path(__file__).with_name("import_art_v072.py")), run_name="__main__")
finally:
    unreal.SystemLibrary.quit_editor()
