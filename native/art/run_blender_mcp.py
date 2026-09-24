"""Send the checked-in authoring script through the installed Blender MCP.

Run with the Python environment that contains the MCP client package. Blender
must already be running with its loopback add-on enabled. No socket shortcut or
background Blender invocation is used for modelling.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
from datetime import timedelta

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True, help="Installed blender-mcp executable")
    parser.add_argument("--script", default="author_models.py")
    parser.add_argument("--output", default="exports")
    parser.add_argument("--context", help="Render one named support context within the MCP script-size limit")
    parser.add_argument("--station-context", help="Canonical StationBox JSON for render_station.py")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    output = (root / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    code = (root / args.script).read_text(encoding="utf-8")
    if "# ASSET_MODULES" in code:
        modules = [root / "train_models.py", root / "track_hardware.py", root / "station_models.py"]
        code = code.replace("# ASSET_MODULES", "\n".join(module.read_text(encoding="utf-8-sig") for module in modules))
    if "__SUPPORT_CONTEXT__" in code:
        context_path = root.parents[1] / "scratch" / "support-tubular-context.json"
        contexts = json.loads(context_path.read_text(encoding="utf-8-sig"))
        contexts = [context for context in contexts if context["name"].startswith("accepted-") and
                    (not args.context or context["name"] == args.context)]
        if not contexts:
            raise RuntimeError("No matching accepted support context")
        terrain_path = root.parents[1] / "scratch" / "support-tubular-terrain.json"
        terrain = json.loads(terrain_path.read_text(encoding="utf-8-sig"))
        for context in contexts:
            context["terrain"] = terrain[context["name"]]
        code = code.replace("__SUPPORT_CONTEXT__", repr(json.dumps(contexts,separators=(",",":"))))
    if "__STATION_CONTEXT__" in code:
        context_path = (Path(args.station_context).resolve() if args.station_context else
                        root.parents[1] / "scratch" / "station-resume" / "canonical-station-7.json")
        context = json.loads(context_path.read_text(encoding="utf-8-sig"))
        code = code.replace("__STATION_CONTEXT__", repr(json.dumps(context, separators=(",", ":"))))
    if "__SUPPORT_GEOMETRY__" in code:
        support_path = root.parents[1] / "scratch" / "v2-support-preview.json"
        code = code.replace("__SUPPORT_GEOMETRY__", repr(json.loads(support_path.read_text(encoding="utf-8-sig"))))
    code = code.replace("__EXPORT_ROOT__", output.as_posix())
    env = dict(os.environ)
    env.update(BLENDER_HOST="127.0.0.1", BLENDER_PORT="9876",
               BLENDER_MCP_SAFE_MODE="1", BLENDER_MCP_DISABLE_TELEMETRY="1",
               DISABLE_TELEMETRY="1", MCP_DISABLE_TELEMETRY="1")
    params = StdioServerParameters(command=args.server, env=env)
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            prompt = ("Render the authored functional station at the injected canonical StationBox poses, "
                      "including a labelled Blender exterior and circulation cutaway."
                      if args.script == "render_station.py" else
                      "Complete the requested coaster models: an original sleek high-speed train with a higher bonnet, and a believable premium station with covered queue and merge, aligned holding lanes, dispatch, separate unload and exit, and step-free access. Use the reviewed canonical track, train and station dimensions.")
            result = await session.call_tool(
                "execute_blender_code",
                {"code": code, "user_prompt": prompt},
                read_timeout_seconds=timedelta(seconds=180),
            )
            messages = "\n".join(item.text for item in result.content if item.type == "text")
            print(messages)
            if result.isError or not messages.startswith("Code executed successfully:"):
                raise RuntimeError("Blender MCP did not complete the authoring script")
            for line in messages.removeprefix("Code executed successfully:").lstrip().splitlines():
                if line.startswith("ASSET_MANIFEST="):
                    (output / "manifest.json").write_text(
                        json.dumps(json.loads(line[len("ASSET_MANIFEST="):]), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    asyncio.run(main())
