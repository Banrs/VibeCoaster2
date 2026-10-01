"""Send the checked-in authoring script through the installed Blender MCP.

Run with the Python environment that contains the MCP client package. Blender
must already be running with its loopback add-on enabled. No socket shortcut or
background Blender invocation is used for modelling.
"""
import argparse
import asyncio
import json
import os
import tomllib
from pathlib import Path
from datetime import timedelta

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", help="MCP executable override; defaults to Codex's configured Blender server")
    parser.add_argument("--script", default="author_models.py")
    parser.add_argument("--output", default="exports")
    parser.add_argument("--prompt", help="Description of the requested Blender operation")
    parser.add_argument("--station-context", help="Canonical StationBox JSON for render_station.py")
    parser.add_argument("--support-context", help="Canonical support_review JSON for author_support_study.py")
    parser.add_argument("--coarse", action="store_true", help="Review support form without fabrication hardware")
    parser.add_argument("--prepare-only", help="Write the request for the directly connected MCP tool; do not start a second server")
    parser.add_argument("--defer-render", action="store_true", help="Save the authored scene and render its cameras in separate MCP calls")
    parser.add_argument("--study-profile", default="track_study_profile.json",
                        help="Track study dimensions, relative to native/art")
    parser.add_argument("--view", default="hero", choices=("hero", "joint", "section", "banked", "inverted", "overview", "rider"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    output = (root / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    code = (root / args.script).read_text(encoding="utf-8")
    if "# ASSET_MODULES" in code:
        modules = [root / "track_hardware.py", root / "station_models.py"]
        code = code.replace("# ASSET_MODULES", "\n".join(module.read_text(encoding="utf-8-sig") for module in modules))
    if "# TRACK_STUDY_GEOMETRY" in code:
        code = code.replace("# TRACK_STUDY_GEOMETRY",
                            (root / "track_study_geometry.py").read_text(encoding="utf-8"))
    if "# SUPPORT_DETAIL_GEOMETRY" in code:
        detail_code = (root / "support_detail_geometry.py").read_text(encoding="utf-8")
        # Its shared dependency is already embedded above. The MCP receives a
        # self-contained script with explicit function calls and stdlib imports.
        detail_code = detail_code.replace("from track_study_geometry import support_head_mesh, support_bearing_parts", "")
        code = code.replace("# SUPPORT_DETAIL_GEOMETRY", detail_code)
    if "__TRACK_PROFILE__" in code:
        profile = json.loads((root / args.study_profile).read_text(encoding="utf-8"))
        code = code.replace("__TRACK_PROFILE__", repr(json.dumps(profile)))
    if "__STATION_CONTEXT__" in code:
        if not args.station_context:
            parser.error("--station-context is required for render_station.py")
        context_path = Path(args.station_context).resolve()
        context = json.loads(context_path.read_text(encoding="utf-8-sig"))
        code = code.replace("__STATION_CONTEXT__", repr(json.dumps(context, separators=(",", ":"))))
    code = code.replace("__EXPORT_ROOT__", output.as_posix())
    code = code.replace("__STUDY_VIEW__", args.view)
    code = code.replace("__SUPPORT_COARSE__", repr(args.coarse))
    code = code.replace("__SUPPORT_DEFER_RENDER__", repr(args.defer_render))
    support_payload = None
    context_name = ".VC2 Support Review Data"
    if "__SUPPORT_CONTEXT__" in code:
        if not args.support_context:
            parser.error("--support-context is required for author_support_study.py")
        support = json.loads(Path(args.support_context).read_text(encoding="utf-8"))
        # Upload plain JSON through bounded MCP calls into a collection property.
        # This keeps the authoring script small without decimating the track or
        # rounding the canonical joint and footing coordinates. A collection
        # property is data only; executable Blender text blocks are not used.
        support_payload = json.dumps(support, separators=(",", ":"))
        code = code.replace("__SUPPORT_CONTEXT__", "bpy.data.collections["+repr(context_name)+"][\"json\"]")
    config_path = Path(os.environ.get("CODEX_HOME", str(Path.home()/".codex")))/"config.toml"
    if args.prepare_only:
        Path(args.prepare_only).write_text(json.dumps({'code':code,'data':support_payload,
            'collection':context_name,'user_prompt':args.prompt or 'Please continue modelling.'}),encoding='utf-8')
        print('Prepared request for the connected Blender MCP')
        return
    configured = {}
    if config_path.is_file():
        configured = tomllib.loads(config_path.read_text(encoding="utf-8")).get("mcp_servers", {}).get("blender", {})
    server = args.server or configured.get("command")
    if not server:
        parser.error("Configure a Blender stdio MCP in Codex or supply --server")
    use_config = not args.server or Path(args.server) == Path(configured.get("command", ""))
    env = dict(os.environ)
    if use_config:
        env.update(configured.get("env", {}))
    env.update(BLENDER_HOST="127.0.0.1", BLENDER_PORT="9876",
               BLENDER_MCP_SAFE_MODE="1", BLENDER_MCP_DISABLE_TELEMETRY="1",
               DISABLE_TELEMETRY="1", MCP_DISABLE_TELEMETRY="1")
    params = StdioServerParameters(command=server, args=configured.get("args", []) if use_config else [], env=env)
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            if support_payload is not None:
                upload = "import bpy\nvc2_data=bpy.data.collections.get("+repr(context_name)+") or bpy.data.collections.new("+repr(context_name)+")\nvc2_data.use_fake_user=True\nvc2_data['json']=''"
                for start in range(0, len(support_payload), 80000):
                    chunk = support_payload[start:start+80000]
                    stage_code = (upload+"\n" if start == 0 else "import bpy\n")+"bpy.data.collections["+repr(context_name)+"][\"json\"] += "+repr(chunk)
                    staged = await session.call_tool("execute_blender_code", {"code": stage_code,
                        "user_prompt": args.prompt or "Please continue modelling."})
                    status = "\n".join(item.text for item in staged.content if item.type == "text")
                    if staged.isError or not status.startswith("Code executed successfully:"):
                        raise RuntimeError("Blender model-data upload failed: "+status[:500])
            prompt = args.prompt or ("Render the authored functional station at the injected canonical StationBox poses, "
                      "including a labelled Blender exterior and circulation cutaway."
                      if args.script == "render_station.py" else
                      "Author the retained station and track hardware at the canonical runtime pivots. Train art is absent pending a new design.")
            result = await session.call_tool(
                "execute_blender_code",
                {"code": code, "user_prompt": prompt},
                read_timeout_seconds=timedelta(seconds=600),
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
