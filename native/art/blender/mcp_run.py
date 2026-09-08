"""Local, project-bounded client for the installed Blender MCP stdio server."""
import argparse, asyncio, base64, datetime, json, os
from pathlib import Path
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
from blender_mcp.safe_mode import validate_code

ROOT = Path(r"D:/Coding/Codex/Vibecoasterjs/native/art").resolve()
REG = Path(r"D:/Toolchains/BlenderMCP/mcp-registration.json")

def bounded(value):
    path = Path(value).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError("Modelling inputs and outputs must remain in native/art")
    return path

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["info", "run", "screenshot", "status"])
    parser.add_argument("--script")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = bounded(args.output)
    if output.exists():
        raise ValueError("Use a fresh result path; prior modelling evidence is preserved")
    output.parent.mkdir(parents=True, exist_ok=True)
    reg = json.loads(REG.read_text())
    env = dict(os.environ); env.update(reg["env"])
    if env.get("BLENDER_MCP_SAFE_MODE") != "1" or env.get("BLENDER_HOST") != "127.0.0.1":
        raise ValueError("Expected the verified local safe-mode Blender configuration")
    tool = {"info":"get_scene_info", "run":"execute_blender_code", "screenshot":"get_viewport_screenshot", "status":"get_addon_status"}[args.action]
    if tool not in reg["recommended_enabled_tools"]:
        raise ValueError("Tool outside local modelling allowlist")
    payload = {"user_prompt":"Author and inspect original coaster assets locally inside the project; no external services."}
    if args.action == "run":
        code = bounded(args.script).read_text(encoding="utf-8-sig")
        validate_code(code)
        payload["code"] = code
    params = StdioServerParameters(command=reg["command"], args=reg["args"], env=env)
    async with stdio_client(params) as (reader, writer):
        async with ClientSession(reader, writer, read_timeout_seconds=datetime.timedelta(seconds=180)) as session:
            await session.initialize()
            result = await session.call_tool(tool, payload)
            data = result.model_dump(mode="json")
            for index, item in enumerate(data.get("content", [])):
                if item.get("type") == "image":
                    ext = ".png" if item.get("mimeType") == "image/png" else ".jpg"
                    image = output.with_name(output.stem+"-"+str(index)+ext)
                    if image.exists(): raise ValueError("Image evidence path already exists")
                    image.write_bytes(base64.b64decode(item.pop("data")))
                    item["saved_image"] = str(image)
            output.write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")
            print(json.dumps(data, ensure_ascii=False)[:6000])
            if result.isError or any(item.get("type") == "text" and item.get("text", "").lstrip().startswith("Error ") for item in data.get("content", [])):
                raise RuntimeError("Blender MCP reported a tool error; see result file")

if __name__ == "__main__":
    asyncio.run(main())
