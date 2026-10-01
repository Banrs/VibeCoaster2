"""Stream a native fabrication export into a small tube-only joint audit file."""
import json
import sys
from pathlib import Path

source = Path(sys.argv[1])
target = source.with_name(source.name.replace('-fabrication.json', '-joints.json'))
if target == source:
    raise SystemExit('Expected a -fabrication.json export')
decoder = json.JSONDecoder()
count = 0
with source.open(encoding='utf-8') as handle, target.open('w', encoding='utf-8') as output:
    buffer = handle.read(1024 * 1024)
    start = buffer.index('"parts":[') + len('"parts":[')
    buffer = buffer[start:]
    output.write('{"parts":[')
    while True:
        buffer = buffer.lstrip(' \r\n\t,')
        if buffer.startswith(']'):
            break
        try:
            part, end = decoder.raw_decode(buffer)
        except json.JSONDecodeError:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                raise RuntimeError('Truncated fabrication export')
            buffer += chunk
            continue
        buffer = buffer[end:]
        if part['name'] in ('Fitted tube', 'Welded node can'):
            if count:
                output.write(',')
            json.dump(part, output, separators=(',', ':'))
            count += 1
    output.write(']}\n')
print(f'{target}: {count} fitted tubes')
