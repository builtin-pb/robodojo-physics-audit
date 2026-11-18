"""One sheet per camera smoke: rows are the replayed states, columns the cameras (PIL; run in the 6.2 container).

usage: python smoke_sheet.py <smoke run dir> <out.jpg> [thumb width]
Each cell is the camera's frame of that state, labelled with the camera name and the state's source and episode step.
"""
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

run, out = sys.argv[1], sys.argv[2]
tw = int(sys.argv[3]) if len(sys.argv) > 3 else 320
ix = json.load(open(os.path.join(run, "frames", "index.json")))
cams = [c["name"] for c in ix["cameras"]]
rows = [f for f in ix["frames"] if f.get("replayed_from") is not None]
w0, h0 = ix["cameras"][0]["res"]
th = int(h0 * tw / w0)
fnt = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16) if os.path.exists(
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf") else ImageFont.load_default()
sheet = Image.new("RGB", (tw * len(cams), th * len(rows)), (255, 255, 255))
d = ImageDraw.Draw(sheet, "RGBA")
for r, fr in enumerate(rows):
    for c, cam in enumerate(cams):
        im = Image.open(os.path.join(run, "frames", cam, f"f_{fr['i']:05d}.jpg")).convert("RGB").resize((tw, th), Image.LANCZOS)
        sheet.paste(im, (c * tw, r * th))
        label = f"{cam}  {fr.get('src', '')} step {fr.get('phys')}"
        d.rectangle([c * tw, r * th, c * tw + d.textlength(label, font=fnt) + 12, r * th + 24], fill=(0, 0, 0, 150))
        d.text((c * tw + 6, r * th + 3), label, font=fnt, fill=(255, 255, 255, 255))
sheet.save(out, quality=85)
print(out, sheet.size)
