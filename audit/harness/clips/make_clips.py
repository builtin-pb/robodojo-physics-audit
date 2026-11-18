"""Compose the blog clips from clip_run.py frame folders: overlays, H.264 MP4 (under 5 MB), stills, contact sheet.

usage (6.2 container python, PIL and imageio-ffmpeg on the path): python make_clips.py <clips.json> <runs_root> <out_dir> [ids]
clips.json: [{"id", "title", "segments": [...], "end_card": {...}, "still": bool}], each segment:
  {"runs": [run tag, ...] (several = side by side; a tag may repeat), "cam": name, "select": {"why": [...], "case": NAME,
   "from": i, "to": j, "key": "action"|"phys"|"case_action"|"case_phys"|"i", "every": n} or "selects": [one per run],
   "repeat": r (frames per sample: slow motion), "labels": [template under each panel], "top": template, "bottom": template}
Templates use {field} from the frame record (action, phys, case_action, case_phys, stages_left, score_tiers_done, ...)
and derived fields: {step} (phys minus the segment's "step_offset"), {t_ms} (case_phys, else {step}, times 4 ms), {count0}/{count1}/{count2} (press counters),
{tilt_<label>} (body z-axis from vertical, change from the first selected frame, degrees), {rot_<label>} (total rotation,
degrees), {dz_<label>}, {dx_<label>} (mm from the first selected frame), {y_<label>} (mm); "crop": [x0 y0 x1 y1] per segment.
Round 3 options, all opt-in: {t_s} (t_ms in seconds, two decimals), {dz_max} (largest dz over rigid objects, mm);
"style": {"font": px, "alpha": 0-255} (smaller, lighter bands, no title band unless "title"); "strikes_from": n
({strikes} = (n + 1 - stages_left) // 2: a strike and a lift per key); "action_log": {"run": tag} (the 5.1 run's
result.json, policy_log.exploit.actions keyed by action: {head_mm} = the witness's head_bottom_above_xylophone_top_mm,
{xyl_contacts} = cumulative xylophone contact points with separation <= 0 in the witness's contact log up to that action);
"hold_last_s" with "hold_bottom" (bottom text while the last frame is held; "\\n" separates lines); "max_s" (fail if longer); "webp":
{"width", "every", "quality"} (animated WebP, falls back to GIF without WebP support); "poster": {"frame": k} (PNG of
composed frame k, before encoding); per segment "ref": a select whose first frame is the reference for dz/dx/tilt,
and "layout": "v" (several runs stacked as full-width panels, labels at each panel's top left) instead of side by side.
Fingers clip options, all opt-in: "width" (composed width in px, default 1280; side-by-side panels are width / n wide),
"label_pos": "top" (side-by-side panel labels in a band at each panel's top instead of its bottom), "note" (one small
line at the composed frame's bottom right on a translucent box, in every frame including the held last one),
"sheet": {"frames": [composed frame k, ...], "labels": [...], "width": px} (the contact sheet stacks these composed frames
in one column, each under its label, instead of the default four thumbnails per clip in a row).
"""
import json
import math
import os
import shutil
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

FPS = 25
W_OUT = 1280


def font(size):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


F_TITLE, F_TEXT, F_SMALL = font(30), font(26), font(20)
STYLE = {"font": F_TEXT, "alpha": 165}
OPTS = {"w_out": W_OUT, "label_pos": "bottom", "note": None}  # per clip: "width", "label_pos", "note"
ACTLOG = {}  # clip "action_log": action -> the witness's own per-action record
STRIKES = {}


def qang(a, b):
    na, nb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
    return math.degrees(2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(a, b))) / (na * nb))))


BOXES = {}  # clips.json "boxes": {field: {"prefix": label prefix, "min": [x y z], "max": [x y z]}} -> count of objects inside


def fields(rec, first, step_offset=0):
    f = {k: v for k, v in rec.items() if isinstance(v, (int, float, str, bool)) or v is None}
    if rec.get("phys") is not None:
        f["step"] = rec["phys"] - step_offset  # e.g. physics steps since a push that follows settle steps
    for name, b in BOXES.items():
        f[name] = sum(1 for lab, v in rec.get("obj", {}).items() if lab.startswith(b["prefix"])
                      and all(lo <= x <= hi for x, lo, hi in zip(v[:3], b["min"], b["max"])))
    n = rec.get("case_phys") if rec.get("case_phys") is not None else (rec.get("phys", 0) or 0) - step_offset
    f["t_ms"] = int(round((n or 0) * 4))
    f["t_s"] = f"{(n or 0) * 0.004:.2f}"
    if STRIKES.get("from") is not None and rec.get("stages_left") is not None:
        f["strikes"] = (STRIKES["from"] + 1 - rec["stages_left"]) // 2
    if ACTLOG and rec.get("action") is not None:
        a = ACTLOG.get(rec["action"]) or {}
        hm = a.get("head_bottom_above_xylophone_top_mm")
        f["head_mm"] = f"{hm:.0f}" if hm is not None else "?"
        f["xyl_contacts"] = sum(sum(v.get("n_sep_le0", 0) for k, v in (r.get("contacts") or {}).items() if "xylophone" in k)
                                for b, r in ACTLOG.items() if b <= rec["action"])
    for i, q in enumerate(rec.get("queries") or []):
        f[f"count{i}"] = q[1]
        f[f"aim{i}"] = q[0]
    for lab, v in rec.get("obj", {}).items():
        v0 = first.get("obj", {}).get(lab, v)
        f[f"tilt_{lab}"] = round(abs(up_tilt(v[3:7]) - up_tilt(v0[3:7])), 1)  # body z-axis from vertical, yaw excluded
        f[f"rot_{lab}"] = round(qang(v[3:7], v0[3:7]), 1)
        f[f"dz_{lab}"] = round((v[2] - v0[2]) * 1000, 1)
        f[f"dx_{lab}"] = round((v[0] - v0[0]) * 1000, 1)
        f[f"y_{lab}"] = round(v[1] * 1000, 1)
    dzs = [(v[2] - first.get("obj", {}).get(lab, v)[2]) * 1000 for lab, v in rec.get("obj", {}).items()]
    f["dz_max"] = f"{max(dzs):.1f}" if dzs else "?"
    for lab, q in rec.get("art", {}).items():
        f[f"q_{lab}_mm"] = round(q[0] * 1000, 2) if q else None
    return f


def up_tilt(q):
    w, x, y, z = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


class SafeDict(dict):
    def __missing__(self, k):
        return "?"


def select(frames, sel):
    key = sel.get("key", "i")
    out = []
    for r in frames:
        if sel.get("why") and not any(r["why"].startswith(w) for w in sel["why"]):
            continue
        if "case" in sel and r.get("case") != sel["case"]:
            continue
        v = r.get(key)
        if v is None or v < sel.get("from", -1e18) or v > sel.get("to", 1e18):
            continue
        out.append(r)
    every = int(sel.get("every", 1))
    return out[::every]


def band(img, lines, top, fnt):
    if not lines:
        return
    d = ImageDraw.Draw(img, "RGBA")
    pad, lh = (10, fnt.size + 8) if fnt.size > 22 else (6, fnt.size + 6)
    h = pad * 2 + lh * len(lines)
    y0 = 0 if top else img.height - h
    d.rectangle([0, y0, img.width, y0 + h], fill=(0, 0, 0, STYLE["alpha"]))
    for k, ln in enumerate(lines):
        d.text((16, y0 + pad + k * lh), ln, font=fnt, fill=(255, 255, 255, 255))


def compose(paths, labels, top_lines, bottom_lines, crop=None, layout="h"):
    W_OUT = OPTS["w_out"]
    ims = [Image.open(p).convert("RGB") for p in paths]
    if crop:
        ims = [im.crop(tuple(crop)) for im in ims]
    if len(ims) > 1 and layout == "v":  # stacked full-width panels, each labelled at its top left
        ims = [im.resize((W_OUT, int(im.height * W_OUT / im.width)), Image.LANCZOS) for im in ims]
        canvas = Image.new("RGB", (W_OUT, sum(im.height for im in ims)))
        y = 0
        for k, im in enumerate(ims):
            canvas.paste(im, (0, y))
            d = ImageDraw.Draw(canvas, "RGBA")
            if labels:
                fnt = STYLE["font"]
                w = d.textlength(labels[k], font=fnt) + 24
                d.rectangle([0, y, w, y + fnt.size + 14], fill=(0, 0, 0, STYLE["alpha"]))
                d.text((12, y + 6), labels[k], font=fnt, fill=(255, 255, 255, 255))
            if k:
                d.line([(0, y), (W_OUT, y)], fill=(255, 255, 255), width=3)
            y += im.height
        img = canvas
    elif len(ims) > 1:
        w = W_OUT // len(ims)
        ims = [im.resize((w, int(im.height * w / im.width)), Image.LANCZOS) for im in ims]
        canvas = Image.new("RGB", (w * len(ims), ims[0].height))
        for k, im in enumerate(ims):
            canvas.paste(im, (k * w, 0))
            if labels:
                d = ImageDraw.Draw(canvas, "RGBA")
                lh = STYLE["font"].size + 18
                y0 = 0 if OPTS["label_pos"] == "top" else ims[0].height - lh
                d.rectangle([k * w, y0, (k + 1) * w, y0 + lh], fill=(0, 0, 0, STYLE["alpha"]))
                d.text((k * w + 12, y0 + 6), labels[k], font=STYLE["font"], fill=(255, 255, 255, 255))
            if k:
                ImageDraw.Draw(canvas).line([(k * w, 0), (k * w, ims[0].height)], fill=(255, 255, 255), width=3)
        img = canvas
    else:
        img = ims[0]
        if img.width != W_OUT:
            img = img.resize((W_OUT, int(img.height * W_OUT / img.width)), Image.LANCZOS)
    band(img, top_lines, True, STYLE["font"])
    band(img, bottom_lines, False, STYLE["font"])
    if OPTS["note"]:
        d = ImageDraw.Draw(img, "RGBA")
        fnt = F_SMALL if STYLE["font"].size > F_SMALL.size else STYLE["font"]
        tw = d.textlength(OPTS["note"], font=fnt)
        x0, y0 = img.width - tw - 20, img.height - fnt.size - 16
        d.rectangle([x0 - 8, y0 - 4, img.width, img.height], fill=(0, 0, 0, STYLE["alpha"]))
        d.text((x0, y0), OPTS["note"], font=fnt, fill=(255, 255, 255, 255))
    return img


def card(size, lines, sub=None):
    img = Image.new("RGB", size, (16, 16, 20))
    d = ImageDraw.Draw(img)
    y = size[1] // 2 - (len(lines) * 44 + (len(sub or []) * 32)) // 2
    for ln in lines:
        w = d.textlength(ln, font=F_TITLE)
        d.text(((size[0] - w) / 2, y), ln, font=F_TITLE, fill=(255, 255, 255))
        y += 44
    for ln in sub or []:
        w = d.textlength(ln, font=F_SMALL)
        d.text(((size[0] - w) / 2, y + 8), ln, font=F_SMALL, fill=(200, 200, 200))
        y += 32
    return img


def ffmpeg_exe():
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return shutil.which("ffmpeg")


def encode(tmp, out, crf=23):
    exe = ffmpeg_exe()
    for c in (crf, crf + 3, crf + 6, crf + 9):
        cmd = [exe, "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", os.path.join(tmp, "%05d.png"), "-c:v", "libx264",
               "-preset", "slow", "-crf", str(c), "-pix_fmt", "yuv420p", "-movflags", "+faststart", out]
        subprocess.run(cmd, check=True)
        if os.path.getsize(out) < 4.8e6:
            return c
    return c


def build(clip, runs_root, out_dir):
    BOXES.clear()
    BOXES.update(clip.get("boxes", {}))
    st = clip.get("style", {})
    STYLE.update(font=font(int(st["font"])) if "font" in st else F_TEXT, alpha=int(st.get("alpha", 165)))
    OPTS.update(w_out=int(clip.get("width", W_OUT)), label_pos=clip.get("label_pos", "bottom"), note=clip.get("note"))
    STRIKES.clear()
    STRIKES["from"] = clip.get("strikes_from")
    ACTLOG.clear()
    if clip.get("action_log"):
        res = json.load(open(os.path.join(runs_root, clip["action_log"]["run"], "result.json")))
        ACTLOG.update({int(r["a"]): r for r in res["runs"][0]["policy_log"]["exploit"]["actions"]})
    tmp = os.path.join(out_dir, f"_tmp_{clip['id']}")
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    n = 0
    keyframes = []
    meta = {"id": clip["id"], "segments": []}
    title = clip.get("title")
    for seg in clip["segments"]:
        idx = [json.load(open(os.path.join(runs_root, r, "frames", "index.json"))) for r in seg["runs"]]
        sel_list = seg.get("selects") or [seg["select"]] * len(idx)
        sels = [select(ix["frames"], sl) for ix, sl in zip(idx, sel_list)]
        m = min(len(s) for s in sels)
        firsts = [s[0] if s else {} for s in sels]
        if seg.get("ref"):  # reference frame for dz/dx/tilt fields instead of the first selected one
            firsts = [(select(ix["frames"], seg["ref"]) or [{}])[0] for ix in idx]
        seg_meta = {"runs": seg["runs"], "cam": seg["cam"], "n_samples": m, "repeat": seg.get("repeat", 1),
                    "first_frame": [s[0]["i"] for s in sels if s], "last_frame": [s[m - 1]["i"] for s in sels if s]}
        for k in range(m):
            recs = [s[k] for s in sels]
            off = int(seg.get("step_offset", 0))
            f = SafeDict(fields(recs[-1], firsts[-1], off))
            top = [title] if title else []
            if seg.get("top"):
                top.append(seg["top"].format_map(f))
            bottom = [seg["bottom"].format_map(f)] if seg.get("bottom") else []
            paths = [os.path.join(runs_root, r, "frames", seg["cam"], f"f_{rec['i']:05d}.jpg") for r, rec in zip(seg["runs"], recs)]
            labels = [t.format_map(SafeDict(fields(rc, fs, off))) for t, rc, fs in zip(seg["labels"], recs, firsts)] if seg.get("labels") else None
            img = compose(paths, labels, top, bottom, seg.get("crop"), seg.get("layout", "h"))
            LAST.update(recs=recs, firsts=firsts, paths=paths, labels=labels)
            for _ in range(int(seg.get("repeat", 1))):
                img.save(os.path.join(tmp, f"{n:05d}.png"))
                n += 1
            if k in (0, m // 3, (2 * m) // 3, m - 1):
                keyframes.append(img.copy())
        meta["segments"].append(seg_meta)
    last = Image.open(os.path.join(tmp, f"{n - 1:05d}.png"))
    if clip.get("end_card"):
        ec = clip["end_card"]
        img = card(last.size, ec["lines"], ec.get("sub"))
        for _ in range(int(ec.get("seconds", 2.0) * FPS)):
            img.save(os.path.join(tmp, f"{n:05d}.png"))
            n += 1
        keyframes.append(img)
    elif clip.get("hold_last_s"):
        if clip.get("hold_bottom"):  # the last composed frame again, with the bottom text replaced
            seg, recs = clip["segments"][-1], LAST["recs"]
            f = SafeDict(fields(recs[-1], LAST["firsts"][-1], int(seg.get("step_offset", 0))))
            top = ([title] if title else []) + ([seg["top"].format_map(f)] if seg.get("top") else [])
            last = compose(LAST["paths"], LAST["labels"], top, clip["hold_bottom"].format_map(f).split("\n"), seg.get("crop"),
                           seg.get("layout", "h"))
        for _ in range(int(clip["hold_last_s"] * FPS)):
            last.save(os.path.join(tmp, f"{n:05d}.png"))
            n += 1
        keyframes.append(last.copy())
    meta["n_frames"] = n
    meta["seconds"] = round(n / FPS, 2)
    if clip.get("sheet"):  # chosen composed frames for a one-column contact sheet
        sh = clip["sheet"]
        keyframes = [("column", int(sh.get("width", 960)), lab, Image.open(os.path.join(tmp, f"{k:05d}.png")).copy())
                     for k, lab in zip(sh["frames"], sh["labels"])]
        meta["sheet"] = {"frames": sh["frames"], "width": int(sh.get("width", 960))}
    if clip.get("max_s") and n / FPS > clip["max_s"]:
        raise SystemExit(f"{clip['id']}: {n / FPS:.2f} s is longer than max_s {clip['max_s']}")
    if clip.get("poster"):
        k = int(clip["poster"]["frame"]) % n
        out = os.path.join(out_dir, f"{clip['id']}_poster.png")
        Image.open(os.path.join(tmp, f"{k:05d}.png")).save(out, optimize=True)
        meta["poster"] = {"file": os.path.basename(out), "frame": k, "bytes": os.path.getsize(out)}
    if clip.get("webp"):
        meta["anim"] = animate(tmp, n, out_dir, clip["id"], clip["webp"])
    if clip.get("still"):
        out = os.path.join(out_dir, f"{clip['id']}.png")
        keyframes[0].save(out, optimize=True)
        meta["file"] = os.path.basename(out)
    else:
        out = os.path.join(out_dir, f"{clip['id']}.mp4")
        meta["crf"] = encode(tmp, out)
        meta["file"] = os.path.basename(out)
    meta["bytes"] = os.path.getsize(out)
    shutil.rmtree(tmp, ignore_errors=True)
    return meta, keyframes


LAST = {}  # the last composed frame's inputs (for "hold_bottom")


def animate(tmp, n, out_dir, cid, opt):
    """Animated WebP (GIF if this Pillow lacks WebP) from every `every`-th composed frame, `width` px wide, under 3 MB."""
    from PIL import features

    every = int(opt.get("every", 2))
    frames = [Image.open(os.path.join(tmp, f"{k:05d}.png")).convert("RGB") for k in range(0, n, every)]
    dur = int(round(1000 * every / FPS))
    webp = features.check_module("webp")  # Pillow 12 writes animated WebP whenever the webp module is built
    out = os.path.join(out_dir, f"{cid}.{'webp' if webp else 'gif'}")
    tries = [(int(opt.get("width", 640)), q) for q in (int(opt.get("quality", 70)), 55, 40)] if webp else \
        [(w, None) for w in (int(opt.get("width", 640)), 560, 480, 400)]
    for width, q in tries:
        ims = [im.resize((width, int(im.height * width / im.width)), Image.LANCZOS) for im in frames]
        if webp:
            ims[0].save(out, save_all=True, append_images=ims[1:], duration=dur, loop=0, quality=q, method=6)
        else:
            pal = [im.quantize(colors=128, method=Image.Quantize.MEDIANCUT) for im in ims]
            pal[0].save(out, save_all=True, append_images=pal[1:], duration=dur, loop=0, optimize=True)
        if os.path.getsize(out) <= 3_000_000:
            break
    return {"file": os.path.basename(out), "format": "webp" if webp else "gif", "frames": len(ims), "every": every,
            "frame_ms": dur, "width": width, "quality": q, "bytes": os.path.getsize(out)}


def column_sheet(cells, out):
    """One column: each cell is ("column", width, label, composed frame) -> label line, then the frame at that width."""
    ims = [(lab, im.resize((w, int(im.height * w / im.width)), Image.LANCZOS)) for _, w, lab, im in cells]
    lh = F_SMALL.size + 16
    sheet = Image.new("RGB", (max(im.width for _, im in ims), sum(lh + im.height for _, im in ims)), (255, 255, 255))
    d, y = ImageDraw.Draw(sheet), 0
    for lab, im in ims:
        d.text((8, y + 7), lab, font=F_SMALL, fill=(0, 0, 0))
        sheet.paste(im, (0, y + lh))
        y += lh + im.height
    sheet.save(out, optimize=True)


def contact_sheet(rows, out):
    if rows and all(kfs and isinstance(kfs[0], tuple) and kfs[0][0] == "column" for _, kfs in rows):
        return column_sheet([c for _, kfs in rows for c in kfs], out)
    tw, th = 400, 225
    lab_w = 260
    sheet = Image.new("RGB", (lab_w + tw * 4, th * len(rows)), (255, 255, 255))
    d = ImageDraw.Draw(sheet)
    for r, (label, kfs) in enumerate(rows):
        d.text((10, r * th + 10), label, font=F_SMALL, fill=(0, 0, 0))
        picks = kfs[:4] if len(kfs) <= 4 else [kfs[int(round(j * (len(kfs) - 1) / 3))] for j in range(4)]
        for c, im in enumerate(picks):
            t = im.copy()
            t.thumbnail((tw, th))
            sheet.paste(t, (lab_w + c * tw, r * th))
    sheet.save(out, optimize=True)


def main():
    clips = json.load(open(sys.argv[1]))
    runs_root, out_dir = sys.argv[2], sys.argv[3]
    only = set(sys.argv[4].split(",")) if len(sys.argv) > 4 else None
    os.makedirs(out_dir, exist_ok=True)
    rows, metas = [], []
    for c in clips:
        if only and c["id"] not in only:
            continue
        meta, kfs = build(c, runs_root, out_dir)
        metas.append(meta)
        rows.append((c.get("sheet_label", c["id"]), kfs))
        print(json.dumps(meta), flush=True)
    contact_sheet(rows, os.path.join(out_dir, "contact_sheet.png"))
    json.dump(metas, open(os.path.join(out_dir, "clips_meta.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
