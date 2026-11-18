"""Blog clips v2: compose rendered replays into a dark card with tag chips, live readouts and a tilt plot (MP4, WebP, poster).

usage: python -I compose_cards.py <cards.json> <frames root> <traces root> <fonts dir> <out dir> [card ids]
- <frames root>/<replay tag>/frames/index.json and frames/<cam>/f_<i>.jpg: clip_run.py's replays (#216's camera ax10z).
- <traces root>/<trace path>: each panel's 5.1 harness result.json(.gz); its witness pose_trace gives the readouts and plot.
  pose_trace[k] is episode step lift0 + 1 + k (the lift starts after step lift0 = 910; fingers_check.py, LIFT0).
- <fonts dir>: Archivo.ttf (axes weight, width) and JetBrainsMono.ttf (weight), Google Fonts' variable TTFs.
Readouts at the frame's episode step s (PREDICTIONS.md, "Readouts and plot"):
  tilt = pose_trace rot_deg at s, one decimal, two under 1 degree (the bottle's rotation in the hand since step lift0);
  grip = min and max of pose_trace f_avg_N over steps s-9..s (40 ms), "lo–hi N", one value when the range is under 2 N.
Plot: rot_deg at every logged step against simulated time since the lift began, (step - lift0) * 4 ms; the whole trace faint,
the part up to the frame's step full, a cursor and a dot per line at the frame's step.
Writes <id>.mp4 (H.264, 25 fps, CRF 23, yuv420p, +faststart; square corners: round them in CSS), <id>.webp (animated,
transparent rounded corners), <id>_poster.png (the last frame, transparent rounded corners), <id>_frames.json (per output
frame: episode step, time, readout values) and <id>_meta.json.
"""
import gzip
import json
import os
import shutil
import subprocess
import sys
import tempfile

from PIL import Image, ImageDraw, ImageFont

FPS = 25
SS = 3  # supersampling for shapes and lines
BG = (18, 21, 24)  # blog dark paper #121518
INK = (229, 232, 234)  # #e5e8ea
MUTED = (154, 164, 175)  # #9aa4af
RULE = (44, 50, 56)  # #2c3238
PANEL_BG = (27, 31, 35)  # #1b1f23
CHIP_BG = (18, 21, 24, 190)


def hexrgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


class Fonts:
    def __init__(self, d):
        self.d = d
        self.cache = {}

    def get(self, fam, size, wght):
        k = (fam, size, wght)
        if k not in self.cache:
            f = ImageFont.truetype(os.path.join(self.d, "Archivo.ttf" if fam == "sans" else "JetBrainsMono.ttf"), size)
            f.set_variation_by_axes([wght, 100] if fam == "sans" else [wght])
            self.cache[k] = f
        return self.cache[k]


def load(p):
    return json.load(gzip.open(p) if p.endswith(".gz") else open(p))


def rounded_mask(w, h, r):
    m = Image.new("L", (w * SS, h * SS), 0)
    ImageDraw.Draw(m).rounded_rectangle([0, 0, w * SS - 1, h * SS - 1], r * SS, fill=255)
    return m.resize((w, h), Image.LANCZOS)


def pill(w, h, fill):
    im = Image.new("RGBA", (w, h), fill)
    im.putalpha(Image.eval(rounded_mask(w, h, h // 2), lambda a: a * fill[3] // 255))
    return im


def dashed(draw, pts, color, width, dash, gap):
    """Polyline with a dash pattern along its length (supersampled coordinates)."""
    on, left = True, dash
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        t = 0.0
        while seg > 0 and t < seg:
            step = min(left, seg - t)
            a, b = t / seg, (t + step) / seg
            if on:
                draw.line([(x0 + (x1 - x0) * a, y0 + (y1 - y0) * a), (x0 + (x1 - x0) * b, y0 + (y1 - y0) * b)],
                          fill=color, width=width)
            t += step
            left -= step
            if left <= 1e-9:
                on = not on
                left = dash if on else gap


def line_key(w, h, color, dash, width):
    im = Image.new("RGBA", (w * SS, h * SS), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    pts = [(0, h * SS / 2), (w * SS, h * SS / 2)]
    if dash:
        dashed(d, pts, color, width * SS, 7 * SS, 5 * SS)
    else:
        d.line(pts, fill=color, width=width * SS)
    return im.resize((w, h), Image.LANCZOS)


def text_tracked(d, xy, text, font, fill, track):
    """Draw text on a baseline ("ls") with extra space between characters; return its advance."""
    x, y = xy
    for c in text:
        d.text((x, y), c, font=font, fill=fill, anchor="ls")
        x += d.textlength(c, font=font) + track
    return x - xy[0] - track


def tilt_text(deg):
    """One decimal, two under 1 degree (0.04 would read as an exact 0.0)."""
    return f"{deg:.2f}" if deg < 1.0 else f"{deg:.1f}"


def grip_text(fs):
    lo, hi = min(fs), max(fs)
    if hi - lo < 2.0:
        return f"{(lo + hi) / 2:.0f}"
    return f"{lo:.0f}–{hi:.0f}"


def build_card(card, frames_root, traces_root, F, out_dir):
    W = card["width"]
    n = len(card["panels"])
    M, G = card.get("margin", 38), card.get("gap", 16)
    pw = (W - 2 * M - (n - 1) * G) // n
    ph = round(pw * 720 / 640)
    M = (W - n * pw - (n - 1) * G) // 2  # absorb rounding into the side margins
    st = card["steps"]
    lift0 = st["lift0"]

    # traces
    traces = []
    for p in card["panels"]:
        res = load(os.path.join(traces_root, p["trace"]))
        pt = res["runs"][0]["policy_log"]["witness_trial"]["pose_trace"]
        assert len(pt) == st["to"] - lift0, (p["trace"], len(pt))
        traces.append(pt)

    # frames: the replay's step records in [from, to], every 5th physics step as rendered
    sel = []
    for p in card["panels"]:
        ix = load(os.path.join(frames_root, p["replay"], "frames", "index.json"))
        recs = [r for r in ix["frames"] if r.get("why", "").startswith("step") and st["from"] <= r["phys"] <= st["to"]]
        recs = [r for r in recs if (r["phys"] - st["from"]) % st["every"] == 0]
        sel.append(recs)
    steps = [r["phys"] for r in sel[0]]
    assert all([r["phys"] for r in s] == steps for s in sel), "panels' frames differ in steps"

    # vertical layout; "text_scale" enlarges every text but the title (a wide card is shown smaller)
    ts = card.get("text_scale", 1.0)

    def z(n):
        return round(n * ts)

    y_title = M - 4
    f_title = F.get("sans", card.get("title_px", 50), 640)
    f_sub = F.get("sans", z(26), 400)
    sub_lines, cur = [], ""
    for word in card["subtitle"].split(" "):
        trial = (cur + " " + word).strip()
        if cur and ImageDraw.Draw(Image.new("L", (1, 1))).textlength(trial, font=f_sub) > W - 2 * M:
            sub_lines.append(cur)
            cur = word
        else:
            cur = trial
    sub_lines.append(cur)
    y_sub = y_title + f_title.size + 14
    y_pan = y_sub + len(sub_lines) * (f_sub.size + 8) + 22
    y_rule = y_pan + ph + 26
    f_val, f_unit, f_lab = F.get("mono", round(46 * ts ** 0.5), 600), F.get("mono", z(26), 500), F.get("sans", z(21), 450)
    y_val = y_rule + 18
    y_lab = y_val + f_val.size + 12
    y_plot = y_lab + f_lab.size + 34
    f_head, f_tick, f_leg = F.get("sans", z(23), 600), F.get("mono", z(19), 450), F.get("sans", z(21), 450)
    plot_head = f_head.size + 24
    plot_h = card.get("plot_h", 170)
    y_body = y_plot + plot_head
    H = y_body + plot_h + 12 + f_tick.size + 34
    H += H % 2

    # static layer
    base = Image.new("RGBA", (W, H), BG + (255,))
    d = ImageDraw.Draw(base)
    d.text((M, y_title), card["title"], font=f_title, fill=INK)
    for k, line in enumerate(sub_lines):
        d.text((M, y_sub + k * (f_sub.size + 8)), line, font=f_sub, fill=MUTED)
    # "slowed 2x" chip, right of the title row
    f_note = F.get("mono", z(24), 500)
    tw = d.textlength(card["note"], font=f_note)
    cw, chh = int(tw + z(36)), z(44)
    cx, cy = W - M - cw, y_title + (f_title.size - chh) // 2 + 4
    ring = Image.new("RGBA", (cw * SS, chh * SS), (0, 0, 0, 0))
    ImageDraw.Draw(ring).rounded_rectangle([0, 0, cw * SS - 1, chh * SS - 1], chh * SS // 2, outline=MUTED + (255,), width=2 * SS)
    base.alpha_composite(ring.resize((cw, chh), Image.LANCZOS), (cx, cy))
    d.text((cx + cw / 2, cy + chh / 2), card["note"], font=f_note, fill=INK, anchor="mm")
    # panel backgrounds (seen only at rounded corners) and chips are drawn per frame; rules and readout labels here
    labels = card["readouts"]
    rx = [round(f * pw) for f in card.get("readout_x", [j / len(labels) for j in range(len(labels))])]
    for i, p in enumerate(card["panels"]):
        x0 = M + i * (pw + G)
        col = hexrgb(p["color"])
        base.alpha_composite(line_key(pw, 6, col + (255,), p.get("dash", False), 4), (x0, y_rule - 3))
        for j, lab in enumerate(labels):
            d.text((x0 + rx[j], y_lab), lab["label"], font=f_lab, fill=MUTED)
            right = rx[j + 1] if j + 1 < len(labels) else pw
            assert rx[j] + d.textlength(lab["label"], font=f_lab) < right - 8, ("readout label too long", lab["label"])
    # plot frame: header, legend, grid, ticks, phases
    px0 = M + 70
    px1 = W - M - 8
    d.text((M, y_plot), card["plot_title"], font=f_head, fill=INK)
    lx = W - M
    for p in reversed(card["panels"]):
        t = p["legend"]
        lw = d.textlength(t, font=f_leg)
        lx -= lw
        d.text((lx, y_plot + f_head.size), t, font=f_leg, fill=MUTED, anchor="ls")
        lx -= z(40)
        base.alpha_composite(line_key(z(32), 8, hexrgb(p["color"]) + (255,), p.get("dash", False), 4),
                             (int(lx), y_plot + f_head.size - f_leg.size // 3 - 4))
        lx -= z(30)
    ymax = card.get("plot_ymax", 90)
    tmax = (st["to"] - lift0) * 0.004

    def X(t):
        return px0 + (px1 - px0) * t / tmax

    def Y(v):
        return y_body + plot_h - plot_h * min(v, ymax) / ymax

    for v in card.get("plot_yticks", [0, 30, 60, 90]):
        d.line([(px0, Y(v)), (px1, Y(v))], fill=RULE, width=1)
        d.text((px0 - 14, Y(v)), f"{v}°", font=f_tick, fill=MUTED, anchor="rm")
    t_lift = card.get("lift_s", 0.8)
    d.line([(X(t_lift), y_body), (X(t_lift), y_body + plot_h)], fill=RULE, width=1)
    f_ph = F.get("sans", z(19), 450)
    for a, b, txt in card["phases"]:  # on the time axis's label row, between its ticks
        d.text(((X(a) + X(b)) / 2, y_body + plot_h + 12), txt, font=f_ph, fill=MUTED, anchor="mt")
    xt = card.get("plot_xticks", [0, 0.8, 1.8])
    for t in xt:
        d.line([(X(t), y_body + plot_h), (X(t), y_body + plot_h + 6)], fill=MUTED, width=1)
        lab = ("0" if t == 0 else f"{t:.1f}") + (" " + card.get("plot_xunit", "s") if t == xt[-1] else "")
        d.text((X(t), y_body + plot_h + 12), lab, font=f_tick, fill=MUTED, anchor="rt" if t == xt[-1] else "mt")

    # series in supersampled plot coordinates (plot layer covers the body plus room for dots)
    lay_x0, lay_y0 = px0 - 12, y_body - 12
    lay_w, lay_h = px1 - px0 + 24, plot_h + 24
    series = []
    for pt in traces:
        pts = [(0.0, 0.0)] + [((k + 1) * 0.004, r["rot_deg"]) for k, r in enumerate(pt)]
        series.append([((X(t) - lay_x0) * SS, (Y(v) - lay_y0) * SS, t, v) for t, v in pts])

    def plot_layer(t_now, faint_only=False):
        im = Image.new("RGBA", (lay_w * SS, lay_h * SS), (0, 0, 0, 0))
        dd = ImageDraw.Draw(im)
        for p, s in zip(card["panels"], series):
            col = hexrgb(p["color"])
            for alpha, upto in ((70, None), (255, t_now)):
                if upto is None:
                    xy = [(a, b) for a, b, _, _ in s]
                else:
                    if faint_only:
                        continue
                    xy = [(a, b) for a, b, t, _ in s if t <= upto + 1e-9]
                if len(xy) < 2:
                    continue
                if p.get("dash"):
                    dashed(dd, xy, col + (alpha,), 4 * SS, 9 * SS, 6 * SS)
                else:
                    dd.line(xy, fill=col + (alpha,), width=4 * SS, joint="curve")
        if not faint_only:
            cx_ = (X(t_now) - lay_x0) * SS
            dd.line([(cx_, 12 * SS), (cx_, (lay_h - 6) * SS)], fill=INK + (150,), width=2 * SS)  # from the 90° line down
            for p, s in zip(card["panels"], series):
                v = next(v for a, b, t, v in s if t >= t_now - 1e-9)
                cy_ = (Y(v) - lay_y0) * SS
                r, ring_w = 7 * SS, 3 * SS
                dd.ellipse([cx_ - r - ring_w, cy_ - r - ring_w, cx_ + r + ring_w, cy_ + r + ring_w], fill=BG + (255,))
                dd.ellipse([cx_ - r, cy_ - r, cx_ + r, cy_ + r], fill=hexrgb(p["color"]) + (255,))
        return im.resize((lay_w, lay_h), Image.LANCZOS)

    pmask = rounded_mask(pw, ph, card.get("radius", 18))
    f_chip = F.get("sans", z(23), 500)
    chips = []
    for p in card["panels"]:
        tw = ImageDraw.Draw(base).textlength(p["chip"], font=f_chip)
        kw_, pad = z(26), z(15)
        cw, chh = int(tw + 2 * pad + kw_ + z(12)), z(44)
        c = pill(cw, chh, CHIP_BG)
        c.alpha_composite(line_key(kw_, 8, hexrgb(p["color"]) + (255,), p.get("dash", False), 4), (pad, chh // 2 - 4))
        ImageDraw.Draw(c).text((pad + kw_ + z(10), chh / 2 + 1), p["chip"], font=f_chip, fill=INK, anchor="lm")
        chips.append(c)

    def frame(k):
        s = steps[k]
        im = base.copy()
        dd = ImageDraw.Draw(im)
        vals = []
        for i, (p, pt) in enumerate(zip(card["panels"], traces)):
            x0 = M + i * (pw + G)
            src = Image.open(os.path.join(frames_root, p["replay"], "frames", card["cam"], f"f_{sel[i][k]['i']:05d}.jpg")).convert("RGB")
            src = src.resize((pw, ph), Image.LANCZOS) if src.size != (pw, ph) else src
            pan = Image.new("RGBA", (pw, ph), PANEL_BG + (255,))
            pan.paste(src, (0, 0))
            pan.putalpha(pmask)
            im.alpha_composite(pan, (x0, y_pan))
            im.alpha_composite(chips[i], (x0 + 16, y_pan + 16))
            r = pt[s - lift0 - 1]
            fs = [q["f_avg_N"] for q in pt[s - lift0 - 10:s - lift0]]
            v = {"tilt": tilt_text(r["rot_deg"]), "grip": grip_text(fs)}
            raw = {"rot_deg": r["rot_deg"], "dz_mm": r["dz_mm"], "f_avg_N_window": [min(fs), max(fs)], "window_steps": [s - 9, s]}
            vals.append({"shown": v, "raw": raw})
            for j, lab in enumerate(card["readouts"]):
                xx = x0 + rx[j]
                txt = v[lab["key"]]
                adv = text_tracked(dd, (xx, y_val + f_val.size), txt, f_val, INK, -4)
                if lab["unit"] == "°":
                    dd.text((xx + adv - 6, y_val + f_val.size), "°", font=f_val, fill=INK, anchor="ls")
                else:
                    dd.text((xx + adv + 8, y_val + f_val.size), lab["unit"], font=f_unit, fill=MUTED, anchor="ls")
        t_now = (s - lift0) * 0.004
        im.alpha_composite(plot_layer(t_now), (lay_x0, lay_y0))
        return im, {"step": s, "t_s": round(t_now, 3), "panels": vals}

    if os.environ.get("CARDS_PREVIEW"):  # layout check: the first frame and the stills only, no encoding
        os.makedirs(out_dir, exist_ok=True)
        for s in [steps[0]] + card.get("stills", []):
            frame(steps.index(s))[0].save(os.path.join(out_dir, f"{card['id']}_preview{s}.png"))
        return {"id": card["id"], "preview": True, "size": [W, H]}
    tmp = tempfile.mkdtemp(prefix=f"cards_{card['id']}_")
    out_frames, records = [], []
    n_first = round(card.get("hold_first_s", 0.5) * FPS)
    n_last = round(card.get("hold_last_s", 0.4) * FPS)
    k_out = 0
    for k in range(len(steps)):
        im, rec = frame(k)
        reps = 1 + (n_first if k == 0 else 0) + (n_last if k == len(steps) - 1 else 0)
        for _ in range(reps):
            im.convert("RGB").save(os.path.join(tmp, f"{k_out:05d}.png"))
            k_out += 1
        rec["out_frames"] = reps
        records.append(rec)
        out_frames.append((im, reps))
    os.makedirs(out_dir, exist_ok=True)
    cid = card["id"]
    mp4 = os.path.join(out_dir, f"{cid}.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", os.path.join(tmp, "%05d.png"),
                    "-c:v", "libx264", "-preset", "slow", "-crf", str(card.get("crf", 23)), "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", mp4], check=True)
    # rounded card corners for the WebP and poster
    cmask = rounded_mask(W, H, card.get("card_radius", 28))
    wp = card["webp"]
    ww = wp["width"]
    wh = round(H * ww / W)
    ims, durs = [], []
    for im, reps in out_frames:
        x = im.copy()
        x.putalpha(cmask)
        ims.append(x.resize((ww, wh), Image.LANCZOS))
        durs.append(reps * 1000 // FPS)
    webp = os.path.join(out_dir, f"{cid}.webp")
    # libwebp's default inter-frame choice; minimize_size=True patches the moving table shadows into visible blocks
    ims[0].save(webp, save_all=True, append_images=ims[1:], duration=durs, loop=0, quality=wp["quality"], method=6)
    poster = os.path.join(out_dir, f"{cid}_poster.png")
    x = out_frames[card.get("poster", -1)][0].copy()
    x.putalpha(cmask)
    x.save(poster, optimize=True)
    for extra in card.get("stills", []):  # composed frames for review, by step
        k = steps.index(extra)
        out_frames[k][0].convert("RGB").save(os.path.join(out_dir, f"{cid}_step{extra}.png"), optimize=True)
    shutil.rmtree(tmp)
    json.dump(records, open(os.path.join(out_dir, f"{cid}_frames.json"), "w"), indent=0)
    meta = {"id": cid, "size": [W, H], "panel": [pw, ph], "steps": [steps[0], steps[-1], st["every"]], "rendered_frames": len(steps),
            "out_frames": k_out, "seconds": k_out / FPS, "fps": FPS, "hold_first_frames": n_first, "hold_last_frames": n_last,
            "mp4": {"file": os.path.basename(mp4), "bytes": os.path.getsize(mp4), "crf": card.get("crf", 23)},
            "webp": {"file": os.path.basename(webp), "bytes": os.path.getsize(webp), "size": [ww, wh], "quality": wp["quality"],
                     "frames_given": len(ims)},
            "poster": {"file": os.path.basename(poster), "bytes": os.path.getsize(poster), "step": steps[card.get("poster", -1)]},
            "panels": [{k: p[k] for k in ("replay", "trace", "chip", "color", "dash") if k in p} for p in card["panels"]]}
    json.dump(meta, open(os.path.join(out_dir, f"{cid}_meta.json"), "w"), indent=1)
    return meta


def main():
    cards_p, frames_root, traces_root, fonts_dir, out_dir = sys.argv[1:6]
    want = set(sys.argv[6:])
    F = Fonts(fonts_dir)
    for card in json.load(open(cards_p)):
        if want and card["id"] not in want:
            continue
        print(json.dumps(build_card(card, frames_root, traces_root, F, out_dir)))


if __name__ == "__main__":
    main()
