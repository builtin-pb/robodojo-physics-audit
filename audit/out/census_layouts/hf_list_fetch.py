"""List every eval layout file at the pinned HF revision and fetch the ones our local Assets copy lacks.

Usage (della-vis1, CPU):  python3 -I hf_list_fetch.py <local Assets dirs, colon-separated> <out dir> [workers]
  <local Assets dirs> existing copies, read only, tried in order (W/robodojo/Assets:W/tmp/rev73b/layouts_8070/Assets)
  <out dir>           new directory; gets listing.json, manifest.json and Assets/Eval_Layout/... for missing files
Each file is verified: SHA-256 of the bytes against the listing's LFS oid (or the git blob SHA-1 if not LFS).
A local file is used only if it verifies; otherwise it is downloaded. Nothing is written into the local dirs.
"""
import concurrent.futures as cf, hashlib, json, os, re, sys, time
import urllib.error as E, urllib.parse as P, urllib.request as U

REPO = "RoboDojo-Benchmark/RoboDojo"
REV = "1127bda94f172d757386945af3eb3a6754d1a4e6"
BASE = "Assets/Eval_Layout/RoboDojo/arx_x5"
API = f"https://huggingface.co/api/datasets/{REPO}/tree/{REV}/"
RES = f"https://huggingface.co/datasets/{REPO}/resolve/{REV}/"
HDR = {"User-Agent": "layout-census/1.0"}

local_roots, out = sys.argv[1].split(":"), sys.argv[2]
workers = int(sys.argv[3]) if len(sys.argv) > 3 else 8


def http(url, tries=8):
    for i in range(tries):
        try:
            r = U.urlopen(U.Request(url, headers=HDR), timeout=60)
            return r.read(), r.headers
        except E.HTTPError as e:
            if e.code in (429, 500, 502, 503, 504) and i < tries - 1:
                wait = float(e.headers.get("Retry-After") or 0) or min(60, 2 ** i)
                print(f"HTTP {e.code} on {url[-60:]}; retry in {wait:.0f}s", flush=True)
                time.sleep(wait)
                continue
            raise
        except (E.URLError, TimeoutError, ConnectionError) as e:
            if i < tries - 1:
                time.sleep(min(60, 2 ** i))
                continue
            raise


def list_dir(path):
    url, entries, pages = API + P.quote(path), [], 0
    while url:
        body, hdr = http(url)
        entries += json.loads(body)
        pages += 1
        m = re.search(r'<([^>]+)>;\s*rel="next"', hdr.get("Link", "") or "")
        url = m.group(1) if m else None
    return entries, pages


def verify(data, ent):
    if ent.get("lfs"):
        return hashlib.sha256(data).hexdigest() == ent["lfs"]["oid"]
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest() == ent["oid"]


os.makedirs(out, exist_ok=True)
listing, pages = {}, {}
for seed in ("0", "1", "2"):
    ents, pages[seed] = list_dir(f"{BASE}/{seed}")
    listing[seed] = [e for e in ents if e["type"] == "file"]
    print(f"seed {seed}: {len(listing[seed])} files in {pages[seed]} pages", flush=True)
with open(os.path.join(out, "listing.json"), "w") as f:
    json.dump({"repo": REPO, "revision": REV, "base": BASE, "pages": pages, "files": listing}, f, indent=0)

jobs = []
for seed, ents in listing.items():
    for e in ents:
        jobs.append((seed, e))


def one(job):
    seed, e = job
    rel = e["path"]
    for root in local_roots:
        loc = os.path.join(root, os.path.relpath(rel, "Assets"))
        if os.path.exists(loc):
            data = open(loc, "rb").read()
            if verify(data, e):
                return rel, "local:" + root, loc, hashlib.sha256(data).hexdigest(), len(data)
    dst = os.path.join(out, rel)
    if os.path.exists(dst):
        data = open(dst, "rb").read()
        if verify(data, e):
            return rel, "downloaded", dst, hashlib.sha256(data).hexdigest(), len(data)
    data, _ = http(RES + P.quote(rel))
    if not verify(data, e):
        raise RuntimeError(f"checksum mismatch: {rel}")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst + ".part", "wb") as f:
        f.write(data)
    os.replace(dst + ".part", dst)
    return rel, "downloaded", dst, hashlib.sha256(data).hexdigest(), len(data)


manifest, t0, n = {}, time.time(), 0
with cf.ThreadPoolExecutor(workers) as ex:
    for rel, src, path, sha, size in ex.map(one, jobs):
        manifest[rel] = {"source": src, "path": path, "sha256": sha, "bytes": size}
        n += 1
        if n % 500 == 0:
            print(f"{n}/{len(jobs)} files, {time.time() - t0:.0f}s", flush=True)
with open(os.path.join(out, "manifest.json"), "w") as f:
    json.dump(manifest, f, indent=0)
src_counts = {}
for v in manifest.values():
    src_counts[v["source"]] = src_counts.get(v["source"], 0) + 1
print("sources:", src_counts, f"{time.time() - t0:.0f}s")
print("DONE")
