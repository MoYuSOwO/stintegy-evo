"""Download Formula 1 race data from OpenF1 (free, 2023 on) into a local folder.

    python3 download_openf1.py                      # 2023-2025 races into ./f1data
    python3 download_openf1.py --years 2024 --out D # one year, elsewhere

Only Python's own library is needed. Run it again after a stop: what is
already down is kept.

For every race session of the given years: the drivers, their laps, their
running order, and per driver the car positions (about 4 Hz) and car data
(speed, throttle, brake, gear, DRS). Each response is stored as it came,
gzipped JSON, under <out>/<year>/<date>-<circuit>/. Files already there are
skipped, so an interrupted run picks up where it stopped. The data is
OpenF1's and stays local: it is not to be committed.
"""

from __future__ import annotations

import argparse
import datetime
import gzip
import json
import time
import urllib.error
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

API = "https://api.openf1.org/v1/"
PAUSE = 0.6  # seconds between requests, inside the free tier's limits
# The server sends each answer slowly, so a driver's race is asked for in
# half-hour pieces, several at once.
WINDOW = datetime.timedelta(minutes=30)
WORKERS = 6


def get(endpoint: str, **params) -> bytes:
    url = API + endpoint + "?" + urllib.parse.urlencode(params)
    for attempt in range(8):
        try:
            with urllib.request.urlopen(url, timeout=120) as response:
                body = response.read()
            time.sleep(PAUSE)
            return body
        except urllib.error.HTTPError as e:
            # Nothing in the window asked for comes back as not found.
            if e.code == 404:
                time.sleep(PAUSE)
                return b"[]"
            if e.code in (429, 500, 502, 503, 504):
                time.sleep(min(60, 5 * 2 ** attempt))
                continue
            raise
        except (urllib.error.URLError, TimeoutError):
            time.sleep(min(60, 5 * 2 ** attempt))
    raise RuntimeError(f"gave up on {url}")


def save(path: Path, body: bytes) -> None:
    tmp = path.with_suffix(".tmp")
    with gzip.open(tmp, "wb") as f:
        f.write(body)
    tmp.rename(path)


def fetch(path: Path, endpoint: str, **params) -> bytes:
    if path.exists():
        with gzip.open(path, "rb") as f:
            return f.read()
    body = get(endpoint, **params)
    save(path, body)
    return body


def windows(s: dict):
    start = datetime.datetime.fromisoformat(s["date_start"]) - datetime.timedelta(minutes=10)
    end = datetime.datetime.fromisoformat(s["date_end"]) + datetime.timedelta(minutes=30)
    while start < end:
        stop = min(end, start + WINDOW)
        yield start, stop
        start = stop


def fetch_windowed(path: Path, endpoint: str, s: dict, driver: int, pool: ThreadPoolExecutor) -> None:
    if path.exists():
        return
    fmt = lambda t: t.strftime("%Y-%m-%dT%H:%M:%S")
    parts = [pool.submit(get, endpoint, **{"session_key": s["session_key"], "driver_number": driver,
                                           "date>": fmt(a), "date<": fmt(b)})
             for a, b in windows(s)]
    rows = []
    for part in parts:
        rows.extend(json.loads(part.result()))
    save(path, json.dumps(rows).encode())


def session(out: Path, s: dict) -> None:
    folder = out / str(s["year"]) / f"{s['date_start'][:10]}-{s['circuit_short_name'].replace(' ', '_')}"
    folder.mkdir(parents=True, exist_ok=True)
    key = s["session_key"]
    (folder / "session.json").write_text(json.dumps(s, indent=1))
    drivers = json.loads(fetch(folder / "drivers.json.gz", "drivers", session_key=key))
    fetch(folder / "laps.json.gz", "laps", session_key=key)
    fetch(folder / "position.json.gz", "position", session_key=key)
    started = time.perf_counter()
    with ThreadPoolExecutor(WORKERS) as pool, ThreadPoolExecutor(4) as files:
        jobs = [files.submit(fetch_windowed, folder / f"{endpoint}-{d['driver_number']}.json.gz",
                             endpoint, s, d["driver_number"], pool)
                for d in drivers for endpoint in ("location", "car_data")]
        for done, job in enumerate(jobs, 1):
            job.result()
            print(f"\r  {folder.name}: {done}/{len(jobs)} files", end="", flush=True)
    print(f"\r  {folder.name}: {len(drivers)} drivers, {time.perf_counter() - started:.0f}s" + " " * 10, flush=True)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="f1data")
    p.add_argument("--years", type=int, nargs="+", default=[2023, 2024, 2025])
    p.add_argument("--circuit", help="only this circuit_short_name")
    args = p.parse_args()
    out = Path(args.out)
    now = time.strftime("%Y-%m-%dT%H:%M:%S")
    for year in args.years:
        sessions = json.loads(get("sessions", year=year, session_name="Race"))
        sessions = [s for s in sorted(sessions, key=lambda x: x["date_start"])
                    if not s.get("is_cancelled") and s["date_end"][:19] <= now
                    and (not args.circuit or s["circuit_short_name"] == args.circuit)]
        print(f"{year}: {len(sessions)} races", flush=True)
        for s in sessions:
            session(out, s)
    total = sum(f.stat().st_size for f in out.rglob("*") if f.is_file())
    print(f"done: {total / 1e9:.2f} GB in {out.resolve()}")


if __name__ == "__main__":
    main()
