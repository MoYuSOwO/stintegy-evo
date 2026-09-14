"""Everything a rented GPU box has to prove before a bake is lit on it.

Run once, right after setup, from Training/python. Each check prints what
it found and stops the script on the first failure, because every later
check assumes the earlier ones. Nothing here trains a model or writes a
checkpoint outside a temporary directory.

    python3 gpu_preflight.py --checkpoint checkpoints/latestparent2i.pt \\
        --sha256 <the hash recorded on the Mac>

With --reference the script also certifies a known checkpoint at the seed
the Mac used and compares the result with the Mac's figures: the Linux
host builds its tracks with OpenBLAS where the Mac used Apple Accelerate,
and the policy runs on CUDA where it ran on the CPU, so the two worlds are
expected to be close rather than bit-identical, and this is the check of
"close".
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time

# The protocol and observation this checkout speaks. A box that reports
# anything else is running a different world from the one being migrated.
EXPECTED_PROTOCOL = 4
EXPECTED_OBSERVATION = 480


class Failed(Exception):
    pass


def step(name: str):
    print(f"\n== {name}", flush=True)


def ok(message: str):
    print(f"   ok   {message}", flush=True)


def fail(message: str):
    raise Failed(message)


def check_python() -> None:
    step("Python and packages")
    ok(f"python {platform.python_version()} on {platform.system()} {platform.machine()}")
    try:
        import numpy
        import torch
    except ImportError as exc:
        fail(f"missing package: {exc.name}")
    ok(f"numpy {numpy.__version__}, torch {torch.__version__}")


def check_cuda(allow_cpu: bool) -> str:
    step("CUDA")
    import torch

    if not torch.cuda.is_available():
        if allow_cpu:
            ok("CUDA not available; continuing on the CPU because --allow-cpu was given")
            return "cpu"
        fail(
            "torch.cuda.is_available() is False. Check `nvidia-smi` and that "
            "the installed torch wheel was built for CUDA."
        )
    name = torch.cuda.get_device_name(0)
    capability = torch.cuda.get_device_capability(0)
    free, total = torch.cuda.mem_get_info()
    ok(f"{name}, compute capability {capability[0]}.{capability[1]}, "
       f"{free / 2**30:.1f} of {total / 2**30:.1f} GiB free")
    # A kernel actually running on the card, not just a driver answering.
    a = torch.randn(512, 512, device="cuda")
    b = a @ a.T
    torch.cuda.synchronize()
    if not torch.isfinite(b).all():
        fail("a matrix product on the card returned non-finite values")
    ok("a 512x512 matrix product ran on the card")
    return "cuda"


def check_dotnet() -> None:
    step(".NET SDK")
    exe = shutil.which("dotnet")
    if exe is None:
        fail("dotnet is not on PATH; the training host is started with `dotnet run`")
    sdks = subprocess.run([exe, "--list-sdks"], capture_output=True, text=True).stdout
    majors = sorted({int(m) for m in re.findall(r"^(\d+)\.", sdks, flags=re.M)})
    if not majors or max(majors) < 8:
        fail(f"need a .NET 8 SDK or newer, found: {sdks.strip() or 'none'}")
    ok(f"SDKs: {', '.join(line.split()[0] for line in sdks.splitlines())}")


def check_host() -> None:
    step("Training host: build, protocol, observation layout")
    from host_env import HostEnv
    from train import assert_observation_layout

    started = time.time()
    with HostEnv(batch=2, seed_base=1, solo=True, track="silverstone",
                 episode_seconds=60) as env:
        if env.obs_size != EXPECTED_OBSERVATION:
            fail(f"host reports {env.obs_size} observation channels, expected "
                 f"{EXPECTED_OBSERVATION}")
        obs = env.reset()
        assert_observation_layout(obs)
        env.step(obs[:, :env.action_size] * 0.0)
        if env.four_wheels_off.shape != (2,):
            fail("host did not return the four-wheel track-limits field")
    # HostEnv refuses any other protocol version during the handshake, so
    # reaching this line is the version check.
    ok(f"protocol {EXPECTED_PROTOCOL}, {EXPECTED_OBSERVATION} channels, layout "
       f"self-check passed, one step taken ({time.time() - started:.0f} s "
       "including the first build)")


def check_checkpoint(path: str, expected_sha: str | None, device: str) -> None:
    step("Checkpoint")
    if not os.path.exists(path):
        fail(f"{path} does not exist; checkpoints are not in git and have to be copied")
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    sha = digest.hexdigest()
    if expected_sha and sha != expected_sha.lower():
        fail(f"sha256 {sha} does not match the expected {expected_sha}")
    ok(f"sha256 {sha}" + (" (matches)" if expected_sha else " (no expected hash given)"))

    from host_env import HostEnv
    from sac import SacAgent, SacConfig

    with HostEnv(batch=2, seed_base=1, solo=True, track="silverstone",
                 episode_seconds=60) as env:
        agent = SacAgent(env.obs_size, env.action_size, SacConfig(device=device))
    report = agent.load(path)
    kind = "continued" if report["optimizers"] and report["rng"] else "warm start"
    ok(f"loaded onto {device}: step {report['step']}, format {report['format']}, "
       f"resume would be a {kind}")


def check_training(device: str, steps: int) -> float:
    step(f"Training smoke on {device} ({steps} steps, 16 lanes, throwaway checkpoints)")
    with tempfile.TemporaryDirectory() as scratch:
        command = [
            sys.executable, "-u", "train.py", "--solo", "--track", "silverstone",
            "--batch", "16", "--seed", "7", "--steps", str(steps),
            "--log-every", str(max(1, steps // 2)), "--eval-every", "100000000",
            "--tag", "preflight", "--device", device, "--checkpoint-dir", scratch,
        ]
        result = subprocess.run(command, capture_output=True, text=True)
    output = result.stdout + result.stderr
    if result.returncode != 0 or "training finished" not in output:
        print(output[-3000:])
        fail("the training smoke did not finish")
    rates = [int(m) for m in re.findall(r"(\d+) tps", output)]
    device_line = next((l for l in output.splitlines() if l.startswith("device:")), "")
    if device not in device_line:
        fail(f"train.py did not report running on {device}: {device_line!r}")
    ok(f"{device_line.strip()}")
    ok(f"finished; transitions per second at each log: {rates}")
    return float(rates[-1]) if rates else 0.0


def check_reference(checkpoint: str, expected: dict) -> None:
    step("Reference certification against the Mac's figures")
    from certify import certify

    r = certify(checkpoint, "silverstone", expected["seed_base"])
    found = {
        "laps": r["laps"],
        "spins": r["spins"],
        "clean_laps": r["clean_laps"],
        "fastest_clean": r["lap"],
    }
    print(f"   Mac:   {json.dumps({k: expected[k] for k in found})}")
    print(f"   here:  {json.dumps(found)}")
    problems = []
    if abs(found["laps"] - expected["laps"]) > 2:
        problems.append("lap count differs by more than two")
    if abs(found["fastest_clean"] - expected["fastest_clean"]) > 0.5:
        problems.append("fastest clean lap differs by more than half a second")
    if abs(found["clean_laps"] - expected["clean_laps"]) > 0.25 * max(expected["laps"], 1):
        problems.append("clean laps differ by more than a quarter of the session")
    if problems:
        fail("this box's world is not close to the Mac's: " + "; ".join(problems))
    ok("close to the Mac's reading (bit identity is not expected across BLAS and CUDA)")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True,
                        help="the checkpoint the bake will resume from")
    parser.add_argument("--sha256", default=None, help="its hash as recorded on the Mac")
    parser.add_argument("--allow-cpu", action="store_true",
                        help="run the other checks on a machine without CUDA")
    parser.add_argument("--train-steps", type=int, default=2000)
    parser.add_argument("--skip-train-smoke", action="store_true")
    parser.add_argument("--reference", default=None,
                        help="JSON: {checkpoint, seed_base, laps, spins, clean_laps, fastest_clean}")
    args = parser.parse_args()

    try:
        check_python()
        device = check_cuda(args.allow_cpu)
        check_dotnet()
        check_host()
        check_checkpoint(args.checkpoint, args.sha256, device)
        if not args.skip_train_smoke:
            check_training(device, args.train_steps)
        if args.reference:
            with open(args.reference, encoding="utf-8") as handle:
                expected = json.load(handle)
            check_reference(expected["checkpoint"], expected)
    except Failed as exc:
        print(f"\nPREFLIGHT FAILED: {exc}", flush=True)
        return 1
    print("\nPREFLIGHT PASSED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
