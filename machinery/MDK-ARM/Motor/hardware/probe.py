import argparse
import json
import re
import struct
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "build" / "pid-probe"
HOME_DIR = Path.home()
EXTENSION = HOME_DIR / ".vscode/extensions/cl.eide-3.27.2"
OPENOCD_ROOT = HOME_DIR / ".eide/tools/openocd_7a1adfbec_mingw32"


def run_process(arguments, log_name, timeout=30):
    result = subprocess.run(arguments, cwd=ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=timeout)
    output = result.stdout.decode("utf-8", errors="replace")
    (OUTPUT / log_name).write_text(output, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(output)
    return output


def openocd(commands, log_name):
    return run_process([
        str(OPENOCD_ROOT / "bin/openocd.exe"),
        "-s", str(OPENOCD_ROOT / "share/openocd/scripts"),
        "-f", "interface/cmsis-dap.cfg", "-c", "transport select swd",
        "-c", "adapter speed 1000", "-f", "target/stm32f1x.cfg",
        "-c", commands,
    ], log_name)


def build():
    params = json.loads((ROOT / "build/machinery/builder.params").read_text())
    params.update(name="pid_probe", target="pid_probe",
                  outDir="build/pid-probe", dumpPath="build/pid-probe")
    params["sourceList"] = [source for source in params["sourceList"]
                            if Path(source).name != "main.c"]
    params["sourceList"].append("Motor/hardware/pid_probe_main.c")
    hal_source = next(source for source in params["sourceList"]
                      if Path(source).name == "stm32f1xx_hal.c")
    params["sourceList"].append(str(Path(hal_source).with_name("stm32f1xx_hal_iwdg.c")))
    params["defines"].append("HAL_IWDG_MODULE_ENABLED")
    params["env"].update(ProjectName="pid_probe", ConfigName="pid_probe",
                         OutDir=str(OUTPUT), OutDirBase="build/pid-probe",
                         ExecutableName=str(OUTPUT / "pid_probe"))
    params_path = OUTPUT / "builder.params"
    params_path.write_text(json.dumps(params, indent=2), encoding="utf-8")
    print(run_process([str(EXTENSION / "res/tools/win32/unify_builder/unify_builder.exe"),
                       "-p", str(params_path), "--rebuild", "--no-color"], "build.log"))


def symbol_address(name):
    map_text = (OUTPUT / "pid_probe.map").read_text()
    match = re.search(r"^\s*" + re.escape(name) + r"\s+(0x[0-9a-fA-F]+)\s+Data", map_text, re.M)
    if not match:
        raise RuntimeError("Missing debug symbol: " + name)
    return int(match.group(1), 16)


def float_word(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def measure(args):
    control = symbol_address("pidProbe")
    trace = symbol_address("pidTrace")
    prefix = OUTPUT / args.name
    if prefix.parent != OUTPUT or not re.fullmatch(r"[a-zA-Z0-9_-]+", args.name):
        raise ValueError("Invalid run name")
    if prefix.with_suffix(".json").exists():
        raise ValueError("Run already exists; choose a new name")
    if not (100 <= args.duration <= 2000 and 0 < args.limit <= 300):
        raise ValueError("Duration must be 100..2000 ms; limit 1..300 permille")
    if args.mode == "open" and not (0 < args.target <= args.limit):
        raise ValueError("Open-loop duty exceeds limit")
    fields = [args.channel - 1, args.duration, float_word(args.limit),
              float_word(args.target), float_word(args.kp), float_word(args.ki),
              float_word(args.kd), args.sign & 0xFFFFFFFF]
    openocd(f"init; dump_image {{{prefix.as_posix()}-before.bin}} 0x{control:x} 64; shutdown",
            args.name + "-before.log")
    before = struct.unpack("<16I", Path(str(prefix) + "-before.bin").read_bytes())
    if before[0] != 0x50494431 or before[2] not in (1, 3, 4) or before[13:15] != (44, 320):
        raise RuntimeError("Expected idle diagnostic firmware; refusing to start")
    commands = ["init"]
    commands.append(f"mww 0x40000028 {72000000 // (3600 * args.pwm_hz) - 1}")
    commands.append("mww 0x40000014 1")
    for index, value in enumerate(fields):
        commands.append(f"mww 0x{control + 16 + index * 4:x} 0x{value:x}")
    flags = int(args.pullups) | (2 if args.decay == "slow" else 0)
    commands.append(f"mww 0x{control + 60:x} {flags}")
    commands.append(f"mww 0x{control + 4:x} {1 if args.mode == 'open' else 2}")
    commands.append("sleep 280")
    commands.append(f"dump_image {{{prefix.as_posix()}-active-tim2.bin}} 0x40000000 68")
    commands.append(f"sleep {args.duration + 720}")
    commands.append(f"dump_image {{{prefix.as_posix()}-control.bin}} 0x{control:x} 64")
    commands.append(f"dump_image {{{prefix.as_posix()}-trace.bin}} 0x{trace:x} 14080")
    commands.append(f"dump_image {{{prefix.as_posix()}-pwm.bin}} 0x40000034 16")
    commands.append("shutdown")
    openocd("; ".join(commands), args.name + ".log")
    header = struct.unpack("<6I5fi4I", Path(str(prefix) + "-control.bin").read_bytes())
    if header[0] != 0x50494431 or header[13] != 44 or header[12] > 320:
        raise RuntimeError("Firmware or trace layout mismatch")
    trace_bytes = Path(str(prefix) + "-trace.bin").read_bytes()
    samples = []
    keys = ["time_ms", "delta_a", "delta_b", "cps_a", "cps_b", "target_cps", "duty_permille", "phase",
            "gpio_levels", "edges_a", "edges_b"]
    for index in range(header[12]):
        samples.append(dict(zip(keys, struct.unpack_from("<Iii4f4I", trace_bytes, index * 44))))
    pwm = struct.unpack("<4I", Path(str(prefix) + "-pwm.bin").read_bytes())
    active_timer = struct.unpack("<17I", Path(str(prefix) + "-active-tim2.bin").read_bytes())
    active = [sample for sample in samples if sample["phase"] == 1]
    edges = {"PA6": sum(sample["edges_a"] & 65535 for sample in active),
             "PA7": sum(sample["edges_a"] >> 16 for sample in active),
             "PB6": sum(sample["edges_b"] & 65535 for sample in active),
             "PB7": sum(sample["edges_b"] >> 16 for sample in active)}
    channel_key = "cps_a" if args.channel == 1 else "cps_b"
    tail = [sample[channel_key] * args.sign for sample in active
            if sample["time_ms"] >= args.duration + 200 - min(500, args.duration // 2)]
    summary = {"status": header[2], "fault": header[3], "sample_count": header[12],
               "final_pwm_ccr": pwm, "active_pwm_ccr": active_timer[13:17],
               "active_timer_cr1": active_timer[0], "active_timer_ccer": active_timer[8],
               "active_timer_psc": active_timer[10],
               "active_gpio_edges": edges,
               "tail_mean_cps": sum(tail) / len(tail) if tail else None,
               "total_counts_a": sum(sample["delta_a"] for sample in samples),
               "total_counts_b": sum(sample["delta_b"] for sample in samples)}
    report = {"parameters": vars(args), "summary": summary, "samples": samples}
    prefix.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    if any(pwm) or header[2] not in (3, 4):
        openocd("init; reset run; shutdown", args.name + "-reset.log")
        raise RuntimeError("Unexpected final state; reset to idle")


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("build")
    sub.add_parser("flash")
    sub.add_parser("restore")
    measure_parser = sub.add_parser("measure")
    measure_parser.add_argument("--name", required=True)
    measure_parser.add_argument("--channel", type=int, choices=[1, 2], required=True)
    measure_parser.add_argument("--mode", choices=["open", "pid"], required=True)
    measure_parser.add_argument("--target", type=float, required=True)
    measure_parser.add_argument("--limit", type=float, default=200)
    measure_parser.add_argument("--duration", type=int, default=1500)
    measure_parser.add_argument("--sign", type=int, choices=[-1, 1], default=1)
    measure_parser.add_argument("--kp", type=float, default=0)
    measure_parser.add_argument("--ki", type=float, default=0)
    measure_parser.add_argument("--kd", type=float, default=0)
    measure_parser.add_argument("--pullups", action="store_true")
    measure_parser.add_argument("--decay", choices=["fast", "slow"], default="fast")
    measure_parser.add_argument("--pwm-hz", type=int, choices=[1000, 20000], default=20000)
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if args.action == "build":
        build()
    elif args.action == "flash":
        print(openocd("program build/pid-probe/pid_probe.hex verify reset exit", "flash.log"))
    elif args.action == "restore":
        print(openocd("program build/machinery/machinery.hex verify reset exit", "restore.log"))
    else:
        measure(args)


if __name__ == "__main__":
    main()
