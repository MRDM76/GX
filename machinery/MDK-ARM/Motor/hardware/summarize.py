import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "build/pid-probe"
RUNS = [
    "cn1-pi3000-1k-v1", "cn2-pi3000-1k-v1", "cn1-pi6000-1k-v1",
    "cn2-pi6000-1k-v1", "cn2-pi6000-1k-repeat",
]


def main():
    summaries = []
    for name in RUNS:
        report = json.loads((OUTPUT / (name + ".json")).read_text())
        parameters = report["parameters"]
        channel = parameters["channel"]
        target = parameters["target"]
        speed_key = "cps_a" if channel == 1 else "cps_b"
        active = [sample for sample in report["samples"] if sample["phase"] == 1]
        tail = [sample for sample in active if sample["time_ms"] >= parameters["duration"] - 300]
        speeds = [sample[speed_key] for sample in tail]
        mean = sum(speeds) / len(speeds)
        summary = {
            "run": name, "channel": channel, "target_counts_per_second": target,
            "last_500ms_mean": mean, "last_500ms_min": min(speeds),
            "last_500ms_max": max(speeds), "mean_error_percent": 100 * (mean - target) / target,
            "last_500ms_mean_pwm_percent": sum(sample["duty_permille"] for sample in tail) / len(tail) / 10,
            "whole_active_peak_counts_per_second": max(sample[speed_key] for sample in active),
            "status": report["summary"]["status"], "fault": report["summary"]["fault"],
        }
        summaries.append(summary)
    (OUTPUT / "pid-results.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
