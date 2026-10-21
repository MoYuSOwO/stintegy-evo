# Learned lap times — `formula_a_mixed`

Flying laps, seconds, **no push-to-pass**. `hot` is the default tyre/power,
`attack` is `TireUsageMode.Attack` with `PowerOutputMode.Normal`.

- `ilc5` / `ilc6` are the previous line sets from the earlier learning runs.
- `ilc6-new` is the current set: **2.0 m car / 1.1 m edge margin**, new bakes,
  and for Zandvoort the manual 19°/18° banking + manual grade.
- All four grip levels (`0.55 / 0.70 / 0.85 / 1.00`) are now learned:
  the first pass learned the full-grip `1.00`; the lower three were appended
  without changing the full-grip factors or steering. The `hot` / `attack`
  table above is still the full-grip `1.00` result.
- The old ilc6 Baku switch had `wall 0.02 s` before the margin fix. All current
  `ilc6-new` rows below have `off 0 / wall 0`.
- `pending` means the new bake/learner has not produced a saved result yet.

| Circuit | ilc5 hot | ilc5 attack | ilc6 hot | ilc6 attack | ilc6-new hot | ilc6-new attack | ilc6-new switch | off / wall |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Baku | 92.61 | 92.34 | 92.69 | 92.38 | 93.27 | **92.94** | 98.77 | 0 / 0 |
| Monaco | 66.75 | 66.22 | 66.13 | 65.74 | 66.74 | **66.41** | 71.79 | 0 / 0 |
| Zandvoort | 70.36 | 69.74 | 70.12 | 69.53 | 71.95 | **71.34** | 74.34 | 0 / 0 |
| Spa | 101.26 | 100.70 | 101.09 | 100.70 | 101.35 | **100.98** | 104.57 | 0 / 0 |
| Interlagos | 69.44 | 69.00 | 69.43 | 69.09 | 69.53 | **69.15** | 71.74 | 0 / 0 |
| Monza | 78.37 | 78.10 | 78.20 | 77.87 | 78.75 | **78.48** | 82.37 | 0 / 0 |
| Singapore | 87.19 | 86.62 | 86.94 | 86.50 | 87.20 | **86.78** | 93.15 | 0 / 0 |
| Portimao | — | — | — | — | 76.42 | **76.00** | 79.95 | 0 / 0 |
| Sepang | — | — | — | — | 89.30 | **88.78** | 91.10 | 0 / 0 |
| Shanghai | — | — | — | — | 91.59 | **91.14** | 93.56 | 0 / 0 |
| Silverstone | — | — | — | — | 87.43 | **87.07** | 88.99 | 0 / 0 |

## Notes

- `ilc5` values are from the previous learning handoff; `ilc6` is the run
  before the 1.1 m edge margin. Both are kept for comparison, not as targets.
- The new Zandvoort surface is intentionally slower than the old aggressive
  bank: 19°/18° with 80/90 m transitions vs 21.5°/19.2° with 35/45 m.
- Silverstone: the learner's flying attack is **87.07 s (1:27.07)** with
  **no push-to-pass**. In-game with push-to-pass the observed lap is
  **1:26.5**. A qualifying **800 kW / low-fuel** run is expected a few
  seconds faster, but it has not been measured yet.
- Shanghai: the learner's flying attack is **91.14 s (1:31.14)** with no
  push-to-pass. In-game with push-to-pass the observed lap is **1:30.301**.
  Real F1 2025 pole is **1:30.641** (2024 pole 1:33.660).
- New lines are learned from `/tmp/bake-all/<circuit>/<circuit>.lanes3d.csv`
  and saved under `/tmp/learn-new/<circuit>/stintegy-lanes/<circuit>/`.
- The F1E is an independent formula; these values are not F1 targets.
