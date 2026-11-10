# Lane driver benchmark

Times `LaneDriver.GetControl` on a real circuit with a real baked table, so a
change to the planner can be judged on what it costs and not only on what it
drives.

```powershell
dotnet run --project Drivers/Bake/Bench -c Release -- `
    official:silverstone .tmp/joint-silverstone/five-runtime.table - 1 solo 600
```

Arguments, in order:

| # | argument | meaning |
|---|---|---|
| 1 | `circuit` | `official:silverstone` and the like |
| 2 | `table` | the joint table to plan on |
| 3 | `geometry` | the mapped geometry, or `-` for none |
| 4 | `cars` | 1 for a solo car, 2 for a duel |
| 5 | `mode` | `solo`, `duel`, or `look` (the beam lookahead) |
| 6 | `steps` | how many 60 Hz steps to time |
| 7 | `flags` | `exact` for exact braking, `noprotect` to drop the defence |

The car is added with no driver and a proxy that returns whatever the driver
last decided, so the stopwatch covers the decision alone while the world still
advances on the real plan.

Reported: the mean, p50, p95 and max over every step, then the same split
between the steps that planned and the steps that did not — the two are very
different, and the mean over all steps hides which one is expensive.
