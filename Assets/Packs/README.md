# Content packs

Everything the game can field lives in a pack, and the built-in car is
`000-base` — pack zero, which declares itself exactly the way a mod does.
Official content goes through the mod door on purpose: a format only
modders use is a format that breaks and nobody notices until a modder
complains.

    <pack>/
      pack.json                 { "id", "car", "label" }
      drivers/
        <circuit>.nn            the exported network
        <circuit>.json          { "formatVersion", "certifiedOn", "cleanLapSeconds" }

**The file names the circuit.** `drivers/silverstone.nn` is the driver for
`silverstone`; there is nothing written down twice and therefore nothing
that can disagree with itself.

A driver may be supplied by whoever has one: a car pack, a circuit pack, or
a pack that is nothing but drivers. A circuit mod can ship a driver for the
official car, and a car mod can ship drivers for the official circuits. The
key is the (car, circuit) pair and who supplied it is not part of it.

## Where they are read from

    res://Assets/Packs      ships with the game
    user://packs            installed by the player

## When two packs claim the same slot

1. A player's pack beats a built-in one — installing something is asking
   for it.
2. Within a tier, the newer certificate wins.
3. A certified driver beats an uncertified one.
4. A tie the rules cannot break is an error naming both packs, because
   quietly picking one is the same silent substitution that the
   missing-driver rule exists to prevent.

## The sidecar

A driver with no sidecar still loads. It claims no contract version and no
certificate, so it loses every conflict it takes part in and shows nothing
on a profile screen — better than refusing a mod over a missing file, and
honest about what is unknown.

## Shipping one

    certify.py  ->  passes  ->  export_policy.py  ->  drop into a pack's drivers/

No global file is edited. The catalogue is a scan result, not a list
somebody maintains.
