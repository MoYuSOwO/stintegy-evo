# The driver catalogue

`manifest.json` is the list of drivers the game can field, keyed by car and
circuit. It is a list of rows, each one:

| field | meaning |
|---|---|
| `car` | which car this driver was baked for; `default` while there is one |
| `track` | which circuit; a driver is baked per circuit, not per fleet |
| `weights` | the exported network, relative to this directory |
| `formatVersion` | which observation contract the network speaks |
| `certifiedOn` | the date it passed large-sample certification |
| `cleanLapSeconds` | the clean lap it passed with |

**The key set is the content list.** A circuit appears here when a policy
has been baked for it and has passed certification, and the game offers
exactly the circuits that appear here. Asking for one that is absent is an
error that says so; there is deliberately nothing to fall back to, because
the analytic driver that could once have stood in was retired as a
baseline and a silent substitution would put a player against something
nobody graded while the game said nothing.

## Adding one

    certify.py  ->  passes  ->  export_policy.py  ->  add a row here

The two graduation fields are filled in at that moment rather than
reconstructed later. They exist because a driver profile screen will want
them, and the only time the number is reliably known is the moment it was
earned.

## The current row is stale, and knowingly so

`silverstone-expert.nn` speaks format version 1, which is the observation
contract from before the resource slots and descriptor block were added.
It cannot be fed by this build. It stays listed, with its true version, so
that the catalogue describes what is actually on disk; the row is replaced
when the parent policy is baked and certified against the current contract.
