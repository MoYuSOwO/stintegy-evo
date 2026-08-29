# Embedded track centreline data

`Silverstone.csv` is sourced from
[TUMFTM/racetrack-database](https://github.com/TUMFTM/racetrack-database/blob/2b44832e938e707103a0975ff5677a6be80362c4/tracks/Silverstone.csv)
at commit `2b44832e938e707103a0975ff5677a6be80362c4`, licensed under
LGPL-3.0; the complete upstream license is included as `LICENSE.LGPL-3.0`.
The upstream centreline originates from OpenStreetMap data and was
smoothed by the Technical University of Munich's Institute of Automotive
Technology.

The four columns are centreline `x`, centreline `y`, right track width and left
track width, all in metres. At runtime the closed centreline is uniformly
scaled and resampled to the FIA-published 5,891 metre Arena Grand Prix length.

`Monaco.csv` is projected from
[bacinger/f1-circuits](https://github.com/bacinger/f1-circuits/blob/b8c7e17c528d54b65b3aa3b0e14a3278bf96b5ac/circuits/mc-1929.geojson)
at commit `b8c7e17c528d54b65b3aa3b0e14a3278bf96b5ac`, licensed under
MIT; the complete upstream license is included as `LICENSE.bacinger-MIT`.
Longitude and latitude are projected to local metres, the duplicated closing
point is removed, and a conservative 10.5 metre street-circuit width is
assigned. At runtime the centreline is scaled to the FIA-published 3,337 metre
length. The source LineString starts near Massenet/Casino, so it is rotated to
source point 118, the closest supplied point to the modern start/finish line on
Boulevard Albert 1er, before spline generation.

`Shanghai.csv` is sourced from the same TUM FTM database and commit as
Silverstone. Its smoothed centreline and satellite-derived left/right widths
are retained, then uniformly scaled from 5,445.25 metres to the FIA-published
5,451 metre centreline length.

`Sepang.csv` is sourced from the same TUM FTM database and commit. The
centreline is retained and scaled from 5,537.35 metres to the official 5,543
metres. The source left/right width proportions are preserved while total width
is normalized to the circuit operator's published 16 metre minimum and 22
metre maximum.

`Zandvoort.csv` is projected from
[bacinger/f1-circuits](https://github.com/bacinger/f1-circuits/blob/master/circuits/nl-1948.geojson),
licensed under MIT; the complete upstream license is included as
`LICENSE.bacinger-MIT`. Longitude and latitude are projected to local metres
about the ring's centroid and the duplicated closing point is removed. The
projected ring measures 4,260.2 metres against the published 4,259, so the
runtime scaling barely moves it. The source LineString already begins on the
start/finish straight, so unlike Monaco it needs no rotation. Width is
assigned rather than sourced: twelve metres, which is a narrow permanent
circuit and is what Zandvoort is.
