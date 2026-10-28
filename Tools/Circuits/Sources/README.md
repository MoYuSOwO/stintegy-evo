# Source track centreline data

These are the sources `Tools/Circuits` makes the circuit folders under
`mods/official/data/circuits/` from; nothing in the game reads them directly.

`Silverstone.csv` is sourced from
[TUMFTM/racetrack-database](https://github.com/TUMFTM/racetrack-database/blob/2b44832e938e707103a0975ff5677a6be80362c4/tracks/Silverstone.csv)
at commit `2b44832e938e707103a0975ff5677a6be80362c4`, licensed under
LGPL-3.0; the complete upstream license is included as `LICENSE.LGPL-3.0`.
The upstream centreline originates from OpenStreetMap data and was
smoothed by the Technical University of Munich's Institute of Automotive
Technology.

The four columns are centreline `x`, centreline `y`, right track width and left
track width, all in metres. When its circuit folder is written, the closed centreline is uniformly
scaled and resampled to the FIA-published 5,891 metre Arena Grand Prix length.

`Monaco.csv` is projected from
[bacinger/f1-circuits](https://github.com/bacinger/f1-circuits/blob/b8c7e17c528d54b65b3aa3b0e14a3278bf96b5ac/circuits/mc-1929.geojson)
at commit `b8c7e17c528d54b65b3aa3b0e14a3278bf96b5ac`, licensed under
MIT; the complete upstream license is included as `LICENSE.bacinger-MIT`.
Longitude and latitude are projected to local metres, the duplicated closing
point is removed, and the width is measured roughly per section from
OpenStreetMap (see below). When its circuit folder is written, the centreline is scaled to the FIA-published 3,337 metre
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
scaling barely moves it. The source LineString already begins on the
start/finish straight, so unlike Monaco it needs no rotation. Width
comes from the circuit's own OpenStreetMap tags (see below): about ten
metres, a narrow permanent circuit, which is what Zandvoort is.

`Baku.csv`, `Spa.csv`, `Monza.csv` and `Interlagos.csv` are projected from
[bacinger/f1-circuits](https://github.com/bacinger/f1-circuits) (`az-2016`,
`be-1925`, `it-1922`, `br-1940`), licensed under MIT; the complete upstream
license is included as `LICENSE.bacinger-MIT`. Longitude and latitude are
projected to local metres about each ring's centroid and duplicated closing
points are removed; projected lengths land within 1.1% of the published
6,003, 7,004, 5,793 and 4,309 metres that they are scaled to. Baku
and Spa are rotated to start on a straight (source points 83 and 118). Baku's
widths are measured roughly per section from OpenStreetMap (see below),
except the castle squeeze, which is kept at 7.6 m, the narrowest point in
the sport; Spa, Monza and Interlagos have no usable widths there and carry
a flat 13, 12 and 12 m. Elevation is authored in the style of each
venue, not surveyed.

`Singapore.csv` and `Portimao.csv` follow the same recipe from the same
MIT-licensed source (`sg-2008`, `pt-2008`): centroid projection, duplicate
closing point removed, projected lengths within 0.3% of the published 4,928
and 4,653 metres. Singapore is rotated to start on a straight (source point
21). Widths are measured roughly per section from OpenStreetMap (see below), and
elevation is authored in the style of each venue: Marina Bay honestly flat
but for its bridges, the Algarve rollercoaster with its plunge past ten
percent.

## Widths from OpenStreetMap

The per-section widths of `Monaco.csv`, `Baku.csv`, `Singapore.csv`,
`Zandvoort.csv` and `Portimao.csv` are derived from OpenStreetMap data,
© OpenStreetMap contributors, available under the Open Database License
(ODbL 1.0, https://opendatacommons.org/licenses/odbl/). Those two width
columns are a derived database under that licence; the centreline columns
remain under their own sources' terms.

They are rough, by design. Across the centreline every two metres, the
first pavement (a footway running alongside the road, taken to stand a
metre back from the kerb) on either side gives the road's edge; where a
circuit's own way carries a `width` tag, that is used instead. Each source
point takes the median of what was found within twelve metres of it,
rounded to half a metre; points where nothing believable was found are
filled in from their neighbours. A street circuit's barriers stand on the
kerb line (`Layouts.StreetBarrierMeters`).
