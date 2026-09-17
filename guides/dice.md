# Dice and shell face-up decoding

`MobRapier.Dice` answers one question: given a settled body's
orientation quaternion, which face is pointing up? Physics does not
know or care — the mapping is a rule about a specific shape's local
axes.

Each rule lives in one function so consumers (dice screens, chopaat's
cowrie shells, future `mob_rapier` embedders) share one authority
instead of re-deriving the mapping per screen.

## Shapes covered

| Function                 | Shape                                                      |
| ------------------------ | ---------------------------------------------------------- |
| `face_up_d6/1`           | Standard cube. Opposite faces sum to 7 (1↔6, 2↔5, 3↔4).    |
| `face_up_d10/1`          | [Pentagonal trapezohedron][d10] — 10 kite faces.           |
| `face_up_d12/1`          | [Regular dodecahedron][d12] — 12 pentagonal faces.         |
| `face_up_d20/1`          | [Regular icosahedron][d20] — 20 triangular faces.          |
| `face_up_cowrie/1`       | Cowrie shell — convex-up vs concave-up binary.             |

[d10]: https://en.wikipedia.org/wiki/Pentagonal_trapezohedron
[d12]: https://en.wikipedia.org/wiki/Regular_dodecahedron
[d20]: https://en.wikipedia.org/wiki/Regular_icosahedron

## The decode recipe

Given a settled quaternion `q` and a table of face normals in the
body's local frame:

1. Rotate every face normal by `q` (so it now points in world space).
2. Pick the one with the largest `+Y` component — that face is
   pointing up in a world with gravity `{0, -9.81, 0}`.

That is the entire rule. What varies per shape is *what "the face
normals" are*.

## Face normals from vertex tables

The d12 and d20 tables use the dual-polyhedron identity to avoid
duplicating data:

- **d12 face normals** are directions to the **vertices of the dual
  icosahedron**. `d12_face_normals/0` reads them straight off
  `icosahedron_vertices/0`.
- **d20 face normals** are directions to the **vertices of the dual
  dodecahedron**. `d20_face_normals/0` reads them off
  `dodecahedron_vertices/0`.

The [dual polyhedron][dual] relationship guarantees this: swap "face"
and "vertex" and you get the dual. There is no separate face-normal
set to keep in sync with vertex changes — edit the vertex table and
the face normals move with it.

[dual]: https://en.wikipedia.org/wiki/Dual_polyhedron

## d10 kite centroids

A pentagonal trapezohedron's kite faces are not planar under the naive
vertex placement `MobRapier.Dice` uses, so a face-normal per polygon
is not defined the way it is for the regular d12 and d20. Instead
`d10_face_normals/0` uses analytic kite centroids computed from the
vertex table — the direction from the die's centre to each kite's
centroid. That is the direction the settled body's rotation takes
back to world space and, again, the max `+Y` one wins.

## Cowrie shell

The cowrie is not a die — the decode is convex-up vs concave-up. The
rule projects the shell's local `+Y` axis (its dorsal / convex side)
into world space via the settled quaternion and reads the sign of the
`+Y` component:

- `+Y > 0` → convex-up
- `+Y < 0` → concave-up

For the game rules that give this a value (three concave-up in seven
counts as a jackpot, etc.), see [chopaat][chopaat].

[chopaat]: https://github.com/GenericJam/chopaat

## Assumptions

- The world's up is `+Y`. This matches the gravity vector
  `MobRapier.Physics.world_new/0` sets (`{0, -9.81, 0}`); ground
  surface is `y = 0` and "up" from a settled body is `+Y`.
- The body has actually settled. A rotating body's face-up answer is
  meaningless — see [physics tuning][tuning] for how to drive the
  velocity thresholds that decide when a screen calls `face_up_*`.

[tuning]: physics_tuning.md
