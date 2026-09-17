# mob_rapier

Rapier 3D physics as a Rustler NIF, plus the named-world registry and
face-up decode rules the rapier_lab spike proved out.

Extracted from [`rapier_lab`](https://github.com/GenericJam/rapier_lab)
after the API stabilised across the dice (d6, d10, d12, d20) and shells
demos — bead `rapier_lab-d00`. `rapier_lab` now depends on this package
and keeps only the demo screens; the physics primitives live here.

## What ships

- **`MobRapier.Physics`** — the Rustler NIF facade. `world_new/0`, plus
  the shape-family constructors (`add_ball`, `add_cuboid`,
  `add_static_cuboid`, `add_oblate`, `add_convex_hull`), impulses
  (`apply_impulse`, `apply_torque_impulse`), the step + read pipeline
  (`step`, `transforms`, `contacts`, `step_with_contacts_in`), and a
  by-name registry (`new_world`, `add_*_in`, `step_in`, …) that lets
  agents drive a world over Erlang distribution without dragging opaque
  NIF resource handles across the wire.

- **`MobRapier.Physics.Registry`** — the named-world storage backing the
  `_in` API.

- **`MobRapier.Dice`** — vertex tables for the d10 / d12 / d20 convex
  hulls, and the face-up decode rules for d6 / d10 / d12 / d20 dice plus
  the up/down cowrie-shell decode. Face-normal tables use the
  dual-polyhedron identity (dodecahedron faces = icosahedron vertices
  and vice versa; trapezohedron faces = pentagonal antiprism vertices)
  so there's no separate face-normal set to keep in sync with vertex
  changes.

## Host integration

`mix mob.deploy` cross-compiles the Rust crate for iOS + Android when
the host's `mob.exs` registers it as a static NIF:

```elixir
# mob.exs
config :mob_dev,
  static_nifs: [
    %{module: :lab_physics, archs: [:all]}
  ]
```

The crate name is `lab_physics` for historical reasons — renaming it
churns more than it clarifies.

## Native crate

`native/lab_physics/` wraps `rapier3d` 0.22 behind a small Rustler
surface. Contact events flow through a `ChannelEventCollector` per step;
the buffer caps at 1024 events per channel with a `contacts_dropped`
counter for saturation detection (bead `rapier_lab-bvr`).
