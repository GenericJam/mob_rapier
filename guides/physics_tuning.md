# Physics tuning

`mob_rapier` wraps [Rapier 3D][rapier-repo] behind a small NIF. Every
knob this package exposes at the Elixir level maps one-to-one to a
Rapier concept — the same names, the same semantics, the same defaults
(the ones Rapier picks when you do not override). If a value here
surprises you, the fastest path to a mental model is to open Rapier's
own docs at the matching section.

[rapier-repo]: https://github.com/dimforge/rapier

## Reference

- [Rapier user guide (Rust)][user-guide] — the canonical explanation
  of rigid bodies, colliders, damping, restitution, friction, sleeping,
  CCD and the solver
- [`rapier3d` API docs on docs.rs][docs-rs] — every builder and every
  parameter, cross-linked
- [Rapier home page][rapier-home] — high-level intro and demos

[user-guide]: https://rapier.rs/docs/user_guides/rust/getting_started
[docs-rs]: https://docs.rs/rapier3d/latest/rapier3d/
[rapier-home]: https://rapier.rs/

## What ships out of the box

`native/lab_physics/src/lib.rs` sets defaults that the demo shells and
dice needed to settle in a small tabletop-scale arena (~60 cm on a side,
bodies ~1 cm to a few cm). They are constants at the top of that file
and every dynamic body / dynamic collider goes through the helpers that
apply them:

| Knob                         | Value  | Rapier reference                                                     |
| ---------------------------- | ------ | -------------------------------------------------------------------- |
| `COLLIDER_DENSITY`           | 1000.0 | [Density → mass][density] — mass driven by collider volume × density |
| `LINEAR_DAMPING`             | 0.5    | [Damping][damping] — first-order drag on linear velocity             |
| `ANGULAR_DAMPING`            | 2.0    | [Damping][damping] — first-order drag on angular velocity            |
| `COLLIDER_FRICTION`          | 0.7    | [Friction][friction] — Coulomb friction coefficient                  |
| `COLLIDER_RESTITUTION`       | 0.15   | [Restitution][restitution] — bounciness (0 = inelastic, 1 = elastic) |

[density]: https://rapier.rs/docs/user_guides/rust/colliders#mass-properties
[damping]: https://rapier.rs/docs/user_guides/rust/rigid_bodies#damping
[friction]: https://rapier.rs/docs/user_guides/rust/colliders#friction
[restitution]: https://rapier.rs/docs/user_guides/rust/colliders#restitution

### Why these values

- **Density 1000 kg/m³ (water)** — Rapier's default is 1 kg/m³, which
  for a 1 cm³ body works out to 1 mg. Impulses sized for a 1 mg body
  send a shell straight through arena walls; density 1000 gives ~1 g
  per cm³ of volume so impulse and mass live on the same scale.
- **Linear damping 0.5, angular damping 2.0** — small tabletop bodies
  benefit from angular damping >> linear damping: without it, a shell
  that lands convex-side-down keeps spinning about its long axis for
  many seconds. See [Rapier damping][damping].
- **Friction 0.7** — high enough that a settled body does not slide on
  the arena floor under residual velocity from neighbouring impacts;
  low enough that impulses still take.
- **Restitution 0.15** — low bounce keeps a shake compact. Values near
  1 turn the arena into a pinball table.

### Sleeping

Auto-sleep is Rapier's built-in mechanism for stopping the solver from
touching a body whose kinetic energy has stayed below a threshold for
long enough; see [Sleeping][sleeping]. `mob_rapier` uses Rapier's
defaults for sleep thresholds and time-until-sleep — override them in
the crate if a specific consumer needs stricter settling.

[sleeping]: https://rapier.rs/docs/user_guides/rust/rigid_bodies#sleeping

## Where the constants live

The tuning surface is a single Rust module — treat it as the source of
truth:

- `native/lab_physics/src/lib.rs` — constants + `build_dynamic_rb`,
  `tune_collider`, `tune_static_collider` helpers. Every dynamic body
  constructor (`add_ball`, `add_cuboid`, `add_oblate`, `add_convex_hull`)
  runs its collider through `tune_collider`; the ground plane and every
  static wall run through `tune_static_collider`.

If a consumer needs different values for a specific world, the cheap
answer is to fork the crate and adjust the constants; the crate is
small on purpose. A per-world tuning API is a follow-up if consumers
converge on needing one.

## Reading physics back

Every tuning decision has a knob on the read side too — you don't have
to guess whether damping is working:

- `MobRapier.Physics.transforms/1` — position + orientation per body
  each tick. Diff two ticks for linear/angular velocity numerators.
- `MobRapier.Physics.contacts/1` — collision + contact-force events for
  a step (a `ChannelEventCollector`). The event buffer caps at 1024
  entries per channel with a `contacts_dropped` counter; a rising
  counter means the world is too energetic for the current solver
  budget.
- Rapier's own `RigidBody::linvel()`, `RigidBody::angvel()` and
  `is_sleeping()` are the ground truth — expose them from the crate
  when a screen wants to visualise settling directly rather than infer
  it from transforms.
