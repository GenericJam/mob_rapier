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
| `LINEAR_DAMPING`             | 3.0    | [Damping][damping] — first-order drag on linear velocity             |
| `ANGULAR_DAMPING`            | 20.0   | [Damping][damping] — first-order drag on angular velocity            |
| `COLLIDER_FRICTION`          | 0.7    | [Friction][friction] — Coulomb friction coefficient                  |
| `COLLIDER_RESTITUTION`       | 0.05   | [Restitution][restitution] — bounciness (0 = inelastic, 1 = elastic) |
| `SLEEP_LINVEL`               | 0.1    | [Sleeping][sleeping] — per-body `normalized_linear_threshold`        |
| `SLEEP_ANGVEL`               | 1.5    | [Sleeping][sleeping] — per-body `angular_threshold`                  |
| `SLEEP_DWELL`                | 0.3    | [Sleeping][sleeping] — per-body `time_until_sleep`, seconds          |

[density]: https://rapier.rs/docs/user_guides/rust/colliders#mass-properties
[damping]: https://rapier.rs/docs/user_guides/rust/rigid_bodies#damping
[friction]: https://rapier.rs/docs/user_guides/rust/colliders#friction
[restitution]: https://rapier.rs/docs/user_guides/rust/colliders#restitution
[sleeping]: https://rapier.rs/docs/user_guides/rust/rigid_bodies#sleeping

### Why these values

- **Density 1000 kg/m³ (water)** — Rapier's default is 1 kg/m³, which
  for a 1 cm³ body works out to 1 mg. Impulses sized for a 1 mg body
  send a shell straight through arena walls; density 1000 gives ~1 g
  per cm³ of volume so impulse and mass live on the same scale.
- **Linear damping 3.0, angular damping 20.0** — telemetry (see
  "Reading physics back" below) showed stacked resting shells drifting
  3-6 mm/s and rotating 15-25°/s under the contact solver's per-step
  energy injection even after damping at 0.5 / 2.0. The current values
  are tuned to pull that residual under `SLEEP_LINVEL` /
  `SLEEP_ANGVEL` within `SLEEP_DWELL` so rapier's own auto-sleep
  actually engages — the recommended-in-Rapier way to detect "at rest"
  rather than hand-tuning velocity thresholds outside the solver.
- **Friction 0.7** — high enough that a settled body does not slide on
  the arena floor under residual velocity from neighbouring impacts;
  low enough that impulses still take.
- **Restitution 0.05** — low bounce keeps the shake compact. Stacked
  resting bodies otherwise re-inject energy into each other via the
  contact solver every step and never settle. Values near 1 turn the
  arena into a pinball table.

### Sleeping

Auto-sleep is Rapier's built-in mechanism for stopping the solver from
touching a body whose kinetic energy has stayed below a threshold for
long enough; see [Sleeping][sleeping]. Rapier's defaults
(`normalized_linear_threshold = 0.4`, `angular_threshold = 0.5`,
`time_until_sleep = 2.0`) are sized for room-scale bodies — for a 3 cm
shell in a small arena the linear threshold is too loose (40 cm/s is
a decent throw, not "at rest") and the angular threshold is too tight
(the contact solver's per-step ω injection routinely exceeds it).

Every dynamic body's `activation` is overridden via `activation_mut()`
in `build_dynamic_rb` to the values in the table above, so
`is_sleeping()` and the telemetry stream both flip cleanly once bodies
come to rest. Prefer the sleep flag over any velocity threshold in
consumer code — it is the value the solver itself gates on.

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

- `MobRapier.Physics.transforms/1` (+ `transforms_in/1`) — position +
  orientation per body each tick. The cheap read for the render loop.
- `MobRapier.Physics.body_states/1` (+ `body_states_in/1`) — the
  **debug/observability surface**. Returns one
  `%MobRapier.Physics.BodyState{}` per body: `pos`, `quat`, `linvel`,
  `angvel`, `|linvel|` as `speed`, `|angvel|` as `ang_speed`,
  Tait-Bryan `{yaw, pitch, roll}`, body-local `+Y` rotated into world
  coords as `up_axis`, and rapier's own `is_sleeping()`. Straight from
  `RigidBody::linvel()` / `angvel()` / `is_sleeping()` — the ground
  truth the solver itself gates on.
- `MobRapier.Physics.Telemetry.stream/2` — the push counterpart. A
  light GenServer polls `body_states_in/1` on a timer and sends
  `{:mob_rapier_telemetry, world_name, %{frame: n, states: [...]}}`
  messages to a subscriber pid. Same `{event, tag, payload}` envelope
  mob's `Mob.Listener` uses for scroll / drag, so consumers pattern
  match with **selective receive** without inventing new grammar:

  ```elixir
  {:ok, _} = MobRapier.Physics.Telemetry.stream("shells_cup",
    interval_ms: 100)

  receive do
    {:mob_rapier_telemetry, "shells_cup", %{states: states}} -> ...
  after 250 -> :timeout end
  ```

  The stream exits when the subscriber pid dies, so an IEx session
  can start-and-forget.

- `MobRapier.Physics.contacts/1` — collision + contact-force events
  for a step (a `ChannelEventCollector`). The event buffer caps at
  1024 entries per channel with a `contacts_dropped` counter; a
  rising counter means the world is too energetic for the current
  solver budget.

### An example diagnostic session

Retuning damping is the canonical use case. Connect to the running
app over Erlang dist and snapshot the world twice, one second apart:

```elixir
node = :"my_app_android_<suffix>@127.0.0.1"

read = fn ->
  :rpc.call(node, MobRapier.Physics, :body_states_in, ["shells_cup"])
end

s1 = read.(); Process.sleep(1000); s2 = read.()

Enum.zip(s1, s2)
|> Enum.each(fn {a, b} ->
  {ax, ay, az} = a.pos; {bx, by, bz} = b.pos
  dp = :math.sqrt((bx-ax)**2 + (by-ay)**2 + (bz-az)**2)
  IO.puts("id=\#{a.id}  Δpos=\#{Float.round(dp * 1000, 2)} mm/s  " <>
          "ang_speed=\#{Float.round(a.ang_speed, 2)}  sleep=\#{a.sleeping}")
end)
```

If bodies read "at rest" visually but `sleep=false` and Δpos is
non-zero, the contact solver is still moving them — bump damping or
raise `SLEEP_ANGVEL` until sleep engages. This is the exact loop that
took `mob_rapier`'s shells from 5-11 rad/s of residual ang velocity
under damping 2.0 to a hard rest under 20.0.
