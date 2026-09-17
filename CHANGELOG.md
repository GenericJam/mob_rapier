# Changelog

All notable changes to `mob_rapier` are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning: [SemVer](https://semver.org/spec/v2.0.0.html).

---

## [0.1.0] - 2026-09-17

### Added
- **Initial extraction from `rapier_lab`** (rapier_lab-d00). Ships the
  Rustler-backed `MobRapier.Physics` NIF (Rapier 3D 0.22), the
  by-name world registry `MobRapier.Physics.Registry` that keeps NIF
  resource handles on-device so an agent can drive a world by string
  name over Erlang distribution, and the `MobRapier.Dice` face-up
  decode rules for d6 / d10 / d12 / d20 dice plus the up/down cowrie
  shell decode.
- **Convex-hull collider primitives** for the standard TTRPG dice
  (`dodecahedron_vertices/0`, `icosahedron_vertices/0`,
  `pentagonal_trapezohedron_vertices/0`). Face-normal tables use the
  dual-polyhedron identity — d12 faces = d20 vertices, d20 faces =
  d12 vertices, pentagonal trapezohedron faces = pentagonal antiprism
  vertices — so there's no separate face-normal set to keep in sync
  with vertex changes.
- **Oblate ellipsoid collider** (`add_oblate/6`) built as the convex
  hull of a low-poly ellipsoid mesh, for cowrie-shell physics — see
  rapier_lab-gmx for the "why convex hull, not scaled sphere"
  discussion.
- **Tuned dynamic-body defaults for tabletop-scale worlds.**
  `COLLIDER_DENSITY = 1000` kg/m³, `LINEAR_DAMPING = 3.0`,
  `ANGULAR_DAMPING = 20.0`, `COLLIDER_FRICTION = 0.7`,
  `COLLIDER_RESTITUTION = 0.05`. Per-body sleep thresholds
  (`normalized_linear_threshold = 0.1`, `angular_threshold = 1.5`,
  `time_until_sleep = 0.3`) are applied via `activation_mut()` so
  rapier's own auto-sleep engages on cm-scale bodies rather than the
  consumer chasing a velocity threshold outside the solver. See
  [`guides/physics_tuning.md`](guides/physics_tuning.md).
- **Per-body CCD** (`ccd_enabled(true)`) on every dynamic body — a
  peak collision impulse can otherwise carry a 45 g shell past a
  5 cm wall in a single 33 ms step. Cheap at demo body counts.
- **`MobRapier.Physics.body_states/1`** (+ `body_states_in/1`) — the
  debug/observability surface. Returns one
  `%MobRapier.Physics.BodyState{}` per body: `pos`, `quat`, `linvel`,
  `angvel`, `speed`, `ang_speed`, Tait-Bryan `{yaw, pitch, roll}`,
  body-local `+Y` in world coords, and rapier's `is_sleeping()`.
- **`MobRapier.Physics.Telemetry.stream/2`** — the push counterpart.
  Emits `{:mob_rapier_telemetry, world_name, %{frame, states}}` in
  the `Mob.Listener` `{event, tag, payload}` envelope so consumers
  pattern-match with selective receive. `emit/4` is the direct entry
  point for callers that already own a physics tick loop.
- **`guides/`** — documentation extras `physics_tuning.md` (constants,
  sleep thresholds, telemetry, upstream Rapier links) and `dice.md`
  (per-shape face-up decode + dual-polyhedron identity).

### Known workarounds
- **Rustler `otp_app` is compile-configurable.** On mob-app hosts,
  mob_dev's flat OTP bundle doesn't register per-dep `code:lib_dir/1`,
  so `Application.app_dir(:mob_rapier)` raises inside Rustler's
  `on_load`. `MobRapier.Physics` declares `use Rustler, otp_app:
  Application.compile_env(:mob_rapier, :otp_app, :mob_rapier)`; hosts
  override in config to route the load through their own known-good
  path (`config :mob_rapier, :otp_app, :my_app`). Follow-up: **MOB-254**.
- **Native crate discoverability from a dep.** `mob_dev`'s
  `MobDev.NativeBuild.classify_project_nif/2` only looks at
  `project_root/native/<name>/Cargo.toml`, so a plugin's Rust crate is
  invisible in `deps/`. Consumers work around by symlinking
  `<host>/native/lab_physics -> deps/mob_rapier/native/lab_physics`
  until MOB-254 lands the `:source` option on `:static_nifs` entries.
