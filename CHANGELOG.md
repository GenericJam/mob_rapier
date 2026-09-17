# Changelog

All notable changes to `mob_rapier` are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning: [SemVer](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

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
