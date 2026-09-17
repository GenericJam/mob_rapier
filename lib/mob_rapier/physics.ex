defmodule MobRapier.Physics do
  @moduledoc """
  Rustler entry point for the `lab_physics` crate — Rapier wrapped as
  an Elixir-side API. A world is a stateful resource handle; add
  bodies to it, step it forward each tick, read transforms back to
  drive the scene.

  Rustler 0.38+ carries the Bionic dlsym fix, so this NIF loads on
  Android without the 0.37-era crash (bead `rapier_lab-2ie`).

  ## otp_app override for mob-app hosts

  On host dev the NIF loads dynamically from `priv/native/lab_physics.so`
  under `Application.app_dir(:mob_rapier)`. On device (mob_dev's static
  NIF pipeline) the code is already linked into the app binary via the
  generated driver_tab and Rustler's on_load only needs a valid path to
  claim success — but `Application.app_dir(:mob_rapier)` raises there
  because mob_dev's flat OTP bundle doesn't register per-dep lib_dirs
  (see MOB-254).

  Consumers work around this by declaring the host app in config:

      # config/config.exs
      config :mob_rapier, :otp_app, :my_app

  which routes the on_load lookup through the host's app_dir instead.
  Defaults to `:mob_rapier`, which is right for host dev + tests.
  """

  use Rustler,
    otp_app: Application.compile_env(:mob_rapier, :otp_app, :mob_rapier),
    crate: "lab_physics"

  @typedoc "A resource handle to a physics world."
  @opaque world :: reference()

  @typedoc "Elixir-side body id — the insertion index into the world."
  @type body_id :: non_neg_integer()

  @typedoc "One body's transform: `{id, {x, y, z}, {qx, qy, qz, qw}}`."
  @type transform ::
          {body_id(), {float(), float(), float()}, {float(), float(), float(), float()}}

  @doc "Returns `:ok` when the NIF is loaded."
  @spec ping() :: :ok
  def ping, do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Creates a new physics world. Comes pre-populated with gravity
  `{0, -9.81, 0}` and a static wide ground plane at `y = 0`.
  """
  @spec world_new() :: world()
  def world_new, do: :erlang.nif_error(:nif_not_loaded)

  @doc "Adds a dynamic ball to `world` at `{x, y, z}` with `radius`."
  @spec add_ball(world(), float(), float(), float(), float()) :: body_id()
  def add_ball(_world, _x, _y, _z, _radius), do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Adds a dynamic cuboid to `world` at `{x, y, z}` with half-extents
  `{hx, hy, hz}`. A unit d6 is `hx = hy = hz = 0.5`.
  """
  @spec add_cuboid(world(), float(), float(), float(), float(), float(), float()) :: body_id()
  def add_cuboid(_world, _x, _y, _z, _hx, _hy, _hz), do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Adds a FIXED (static) cuboid — walls, tables, anything that shouldn't
  move. No mass, no impulses, no gravity. Same collider shape as
  `add_cuboid/6` so contacts still fire.
  """
  @spec add_static_cuboid(world(), float(), float(), float(), float(), float(), float()) ::
          body_id()
  def add_static_cuboid(_world, _x, _y, _z, _hx, _hy, _hz),
    do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Adds a dynamic oblate-of-revolution ellipsoid to `world` at `{x, y, z}`
  (bead rapier_lab-gmx). `equatorial_r` is the x/z semi-axis, `polar_r`
  the shorter y semi-axis — the shape a shaken cowrie shell settles into.

  The collider is the convex hull of a low-poly ellipsoid mesh (Rapier
  ships no ellipsoid primitive). Contacts fire on the same event mask as
  every other dynamic body.
  """
  @spec add_oblate(world(), float(), float(), float(), float(), float()) :: body_id()
  def add_oblate(_world, _x, _y, _z, _equatorial_r, _polar_r),
    do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Adds a dynamic convex polyhedron to `world` at `{x, y, z}`, built from
  `points` (each `(x, y, z)` tuple scaled by `scale`). Bead rapier_lab-ry2:
  the d10 / d12 / d20 collider. Rapier's convex-hull builder does the
  actual work — interior points in the cloud are discarded — so the caller
  passes a canonical vertex table without pre-computing the hull.

  See `MobRapier.Dice.dodecahedron_vertices/0`,
  `MobRapier.Dice.icosahedron_vertices/0`, and
  `MobRapier.Dice.pentagonal_trapezohedron_vertices/0` for the vertex sets.
  """
  @spec add_convex_hull(
          world(),
          float(),
          float(),
          float(),
          [{float(), float(), float()}],
          float()
        ) :: body_id()
  def add_convex_hull(_world, _x, _y, _z, _points, _scale),
    do: :erlang.nif_error(:nif_not_loaded)

  @doc "Applies a linear impulse to `body_id`'s centre of mass."
  @spec apply_impulse(world(), body_id(), float(), float(), float()) :: :ok
  def apply_impulse(_world, _body_id, _ix, _iy, _iz), do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Applies an angular impulse (torque) to `body_id`. Needed for dice +
  shell demos so a shape actually spins, not just slides.
  """
  @spec apply_torque_impulse(world(), body_id(), float(), float(), float()) :: :ok
  def apply_torque_impulse(_world, _body_id, _tx, _ty, _tz),
    do: :erlang.nif_error(:nif_not_loaded)

  @doc "Steps `world` forward by `dt` seconds."
  @spec step(world(), float()) :: :ok
  def step(_world, _dt), do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Every body's transform as `[{id, {x, y, z}, {qx, qy, qz, qw}}]`, in
  insertion order.
  """
  @spec transforms(world()) :: [transform()]
  def transforms(_world), do: :erlang.nif_error(:nif_not_loaded)

  defmodule BodyState do
    @moduledoc """
    Full per-body telemetry for debugging + settle detection. The NIF
    returns a list of these; every field is derived on the Rust side from
    rapier's own body state, so the values match what the solver sees.

    Fields:

      * `id` — body index (u32), the same one `transforms/1` and the
        contacts NIFs use.
      * `pos` — `{x, y, z}` centre position in world coordinates, metres.
      * `quat` — `{qx, qy, qz, qw}` orientation quaternion.
      * `linvel` — `{lvx, lvy, lvz}` linear velocity, m/s.
      * `angvel` — `{avx, avy, avz}` angular velocity, rad/s.
      * `speed` — `|linvel|`, m/s.
      * `ang_speed` — `|angvel|`, rad/s.
      * `euler` — `{yaw, pitch, roll}` intrinsic Z-Y-X Tait-Bryan
        angles, radians. Yaw about world +Y, pitch about world +Z after
        yaw, roll about world +X after pitch. Useful for "what does this
        die's face-up look like" without recomputing per-shape tables.
      * `up_axis` — body-local +Y rotated into world coords. Sign of
        `elem(up_axis, 1)` is the cheap convex-up vs concave-up decode
        for a cowrie, or "top face pointing which way" for a die.
      * `sleeping` — rapier's own `is_sleeping()`. Ground-truth "at rest"
        flag the solver gates on; prefer over hand-tuned velocity
        thresholds where possible.
    """

    @type t :: %__MODULE__{
            id: non_neg_integer(),
            pos: {float(), float(), float()},
            quat: {float(), float(), float(), float()},
            linvel: {float(), float(), float()},
            angvel: {float(), float(), float()},
            speed: float(),
            ang_speed: float(),
            euler: {float(), float(), float()},
            up_axis: {float(), float(), float()},
            sleeping: boolean()
          }

    defstruct [
      :id,
      :pos,
      :quat,
      :linvel,
      :angvel,
      :speed,
      :ang_speed,
      :euler,
      :up_axis,
      :sleeping
    ]
  end

  @doc """
  Every body's full state as a list of `#{inspect(__MODULE__)}.BodyState`
  structs — position, orientation, velocities, derived speeds, Tait-Bryan
  angles, body-local +Y in world coords, and rapier's own `is_sleeping()`.

  Prefer this over diffing consecutive `transforms/1` calls when you need
  velocity or a rest signal: it reads directly from rapier's
  `RigidBody::linvel`, `angvel`, and `is_sleeping`, which is the ground
  truth the auto-sleep + solver themselves gate on. Handy in IEx over
  dist for observing why a body is or isn't settling.
  """
  @spec body_states(world()) :: [BodyState.t()]
  def body_states(_world), do: :erlang.nif_error(:nif_not_loaded)

  @typedoc "One collision-start/stop event."
  @type collision ::
          {body_id(), body_id(), :started | :stopped}

  @typedoc "One aggregated contact-force event for a body pair."
  @type contact_force ::
          {body_id(), body_id(), float(), {float(), float(), float()}}

  @doc """
  Drains and returns every contact event accumulated in `world` since the
  last call to this NIF (bead rapier_lab-bvr).

  Returns `{collisions, forces, dropped}`.

  Colliders parented to nothing (the static ground) report body id
  `4_294_967_295` (`0xFFFFFFFF`) so a caller can filter on 'body A hit the
  ground' without extra bookkeeping.
  """
  @spec contacts(world()) :: {[collision()], [contact_force()], non_neg_integer()}
  def contacts(_world), do: :erlang.nif_error(:nif_not_loaded)

  @doc """
  Legacy smoke: canned drop, returns the ball's final y. Kept for the
  boot-time NIF check on MainScreen until the demo takes its place.
  """
  @spec smoke_drop() :: float()
  def smoke_drop, do: :erlang.nif_error(:nif_not_loaded)

  # ── Named-world API (bead rapier_lab-ou4) ────────────────────────────────
  #
  # NIF resources cannot cross Erlang dist as usable handles. An agent that
  # RPCs into the device gets a reference that reads as `:badarg` on any
  # follow-up NIF call. The by-name variants keep the resource on-device
  # and let the caller reference it by a plain string.

  alias MobRapier.Physics.Registry

  @doc """
  Creates a fresh world and registers it under `name`.

  Returns `:ok`. Any world previously registered under `name` is replaced.
  Call this from an on-device process (a Screen, an IEx over
  `mix mob.connect`, `:rpc.call/4` from an agent).
  """
  @spec new_world(String.t()) :: :ok
  def new_world(name) when is_binary(name) do
    Registry.register(name, world_new())
  end

  @doc """
  Returns the resource handle for `name`, or `{:error, :not_found}`.

  Meaningful only on-device — a resource sent across dist is inert.
  """
  @spec world(String.t()) :: {:ok, world()} | {:error, :not_found}
  def world(name) when is_binary(name), do: Registry.lookup(name)

  @doc "Names of every registered world."
  @spec list_worlds() :: [String.t()]
  def list_worlds, do: Registry.names()

  @doc "Removes the world named `name` from the registry."
  @spec destroy_world(String.t()) :: :ok
  def destroy_world(name) when is_binary(name), do: Registry.unregister(name)

  @doc "Drops every registered world."
  @spec destroy_all() :: :ok
  def destroy_all, do: Registry.clear()

  @doc "Adds a dynamic ball to the named world. Returns the body id."
  @spec add_ball_in(String.t(), float(), float(), float(), float()) ::
          body_id() | {:error, :not_found}
  def add_ball_in(name, x, y, z, radius) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name), do: add_ball(world, x, y, z, radius)
  end

  @doc "Adds a dynamic cuboid to the named world. Returns the body id."
  @spec add_cuboid_in(String.t(), float(), float(), float(), float(), float(), float()) ::
          body_id() | {:error, :not_found}
  def add_cuboid_in(name, x, y, z, hx, hy, hz) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name),
         do: add_cuboid(world, x, y, z, hx, hy, hz)
  end

  @doc "Adds a fixed cuboid to the named world. Returns the body id."
  @spec add_static_cuboid_in(String.t(), float(), float(), float(), float(), float(), float()) ::
          body_id() | {:error, :not_found}
  def add_static_cuboid_in(name, x, y, z, hx, hy, hz) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name),
         do: add_static_cuboid(world, x, y, z, hx, hy, hz)
  end

  @doc "Adds a dynamic oblate to the named world. Returns the body id."
  @spec add_oblate_in(String.t(), float(), float(), float(), float(), float()) ::
          body_id() | {:error, :not_found}
  def add_oblate_in(name, x, y, z, equatorial_r, polar_r) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name),
         do: add_oblate(world, x, y, z, equatorial_r, polar_r)
  end

  @doc "Adds a dynamic convex polyhedron to the named world. Returns the body id."
  @spec add_convex_hull_in(
          String.t(),
          float(),
          float(),
          float(),
          [{float(), float(), float()}],
          float()
        ) :: body_id() | {:error, :not_found}
  def add_convex_hull_in(name, x, y, z, points, scale) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name),
         do: add_convex_hull(world, x, y, z, points, scale)
  end

  @doc "Applies a linear impulse to `body_id` in the named world."
  @spec apply_impulse_in(String.t(), body_id(), float(), float(), float()) ::
          :ok | {:error, :not_found}
  def apply_impulse_in(name, body_id, ix, iy, iz) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name),
         do: apply_impulse(world, body_id, ix, iy, iz)
  end

  @doc "Applies an angular impulse to `body_id` in the named world."
  @spec apply_torque_impulse_in(String.t(), body_id(), float(), float(), float()) ::
          :ok | {:error, :not_found}
  def apply_torque_impulse_in(name, body_id, tx, ty, tz) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name),
         do: apply_torque_impulse(world, body_id, tx, ty, tz)
  end

  @doc "Steps the named world forward by `dt` seconds."
  @spec step_in(String.t(), float()) :: :ok | {:error, :not_found}
  def step_in(name, dt) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name), do: step(world, dt)
  end

  @doc "Every body's transform in the named world, or `{:error, :not_found}`."
  @spec transforms_in(String.t()) :: [transform()] | {:error, :not_found}
  def transforms_in(name) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name), do: transforms(world)
  end

  @doc """
  Every body's full state (position, orientation, linvel, angvel,
  derived speeds + Euler angles + local +Y, is_sleeping) in the named
  world, or `{:error, :not_found}`.
  """
  @spec body_states_in(String.t()) :: [BodyState.t()] | {:error, :not_found}
  def body_states_in(name) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name), do: body_states(world)
  end

  @doc """
  Drains contact events in the named world (bead rapier_lab-bvr).

  Returns `{collisions, forces, dropped}` or `{:error, :not_found}`.
  """
  @spec contacts_in(String.t()) ::
          {[collision()], [contact_force()], non_neg_integer()}
          | {:error, :not_found}
  def contacts_in(name) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name), do: contacts(world)
  end

  @doc """
  Steps the named world by `dt` and immediately drains its contact events.

  Returns `{collisions, forces, dropped}` for the events that fired during
  this step (the buffer is drained fresh here, so a caller mixing this with
  `contacts_in/1` sees no overlap).
  """
  @spec step_with_contacts_in(String.t(), float()) ::
          {[collision()], [contact_force()], non_neg_integer()}
          | {:error, :not_found}
  def step_with_contacts_in(name, dt) when is_binary(name) do
    with {:ok, world} <- Registry.lookup(name) do
      step(world, dt)
      contacts(world)
    end
  end
end
