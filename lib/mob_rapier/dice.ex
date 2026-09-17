defmodule MobRapier.Dice do
  @moduledoc """
  Face-up detection primitives per shape (bead rapier_lab-le5).

  Given a settled body's orientation quaternion, which face is pointing
  up? The physics engine does not know or care — the mapping is a rule
  about a specific shape's local axes.

  This module keeps each shape's rule in one place so the demos (dice
  screens, the eventual `mob_rapier` extraction, chopaat's shells
  once it lives on Rapier) can share a single face-up authority.

  ## Conventions

  A standard d6 has opposite faces summing to 7, with faces named by
  their pip count (1..6). We define the mapping from the local axis
  pointing to a face:

      +Y → 1     -Y → 6      (top face and bottom face)
      +X → 2     -X → 5
      +Z → 3     -Z → 4

  Applying the body's rotation quaternion to each of the six local
  axes yields six world-space unit vectors; the one with the largest
  y-component is the axis facing "up" in the physics world.
  """

  @doc """
  Face of a settled d6 whose current orientation is `quat`
  (`{qx, qy, qz, qw}`).

  Returns an integer 1..6. Assumes the world's up is `+Y` (matches the
  world_new gravity `{0, -9.81, 0}` — the ground surface is at y = 0
  and "up" from the die is `+Y`).
  """
  @spec face_up_d6({number(), number(), number(), number()}) :: 1..6
  def face_up_d6({qx, qy, qz, qw}) do
    axes = [
      {:pos_x, {1.0, 0.0, 0.0}},
      {:neg_x, {-1.0, 0.0, 0.0}},
      {:pos_y, {0.0, 1.0, 0.0}},
      {:neg_y, {0.0, -1.0, 0.0}},
      {:pos_z, {0.0, 0.0, 1.0}},
      {:neg_z, {0.0, 0.0, -1.0}}
    ]

    {label, _y} =
      axes
      |> Enum.map(fn {label, v} ->
        {_wx, wy, _wz} = rotate_vec({qx, qy, qz, qw}, v)
        {label, wy}
      end)
      |> Enum.max_by(fn {_label, y} -> y end)

    face_for(label)
  end

  # Standard d6 convention: opposite faces sum to 7.
  defp face_for(:pos_y), do: 1
  defp face_for(:neg_y), do: 6
  defp face_for(:pos_x), do: 2
  defp face_for(:neg_x), do: 5
  defp face_for(:pos_z), do: 3
  defp face_for(:neg_z), do: 4

  @doc """
  Face of a settled cowrie shell (bead rapier_lab-yyv).

  An oblate ellipsoid has two stable orientations: polar axis (local +y)
  pointing roughly world-up (`:up`), or roughly world-down (`:down`).
  Physically the shape is symmetric top-bottom, so the choice of which
  face to name `:up` is arbitrary — we take the convention that a shell
  whose local +y ends up pointing at world +y is `:up`. Every shell has
  ~50 % chance per settle under a symmetric shake, matching the
  chopaat sim.

  Assumes the world's up is `+Y` (world_new gravity `{0, -9.81, 0}`).
  """
  @spec face_up_cowrie({number(), number(), number(), number()}) :: :up | :down
  def face_up_cowrie({qx, qy, qz, qw}) do
    {_wx, wy, _wz} = rotate_vec({qx, qy, qz, qw}, {0.0, 1.0, 0.0})
    if wy >= 0.0, do: :up, else: :down
  end

  @doc """
  Rotates the vector `v` by the unit quaternion `q` using the standard
  `q * v * q_conj` formula, expanded for a `(x, y, z, w)` layout.
  """
  @spec rotate_vec(
          {number(), number(), number(), number()},
          {number(), number(), number()}
        ) :: {float(), float(), float()}
  def rotate_vec({qx, qy, qz, qw}, {vx, vy, vz}) do
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v' = v + q.w * t + cross(q.xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)

    {rx * 1.0, ry * 1.0, rz * 1.0}
  end

  # ── Bead rapier_lab-ry2: d10 / d12 / d20 geometry ─────────────────────
  #
  # Vertex tables for the three convex-hull dice, plus the face-normal
  # tables that face_up_dN/1 uses. Where possible the face-normals come
  # from the dual polyhedron (d12 ↔ d20 are duals; the pentagonal
  # trapezohedron's dual is the pentagonal antiprism) — no separate
  # geometry to keep in sync.
  #
  # Every vector here is centred on the origin; `Physics.add_convex_hull`
  # takes a `scale` argument that grows the whole cloud at spawn time.

  @phi (1.0 + :math.sqrt(5.0)) / 2.0
  @inv_phi 1.0 / ((1.0 + :math.sqrt(5.0)) / 2.0)

  @doc """
  20 vertices of a regular dodecahedron centred at the origin. Feed to
  `MobRapier.Physics.add_convex_hull/6` with a `scale` to build a d12
  collider.
  """
  @spec dodecahedron_vertices() :: [{float(), float(), float()}]
  def dodecahedron_vertices do
    phi = @phi
    inv_phi = @inv_phi

    # 8 cube corners.
    cube =
      for sx <- [1.0, -1.0], sy <- [1.0, -1.0], sz <- [1.0, -1.0],
          do: {sx, sy, sz}

    yz =
      for sy <- [inv_phi, -inv_phi], sz <- [phi, -phi],
          do: {0.0, sy, sz}

    xy =
      for sx <- [inv_phi, -inv_phi], sy <- [phi, -phi],
          do: {sx, sy, 0.0}

    xz =
      for sx <- [phi, -phi], sz <- [inv_phi, -inv_phi],
          do: {sx, 0.0, sz}

    cube ++ yz ++ xy ++ xz
  end

  @doc """
  12 vertices of a regular icosahedron centred at the origin. Feed to
  `MobRapier.Physics.add_convex_hull/6` with a `scale` to build a d20
  collider.
  """
  @spec icosahedron_vertices() :: [{float(), float(), float()}]
  def icosahedron_vertices do
    phi = @phi

    yz =
      for sy <- [1.0, -1.0], sz <- [phi, -phi],
          do: {0.0, sy, sz}

    xy =
      for sx <- [1.0, -1.0], sy <- [phi, -phi],
          do: {sx, sy, 0.0}

    xz =
      for sx <- [phi, -phi], sz <- [1.0, -1.0],
          do: {sx, 0.0, sz}

    yz ++ xy ++ xz
  end

  @doc """
  12 vertices of a pentagonal trapezohedron centred at the origin — the
  standard d10 shape. Two apexes on the y-axis, two five-vertex equatorial
  rings offset by 36°. Feed to `MobRapier.Physics.add_convex_hull/6` with a
  `scale` to build a d10 collider.
  """
  @spec pentagonal_trapezohedron_vertices() :: [{float(), float(), float()}]
  def pentagonal_trapezohedron_vertices do
    half_h = 0.35
    r = 1.0

    upper =
      for k <- 0..4 do
        theta = k * 2.0 * :math.pi() / 5.0
        {r * :math.cos(theta), half_h, r * :math.sin(theta)}
      end

    lower =
      for k <- 0..4 do
        theta = (k + 0.5) * 2.0 * :math.pi() / 5.0
        {r * :math.cos(theta), -half_h, r * :math.sin(theta)}
      end

    [{0.0, 1.0, 0.0}, {0.0, -1.0, 0.0}] ++ upper ++ lower
  end

  # ── Face normals ──────────────────────────────────────────────────────

  # Icosahedron face normals = normalized dodecahedron vertices (dual).
  defp d20_face_normals do
    dodecahedron_vertices()
    |> Enum.map(&normalize/1)
  end

  # Dodecahedron face normals = normalized icosahedron vertices (dual).
  defp d12_face_normals do
    icosahedron_vertices()
    |> Enum.map(&normalize/1)
  end

  # Pentagonal trapezohedron face normals: 10 unit vectors, 5 tilted up
  # and 5 tilted down, offset by 36°. This is the dual (pentagonal
  # antiprism vertex set) — the geometrically-exact face-normals for a
  # trapezohedron of these proportions. Face 1 is the +y-most top kite;
  # numbering ascends around the ring, then continues with the bottom kites.
  defp d10_face_normals do
    # tilt from y-axis chosen so face-normals form a pentagonal antiprism
    # inscribed in the unit sphere at y = ±cos(tilt).
    tilt = :math.pi() * 60.0 / 180.0
    y_top = :math.cos(tilt)
    r = :math.sin(tilt)

    top =
      for k <- 0..4 do
        theta = k * 2.0 * :math.pi() / 5.0
        {r * :math.cos(theta), y_top, r * :math.sin(theta)}
      end

    bottom =
      for k <- 0..4 do
        theta = (k + 0.5) * 2.0 * :math.pi() / 5.0
        {r * :math.cos(theta), -y_top, r * :math.sin(theta)}
      end

    top ++ bottom
  end

  defp normalize({x, y, z}) do
    len = :math.sqrt(x * x + y * y + z * z)
    {x / len, y / len, z / len}
  end

  # ── Face-up decoding ──────────────────────────────────────────────────

  @doc """
  Face of a settled d20 whose current orientation is `quat`. Returns an
  integer 1..20 — the face whose outward normal aligns most closely with
  world +y after the body's rotation is applied.
  """
  @spec face_up_d20({number(), number(), number(), number()}) :: 1..20
  def face_up_d20(quat), do: face_up_by_normal(quat, d20_face_normals())

  @doc """
  Face of a settled d12 whose current orientation is `quat`. Returns an
  integer 1..12.
  """
  @spec face_up_d12({number(), number(), number(), number()}) :: 1..12
  def face_up_d12(quat), do: face_up_by_normal(quat, d12_face_normals())

  @doc """
  Face of a settled d10 whose current orientation is `quat`. Returns an
  integer 1..10 — with face 1 being the top-most top kite, ascending
  around the top ring 1..5, then the bottom ring 6..10.
  """
  @spec face_up_d10({number(), number(), number(), number()}) :: 1..10
  def face_up_d10(quat), do: face_up_by_normal(quat, d10_face_normals())

  # Rotate every face normal by `quat`, return the 1-based index of the
  # one whose world-space y-component is largest — that is the face
  # pointing at world up.
  defp face_up_by_normal(quat, normals) do
    normals
    |> Enum.with_index(1)
    |> Enum.map(fn {n, i} ->
      {_x, y, _z} = rotate_vec(quat, n)
      {i, y}
    end)
    |> Enum.max_by(fn {_i, y} -> y end)
    |> elem(0)
  end
end
