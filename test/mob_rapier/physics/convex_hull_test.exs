defmodule MobRapier.Physics.ConvexHullTest do
  # Bead rapier_lab-ry2: d10 / d12 / d20 convex-hull colliders. This is
  # the integration half — the Rust NIF builds the hull, physics runs,
  # face-up decode returns something valid.
  use ExUnit.Case, async: false

  alias MobRapier.Dice
  alias MobRapier.Physics

  @world "hulls_test"
  @settle_steps 240
  @drop_height 0.5

  setup do
    :ok = Physics.new_world(@world)
    on_exit(fn -> Physics.destroy_world(@world) end)
    :ok
  end

  # Each test drops a die from `@drop_height` with a small torque,
  # steps the world @settle_steps times (~4 s at 60 Hz), then decodes
  # the settled face. All that matters here is:
  #   * The convex-hull NIF returned a body id
  #   * The die is at rest near y = 0 after settling
  #   * face_up_dN/1 returns a valid face number in the expected range

  test "d20 (icosahedron) settles and decodes to a face in 1..20" do
    body =
      Physics.add_convex_hull_in(
        @world,
        0.0,
        @drop_height,
        0.0,
        Dice.icosahedron_vertices(),
        0.05
      )

    assert is_integer(body) and body >= 0

    Physics.apply_torque_impulse_in(@world, body, 3.0e-6, 5.0e-6, 4.0e-6)
    for _ <- 1..@settle_steps, do: Physics.step_in(@world, 1.0 / 60.0)

    {_id, {_x, y, _z}, rot} =
      Physics.transforms_in(@world) |> Enum.find(fn {id, _, _} -> id == body end)

    assert y < 0.2, "die should be settled near the ground; got y=#{y}"
    assert Dice.face_up_d20(rot) in 1..20
  end

  test "d12 (dodecahedron) settles and decodes to a face in 1..12" do
    body =
      Physics.add_convex_hull_in(
        @world,
        0.0,
        @drop_height,
        0.0,
        Dice.dodecahedron_vertices(),
        0.05
      )

    assert is_integer(body) and body >= 0
    Physics.apply_torque_impulse_in(@world, body, 3.0e-6, 5.0e-6, 4.0e-6)
    for _ <- 1..@settle_steps, do: Physics.step_in(@world, 1.0 / 60.0)

    {_id, {_x, y, _z}, rot} =
      Physics.transforms_in(@world) |> Enum.find(fn {id, _, _} -> id == body end)

    assert y < 0.2
    assert Dice.face_up_d12(rot) in 1..12
  end

  test "d10 (pentagonal trapezohedron) settles and decodes to a face in 1..10" do
    body =
      Physics.add_convex_hull_in(
        @world,
        0.0,
        @drop_height,
        0.0,
        Dice.pentagonal_trapezohedron_vertices(),
        0.05
      )

    assert is_integer(body) and body >= 0
    Physics.apply_torque_impulse_in(@world, body, 3.0e-6, 5.0e-6, 4.0e-6)
    for _ <- 1..@settle_steps, do: Physics.step_in(@world, 1.0 / 60.0)

    {_id, {_x, y, _z}, rot} =
      Physics.transforms_in(@world) |> Enum.find(fn {id, _, _} -> id == body end)

    assert y < 0.2
    assert Dice.face_up_d10(rot) in 1..10
  end

  test "the NIF discards interior points — a hull with padding still spawns" do
    # Deliberately drop the origin inside the vertex cloud. Rapier's
    # convex_hull builder should ignore it (interior points don't extend
    # the hull) and produce the same collider as the clean vertex set.
    padded = [{0.0, 0.0, 0.0} | Dice.icosahedron_vertices()]

    body =
      Physics.add_convex_hull_in(@world, 0.0, @drop_height, 0.0, padded, 0.05)

    assert is_integer(body) and body >= 0
    for _ <- 1..60, do: Physics.step_in(@world, 1.0 / 60.0)
    assert Physics.transforms_in(@world) != []
  end
end
