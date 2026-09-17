defmodule MobRapier.Physics.OblateTest do
  # Runs on the host BEAM — the NIF is loaded from priv/native/lab_physics.so.
  # bead rapier_lab-gmx: oblate ellipsoid collider for cowries.
  use ExUnit.Case, async: false

  alias MobRapier.Physics

  @ground 0xFFFFFFFF

  test "add_oblate returns a body id and reports a first-contact against the ground" do
    world = Physics.world_new()
    # Cowrie-scale oblate: equatorial 1.0, polar 0.35. Drop from y = 1.5 so
    # the ground contact fires within a handful of frames.
    body = Physics.add_oblate(world, 0.0, 1.5, 0.0, 1.0, 0.35)
    assert is_integer(body) and body >= 0

    Physics.contacts(world)

    first_started =
      Enum.reduce_while(1..180, nil, fn _, _acc ->
        Physics.step(world, 1.0 / 60.0)
        {collisions, _forces, _dropped} = Physics.contacts(world)

        case Enum.find(collisions, fn {_a, _b, kind} -> kind == :started end) do
          nil -> {:cont, nil}
          found -> {:halt, found}
        end
      end)

    assert first_started != nil, "expected a :started collision within 180 steps"
    {a, b, :started} = first_started
    assert @ground in [a, b]
    assert if(a == @ground, do: b, else: a) == body
  end

  test "an oblate dropped tilted settles with the polar axis vertical" do
    # A cowrie's whole point is that it's stable on the flat side, not on
    # its edge. Drop it slightly off-axis and give it a light spin; after
    # ~4 s of sim time the polar axis (initially local +y) should align
    # with world +y (up) — |q.i| + |q.k| ≈ 0, i.e. rotation is around the
    # y-axis only, i.e. the oblate is lying flat.
    world = Physics.world_new()
    _body = Physics.add_oblate(world, 0.0, 1.2, 0.0, 1.0, 0.35)
    # Nudge it so it doesn't drop dead-flat by accident.
    Physics.apply_torque_impulse(world, 0, 0.6, 0.0, 0.4)

    # 240 steps at 60 Hz = 4 s. More than enough for a light oblate to
    # settle on this ground restitution.
    for _ <- 1..240, do: Physics.step(world, 1.0 / 60.0)

    [{_id, _pos, {qi, _qj, qk, _qw}}] = Physics.transforms(world)
    tilt = abs(qi) + abs(qk)

    assert tilt < 0.15,
           "polar axis should be roughly world-vertical after settling; " <>
             "quat i+k magnitude was #{tilt}"
  end
end
