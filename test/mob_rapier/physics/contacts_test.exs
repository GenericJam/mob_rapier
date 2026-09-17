defmodule MobRapier.Physics.ContactsTest do
  # Runs on the host BEAM — the NIF is loaded from priv/native/lab_physics.so.
  # Same physics as on-device; only difference is the ResourceArc lives in a
  # host process instead of a device process.
  use ExUnit.Case, async: false

  alias MobRapier.Physics

  # u32::MAX — the sentinel that :mob_scene3d_nif / lab_physics's `contacts`
  # NIF uses for ground contacts. Keep it here so the assertions read as
  # intent instead of magic numbers.
  @ground 0xFFFFFFFF

  test "idle world drains empty" do
    world = Physics.world_new()
    assert {[], [], 0} = Physics.contacts(world)
    Physics.step(world, 1.0 / 60.0)
    assert {[], [], 0} = Physics.contacts(world)
  end

  test "ball dropped onto ground reports first-contact" do
    world = Physics.world_new()
    _ball = Physics.add_ball(world, 0.0, 1.5, 0.0, 0.3)

    # Drain any spurious startup events (none expected, but be explicit).
    Physics.contacts(world)

    first_started =
      Enum.reduce_while(1..120, nil, fn _, _acc ->
        Physics.step(world, 1.0 / 60.0)
        {collisions, _forces, _dropped} = Physics.contacts(world)

        case Enum.find(collisions, fn {_a, _b, kind} -> kind == :started end) do
          nil -> {:cont, nil}
          found -> {:halt, found}
        end
      end)

    assert first_started != nil, "expected a :started collision within 120 steps"
    {a, b, :started} = first_started
    assert @ground in [a, b], "ground body should be one side of the first contact"
    other = if a == @ground, do: b, else: a
    assert other == 0, "the other body should be the only dynamic ball (id 0)"
  end

  test "step_with_contacts_in returns just this step's events" do
    :ok = Physics.new_world("bench")
    _ball = Physics.add_ball_in("bench", 0.0, 1.5, 0.0, 0.3)

    started_step =
      Enum.reduce_while(1..120, nil, fn i, _acc ->
        {collisions, _forces, _dropped} =
          Physics.step_with_contacts_in("bench", 1.0 / 60.0)

        if Enum.any?(collisions, fn {_a, _b, kind} -> kind == :started end),
          do: {:halt, i},
          else: {:cont, nil}
      end)

    assert started_step != nil
    # After the started event is drained, subsequent steps should not
    # re-report it — the buffer is per-call.
    {collisions, _forces, _dropped} = Physics.step_with_contacts_in("bench", 1.0 / 60.0)
    assert Enum.count(collisions, fn {_a, _b, k} -> k == :started end) == 0

    Physics.destroy_world("bench")
  end
end
