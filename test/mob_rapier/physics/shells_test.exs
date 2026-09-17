defmodule MobRapier.Physics.ShellsTest do
  # bead rapier_lab-yyv: 7 oblate cowries shaken in a small arena settle
  # into a mix of :up and :down orientations. This is the physics half of
  # the demo; the visual half is `MobRapier.Screens.ShellsScreen` and the
  # `Mob.Scene3d.Test.Physics.assert_scene_tracks_physics/3` coherence
  # helper (mob_scene3d-8oh) proves the two agree at runtime.
  use ExUnit.Case, async: false

  alias MobRapier.Dice
  alias MobRapier.Physics

  @world "shells_test"
  @count 7
  @equatorial_r 0.03
  @polar_r 0.012
  @arena_half 0.20

  setup do
    :ok = Physics.new_world(@world)

    on_exit(fn -> Physics.destroy_world(@world) end)

    for {x, z} <- [{@arena_half, 0.0}, {-@arena_half, 0.0}, {0.0, @arena_half}, {0.0, -@arena_half}] do
      Physics.add_static_cuboid_in(@world, x, 0.05, z, 0.02, 0.05, @arena_half)
    end

    :ok
  end

  test "7 shells shaken in a cup all settle within 4 seconds and each reads :up or :down" do
    ids =
      for i <- 0..(@count - 1) do
        x = -0.06 + rem(i, 3) * 0.06
        z = -0.06 + div(i, 3) * 0.06
        y = 0.15 + i * 0.015

        id = Physics.add_oblate_in(@world, x, y, z, @equatorial_r, @polar_r)

        # Same shake profile as ShellsScreen.spawn_shells/1.
        Physics.apply_impulse_in(
          @world,
          id,
          (:rand.uniform() - 0.5) * 3.0e-4,
          2.0e-5 + :rand.uniform() * 5.0e-5,
          (:rand.uniform() - 0.5) * 3.0e-4
        )

        Physics.apply_torque_impulse_in(
          @world,
          id,
          (:rand.uniform() - 0.5) * 4.0e-6,
          (:rand.uniform() - 0.5) * 4.0e-6,
          (:rand.uniform() - 0.5) * 4.0e-6
        )

        id
      end

    # 240 steps at 60 Hz = 4 s. The tumble is small so shells settle fast.
    for _ <- 1..240, do: Physics.step_in(@world, 1.0 / 60.0)

    transforms = Physics.transforms_in(@world)
    reads = for id <- ids, {tid, _pos, rot} <- transforms, tid == id, do: Dice.face_up_cowrie(rot)

    assert length(reads) == @count,
           "expected face-up read for every shell; got #{length(reads)} of #{@count}"

    assert Enum.all?(reads, &(&1 in [:up, :down])),
           "every shell must decode to :up or :down; got #{inspect(reads)}"
  end

  test "shell body ids are dense (consecutive) — the transform lookup relies on it" do
    # ShellsScreen keys per-tick state by body id; a hole in the id sequence
    # would still work (Enum.find on transforms doesn't care) but a gap that
    # skipped a valid id here would signal something has changed in the NIF
    # about how insertion indices are assigned, which the screen would want
    # to know about. The static arena walls consume the first N ids; shells
    # follow directly.
    ids =
      for i <- 0..(@count - 1) do
        x = 0.02 * i
        Physics.add_oblate_in(@world, x, 0.2, 0.0, @equatorial_r, @polar_r)
      end

    assert length(ids) == @count
    assert ids == Enum.to_list(hd(ids)..(hd(ids) + @count - 1)),
           "shell body ids should be consecutive; got #{inspect(ids)}"
  end
end
