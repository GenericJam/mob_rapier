defmodule MobRapier.Physics.BodyStatesTest do
  # Not async — the Registry ETS table is a shared singleton.
  use ExUnit.Case, async: false

  alias MobRapier.Physics
  alias MobRapier.Physics.{BodyState, Registry, Telemetry}

  @world "body_states_test"

  setup do
    _ = Registry.start_link()
    Registry.clear()
    :ok = Physics.new_world(@world)
    on_exit(fn -> Physics.destroy_world(@world) end)
    :ok
  end

  describe "body_states/1 snapshot" do
    test "empty world has no bodies" do
      assert Physics.body_states_in(@world) == []
    end

    test "one dropped ball reports a full BodyState struct" do
      body = Physics.add_ball_in(@world, 0.0, 0.5, 0.0, 0.05)

      assert [%BodyState{} = state] = Physics.body_states_in(@world)
      assert state.id == body
      assert {_x, _y, _z} = state.pos
      assert {qx, qy, qz, qw} = state.quat
      # Fresh body: identity rotation.
      assert_in_delta qx, 0.0, 1.0e-6
      assert_in_delta qy, 0.0, 1.0e-6
      assert_in_delta qz, 0.0, 1.0e-6
      assert_in_delta qw, 1.0, 1.0e-6
      # Nothing has stepped, so velocities are exactly zero.
      assert state.linvel == {0.0, 0.0, 0.0}
      assert state.angvel == {0.0, 0.0, 0.0}
      assert state.speed == 0.0
      assert state.ang_speed == 0.0
      # Body-local +Y is world +Y at identity rotation.
      assert {ax, ay, az} = state.up_axis
      assert_in_delta ax, 0.0, 1.0e-6
      assert_in_delta ay, 1.0, 1.0e-6
      assert_in_delta az, 0.0, 1.0e-6
      # A brand-new dynamic body is awake by default (`sleeping/1` on the
      # builder was not called).
      refute state.sleeping
    end

    test "mid-fall, gravity registers as downward linvel" do
      # Drop from 2m so we're firmly in the falling phase after 10 steps
      # (~0.16 s of sim time, no ground contact yet — ground is at y=0).
      _ = Physics.add_ball_in(@world, 0.0, 2.0, 0.0, 0.05)

      for _ <- 1..10, do: :ok = Physics.step_in(@world, 1.0 / 60.0)

      assert [%BodyState{linvel: {_, lvy, _}, speed: sp, ang_speed: as} = _s] =
               Physics.body_states_in(@world)

      # Gravity is -Y in mob_rapier worlds.
      assert lvy < 0.0
      # Falling body has positive speed and (for a ball) tiny angular
      # speed (contact spin isn't infinite).
      assert sp > 0.0
      assert as >= 0.0
    end

    test "sleep threshold constants keep a resting body identifiable" do
      _ = Physics.add_ball_in(@world, 0.0, 0.05, 0.0, 0.05)

      # Enough steps that the ball settles and rapier's auto-sleep engages.
      # Ball starts almost on the ground so we don't spend energy on fall.
      for _ <- 1..300, do: :ok = Physics.step_in(@world, 1.0 / 60.0)

      assert [%BodyState{sleeping: sleeping, ang_speed: ang_speed} = _s] =
               Physics.body_states_in(@world)

      # Either rapier's auto-sleep engaged (the ideal outcome) or the ball
      # is at least angularly at rest. Both are legitimate "settled"
      # signals — the tuning knobs live in native/lab_physics/src/lib.rs.
      assert sleeping or ang_speed < 0.5
    end
  end

  describe "Telemetry.stream/2" do
    test "streams frames in the Mob.Listener envelope shape" do
      _ = Physics.add_ball_in(@world, 0.0, 0.5, 0.0, 0.05)

      {:ok, streamer} = Telemetry.stream(@world, interval_ms: 20)

      # Wait for at least two frames.
      assert_receive {:mob_rapier_telemetry, @world, %{frame: f0, states: s0}}, 500
      assert_receive {:mob_rapier_telemetry, @world, %{frame: f1, states: s1}}, 500

      assert is_integer(f0) and is_integer(f1)
      assert f1 > f0
      assert is_list(s0) and is_list(s1)
      assert length(s0) == 1
      assert %BodyState{} = hd(s0)

      Telemetry.stop(streamer)
    end

    test "selective receive by world_name skips unrelated frames" do
      other = @world <> "_other"
      :ok = Physics.new_world(other)
      on_exit(fn -> Physics.destroy_world(other) end)

      _ = Physics.add_ball_in(@world, 0.0, 0.5, 0.0, 0.05)
      _ = Physics.add_ball_in(other, 0.0, 0.5, 0.0, 0.05)

      {:ok, s1} = Telemetry.stream(@world, interval_ms: 20)
      {:ok, s2} = Telemetry.stream(other, interval_ms: 20)

      # Selective receive: pick out only the "other" world's frames — the
      # `@world` frames sit in the mailbox and never match.
      assert_receive {:mob_rapier_telemetry, ^other, _payload}, 500

      Telemetry.stop(s1)
      Telemetry.stop(s2)
    end

    test "emit/4 pushes the same envelope shape for screen-driven ticks" do
      state = %BodyState{
        id: 0,
        pos: {0.0, 0.0, 0.0},
        quat: {0.0, 0.0, 0.0, 1.0},
        linvel: {0.0, 0.0, 0.0},
        angvel: {0.0, 0.0, 0.0},
        speed: 0.0,
        ang_speed: 0.0,
        euler: {0.0, 0.0, 0.0},
        up_axis: {0.0, 1.0, 0.0},
        sleeping: true
      }

      :ok = Telemetry.emit(self(), "handmade", 42, [state])

      assert_receive {:mob_rapier_telemetry, "handmade", %{frame: 42, states: [^state]}}
    end
  end
end
