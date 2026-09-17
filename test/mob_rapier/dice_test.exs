defmodule MobRapier.DiceTest do
  use ExUnit.Case, async: true

  alias MobRapier.Dice

  describe "rotate_vec/2" do
    test "identity quaternion leaves vectors unchanged" do
      q = {0.0, 0.0, 0.0, 1.0}

      for v <- [{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {0.3, -0.4, 0.5}] do
        {rx, ry, rz} = Dice.rotate_vec(q, v)
        {vx, vy, vz} = v
        assert_in_delta rx, vx, 1.0e-6
        assert_in_delta ry, vy, 1.0e-6
        assert_in_delta rz, vz, 1.0e-6
      end
    end

    test "90° rotation about Z sends +X to +Y" do
      # (0, 0, sin(45°), cos(45°))
      q = {0.0, 0.0, 0.7071067811865475, 0.7071067811865476}
      {x, y, z} = Dice.rotate_vec(q, {1.0, 0.0, 0.0})
      assert_in_delta x, 0.0, 1.0e-6
      assert_in_delta y, 1.0, 1.0e-6
      assert_in_delta z, 0.0, 1.0e-6
    end

    test "180° rotation about X sends +Y to -Y" do
      # (sin(90°), 0, 0, cos(90°)) = (1, 0, 0, 0)
      q = {1.0, 0.0, 0.0, 0.0}
      {x, y, z} = Dice.rotate_vec(q, {0.0, 1.0, 0.0})
      assert_in_delta x, 0.0, 1.0e-6
      assert_in_delta y, -1.0, 1.0e-6
      assert_in_delta z, 0.0, 1.0e-6
    end
  end

  describe "face_up_d6/1" do
    test "identity orientation: +Y is up → face 1" do
      assert Dice.face_up_d6({0.0, 0.0, 0.0, 1.0}) == 1
    end

    test "180° flip about X: -Y is up → face 6" do
      assert Dice.face_up_d6({1.0, 0.0, 0.0, 0.0}) == 6
    end

    test "90° about +Z rotates die's +X to world up → face 2" do
      q = {0.0, 0.0, 0.7071067811865475, 0.7071067811865476}
      assert Dice.face_up_d6(q) == 2
    end

    test "opposite faces sum to 7 for every canonical orientation" do
      pairs = [
        # +Y and -Y
        {{0.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 0.0}},
        # +X (90° about -Z) and -X (90° about +Z)
        {{0.0, 0.0, -0.7071067811865475, 0.7071067811865476},
         {0.0, 0.0, 0.7071067811865475, 0.7071067811865476}},
        # +Z (90° about +X) and -Z (90° about -X)
        {{0.7071067811865475, 0.0, 0.0, 0.7071067811865476},
         {-0.7071067811865475, 0.0, 0.0, 0.7071067811865476}}
      ]

      for {qa, qb} <- pairs do
        assert Dice.face_up_d6(qa) + Dice.face_up_d6(qb) == 7
      end
    end
  end

  describe "face_up_cowrie/1" do
    test "identity orientation: local +Y aligned with world +Y → :up" do
      assert Dice.face_up_cowrie({0.0, 0.0, 0.0, 1.0}) == :up
    end

    test "180° flip about X: local +Y now points at world -Y → :down" do
      assert Dice.face_up_cowrie({1.0, 0.0, 0.0, 0.0}) == :down
    end

    test "180° flip about Z (also inverts +Y) → :down" do
      assert Dice.face_up_cowrie({0.0, 0.0, 1.0, 0.0}) == :down
    end

    test "90° about +X (local +Y goes to world -Z, y-component = 0) → :up (tie)" do
      # sin(45°), 0, 0, cos(45°): rotated (0,1,0) becomes (0, 0, 1).
      # y-component is 0; convention picks :up on the boundary.
      q = {0.7071067811865475, 0.0, 0.0, 0.7071067811865476}
      assert Dice.face_up_cowrie(q) == :up
    end
  end

  # ── Bead rapier_lab-ry2 ─────────────────────────────────────────────

  describe "dodecahedron_vertices/0" do
    test "returns 20 vertices" do
      assert length(Dice.dodecahedron_vertices()) == 20
    end

    test "every vertex has the same distance from origin (circumsphere)" do
      dists =
        Dice.dodecahedron_vertices()
        |> Enum.map(fn {x, y, z} -> :math.sqrt(x * x + y * y + z * z) end)
        |> Enum.map(&Float.round(&1, 6))
        |> Enum.uniq()

      assert length(dists) == 1,
             "dodecahedron vertices lie on one sphere; got distinct radii #{inspect(dists)}"
    end
  end

  describe "icosahedron_vertices/0" do
    test "returns 12 vertices" do
      assert length(Dice.icosahedron_vertices()) == 12
    end

    test "every vertex has the same circumsphere radius" do
      dists =
        Dice.icosahedron_vertices()
        |> Enum.map(fn {x, y, z} -> :math.sqrt(x * x + y * y + z * z) end)
        |> Enum.map(&Float.round(&1, 6))
        |> Enum.uniq()

      assert length(dists) == 1
    end
  end

  describe "pentagonal_trapezohedron_vertices/0" do
    test "returns 12 vertices (2 apexes + 5 upper + 5 lower)" do
      verts = Dice.pentagonal_trapezohedron_vertices()
      assert length(verts) == 12
      assert Enum.count(verts, fn {_x, y, _z} -> y == 1.0 end) == 1
      assert Enum.count(verts, fn {_x, y, _z} -> y == -1.0 end) == 1
    end
  end

  describe "face_up_d20/1" do
    test "identity orientation lands on some face 1..20" do
      f = Dice.face_up_d20({0.0, 0.0, 0.0, 1.0})
      assert f in 1..20
    end

    test "flipping 180° about X picks a different face" do
      f_id = Dice.face_up_d20({0.0, 0.0, 0.0, 1.0})
      f_flip = Dice.face_up_d20({1.0, 0.0, 0.0, 0.0})
      refute f_id == f_flip,
             "a 180° X-flip should surface the opposite face"
    end
  end

  describe "face_up_d12/1" do
    test "identity orientation lands on some face 1..12" do
      f = Dice.face_up_d12({0.0, 0.0, 0.0, 1.0})
      assert f in 1..12
    end

    test "flipping 180° about X picks a different face" do
      f_id = Dice.face_up_d12({0.0, 0.0, 0.0, 1.0})
      f_flip = Dice.face_up_d12({1.0, 0.0, 0.0, 0.0})
      refute f_id == f_flip
    end
  end

  describe "face_up_d10/1" do
    test "identity orientation lands on a top kite (face 1..5)" do
      # d10_face_normals lists the 5 top kites first with +y bias, so an
      # identity-rotated die sits with a top face up.
      f = Dice.face_up_d10({0.0, 0.0, 0.0, 1.0})
      assert f in 1..5
    end

    test "flipping 180° about X lands on a bottom kite (face 6..10)" do
      f = Dice.face_up_d10({1.0, 0.0, 0.0, 0.0})
      assert f in 6..10
    end
  end
end
