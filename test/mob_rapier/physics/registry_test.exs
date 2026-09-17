defmodule MobRapier.Physics.RegistryTest do
  # Not async — ETS table is a named singleton shared across the module.
  use ExUnit.Case, async: false

  alias MobRapier.Physics
  alias MobRapier.Physics.Registry

  setup do
    _ = Registry.start_link()
    Registry.clear()
    :ok
  end

  describe "register/lookup/names" do
    test "empty registry lists no names" do
      assert Registry.names() == []
    end

    test "register + lookup round-trip" do
      ref = make_ref()
      assert :ok = Registry.register("only", ref)
      assert {:ok, ^ref} = Registry.lookup("only")
      assert Registry.names() == ["only"]
    end

    test "lookup of an unknown name is honest" do
      assert Registry.lookup("no-such-world") == {:error, :not_found}
    end

    test "re-registering the same name replaces" do
      Registry.register("swap", make_ref())
      new_ref = make_ref()
      Registry.register("swap", new_ref)
      assert {:ok, ^new_ref} = Registry.lookup("swap")
      assert Registry.names() == ["swap"]
    end

    test "unregister drops the name" do
      Registry.register("gone", make_ref())
      assert :ok = Registry.unregister("gone")
      assert Registry.lookup("gone") == {:error, :not_found}
    end

    test "unregister of an unknown name is a no-op, not a raise" do
      assert :ok = Registry.unregister("never-registered")
    end

    test "names are sorted" do
      Registry.register("c", make_ref())
      Registry.register("a", make_ref())
      Registry.register("b", make_ref())
      assert Registry.names() == ["a", "b", "c"]
    end
  end

  describe "Physics by-name API on host" do
    # The NIF is not loaded on the host BEAM, so the by-name paths that
    # call into it (add_ball_in, transforms_in, ...) can't complete — but
    # the resolution shape (bad name -> {:error, :not_found}) is host-side
    # and worth pinning here so bad-name callers don't crash.

    test "add_ball_in returns :not_found for an unknown world" do
      assert Physics.add_ball_in("no-world", 0.0, 1.0, 0.0, 0.3) ==
               {:error, :not_found}
    end

    test "transforms_in returns :not_found for an unknown world" do
      assert Physics.transforms_in("no-world") == {:error, :not_found}
    end

    test "step_in returns :not_found for an unknown world" do
      assert Physics.step_in("no-world", 0.016) == {:error, :not_found}
    end
  end
end
