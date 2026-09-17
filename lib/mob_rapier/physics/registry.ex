defmodule MobRapier.Physics.Registry do
  @moduledoc """
  On-device name → physics world registry (bead rapier_lab-ou4).

  A NIF `ResourceArc` cannot cross Erlang dist as a usable resource — a
  handle returned from an `:rpc.call` to `MobRapier.Physics.world_new/0`
  reads as an opaque reference on the caller, and any follow-up NIF call
  with it from a different node raises `:badarg` on the device side.

  The registry keeps the resource local: an agent (or IEx) creates a world
  by name, then every subsequent NIF call rides the name — a plain string
  that survives dist round-trips just fine. `MobRapier.Physics.world/1`
  looks the resource up in ETS and returns it, so on-device callers can
  still get the raw handle when they want it (e.g. inside a screen tick).

  See `MobRapier.Physics` for the by-name convenience wrappers.
  """

  use GenServer

  @table :mob_rapier_worlds

  def start_link(_opts \\ []) do
    GenServer.start_link(__MODULE__, :ok, name: __MODULE__)
  end

  @doc "Registers `resource` under `name`, replacing any previous entry."
  @spec register(String.t(), reference()) :: :ok
  def register(name, resource) when is_binary(name) do
    ensure_table()
    :ets.insert(@table, {name, resource})
    :ok
  end

  @doc """
  Returns `{:ok, resource}` for `name`, or `{:error, :not_found}`.

  Only useful on-device — a resource returned across dist is inert.
  """
  @spec lookup(String.t()) :: {:ok, reference()} | {:error, :not_found}
  def lookup(name) when is_binary(name) do
    case safe_lookup(name) do
      [{^name, resource}] -> {:ok, resource}
      [] -> {:error, :not_found}
    end
  end

  @doc "Names of every registered world, in insertion order."
  @spec names() :: [String.t()]
  def names do
    case :ets.whereis(@table) do
      :undefined -> []
      _ -> @table |> :ets.tab2list() |> Enum.map(&elem(&1, 0)) |> Enum.sort()
    end
  end

  @doc "Drops `name` from the registry — a no-op if unknown."
  @spec unregister(String.t()) :: :ok
  def unregister(name) when is_binary(name) do
    case :ets.whereis(@table) do
      :undefined -> :ok
      _ -> :ets.delete(@table, name) && :ok
    end
  end

  @doc "Drops every world — useful in tests + between agent runs."
  @spec clear() :: :ok
  def clear do
    case :ets.whereis(@table) do
      :undefined -> :ok
      _ -> :ets.delete_all_objects(@table) && :ok
    end
  end

  @impl true
  def init(:ok) do
    ensure_table()
    {:ok, %{}}
  end

  defp ensure_table do
    case :ets.whereis(@table) do
      :undefined ->
        :ets.new(@table, [
          :named_table,
          :public,
          :set,
          read_concurrency: true,
          write_concurrency: true
        ])

      _ ->
        @table
    end
  end

  defp safe_lookup(name) do
    case :ets.whereis(@table) do
      :undefined -> []
      _ -> :ets.lookup(@table, name)
    end
  end
end
