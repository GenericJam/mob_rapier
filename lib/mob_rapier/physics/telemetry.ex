defmodule MobRapier.Physics.Telemetry do
  @moduledoc """
  Streamed body-state telemetry — the push counterpart to the snapshot
  API on `MobRapier.Physics.body_states_in/1`.

  Two shapes, one payload:

    * **Snapshot** — `MobRapier.Physics.body_states_in(world_name)`
      returns `[%MobRapier.Physics.BodyState{}]` right now. Use this from
      IEx when you want to peek once.

    * **Stream** — `stream/2` (this module) starts a light GenServer that
      samples the world on a timer and sends `mob` `Mob.Listener`-shape
      messages to a subscriber. Use this when you want to watch how the
      state changes tick over tick — e.g. an agent RPC'd in over Erlang
      dist watching a shell settle, or a screen wanting to log frames.

  ## Message envelope — matches `Mob.Listener`

  The stream emits `{event, tag, payload}` triples, the same shape mob's
  own scroll / drag / touch events use. `event` is the atom
  `:mob_rapier_telemetry`, `tag` is the world name (a binary), and
  `payload` is a map:

      {:mob_rapier_telemetry, world_name, %{frame: n, states: [%BodyState{}]}}

  This shape gives you **selective receive** for free — a screen watching
  two worlds picks off one with:

      receive do
        {:mob_rapier_telemetry, "shells", %{states: states}} -> ...
      after
        200 -> :timeout
      end

  and the mailbox scan skips everything else. That is the pattern
  `Mob.Listener` follows for gesture streams; this module borrows it so
  consumers write one `receive` grammar for every kind of high-frequency
  device event.
  """

  use GenServer

  alias MobRapier.Physics
  alias MobRapier.Physics.BodyState

  @typedoc "One streamed telemetry frame."
  @type frame ::
          {:mob_rapier_telemetry, world_name :: String.t(),
           %{frame: non_neg_integer(), states: [BodyState.t()]}}

  @doc """
  Start a stream on `world_name`, sending one telemetry frame per sample
  to `opts[:to]` (defaults to the caller pid).

  Options:

    * `:to` — pid to receive the frames (default: `self()`). The stream
      exits normally when this pid dies, so it's safe to start-and-forget
      from an IEx session — leave IEx, the streamer stops.

    * `:interval_ms` — sample interval, milliseconds (default: 100).
      Independent of the physics tick — this is *how often we peek*, not
      how often the world advances.

    * `:name` — GenServer name (default: unregistered). Handy when you
      want to `stop/1` a stream by name from another session.

  Returns `{:ok, pid}` for the streamer. Stop it with `stop/1`, or just
  let the subscriber die.
  """
  @spec stream(String.t(), keyword()) :: GenServer.on_start()
  def stream(world_name, opts \\ []) when is_binary(world_name) do
    to = Keyword.get(opts, :to, self())
    interval = Keyword.get(opts, :interval_ms, 100)
    gen_opts = if name = opts[:name], do: [name: name], else: []

    GenServer.start_link(
      __MODULE__,
      %{world: world_name, to: to, interval_ms: interval, frame: 0},
      gen_opts
    )
  end

  @doc "Stop a running stream (see `stream/2`)."
  @spec stop(GenServer.server()) :: :ok
  def stop(server), do: GenServer.stop(server)

  @doc """
  Emit one telemetry frame right now to the subscribers a caller manages.

  Callers that already own a tick loop (e.g. a `Mob.Screen` calling
  `Physics.step_in/2` per tick) can invoke this directly instead of
  spinning up a separate GenServer via `stream/2`. Same envelope shape,
  so the same selective-receive grammar works for both push paths.
  """
  @spec emit(pid() | [pid()], String.t(), non_neg_integer(), [BodyState.t()]) :: :ok
  def emit(subscribers, world_name, frame, states)
      when is_binary(world_name) and is_integer(frame) and is_list(states) do
    payload = %{frame: frame, states: states}
    envelope = {:mob_rapier_telemetry, world_name, payload}

    subscribers
    |> List.wrap()
    |> Enum.each(fn pid when is_pid(pid) -> send(pid, envelope) end)
  end

  @impl true
  def init(state) do
    Process.monitor(state.to)
    Process.send_after(self(), :sample, state.interval_ms)
    {:ok, state}
  end

  @impl true
  def handle_info(:sample, state) do
    case Physics.body_states_in(state.world) do
      states when is_list(states) ->
        emit(state.to, state.world, state.frame, states)

      _ ->
        # World gone; keep polling — a caller may destroy_world + new_world
        # under the same name mid-stream (Reset button in the screens).
        :ok
    end

    Process.send_after(self(), :sample, state.interval_ms)
    {:noreply, %{state | frame: state.frame + 1}}
  end

  def handle_info({:DOWN, _ref, :process, pid, _reason}, %{to: pid} = state),
    do: {:stop, :normal, state}

  def handle_info(_msg, state), do: {:noreply, state}
end
