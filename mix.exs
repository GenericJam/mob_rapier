defmodule MobRapier.MixProject do
  use Mix.Project

  @version "0.1.0"
  @source_url "https://github.com/GenericJam/mob_rapier"

  def project do
    [
      app: :mob_rapier,
      version: @version,
      description:
        "Rapier 3D physics wrapped as a Rustler NIF, with a named-world " <>
          "registry and face-up decode rules for the dice + shells shapes " <>
          "the rapier_lab spike proved out",
      elixir: "~> 1.20",
      start_permanent: Mix.env() == :prod,
      elixirc_paths: elixirc_paths(Mix.env()),
      deps: deps(),
      source_url: @source_url,
      package: package(),
      docs: docs()
    ]
  end

  defp elixirc_paths(:test), do: ["lib", "test/support"]
  defp elixirc_paths(_env), do: ["lib"]

  defp package do
    [
      licenses: ["MIT"],
      links: %{"GitHub" => @source_url},
      # Native Rust crate must ship in the package so the host app can
      # cross-compile it (mob_dev's static_nifs pipeline reads from
      # deps/mob_rapier/native/lab_physics). Everything the compile step
      # reads at COMPILE time lives here — no repo-root dotfiles.
      files:
        ~w(lib native/lab_physics/Cargo.toml native/lab_physics/src
           mix.exs README* CHANGELOG* LICENSE)
    ]
  end

  defp docs do
    [
      main: "readme",
      source_url: @source_url,
      source_url_pattern: "#{@source_url}/blob/master/%{path}#L%{line}",
      extras: [
        "README.md": [title: "mob_rapier"],
        "CHANGELOG.md": [title: "Changelog"]
      ]
    ]
  end

  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp deps do
    [
      # Rustler drives the lab_physics NIF crate (Rapier 3D wrapper). 0.38+
      # carries the Bionic dlsym fix without which nif_init aborts on
      # Android — see rapier_lab-2ie.
      {:rustler, "~> 0.38"},
      {:ex_doc, "~> 0.34", only: :dev, runtime: false}
    ]
  end
end
