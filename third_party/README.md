# Third-Party Runtimes

Place the downloaded CARLA Linux distribution here as:

```text
third_party/CARLA_0.9.15/
|-- CarlaUE4.sh
`-- CarlaUE4/
```

The CARLA distribution is intentionally ignored by Git because it contains
large external binaries and assets. Project-owned CARLA client code belongs in
`simulators/carla_3d/`.
