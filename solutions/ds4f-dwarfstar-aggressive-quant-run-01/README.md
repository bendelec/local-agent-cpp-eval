# VWmini

Implement the library described by these documents:

1. `docs/requirements/requirements.md`
2. `docs/requirements/mesh.md`
3. `docs/requirements/simulation.md`
4. `docs/requirements/non-functional.md`
5. `docs/architecture/public-api.md`

Before implementation, create the required
`docs/architecture/architecture.md`; it is intentionally not supplied in this package.
Keep it current as the design evolves.

The public headers already define the fixed interface. Preserve every public declaration; you may add only private implementation details to the headers. Implement the library under `src/`, add a
CMake target `vwmini::vwmini`, add deterministic tests under `tests/`, and create the
architecture document required at `docs/architecture/architecture.md`.

Typical local verification:

```sh
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```
