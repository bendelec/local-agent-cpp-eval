# Consumer smoke check

A scratch consumer that links the installed-style target name `vwmini::vwmini` and
exercises the public surface only: create a mesh, add an agent with a goal, step it to
arrival, and confirm an unknown id reports `NotFound`. It is deliberately *not* part of
the library's own build.

```sh
cmake -S tests/consumer -B build/consumer -DVWMINI_SOURCE_DIR=$PWD -DVWMINI_BUILD_TESTS=OFF
cmake --build build/consumer --parallel
./build/consumer/consumer && echo OK
```

Exit status 0 means the library behaves as documented from a consumer's point of view.
