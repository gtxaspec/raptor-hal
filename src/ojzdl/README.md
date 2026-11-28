# ojzdl (vendored)

Open runtime for Ingenic JZDL models (the format of `libjzdl.m.so`), used by
`src/hal_ivs_ojzdl.c` when raptor-hal is built with `OJZDL=1`.

- Runs JZDL version 4 and 5 models (e.g. the magik `yolov5` and `persondet`
  samples) with output bit-identical to libjzdl 5.4.0.
- T31 only (`OJZDL=1` is refused elsewhere): MXU2 vector kernels (`-DOJZDL_MXU2`).
  The plain C path still exists for host tests; on a camera it is ~30x slower than libjzdl.
- `ojzdl_resize` reproduces `jzdl::resize` exactly for 640x360 -> 416x320 / 320x416.

Host test in this repo: `make -C tests test` runs `tests/test_ojzdl.c`, which checks
ojzdl's output for a small random-weight model (`tests/ojzdl/gen_tiny.py`) against
libjzdl's stored output for it, byte for byte, plus the loader's refusals.

Upstream source, tests (bit-exact gate against libjzdl, MXU2 self-test) and
notes: Small6oy/thingino-ai, `tools/ojzdl/` and `docs/jzdl-runtime.md`.
Files here are copied unchanged from there; change them there first.

Licence: MIT (SPDX header in each file), as raptor-hal's own sources; Copyright (C) 2026 Thingino Project.
