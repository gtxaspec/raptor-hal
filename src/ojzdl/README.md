# ojzdl (vendored)

Open runtime for Ingenic JZDL models (the format of `libjzdl.m.so`), used by
`src/hal_ivs_ojzdl.c` when raptor-hal is built with `OJZDL=1`.

- Runs JZDL version 4 and 5 models (e.g. the magik `yolov5` and `persondet`
  samples) with output bit-identical to libjzdl 5.4.0.
- MXU2 vector kernels on the T31 (`-DOJZDL_MXU2`); plain C everywhere else.
- `ojzdl_resize` reproduces `jzdl::resize` exactly for 640x360 -> 416x320 / 320x416.

Upstream source, tests (bit-exact gate against libjzdl, MXU2 self-test) and
notes: Small6oy/thingino-ai, `tools/ojzdl/` and `docs/jzdl-runtime.md`.
Files here are copied unchanged from there; change them there first.

Licence: MIT (SPDX header in each file), as raptor-hal's own sources; Copyright (C) 2026 Thingino Project.
