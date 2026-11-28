# Provenance: ojzdl and onna

How the libjzdl (T31) and venus (T41) model formats and arithmetic were worked out, for the
open runtimes in `src/ojzdl/` (`OJZDL=1`) and `src/onna/` (`ONNA=1`). The full write-ups live in
Small6oy/thingino-ai: `docs/jzdl-format.md`, `docs/jzdl-runtime.md`, `docs/nna-runtime.md`.

## ojzdl / libjzdl format (T31)

- **Reference binaries:** `libjzdl.m.so` (it reports jzdl 1.7.0; the 5.4.0 uClibc "simd" build)
  from ingenic-lib (`acceleration-modules/jzdl/lib/5.4.0/uclibc`), and Ingenic's public sample models
  `magik_model_persondet.bin` (version 5) and `magik_model_yolov5.bin` (version 4) from
  gtxaspec/magik_models (`Txx/jzdl`). Later also seven public Magik 1.0 kit models from
  andylee77/thingino-sigmastar-fwd (`models/t31`).
- **Loader / file layout:** libjzdl's Net/Layer code is ncnn-derived but stripped. The creator
  table and per-layer `load_param`/`load_model` vtable slots were located, then libjzdl's own
  loader was run under Unicorn (libc stubbed) and every `fread` logged. A Python parser/writer
  round-trips both samples byte for byte, and its blob offsets match the logged freads
  (100/100 persondet, 353/353 yolov5).
- **Arithmetic:** a NumPy reference (`jzdl_ref.py`) was fitted against libjzdl's own output,
  run under QEMU user-mode (`qemu-mipsel -cpu XBurstR2`, an Ingenic-focused fork of
  gtxaspec/qemu with MXU2) with a small harness that dumps any intermediate blob. Version-4 rules
  were pinned down layer by layer by feeding each layer libjzdl's own inputs. Checked on
  real and synthetic frames (noise, black, white, checkerboard), then on a Wyze Cam v3.
- **MXU2 kernels:** instruction encodings come from QEMU's `target/mips/tcg/mxu2.decode`.
  libjzdl's code was only scanned to list which instructions it uses; the kernels are written
  from the instruction semantics, and each instruction form is tested against a C model.
- **Result:** bit-identical to libjzdl (36/36 outputs for v5, 54/54 for v4, 63/63 for the kit
  models) under QEMU and on the camera; detections identical to raptor-hal's libjzdl backend.

## onna / venus format (T41 NNA)

- **Reference binaries:** venus 0.1.2.5 from Ingenic's magik-toolkit (InferenceKit), used with
  magik-transform-tools 2.0.4 (its matching converter) to convert our own ONNX models
  (YOLOv5n and the thingino-ai detectors). venus 0.3.7.1 from ingenic-lib was checked as well.
- **NNA behaviour:** gtxaspec/qemu's `ingenic-nna` model (release `ingenic-0.7.26`) runs venus
  byte for byte against silicon, and its existing `-global ingenic-nna.log` and `-trace` options
  log every NN instruction and NNDMA descriptor. One YOLOv5n frame under venus is ~1.3M NN
  instructions; that trace, plus QEMU's `hw/misc/ingenic-nna.c` as the unit spec, gave the
  register meanings and data layouts. Instruction encodings come from QEMU's `mxu3.decode`.
- **Arithmetic:** a NumPy golden model reads the venus `.bin` and the ONNX graph and reproduces
  venus's head outputs bit for bit; later models were compared op by op with Ingenic's PC
  executor from the magik Docker image. Silicon behaviour QEMU did not model (F RAM slicing,
  W RAM size, register `0x5b`) was measured with small probes on a Wyze Cam v4 and reported
  upstream (gtxaspec/qemu#63, #65).
- **Venus name key:** the venus `.bin` header is the converter's op list, with each name stored
  as an int32 length followed by the bytes XORed with a fixed 9-byte cycle
  (`f4 36 a5 3e 96 7c ce 94 e8`). It was recovered as a known-plaintext XOR: the models were our
  own conversions, so the tensor names in them were known from the ONNX graphs we converted.
- **Result:** heads byte-identical to venus / the PC executor on the host, under QEMU and on a
  Wyze Cam v4.

## Licensing

ojzdl and onna are our own code, MIT (SPDX header in every source file, like raptor-hal's own
sources), Copyright (C) 2026 Thingino Project; thingino-ai's LICENSE is MIT. No Ingenic code is
copied and neither runtime includes an Ingenic header. onna programs some NNA register values
that were observed in traces of venus runs (data, not code). QEMU's GPL NNA engine is used only
as a host test oracle in thingino-ai, extracted at build time, and is not part of either runtime
or of this repository.
