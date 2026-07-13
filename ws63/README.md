# ws63-flash-algorithm (probe-rs loader)

A probe-rs flash loader for the HiSilicon WS63 (Hi3863) on-chip SFC NOR flash. It
drives the Serial Flash Controller (SFC v150) in register/command mode for status
and erase commands and bus-DMA mode for programming. It clears the flash chip's
block-protect bits on Init.

> **Status: hardware-verified.** `probe-rs download` erases and programs WS63
> flash on a real board (GD25Q32). The extracted algorithm is embedded into
> `probe-rs/targets/HiSilicon_WS63.yaml` (`flash_algorithms: [ws63-sfc]`).

## Why a `riscv32imc` blob

A flash algorithm is a `#![no_std]` blob built for the **target** ISA, not the
host. Built for **`riscv32imc-unknown-none-elf`** — a subset of the WS63
`RV32IMFC` ISA: WS63 has **no atomics** (so not `imac`), and probe-rs does **not**
preserve FP across flash-algo calls (so no `F`). This crate builds on **stable
rust** with the standard rustup target — it does **not** need the custom hisi-riscv
toolchain.

The only external dependency is crates.io's `flash-algorithm`, which supplies the
panic handler and the `PrgCode`/`PrgData` linker sections.

## Key implementation details (do not regress)

- **Init clears block-protect without wearing status flash.** On this board the
  GD25Q32 powers up with BP0..BP2 set and silently rejects erase/program until
  cleared. Mutating operations use the vendor `WRVSR (0x50) + WRSR` sequence,
  confirm the BP bits read back clear, and restore the non-volatile status by a
  flash software reset during UnInit.
- **`.trampoline` ebreak at PrgCode offset 0.** probe-rs sets `ra = load_address`
  and (CMSIS-Pack convention) expects a routine's `ret` to self-trap there.
  `link.x` KEEPs `.trampoline` first in PrgCode and `main.rs` emits a single
  `ebreak` into it, so Init/EraseSector/ProgramPage/UnInit all `ret` here and halt.
- **`code-model=medium` (RISC-V medany).** probe-rs loads the algo at a
  runtime-chosen RAM address; the default medlow model emits absolute addresses for
  statics and runs off the rails once relocated. medany uses PC-relative `auipc`.
  Set in the workspace `.cargo/config.toml` alongside `-Tlink.x`.
- **Reliable, bounded polling.** Every WIP/`start`-bit poll is bounded. WIP samples
  are separated by about 100 us; a 64 KiB erase otherwise exhausts a tight
  CPU-speed loop before the NOR finishes and used to fall through into program.
- **Cache-coherent verification.** UnInit cleans/invalidates D-cache and
  invalidates I-cache before probe-rs reads XIP flash. The optional CMSIS Verify
  entry point compares on target and remains available for release evidence.

## Build

```bash
rustup target add riscv32imc-unknown-none-elf
# from the workspace root:
cargo build --release -p ws63-flash-algorithm
# or from this directory:
cd ws63 && cargo build --release
# -> target/riscv32imc-unknown-none-elf/release/ws63-flash-algorithm
```

## Extract + embed into probe-rs

```bash
# from this workspace root, with a probe-rs checkout at <probe-rs>:
ELF=target/riscv32imc-unknown-none-elf/release/ws63-flash-algorithm
cargo run -p target-gen --manifest-path <probe-rs>/Cargo.toml -- \
  elf "$ELF" -n ws63-sfc --update <probe-rs>/probe-rs/targets/HiSilicon_WS63.yaml
```

`target-gen` fills `instructions` (base64), `pc_init`, `pc_uninit`,
`pc_program_page`, `pc_erase_sector`, `data_section_offset`, and
`pc_verify`, and `flash_properties`. The range is `0x200000..0xa00000`, the host
batch is 64 KiB, and erase geometry is 4 KiB / 64 KiB / 4 KiB. The variant's
`flash_algorithms: [ws63-sfc]` is already set in the WS63 target.

## What it does (SFC v150 command + bus-DMA modes)

- SFC base `0x4800_0000`: `cmd_config@0x300`, `cmd_ins@0x308`, `cmd_addr@0x30c`,
  `cmd_databuf[0..15]@0x400` (64 B max per reg-mode transfer).
- `erase_sector`: WREN, then 64 KiB opcode `0xD8` for aligned app blocks or 4 KiB
  opcode `0x20` at protected boundaries, then poll the real RDSR WIP state.
- The 64 KiB geometry is limited to `0x230000..0x5f0000`. The final range before
  the WS63 NV partition at `0x5fc000` remains 4 KiB, so a block erase cannot cross
  into NV. This mirrors the vendor SDK's greedy erase order (`0xD8`, `0x52`,
  `0x20`) without teaching probe-rs any HiSilicon image format.
- `program_page`: WREN → configure the bus-DMA source/flash address/length → start
  one hardware transfer. The 64 KiB host batch is passed directly from SRAM to
  SFC; page-program sequencing is handled by the same DMA engine used by the
  vendor SDK's `hal_sfc_dma_write()`.
- CPU XIP address → flash offset via `XIP_BASE = 0x200000`.

## Performance verification

Use the same planned `.img`, probe, cable and board for every comparison. Keep
`--verify` enabled for explicit release verification; a smoke run may omit the
second upload only when JLink nRST is followed by flashboot's authoritative body
hash check and the firmware UART marker.

The 2026-07-13 WS63 reference run used a 656,584-byte WPA connectivity image:

| Configuration | Erase calls | Program batches | Three runs |
| --- | ---: | ---: | --- |
| legacy 4 KiB geometry, J-Link 400 kHz | 161 | 11 | 202.72 s / 206.20 s / 201.04 s |
| safe 64 KiB geometry, J-Link 2 MHz | 11 | 11 | 79.71 s / 79.98 s / 79.72 s |
| above + target-opt-in DMI batch=64 | 11 | 11 | 29.65 s / 29.45 s / 29.49 s |

The final row also requires probe-rs's explicit WS63 repeated-DMI-write
capability. It is not an algorithm-only improvement and must not be enabled for
other `ArmWithRiscv` targets without separate verification.

Detailed probe-rs tracing must show two 64 KiB page buffers and
`Double Buffering enabled: true`; do not infer double buffering only from SRAM
capacity. `4 MHz` improved small-image transfer but produced DAP NoAck during a
long transfer on the reference probe, so runners default to the repeatedly tested
`2 MHz` setting.
