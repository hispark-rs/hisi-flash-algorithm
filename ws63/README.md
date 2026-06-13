# ws63-flash-algorithm (probe-rs loader)

A probe-rs flash loader for the HiSilicon WS63 (Hi3863) on-chip SFC NOR flash. It
drives the Serial Flash Controller (SFC v150) in register/command mode to issue
standard SPI-NOR commands (WREN / RDSR / 4K sector-erase / page-program), and
clears the flash chip's block-protect bits on Init.

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

- **Init clears block-protect.** On this board the GD25Q32 powers up with BP0..BP2
  set (RDSR=0x1e) and silently rejects every erase/program until cleared. Init does
  `WREN + WRSR(status=0x00)`.
- **`.trampoline` ebreak at PrgCode offset 0.** probe-rs sets `ra = load_address`
  and (CMSIS-Pack convention) expects a routine's `ret` to self-trap there.
  `link.x` KEEPs `.trampoline` first in PrgCode and `main.rs` emits a single
  `ebreak` into it, so Init/EraseSector/ProgramPage/UnInit all `ret` here and halt.
- **`code-model=medium` (RISC-V medany).** probe-rs loads the algo at a
  runtime-chosen RAM address; the default medlow model emits absolute addresses for
  statics and runs off the rails once relocated. medany uses PC-relative `auipc`.
  Set in the workspace `.cargo/config.toml` alongside `-Tlink.x`.
- **Bounded polling.** Every WIP/`start`-bit poll is bounded so a routine can never
  spin forever and always returns to the trampoline.

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
`flash_properties` (range `0x200000..0xa00000`, page `0x100`, 4K sectors). The
variant's `flash_algorithms: [ws63-sfc]` is already set in the WS63 target.

## What it does (SFC v150 register/command mode)

- SFC base `0x4800_0000`: `cmd_config@0x300`, `cmd_ins@0x308`, `cmd_addr@0x30c`,
  `cmd_databuf[0..15]@0x400` (64 B max per reg-mode transfer).
- `erase_sector`: WREN → opcode `0x20` (4K) at the flash offset → poll RDSR WIP.
- `program_page`: for each ≤64 B chunk → WREN → load `cmd_databuf` → opcode `0x02`
  → poll RDSR WIP.
- CPU XIP address → flash offset via `XIP_BASE = 0x200000`.
