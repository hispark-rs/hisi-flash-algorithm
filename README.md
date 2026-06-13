# hisi-flash-algorithm

probe-rs flash loaders ("flash algorithms") for HiSilicon RISC-V SoCs.

A flash algorithm is a tiny `#![no_std]` blob, built for the **target** ISA, that
probe-rs uploads into the chip's RAM and calls to erase and program on-chip flash.
This repository is a Cargo **workspace**; each member is the loader for one chip
family. The compiled ELF is extracted with probe-rs's `target-gen` and embedded
(base64) into the matching probe-rs target YAML.

## Crates

| Crate                   | SoC / flash                                  | Status            |
| ----------------------- | -------------------------------------------- | ----------------- |
| [`ws63`](./ws63)        | HiSilicon WS63 (Hi3863) — SFC v150 NOR flash | hardware-verified |
| `bs2x` *(planned)*      | HiSilicon BS21/BS2X                          | not yet added     |

To add another chip family, create a new member directory (e.g. `bs2x/`) and add
it to the `members` list in the root `Cargo.toml`.

## Build

All members target `riscv32imc-unknown-none-elf` (a subset of the WS63 `RV32IMFC`
ISA: no atomics, and probe-rs does not preserve FP across flash-algo calls). They
build on **stable rust** with the standard rustup target — **no** custom hisi-riscv
toolchain required. The shared `riscv32imc` target, `-Tlink.x` and
`code-model=medium` flags live in the workspace `.cargo/config.toml`.

```bash
rustup target add riscv32imc-unknown-none-elf

# build everything:
cargo build --release
# or one crate:
cargo build --release -p ws63-flash-algorithm

# ELFs land in:
#   target/riscv32imc-unknown-none-elf/release/<crate-bin-name>
```

## Embed into probe-rs

After building, extract the algorithm and splice it into the probe-rs target YAML
with `target-gen`. From this workspace root, with a probe-rs checkout at
`<probe-rs>`:

```bash
cargo run -p target-gen --manifest-path <probe-rs>/Cargo.toml -- \
  elf target/riscv32imc-unknown-none-elf/release/ws63-flash-algorithm \
  -n ws63-sfc \
  --update <probe-rs>/probe-rs/targets/HiSilicon_WS63.yaml
```

`target-gen` fills `instructions` (base64), the `pc_*` routine entry points,
`data_section_offset`, and `flash_properties`. See each crate's README for
chip-specific notes.

## Repository layout

```
hisi-flash-algorithm/
├── Cargo.toml          # workspace (members = ["ws63"]) + shared release profile
├── .cargo/config.toml  # riscv32imc target + -Tlink.x + code-model=medium
├── ws63/               # WS63 SFC NOR flash loader
│   ├── Cargo.toml
│   ├── build.rs        # emits link.x into OUT_DIR
│   ├── link.x          # PrgCode/PrgData + .trampoline ebreak first
│   └── src/main.rs
└── README.md
```
