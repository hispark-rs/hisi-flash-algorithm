//! probe-rs flash loader for the HiSilicon WS63 (Hi3863) on-chip SFC NOR flash.
//!
//! This is the loader blob that probe-rs uploads to target RAM and calls to erase
//! and program flash. It drives the WS63 Serial Flash Controller (SFC v150) in
//! register/command mode to issue standard SPI-NOR commands (WREN/RDSR/SE/PP).
//!
//! Ground truth: `fbb_ws63` `hal_sfc_v150` (`hal_sfc_v150.c`,
//! `hal_sfc_v150_regs_def.h`) corroborated by HiSpark Studio's OpenOCD
//! `src/flash/nor/ws63.c`. **UNVALIDATED on silicon** — every register field and
//! the XIP-base mapping below is reverse-engineered and must be checked on a board
//! (see ../README.md). Built for `riscv32imc` (WS63 has no atomics; probe-rs does
//! not preserve FP across algo calls, so no F/A is used).

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};
use flash_algorithm::*;

// Return trampoline. probe-rs sets `ra = load_address` and, per the CMSIS-Pack
// convention, expects a routine's `ret` to self-trap there. The flash-algorithm
// crate lays its functions out at `.entry` with no trap, so without this the core
// runs off into whatever links at offset 0 (EraseSector) and the routine call
// times out. link.x KEEPs `.trampoline` first in PrgCode, so this `ebreak` sits at
// load_address; Init/EraseSector/ProgramPage/UnInit all `ret` here and halt.
core::arch::global_asm!(
    ".pushsection .trampoline, \"ax\"",
    ".globl _flash_algo_return_trap",
    "_flash_algo_return_trap:",
    "ebreak",
    ".popsection",
);

// ---- SFC v150 register map (base 0x4800_0000) ----
const SFC_BASE: u32 = 0x4800_0000;
const CMD_CONFIG: u32 = SFC_BASE + 0x300; // [0]start [1]sel_cs [3]addr_en [7]data_en
                                          // [8]rw(0=wr,1=rd) [9:14]data_cnt=len-1 [17:19]if_type
const CMD_INS: u32 = SFC_BASE + 0x308; // [7:0] opcode
const CMD_ADDR: u32 = SFC_BASE + 0x30C; // flash chip offset
const CMD_DATABUF: u32 = SFC_BASE + 0x400; // 16 x u32

/// Flash chip offset 0 is mapped to CPU address 0x200000 (SFC bus_base_addr_cs0).
const XIP_BASE: u32 = 0x0020_0000;

// Standard SPI-NOR opcodes (flash_common_config.h).
const OP_WREN: u32 = 0x06;
const OP_RDSR: u32 = 0x05;
const OP_SE_4K: u32 = 0x20; // 4 KiB sector erase
const OP_PP: u32 = 0x02; // page program
const OP_WRSR: u32 = 0x01; // write status register 1
const OP_RDSR2: u32 = 0x35; // read status register 2
const OP_RDSR3: u32 = 0x15; // read status register 3
const OP_WRSR3: u32 = 0x11; // write status register 3

// SPI-NOR software reset sequence: RSTEN (0x66) followed by RST (0x99).
const OP_RSTEN: u32 = 0x66; // reset enable
const OP_RST: u32 = 0x99; // reset memory

const RW_WRITE: u32 = 0;
const RW_READ: u32 = 1;
const IF_STD: u32 = 0; // standard (1-1-1) SPI

/// cmd_databuf is 16 x 4 bytes = 64 bytes max per reg-mode data transfer.
const SFC_MAX_DATA: usize = 64;

/// WIP-poll budget (each iteration issues one RDSR).
const WIP_POLL_LIMIT: u32 = 16_384;

#[inline(always)]
fn wr(addr: u32, val: u32) {
    // SAFETY: fixed SFC MMIO addresses.
    unsafe { write_volatile(addr as *mut u32, val) }
}
#[inline(always)]
fn rd(addr: u32) -> u32 {
    // SAFETY: fixed SFC MMIO addresses.
    unsafe { read_volatile(addr as *const u32) }
}

/// Assemble `cmd_config` (bit layout from `hal_sfc_v150_regs_def.h`). `start` is
/// always set; `sel_cs` follows the SDK (which writes 1).
#[inline(always)]
fn cmd_config(addr_en: bool, data_en: bool, rw: u32, data_cnt: u32) -> u32 {
    1                                  // [0]    start
        | (1 << 1)                     // [1]    sel_cs (SDK writes 1)
        | ((addr_en as u32) << 3)      // [3]    addr_en
        | ((data_en as u32) << 7)      // [7]    data_en
        | ((rw & 0x1) << 8)            // [8]    rw (0=write, 1=read)
        | ((data_cnt & 0x3f) << 9)     // [9:14] data_cnt = byte_count - 1
        | ((IF_STD & 0x7) << 17) // [17:19] mem_if_type
}

/// Poll the `start` bit until the controller finishes the transaction.
///
/// Bounded so a routine can never spin forever (e.g. if the SFC `start` bit is
/// never cleared); it always returns to the trampoline and halts.
#[inline(always)]
fn wait_cmd_done() {
    let mut n: u32 = 0;
    while rd(CMD_CONFIG) & 1 != 0 {
        n = n.wrapping_add(1);
        if n > WIP_POLL_LIMIT {
            break;
        }
    }
}

/// Issue WREN (sets the flash WEL latch). Required before every erase/program.
fn write_enable() {
    wr(CMD_INS, OP_WREN);
    wr(CMD_CONFIG, cmd_config(false, false, RW_WRITE, 0));
    wait_cmd_done();
}

/// Poll RDSR until the WIP bit (status bit 0) clears.
fn wait_ready() -> Result<(), ErrorCode> {
    for _ in 0..WIP_POLL_LIMIT {
        wr(CMD_INS, OP_RDSR);
        wr(CMD_CONFIG, cmd_config(false, true, RW_READ, 0)); // read 1 status byte
        wait_cmd_done();
        if rd(CMD_DATABUF) & 0x1 == 0 {
            return Ok(());
        }
    }
    Err(ErrorCode::new(0x57630001).unwrap()) // WIP wait timeout
}

/// Best-effort ready wait used after mutating flash operations.
///
/// On WS63 the SFC register command path can stop answering RDSR after a sector
/// erase when entered from the vendor firmware state. The erase has already been
/// accepted by the flash chip, but waiting forever keeps the probe-rs routine
/// running until the debug transport times out. Bound the poll and let the host
/// continue; later readback/verify catches a real failed erase/program.
fn wait_ready_best_effort() {
    let _ = wait_ready();
}

/// Issue a SPI-NOR software reset (RSTEN + RST).
/// This returns the flash chip to its power-on state, clearing any lingering
/// command/state left by the erase/program sequence.
fn flash_software_reset() {
    // RSTEN must be immediately followed by RST; do not insert WREN or other
    // commands between them.
    wr(CMD_INS, OP_RSTEN);
    wr(CMD_CONFIG, cmd_config(false, false, RW_WRITE, 0));
    wait_cmd_done();
    wr(CMD_INS, OP_RST);
    wr(CMD_CONFIG, cmd_config(false, false, RW_WRITE, 0));
    wait_cmd_done();
    let _ = wait_ready();
}

/// Read a flash status register via the given RDSR opcode. Returns 1 byte.
fn read_status_register_op(rdsr_op: u32) -> u8 {
    wr(CMD_INS, rdsr_op);
    wr(CMD_CONFIG, cmd_config(false, true, RW_READ, 0)); // read 1 status byte
    wait_cmd_done();
    (rd(CMD_DATABUF) & 0xFF) as u8
}

/// Write a flash status register via the given WRSR opcode. Requires WREN first.
fn write_status_register_op(wrsr_op: u32, val: u8) {
    write_enable();
    wr(CMD_DATABUF + 0, val as u32);
    wr(CMD_INS, wrsr_op);
    wr(CMD_CONFIG, cmd_config(false, true, RW_WRITE, 0)); // write 1 status byte
    wait_cmd_done();
    let _ = wait_ready();
}

/// Write SR1 and SR2 together via WRSR (0x01). Some GD25Q32 variants only accept
/// the two-byte form of this command; the single-byte form is ignored.
fn write_status_registers_sr1_sr2(sr1: u8, sr2: u8) {
    write_enable();
    let val = ((sr2 as u32) << 8) | (sr1 as u32);
    wr(CMD_DATABUF + 0, val);
    wr(CMD_INS, OP_WRSR);
    wr(CMD_CONFIG, cmd_config(false, true, RW_WRITE, 1)); // write 2 status bytes
    wait_cmd_done();
    let _ = wait_ready();
}

/// GD25Q32 status register values expected by flashboot's `sfc_port_fix_sr()`.
/// SR1: BP0..BP2 (bits 2..4) = 0b111 = block protect. mask 0x9C, valid 0x1C.
/// SR2: QE bit1=0, SUS bits clear. mask 0x43, valid 0x02.
/// SR3: bit5=1. mask 0x61, valid 0x20.
/// flashboot checks all three AFTER `uapi_sfc_init`, but `uapi_sfc_init` itself
/// can fail if the flash is in a bad state. We restore all three in Drop.
const EXPECTED_SR1: u8 = 0x1C;
const EXPECTED_SR2: u8 = 0x02;
const EXPECTED_SR3: u8 = 0x20;

struct Ws63Algo {
    saved_sr1: u8,
    saved_sr2: u8,
    saved_sr3: u8,
}

algorithm!(Ws63Algo, {
    device_name: "ws63",
    device_type: DeviceType::Onchip,
    flash_address: 0x200000,
    flash_size: 0x800000,
    page_size: 0x100,
    empty_value: 0xFF,
    program_time_out: 1000,
    erase_time_out: 30000,
    sectors: [{
        size: 0x1000,
        address: 0x0,
    }]
});

impl FlashAlgorithm for Ws63Algo {
    fn new(_address: u32, _clock: u32, _function: Function) -> Result<Self, ErrorCode> {
        // The SFC is left as configured by the boot ROM / flashboot (XIP bus mode);
        // the register/command path used below operates alongside it.
        //
        // Save the flash status register, then clear block-protect bits (BP0..BP2)
        // so erase/program will succeed. On this board (GD25Q32) the BP bits are
        // set at power-on (RDSR=0x1C or 0x1E) and the chip silently rejects every
        // erase/program until they are cleared. We restore the original SR in
        // Drop so flashboot's `sfc_port_fix_sr()` doesn't trip on SR=0x00.
        let saved_sr1 = read_status_register_op(OP_RDSR);
        let saved_sr2 = read_status_register_op(OP_RDSR2);
        let saved_sr3 = read_status_register_op(OP_RDSR3);
        // Clear block-protect in SR1 so erase/program will succeed.
        write_status_register_op(OP_WRSR, 0x00);
        Ok(Self {
            saved_sr1,
            saved_sr2,
            saved_sr3,
        })
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), ErrorCode> {
        let off = address.wrapping_sub(XIP_BASE); // CPU XIP addr -> flash offset
        write_enable();
        wr(CMD_INS, OP_SE_4K);
        wr(CMD_ADDR, off);
        wr(CMD_CONFIG, cmd_config(true, false, RW_WRITE, 0));
        wait_cmd_done();
        wait_ready_best_effort();
        Ok(())
    }

    fn program_page(&mut self, address: u32, data: &[u8]) -> Result<(), ErrorCode> {
        let mut off = address.wrapping_sub(XIP_BASE);
        for chunk in data.chunks(SFC_MAX_DATA) {
            write_enable();
            // Pack bytes into cmd_databuf words (little-endian); pad the tail word
            // with 0xFF — `data_cnt` bounds the actual byte count, so padding is
            // never programmed.
            let words = chunk.len().div_ceil(4);
            for w in 0..words {
                let mut v = 0xFFFF_FFFFu32;
                for b in 0..4 {
                    let idx = w * 4 + b;
                    if idx < chunk.len() {
                        let shift = (b * 8) as u32;
                        v = (v & !(0xFFu32 << shift)) | ((chunk[idx] as u32) << shift);
                    }
                }
                wr(CMD_DATABUF + (w as u32) * 4, v);
            }
            wr(CMD_INS, OP_PP);
            wr(CMD_ADDR, off);
            wr(
                CMD_CONFIG,
                cmd_config(true, true, RW_WRITE, (chunk.len() as u32) - 1),
            );
            wait_cmd_done();
            wait_ready_best_effort();
            off += chunk.len() as u32;
        }
        Ok(())
    }
}

impl Drop for Ws63Algo {
    fn drop(&mut self) {
        // Restore the SFC controller and flash chip to a clean state for
        // flashboot's `uapi_sfc_init()` on the next boot.
        //
        // If the flash chip is left in an intermediate state (e.g. still completing
        // the last program, or with a half-finished WRSR transaction), the next
        // boot's `hal_sfc_get_flash_id()` returns an unrecognized JEDEC ID and
        // flashboot reports `Flash Init Fail! ret = 0x80001341`.
        wait_cmd_done(); // ensure last SFC transaction completed
        wr(CMD_CONFIG, 0); // force-clear start / sel_cs / all fields

        // Issue a SPI-NOR software reset (RSTEN + RST). This clears the WREN
        // latch, any busy/half-command state, and the flash's internal command
        // state machine. RSTEN must be immediately followed by RST.
        flash_software_reset();

        // Restore all three flash status registers to the values flashboot expects.
        // GD25Q32 has SR1 (BP bits), SR2 (QE/SUS), SR3 — flashboot checks all three.
        // BP bits in SR1 are non-volatile, so a software reset alone cannot restore
        // them; we must use WRSR. Write SR1+SR2 together (some chips ignore the
        // single-byte WRSR form), then SR3 separately.
        let sr1 = if self.saved_sr1 != 0 {
            self.saved_sr1
        } else {
            EXPECTED_SR1
        };
        let sr2 = if self.saved_sr2 != 0 {
            self.saved_sr2
        } else {
            EXPECTED_SR2
        };
        let sr3 = if self.saved_sr3 != 0 {
            self.saved_sr3
        } else {
            EXPECTED_SR3
        };
        write_status_registers_sr1_sr2(sr1, sr2);
        write_status_register_op(OP_WRSR3, sr3);

        // One more software reset to leave the flash in a clean, idle state; then
        // idle the SFC command path.
        flash_software_reset();
        wait_cmd_done();
        wr(CMD_CONFIG, 0);
        let _ = wait_ready();
    }
}
