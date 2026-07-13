//! probe-rs flash loader for the HiSilicon WS63 (Hi3863) on-chip SFC NOR flash.
//!
//! This is the loader blob that probe-rs uploads to target RAM and calls to erase
//! and program flash. It drives the WS63 Serial Flash Controller (SFC v150) in
//! register/command mode for status/erase commands and bus-DMA mode for writes.
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

// Bus-mode configuration and its DMA engine. These offsets and the programming
// sequence match the vendor `hal_sfc_dma_write()` implementation.
const BUS_CONFIG1: u32 = SFC_BASE + 0x200;
const BUS_DMA_CTRL: u32 = SFC_BASE + 0x240;
const BUS_DMA_MEM_SADDR: u32 = SFC_BASE + 0x244;
const BUS_DMA_FLASH_SADDR: u32 = SFC_BASE + 0x248;
const BUS_DMA_LEN: u32 = SFC_BASE + 0x24C;
const BUS_DMA_AHB_CTRL: u32 = SFC_BASE + 0x250;

/// Flash chip offset 0 is mapped to CPU address 0x200000 (SFC bus_base_addr_cs0).
const XIP_BASE: u32 = 0x0020_0000;

// The application bulk area is 64-KiB aligned at both ends. The final 64 KiB
// before the 0x5fc000 NV partition deliberately remains 4-KiB sectors so an
// application update cannot erase NV. These boundaries match the vendor WS63
// partition layout and `do_greedy_erase()` policy.
const APP_BULK_START: u32 = 0x0023_0000;
const APP_BULK_END: u32 = 0x005F_0000;

// Standard SPI-NOR opcodes (flash_common_config.h).
const OP_WREN: u32 = 0x06;
const OP_WRVSR: u32 = 0x50; // volatile status-register write enable
const OP_RDSR: u32 = 0x05;
const OP_SE_4K: u32 = 0x20; // 4 KiB sector erase
const OP_BE_64K: u32 = 0xD8; // 64 KiB block erase
const OP_PP: u32 = 0x02; // page program
const OP_WRSR: u32 = 0x01; // write status register 1

// SPI-NOR software reset sequence: RSTEN (0x66) followed by RST (0x99).
const OP_RSTEN: u32 = 0x66; // reset enable
const OP_RST: u32 = 0x99; // reset memory

const RW_WRITE: u32 = 0;
const RW_READ: u32 = 1;
const IF_STD: u32 = 0; // standard (1-1-1) SPI

/// Vendor-compatible WIP polling: at most 50,000 samples separated by roughly
/// 100 us. The interval prevents a large block erase from exhausting a tight
/// CPU-speed loop before the NOR can finish.
const WIP_POLL_LIMIT: u32 = 50_000;
const WIP_POLL_INTERVAL_CYCLES: u32 = 16_000;
const CMD_POLL_LIMIT: u32 = 50_000;

/// A 64 KiB bus-DMA write can take substantially longer than one register-mode
/// command. The host also enforces `program_time_out`; this bound only prevents
/// a wedged controller from spinning forever on target.
const DMA_POLL_LIMIT: u32 = 50_000_000;

const BUS_CONFIG1_WRITE_MASK: u32 = (0x7 << 16) | (0x7 << 19) | (0xFF << 22) | (1 << 30);
const BUS_CONFIG1_STANDARD_PP: u32 = (OP_PP << 22) | (1 << 30);
const BUS_DMA_CTRL_START_WRITE_CS1: u32 = 1 | (1 << 4);

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

#[inline(always)]
fn cycle_count() -> u32 {
    let cycles: u32;
    // SAFETY: reading mcycle has no side effects and is available in M-mode.
    unsafe {
        core::arch::asm!("csrr {cycles}, mcycle", cycles = out(reg) cycles, options(nomem, nostack));
    }
    cycles
}

/// Delay between RDSR samples without depending on a vendor timer service.
///
/// WS63 runs at no more than 160 MHz in the supported boot paths, so 16,000
/// cycles is at least 100 us. At a lower early-boot clock the delay is longer,
/// which is conservative for flash completion and remains bounded.
#[inline(always)]
fn wait_poll_interval() {
    let start = cycle_count();
    while cycle_count().wrapping_sub(start) < WIP_POLL_INTERVAL_CYCLES {
        core::hint::spin_loop();
    }
}

/// Make subsequent CPU/XIP reads observe the newly programmed flash.
///
/// probe-rs verifies through the CPU memory map. The WS63 caches can otherwise
/// retain the pre-erase body and cause a false verification failure. Clean the
/// D-cache before invalidating it because this loader's stack and state also
/// live in SRAM, then invalidate the I-cache for the next boot path.
#[inline(always)]
fn flush_cpu_caches() {
    const ICACHE_INVALIDATE_ALL: u32 = 1 << 2;
    const DCACHE_CLEAN_INVALIDATE_ALL: u32 = (1 << 3) | (1 << 2);
    // SAFETY: these are the WS63 cache-maintenance CSRs documented by the
    // vendor LiteOS port. The loader executes in M-mode from SRAM.
    unsafe {
        core::arch::asm!(
            "csrw 0x7c3, {dcache}",
            "fence rw, rw",
            "csrw 0x7c2, {icache}",
            "fence.i",
            dcache = in(reg) DCACHE_CLEAN_INVALIDATE_ALL,
            icache = in(reg) ICACHE_INVALIDATE_ALL,
            options(nostack),
        );
    }
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
fn wait_cmd_done() -> Result<(), ErrorCode> {
    let mut n: u32 = 0;
    while rd(CMD_CONFIG) & 1 != 0 {
        n = n.wrapping_add(1);
        if n > CMD_POLL_LIMIT {
            break;
        }
    }
    if n > CMD_POLL_LIMIT {
        Err(ErrorCode::new(0x5763_0004).unwrap())
    } else {
        Ok(())
    }
}

/// Issue WREN (sets the flash WEL latch). Required before every erase/program.
fn write_enable() -> Result<(), ErrorCode> {
    wr(CMD_INS, OP_WREN);
    wr(CMD_CONFIG, cmd_config(false, false, RW_WRITE, 0));
    wait_cmd_done()
}

/// Enable a volatile status-register write.
///
/// The vendor GD25Q32 path uses 0x50 before WRSR when temporarily changing the
/// protection bits. Unlike WREN (0x06), this does not consume a non-volatile
/// status-register program cycle on every debug download.
fn write_enable_volatile_status() -> Result<(), ErrorCode> {
    wr(CMD_INS, OP_WRVSR);
    wr(CMD_CONFIG, cmd_config(false, false, RW_WRITE, 0));
    wait_cmd_done()
}

/// Poll RDSR until the WIP bit (status bit 0) clears.
fn wait_ready() -> Result<(), ErrorCode> {
    for _ in 0..WIP_POLL_LIMIT {
        wr(CMD_INS, OP_RDSR);
        wr(CMD_CONFIG, cmd_config(false, true, RW_READ, 0)); // read 1 status byte
        wait_cmd_done()?;
        if rd(CMD_DATABUF) & 0x1 == 0 {
            return Ok(());
        }
        wait_poll_interval();
    }
    Err(ErrorCode::new(0x57630001).unwrap()) // WIP wait timeout
}

/// Program one host batch through the SFC bus-DMA engine.
///
/// Unlike command mode's 64-byte data window, the bus-DMA engine accepts the
/// whole probe-rs page from SRAM and performs the page-program sequencing in
/// hardware. This is the same path used by the vendor SDK's
/// `hal_sfc_dma_write()`.
fn dma_write(flash_offset: u32, data: &[u8]) -> Result<(), ErrorCode> {
    if data.is_empty() {
        return Ok(());
    }

    write_enable()?;
    wr(BUS_DMA_FLASH_SADDR, flash_offset);
    wr(BUS_DMA_MEM_SADDR, data.as_ptr() as u32);
    wr(BUS_DMA_LEN, (data.len() as u32) - 1);
    wr(BUS_DMA_AHB_CTRL, 0x7); // enable INCR4/INCR8/INCR16 AHB bursts
    wr(BUS_DMA_CTRL, BUS_DMA_CTRL_START_WRITE_CS1);

    wait_dma_done()
}

fn wait_dma_done() -> Result<(), ErrorCode> {
    for _ in 0..DMA_POLL_LIMIT {
        if rd(BUS_DMA_CTRL) & 1 == 0 {
            return Ok(());
        }
    }
    Err(ErrorCode::new(0x57630002).unwrap())
}

/// Issue a SPI-NOR software reset (RSTEN + RST).
/// This returns the flash chip to its power-on state, clearing any lingering
/// command/state left by the erase/program sequence.
fn flash_software_reset() -> Result<(), ErrorCode> {
    // RSTEN must be immediately followed by RST; do not insert WREN or other
    // commands between them.
    wr(CMD_INS, OP_RSTEN);
    wr(CMD_CONFIG, cmd_config(false, false, RW_WRITE, 0));
    wait_cmd_done()?;
    wr(CMD_INS, OP_RST);
    wr(CMD_CONFIG, cmd_config(false, false, RW_WRITE, 0));
    wait_cmd_done()?;
    wait_ready()
}

/// Read a flash status register via the given RDSR opcode. Returns 1 byte.
fn read_status_register_op(rdsr_op: u32) -> Result<u8, ErrorCode> {
    wr(CMD_INS, rdsr_op);
    wr(CMD_CONFIG, cmd_config(false, true, RW_READ, 0)); // read 1 status byte
    wait_cmd_done()?;
    Ok((rd(CMD_DATABUF) & 0xFF) as u8)
}

/// Write one status register through the volatile-write path used by the SDK.
fn write_status_register_op_volatile(wrsr_op: u32, val: u8) -> Result<(), ErrorCode> {
    write_enable_volatile_status()?;
    wr(CMD_DATABUF, val as u32);
    wr(CMD_INS, wrsr_op);
    wr(CMD_CONFIG, cmd_config(false, true, RW_WRITE, 0));
    wait_cmd_done()?;
    wait_ready()
}

struct Ws63Algo {
    saved_bus_config1: u32,
    saved_bus_dma_ahb_ctrl: u32,
    mutating: bool,
}

algorithm!(Ws63Algo, {
    device_name: "ws63",
    device_type: DeviceType::Onchip,
    flash_address: 0x200000,
    flash_size: 0x800000,
    // Host-side transfer batch. The algorithm still emits independent <=64-byte
    // SFC page-program commands, so a 64 KiB batch never crosses a hardware page
    // within one command. probe-rs places the buffers in the 576 KiB SRAM region;
    // this reduces debug run/halt round trips from about 1,100 to five for a
    // typical 282 KiB RF image.
    page_size: 0x10000,
    empty_value: 0xFF,
    program_time_out: 30000,
    erase_time_out: 30000,
    sectors: [
        {
            size: 0x1000,
            address: 0x0,
        },
        {
            size: 0x10000,
            address: APP_BULK_START - XIP_BASE,
        },
        {
            size: 0x1000,
            address: APP_BULK_END - XIP_BASE,
        }
    ]
});

impl FlashAlgorithm for Ws63Algo {
    fn new(_address: u32, _clock: u32, function: Function) -> Result<Self, ErrorCode> {
        // The SFC is left as configured by the boot ROM / flashboot (XIP bus mode);
        // the register/command path used below operates alongside it.
        //
        // Clear block-protect bits only for mutating operations. The volatile
        // status-write path is reset back to the non-volatile values in Drop.
        let mutating = matches!(function, Function::Erase | Function::Program);
        let saved_bus_config1 = rd(BUS_CONFIG1);
        let saved_bus_dma_ahb_ctrl = rd(BUS_DMA_AHB_CTRL);

        if mutating {
            // Bus-DMA uses the write operation configured in bus_config1. Do not
            // rely on the previously-running firmware to have left this as 0x02.
            wr(
                BUS_CONFIG1,
                (saved_bus_config1 & !BUS_CONFIG1_WRITE_MASK) | BUS_CONFIG1_STANDARD_PP,
            );
            // Clear block-protect in SR1 through the vendor's volatile
            // status-write path. Verification is read-only and skips this.
            write_status_register_op_volatile(OP_WRSR, 0x00)?;
            if read_status_register_op(OP_RDSR)? & 0x7C != 0 {
                return Err(ErrorCode::new(0x5763_0003).unwrap());
            }
        }
        Ok(Self {
            saved_bus_config1,
            saved_bus_dma_ahb_ctrl,
            mutating,
        })
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), ErrorCode> {
        let off = address.wrapping_sub(XIP_BASE); // CPU XIP addr -> flash offset
        let opcode = if (APP_BULK_START..APP_BULK_END).contains(&address) {
            OP_BE_64K
        } else {
            OP_SE_4K
        };
        write_enable()?;
        wr(CMD_INS, opcode);
        wr(CMD_ADDR, off);
        wr(CMD_CONFIG, cmd_config(true, false, RW_WRITE, 0));
        wait_cmd_done()?;
        wait_ready()
    }

    fn program_page(&mut self, address: u32, data: &[u8]) -> Result<(), ErrorCode> {
        dma_write(address.wrapping_sub(XIP_BASE), data)
    }

    fn verify(&mut self, address: u32, size: u32, data: Option<&[u8]>) -> Result<(), u32> {
        let Some(expected) = data else {
            return Err(address);
        };
        if expected.len() != size as usize {
            return Err(address);
        }

        // The preceding program operation invalidates caches in UnInit. Repeat
        // the maintenance here so Verify is also correct when invoked on its
        // own after arbitrary firmware execution.
        flush_cpu_caches();
        for (index, expected_byte) in expected.iter().copied().enumerate() {
            let actual = unsafe { read_volatile((address as *const u8).add(index)) };
            if actual != expected_byte {
                return Err(address + index as u32);
            }
        }
        Ok(())
    }
}

impl Drop for Ws63Algo {
    fn drop(&mut self) {
        if !self.mutating {
            flush_cpu_caches();
            return;
        }

        // Restore the SFC controller and flash chip to a clean state for
        // flashboot's `uapi_sfc_init()` on the next boot.
        //
        // If the flash chip is left in an intermediate state (e.g. still completing
        // the last program, or with a half-finished WRSR transaction), the next
        // boot's `hal_sfc_get_flash_id()` returns an unrecognized JEDEC ID and
        // flashboot reports `Flash Init Fail! ret = 0x80001341`.
        let _ = wait_cmd_done(); // ensure last SFC transaction completed
        let _ = wait_dma_done();
        wr(BUS_DMA_CTRL, 0);
        wr(BUS_DMA_AHB_CTRL, self.saved_bus_dma_ahb_ctrl);
        wr(BUS_CONFIG1, self.saved_bus_config1);
        wr(CMD_CONFIG, 0); // force-clear start / sel_cs / all fields

        // Issue a SPI-NOR software reset (RSTEN + RST). This clears the WREN
        // latch, any busy/half-command state, and the flash's internal command
        // state machine. RSTEN must be immediately followed by RST.
        let _ = flash_software_reset();

        // WRVSR made the protection change volatile, so reset restores the
        // original non-volatile status registers without another WRSR cycle.
        let _ = wait_cmd_done();
        wr(CMD_CONFIG, 0);
        let _ = wait_ready();
        flush_cpu_caches();
    }
}
