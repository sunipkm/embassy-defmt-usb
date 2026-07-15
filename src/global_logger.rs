//! Combined SEGGER RTT + USB defmt global logger.
//!
//! Enabled with the `global-logger` Cargo feature.  Writes every defmt frame
//! to both the SEGGER RTT up-channel **and** the USB double-buffer (drained by
//! the `UsbDefmtTask`).
//!
//! # Targets
//! Designed for Cortex-M boards debugged via probe-rs or OpenOCD.  The
//! `_SEGGER_RTT` control block and its `link_section` attributes are
//! Cortex-M / SEGGER-specific.

#![allow(clippy::missing_safety_doc)]

use core::{
    cell::UnsafeCell,
    ptr,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

// ─────────────────────────────────────────────────────────────────────────────
// RTT infrastructure  (ported verbatim from defmt-rtt 1.1.0)
// ─────────────────────────────────────────────────────────────────────────────

const RTT_BUF_SIZE: usize = 1024;

const MODE_MASK: usize = 0b11;
const MODE_BLOCK_IF_FULL: usize = 2;
const MODE_NON_BLOCKING_TRIM: usize = 1;

#[repr(C)]
struct RttHeader {
    id: [u8; 16],
    max_up_channels: usize,
    max_down_channels: usize,
    up_channel: RttChannel,
}
// SAFETY: only written within a critical section held by the defmt global logger.
unsafe impl Sync for RttHeader {}

#[repr(C)]
struct RttChannel {
    name: *const u8,
    buffer: *mut u8,
    size: usize,
    write: AtomicUsize,
    read: AtomicUsize,
    flags: AtomicUsize,
}
// SAFETY: access is serialised by the global-logger critical section.
unsafe impl Sync for RttChannel {}

impl RttChannel {
    fn write_all(&self, mut bytes: &[u8]) {
        let write_fn: fn(&Self, &[u8]) -> usize = if self.host_is_connected() {
            Self::blocking_write
        } else {
            Self::nonblocking_write
        };
        while !bytes.is_empty() {
            let consumed = write_fn(self, bytes);
            if consumed != 0 {
                bytes = &bytes[consumed..];
            }
        }
    }

    fn blocking_write(&self, bytes: &[u8]) -> usize {
        if bytes.is_empty() { return 0; }
        let read  = self.read.load(Ordering::Relaxed);
        let write = self.write.load(Ordering::Acquire);
        let avail = rtt_available(read, write, RTT_BUF_SIZE);
        if avail == 0 { return 0; }
        self.write_impl(bytes, write, avail)
    }

    fn nonblocking_write(&self, bytes: &[u8]) -> usize {
        let write = self.write.load(Ordering::Acquire);
        self.write_impl(bytes, write, RTT_BUF_SIZE)
    }

    fn write_impl(&self, bytes: &[u8], cursor: usize, available: usize) -> usize {
        let len = bytes.len().min(available);
        // SAFETY: `self.buffer` points to a static array of `RTT_BUF_SIZE` bytes.
        unsafe {
            if cursor + len > RTT_BUF_SIZE {
                let pivot = RTT_BUF_SIZE - cursor;
                ptr::copy_nonoverlapping(bytes.as_ptr(), self.buffer.add(cursor), pivot);
                ptr::copy_nonoverlapping(bytes.as_ptr().add(pivot), self.buffer, len - pivot);
            } else {
                ptr::copy_nonoverlapping(bytes.as_ptr(), self.buffer.add(cursor), len);
            }
        }
        self.write.store(cursor.wrapping_add(len) % RTT_BUF_SIZE, Ordering::Release);
        len
    }

    fn flush(&self) {
        if !self.host_is_connected() { return; }
        while self.read.load(Ordering::Relaxed) != self.write.load(Ordering::Relaxed) {
            core::hint::spin_loop();
        }
    }

    fn host_is_connected(&self) -> bool {
        self.flags.load(Ordering::Relaxed) & MODE_MASK == MODE_BLOCK_IF_FULL
    }
}

fn rtt_available(read: usize, write: usize, size: usize) -> usize {
    if read > write        { read - write - 1 }
    else if read == 0      { size - write - 1 }
    else                   { size - write      }
}

struct RttBuffer { inner: UnsafeCell<[u8; RTT_BUF_SIZE]> }
impl RttBuffer {
    const fn new() -> Self { Self { inner: UnsafeCell::new([0; RTT_BUF_SIZE]) } }
    const fn get(&self) -> *mut u8 { self.inner.get() as *mut u8 }
}
// SAFETY: only accessed through the serialised global logger.
unsafe impl Sync for RttBuffer {}

#[unsafe(link_section = ".uninit.defmt-rtt.BUFFER")]
static RTT_BUFFER: RttBuffer = RttBuffer::new();

#[unsafe(link_section = ".data.defmt-rtt.NAME")]
static RTT_NAME: [u8; 6] = *b"defmt\0";

/// SEGGER RTT control block — must be named `_SEGGER_RTT` and exported
/// without mangling so probe-rs / OpenOCD can locate it in RAM.
#[unsafe(no_mangle)]
static _SEGGER_RTT: RttHeader = RttHeader {
    id: *b"SEGGER RTT\0\0\0\0\0\0",
    max_up_channels: 1,
    max_down_channels: 0,
    up_channel: RttChannel {
        name: RTT_NAME.as_ptr(),
        buffer: RTT_BUFFER.get(),
        size: RTT_BUF_SIZE,
        write: AtomicUsize::new(0),
        read: AtomicUsize::new(0),
        flags: AtomicUsize::new(MODE_NON_BLOCKING_TRIM),
    },
};

// ─────────────────────────────────────────────────────────────────────────────
// Combined defmt global logger
// ─────────────────────────────────────────────────────────────────────────────

struct MultiEncoder {
    /// Re-entrancy guard / exclusive-access flag.
    taken: AtomicBool,
    /// Saved interrupt-enable state for the critical section.
    restore: UnsafeCell<critical_section::RestoreState>,
    /// The defmt COBS encoder that frames outgoing bytes.
    encoder: UnsafeCell<defmt::Encoder>,
}

// SAFETY: access is serialised by the `taken` flag and the critical section.
unsafe impl Sync for MultiEncoder {}

impl MultiEncoder {
    const fn new() -> Self {
        Self {
            taken: AtomicBool::new(false),
            restore: UnsafeCell::new(critical_section::RestoreState::invalid()),
            encoder: UnsafeCell::new(defmt::Encoder::new()),
        }
    }

    fn acquire(&self) {
        // Enter a critical section so only one caller can proceed.
        // SAFETY: paired with the `release` call below.
        let restore = unsafe { critical_section::acquire() };

        if self.taken.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly");
        }
        self.taken.store(true, Ordering::Relaxed);

        // SAFETY: we are in a critical section.
        unsafe {
            self.restore.get().write(restore);
            (*self.encoder.get()).start_frame(Self::sink);
        }
    }

    unsafe fn write(&self, bytes: &[u8]) {
        // SAFETY: inside a critical section held since `acquire`.
        unsafe { (*self.encoder.get()).write(bytes, Self::sink); }
    }

    unsafe fn flush(&self) {
        // SAFETY: inside a critical section.
        unsafe { super::CONTROLLER.swap(); }
        _SEGGER_RTT.up_channel.flush();
    }

    unsafe fn release(&self) {
        if !self.taken.load(Ordering::Relaxed) {
            panic!("defmt release out of context");
        }
        // SAFETY: inside a critical section.
        unsafe {
            (*self.encoder.get()).end_frame(Self::sink);
            let restore = self.restore.get().read();
            self.taken.store(false, Ordering::Relaxed);
            critical_section::release(restore);
        }
    }

    /// Byte sink: writes encoded defmt bytes to **both** RTT and the USB
    /// double-buffer.
    ///
    /// Called by `defmt::Encoder` inside `start_frame`, `write`, and
    /// `end_frame` — all within the critical section held by `acquire`/`release`.
    fn sink(bytes: &[u8]) {
        _SEGGER_RTT.up_channel.write_all(bytes);
        // SAFETY: called within the critical section held since `acquire`.
        unsafe { super::CONTROLLER.write(bytes); }
    }
}

static LOGGER: MultiEncoder = MultiEncoder::new();

#[defmt::global_logger]
struct MultiLogger;

unsafe impl defmt::Logger for MultiLogger {
    fn acquire() { LOGGER.acquire(); }

    unsafe fn write(bytes: &[u8]) {
        // SAFETY: contract delegated to `MultiEncoder::write`.
        unsafe { LOGGER.write(bytes) }
    }

    unsafe fn flush() {
        // SAFETY: contract delegated to `MultiEncoder::flush`.
        unsafe { LOGGER.flush() }
    }

    unsafe fn release() {
        // SAFETY: contract delegated to `MultiEncoder::release`.
        unsafe { LOGGER.release() }
    }
}
