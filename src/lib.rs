//! Generic defmt USB CDC-ACM drain transport for embassy.
//!
//! This crate provides:
//! - A lock-free double-buffer that your `defmt::global_logger` writes into
//!   (via [`write()`] / [`swap`]).
//! - [`UsbDefmtLogger`] — a builder that registers a properly-described
//!   CDC-ACM interface (with `iInterface = "defmt"`) on any `embassy-usb`
//!   [`Builder`].
//! - [`UsbDefmtTask<D>`] — holds the allocated bulk-IN endpoint and exposes a
//!   [`run`](UsbDefmtTask::run) future that drains the buffer over USB.
//!
//! Because `#[embassy_executor::task]` does not support generic functions, the
//! drain loop is exposed as an ordinary `async fn`.  In your binary, wrap it
//! in a single concrete task:
//!
//! ```no_run
//! #[embassy_executor::task]
//! async fn defmt_drain(task: embassy_defmt_usb::UsbDefmtTask<YourDriver>) {
//!     task.run().await;
//! }
//! ```
//!
//! # Usage
//!
//! In your global-logger `sink` / `flush` callbacks:
//!
//! ```no_run
//! fn sink(bytes: &[u8]) {
//!     // ... write to RTT / other sinks ...
//!     // SAFETY: called inside the defmt critical section.
//!     unsafe { embassy_defmt_usb::write(bytes); }
//! }
//!
//! unsafe fn flush() {
//!     // SAFETY: inside the defmt critical section.
//!     unsafe { embassy_defmt_usb::swap(); }
//! }
//! ```
//!
//! # Feature flags
//!
//! | Feature | Default | Description |
//! |---------|---------|-------------|
//! | `global-logger` | off | Registers a `#[defmt::global_logger]` that writes every frame to both SEGGER RTT and the USB double-buffer.  Suitable for Cortex-M targets debugged via probe-rs / OpenOCD.  When disabled, bring your own `#[defmt::global_logger]` and call [`write()`] / [`swap()`] from within it. |

#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicUsize, Ordering},
};

use embassy_time::{Duration, Timer};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn};
use embassy_usb::{
    Builder, Handler,
    types::{InterfaceNumber, StringIndex},
};
use portable_atomic::AtomicBool;
use static_cell::StaticCell;

// ─────────────────────────────────────────────────────────────────────────────
// USB double-buffer
// ─────────────────────────────────────────────────────────────────────────────

const USB_BUF_SIZE: usize = 32;

#[derive(Clone, Copy, PartialEq, Eq)]
enum BufState {
    Active,
    Flush,
}

struct LogBuffer {
    state: BufState,
    cursor: usize,
    data: [u8; USB_BUF_SIZE],
}

impl LogBuffer {
    const fn new() -> Self {
        Self {
            state: BufState::Active,
            cursor: 0,
            data: [0u8; USB_BUF_SIZE],
        }
    }

    fn set_flushing(&mut self) {
        self.state = BufState::Flush;
    }

    fn reset(&mut self) {
        self.state = BufState::Active;
        self.cursor = 0;
    }

    fn write(&mut self, bytes: &[u8]) {
        let c = self.cursor;
        self.data[c..c + bytes.len()].copy_from_slice(bytes);
        self.cursor += bytes.len();
    }

    fn accepts(&self, n: usize) -> bool {
        (self.cursor + n) < USB_BUF_SIZE && self.state == BufState::Active
    }

    fn is_flushing(&self) -> bool {
        self.state == BufState::Flush
    }
}

struct Controller {
    current_idx: AtomicUsize,
    enabled: AtomicBool,
    /// SAFETY: writes only inside a defmt critical section; reads only after
    /// the buffer transitions to `Flush` (disjoint from writes).
    buffers: [UnsafeCell<LogBuffer>; 2],
}

// SAFETY: see field comments above.
unsafe impl Sync for Controller {}

impl Controller {
    const fn new() -> Self {
        Self {
            current_idx: AtomicUsize::new(0),
            enabled: AtomicBool::new(true),
            buffers: [
                UnsafeCell::new(LogBuffer::new()),
                UnsafeCell::new(LogBuffer::new()),
            ],
        }
    }

    fn enable(&self) {
        self.enabled.store(true, Ordering::Relaxed);
    }

    fn disable(&self) {
        self.enabled.store(false, Ordering::Relaxed);
        critical_section::with(|_| unsafe {
            // SAFETY: inside a critical section.
            (*self.buffers[0].get()).reset();
            (*self.buffers[1].get()).reset();
        });
    }

    /// # Safety: must be called from within a critical section.
    pub(crate) unsafe fn swap(&self) {
        if !self.enabled.load(Ordering::Relaxed) {
            return;
        }
        let idx = self.current_idx.load(Ordering::Relaxed);
        // SAFETY: inside a critical section, no concurrent mutation.
        unsafe { (*self.buffers[idx].get()).set_flushing() };
        self.current_idx.store(idx ^ 1, Ordering::Relaxed);
    }

    /// # Safety: must be called from within a critical section.
    pub(crate) unsafe fn write(&self, bytes: &[u8]) {
        if !self.enabled.load(Ordering::Relaxed) {
            return;
        }
        let idx = self.current_idx.load(Ordering::Relaxed);
        let other = idx ^ 1;
        // SAFETY: inside critical section.
        let cur = unsafe { &mut *self.buffers[idx].get() };
        let oth = unsafe { &mut *self.buffers[other].get() };

        if cur.accepts(bytes.len()) {
            cur.write(bytes);
        } else {
            unsafe { self.swap() };
            if oth.accepts(bytes.len()) {
                oth.write(bytes);
            }
        }
    }

    fn get_flushing(&self) -> Option<(usize, &LogBuffer)> {
        for (i, cell) in self.buffers.iter().enumerate() {
            // SAFETY: the drain loop is the only reader, and only reads
            // buffers in Flush state — disjoint from concurrent writes.
            let buf = unsafe { &*cell.get() };
            if buf.is_flushing() {
                return Some((i, buf));
            }
        }
        None
    }

    fn reset_buffer(&self, idx: usize) {
        critical_section::with(|_| unsafe {
            // SAFETY: inside a critical section.
            (*self.buffers[idx].get()).reset();
        });
    }
}

pub(crate) static CONTROLLER: Controller = Controller::new();

// ─────────────────────────────────────────────────────────────────────────────
// Global logger hooks
// ─────────────────────────────────────────────────────────────────────────────

/// Write encoded defmt bytes to the USB double-buffer.
///
/// Call this from your `#[defmt::global_logger]` sink function inside the
/// critical section held between `acquire` and `release`.
///
/// # Safety
/// Must be called inside a defmt critical section.
pub unsafe fn write(bytes: &[u8]) {
    // SAFETY: contract delegated to caller.
    unsafe { CONTROLLER.write(bytes) }
}

/// Swap the active and flush buffers at end-of-frame.
///
/// Call this from your `#[defmt::global_logger]` flush function.
///
/// # Safety
/// Must be called inside a defmt critical section.
pub unsafe fn swap() {
    // SAFETY: contract delegated to caller.
    unsafe { CONTROLLER.swap() }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private CDC string handler
// ─────────────────────────────────────────────────────────────────────────────

struct DefmtStringHandler {
    defmt_str: StringIndex,
    /// Comm interface number — used to filter CDC class requests.
    comm_if: InterfaceNumber,
    /// Default line coding returned to GET_LINE_CODING (9600 8N1).
    line_coding: [u8; 7],
}

impl Handler for DefmtStringHandler {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        (index == self.defmt_str).then_some("defmt")
    }

    /// Accept SET_LINE_CODING and SET_CONTROL_LINE_STATE so Windows does not
    /// receive a STALL when opening the defmt COM port (error 31).
    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if req.request_type != RequestType::Class
            || req.recipient != Recipient::Interface
            || req.index != self.comm_if.0 as u16
        {
            return None;
        }
        match req.request {
            0x20 | 0x22 => Some(OutResponse::Accepted), // SET_LINE_CODING | SET_CONTROL_LINE_STATE
            _ => None,
        }
    }

    /// Return default line coding for GET_LINE_CODING.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Class
            || req.recipient != Recipient::Interface
            || req.index != self.comm_if.0 as u16
        {
            return None;
        }
        if req.request == 0x21 {
            // GET_LINE_CODING
            let n = buf.len().min(self.line_coding.len());
            buf[..n].copy_from_slice(&self.line_coding[..n]);
            Some(InResponse::Accepted(&buf[..n]))
        } else {
            None
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Builder
// ─────────────────────────────────────────────────────────────────────────────

/// Registers the defmt CDC-ACM interface on a USB builder.
///
/// Call [`build`](Self::build) to install the interface (with the `"defmt"`
/// iInterface string) before [`Builder::build`], then call
/// [`UsbDefmtTask::run`] from a concrete embassy task.
///
/// # Example
///
/// ```no_run
/// // Before builder.build():
/// let task = UsbDefmtLogger::new()
///     .with_timeout(Duration::from_millis(5))
///     .build(&mut builder);
///
/// // After builder.build(), inside a concrete #[embassy_executor::task]:
/// task.run().await;
/// ```
pub struct UsbDefmtLogger {
    timeout: Duration,
}

impl Default for UsbDefmtLogger {
    fn default() -> Self {
        Self::new()
    }
}

impl UsbDefmtLogger {
    /// Create with the default drain poll interval (10 ms).
    pub const fn new() -> Self {
        Self {
            timeout: Duration::from_millis(10),
        }
    }

    /// Override the idle poll interval between drain attempts.
    pub const fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Register the defmt CDC-ACM interface on `builder`.
    ///
    /// Allocates a CDC function with:
    /// - Comm interface (class `0x02/0x02/0x01`) with `iInterface = "defmt"`
    /// - Data interface (class `0x0A/0x00/0x00`) with bulk IN/OUT endpoints
    ///
    /// Must be called before `builder.build()`.
    pub fn build<D>(self, builder: &mut Builder<'static, D>) -> UsbDefmtTask<D>
    where
        D: Driver<'static>,
    {
        static DEFMT_STRING_HANDLER: StaticCell<DefmtStringHandler> = StaticCell::new();

        let (ep_in, defmt_str, comm_if) = {
            let mut func = builder.function(0x02, 0x02, 0x00);

            // Comm interface: carries the "defmt" iInterface string.
            let (defmt_str, comm_if) = {
                let mut comm = func.interface();
                let str_idx = comm.string();
                let num = comm.interface_number();
                let mut alt = comm.alt_setting(0x02, 0x02, 0x01, Some(str_idx));
                // CDC Header functional descriptor (CDC spec v1.10).
                alt.descriptor(0x24, &[0x00, 0x10, 0x01]);
                // CDC ACM functional descriptor (capabilities 0x06).
                alt.descriptor(0x24, &[0x02, 0x06]);
                // CDC Union functional descriptor (comm = this, data = comm+1).
                alt.descriptor(0x24, &[0x06, num.0, num.0 + 1]);
                // Notification endpoint (required by spec; not used for defmt).
                alt.endpoint_interrupt_in(None, 8, 255);
                (str_idx, num)
            };

            // Data interface: the bulk IN endpoint is the defmt byte stream.
            let ep_in = {
                let mut data = func.interface();
                let mut alt = data.alt_setting(0x0A, 0x00, 0x00, None);
                let _ = alt.endpoint_bulk_out(None, 64);
                alt.endpoint_bulk_in(None, 64)
            };

            (ep_in, defmt_str, comm_if)
        };

        let handler = DEFMT_STRING_HANDLER.init(DefmtStringHandler {
            defmt_str,
            comm_if,
            // Default line coding: 9600 baud, 1 stop bit, no parity, 8 data bits.
            line_coding: [0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08],
        });
        builder.handler(handler);

        UsbDefmtTask {
            timeout: self.timeout,
            sender: ep_in,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Task handle
// ─────────────────────────────────────────────────────────────────────────────

/// Holds the allocated bulk-IN endpoint ready to drain.
///
/// Because `#[embassy_executor::task]` does not support generic functions,
/// wrap `run` in a concrete task in your binary:
///
/// ```no_run
/// type MyTask = embassy_defmt_usb::UsbDefmtTask<MyUsbDriver>;
///
/// #[embassy_executor::task]
/// async fn defmt_drain(task: MyTask) {
///     task.run().await;
/// }
/// ```
pub struct UsbDefmtTask<D>
where
    D: Driver<'static>,
{
    timeout: Duration,
    sender: D::EndpointIn,
}

impl<D> UsbDefmtTask<D>
where
    D: Driver<'static>,
{
    /// Drain the USB double-buffer over the bulk-IN endpoint.
    ///
    /// This future runs forever.  Spawn it from a concrete
    /// `#[embassy_executor::task]`.
    pub async fn run(self) {
        let mut sender = self.sender;
        let timeout = self.timeout;

        'main: loop {
            // Wait for the endpoint to be enabled (USB enumeration complete).
            sender.wait_enabled().await;
            CONTROLLER.enable();

            loop {
                if let Some((idx, buf)) = CONTROLLER.get_flushing() {
                    let bytes = &buf.data[..buf.cursor];
                    let max = sender.info().max_packet_size as usize;
                    let mut last_was_max = false;

                    let mut write_err: Option<EndpointError> = None;
                    for chunk in bytes.chunks(max) {
                        last_was_max = chunk.len() == max;
                        match sender.write(chunk).await {
                            Ok(()) => {}
                            Err(e) => {
                                write_err = Some(e);
                                break;
                            }
                        }
                    }

                    // Per USB spec: send a ZLP if the last transfer filled a
                    // full packet, to signal end-of-transfer to the host.
                    if write_err.is_none() && last_was_max {
                        write_err = sender.write(&[]).await.err();
                    }

                    // Always reset the buffer whether or not an error occurred.
                    CONTROLLER.reset_buffer(idx);

                    match write_err {
                        Some(EndpointError::Disabled) => {
                            CONTROLLER.disable();
                            continue 'main;
                        }
                        Some(EndpointError::BufferOverflow) => {
                            unreachable!("chunks are bounded by max_packet_size")
                        }
                        None => {}
                    }
                }

                Timer::after(timeout).await;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Optional: combined RTT + USB defmt global logger
// ─────────────────────────────────────────────────────────────────────────────

/// Registers a `#[defmt::global_logger]` that writes every frame to both
/// SEGGER RTT **and** the USB double-buffer.
///
/// Enabled with Cargo feature `global-logger`.
#[cfg(feature = "global-logger")]
#[cfg_attr(docsrs, doc(cfg(feature = "global-logger")))]
mod global_logger;
