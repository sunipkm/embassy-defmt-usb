# embassy-defmt-usb

Generic [defmt] USB CDC-ACM drain transport for [embassy], compatible with any
`embassy-usb` driver implementation (RP2040/RP2350, STM32, nRF52, ESP32, etc.).

## Features

| Feature | Default | Description |
|---|---|---|
| `global-logger` | off | Registers a `#[defmt::global_logger]` that writes every frame to both SEGGER RTT and the USB double-buffer. Suitable for Cortex-M targets debugged via probe-rs / OpenOCD. When disabled, bring your own `#[defmt::global_logger]` and call [`write()`] / [`swap()`] from within it. |

Without `global-logger` you bring your own `#[defmt::global_logger]` and call
the provided hooks from within it (see [Manual logger](#manual-logger) below).

## Quick start — `global-logger` feature (RTT + USB)

```toml
# Cargo.toml
[dependencies]
embassy-defmt-usb = { version = "0.0.1", features = ["global-logger"] }
```

The `#[defmt::global_logger]` is registered automatically. All you need is
the USB wiring in your main task:

```rust
// 1. Register the CDC-ACM interface before builder.build():
let defmt_task = embassy_defmt_usb::UsbDefmtLogger::new()
    .with_timeout(Duration::from_millis(10))
    .build(&mut usb_builder);          // generic over any embassy-usb Driver

// 2. After builder.build(), spawn the concrete drain task:
spawner.spawn(defmt_drain(defmt_task)).unwrap();
```

Because `#[embassy_executor::task]` does not support generic functions, you must
define one thin wrapper in your binary:

```rust
type DefmtTask = embassy_defmt_usb::UsbDefmtTask<YourDriver>;

#[embassy_executor::task]
async fn defmt_drain(task: DefmtTask) {
    task.run().await;
}
```

## Manual logger

When you want a custom logger (e.g. USB-only without RTT, or with a UART
transport), omit the feature and hook the USB buffer from your own
`#[defmt::global_logger]`:

```toml
embassy-defmt-usb = { version = "0.0.1" }   # no global-logger feature
```

```rust
// In your global_logger module:
fn sink(bytes: &[u8]) {
    // … other transports (UART, custom RTT, …) …
    // SAFETY: called within the defmt critical section.
    unsafe { embassy_defmt_usb::write(bytes); }
}

unsafe fn flush() {
    // SAFETY: inside the defmt critical section.
    unsafe { embassy_defmt_usb::swap(); }
}
```

## Architecture

```
defmt frame
    └─ #[defmt::global_logger]  (in your binary, or from global-logger feature)
            ├─ SEGGER RTT  →  probe-rs / OpenOCD  (Cortex-M, optional)
            └─ embassy_defmt_usb::write()
                    └─ USB double-buffer
                            └─ CDC-ACM bulk IN  →  host
```

The CDC-ACM communication interface advertises `iInterface = "defmt"`, making the
port easily identifiable to host tools such as `defmt-print` and `probe-rs`.

## CDC-ACM descriptor layout

| Interface | Class | Subclass | Protocol | iInterface |
|---|---|---|---|---|
| Comm (notifications) | `0x02` | `0x02` | `0x01` | `"defmt"` |
| Data (bulk IN/OUT) | `0x0A` | `0x00` | `0x00` | — |

## Public API hooks

| Function | Safety | Description |
|---|---|---|
| `write(bytes: &[u8])` | Must be in a `defmt` critical section | Write COBS-encoded bytes to the USB double-buffer |
| `swap()` | Must be in a `defmt` critical section | Swap active/flush buffers at end-of-frame |

## Compatibility

Tested with `embassy-usb 0.6` and `embassy-executor 0.9` on:
- `embassy-rp` (RP2040 / RP2350)

This crate should work with any `D: embassy_usb::driver::Driver<'static>` implementation.

[defmt]: https://defmt.ferrous-systems.com
[embassy]: https://embassy.dev
