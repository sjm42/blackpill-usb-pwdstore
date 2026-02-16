# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded Rust firmware that turns a WeAct BlackPill (STM32F411CEU6) into a USB password manager. Stores up to 2047 encrypted credentials on the onboard 8MB W25Q64 SPI flash using XChaCha20Poly1305 AEAD encryption. Exposes a USB CDC-ACM serial CLI for interaction.

## Build Commands

The `./build` script wraps all build/flash operations:

```bash
./build k b    # Build for BlackPill (default target)
./build k c    # Clippy lint for BlackPill
./build k d    # DFU flash (interactive - prompts for DFU mode)
./build k f    # cargo-flash (requires debug probe)
```

Hardware selector: `k` = BlackPill, `b` = Blue Pill, `n` = Nucleo F411

Manual build equivalent:
```bash
cargo build --release --no-default-features --features black_pill --target thumbv7em-none-eabihf
```

No test suite — testing is manual via USB serial terminal (`minicom -D /dev/PwdTrove`).

## Toolchain Requirements

- Rust **nightly** (specified in `rust-toolchain.toml`)
- Target: `thumbv7em-none-eabihf` (Cortex-M4F with FPU)
- `cargo-binutils` and `llvm-tools-preview` for objcopy (DFU flashing)
- `dfu-util` for USB DFU programming

## Architecture

### RTIC Application (`src/bin/blackpill-usb-pwdstore.rs`)
Entry point using `cortex-m-rtic` for interrupt-driven task scheduling. Initializes all hardware (clocks at 84MHz, USB OTG_FS, SPI1 for flash, USART1 at 115200, watchdog, 32KB heap). Key tasks:
- `usb_fs` — USB interrupt handler, routes serial input to `feed_runner`
- `feed_runner` — processes CLI input through the menu system
- `task_init/open/store` — background crypto operations (slow, feeds watchdog)
- `periodic` — 200ms watchdog feed
- `button` — PA0 button handler with 500ms debounce

### Password Store (`src/pwdstore.rs`)
Core storage and encryption engine. `PwdStore` manages the SPI flash and crypto context. Each entry is a 4KB `PwdRepr` block: 6 bytes of lengths + 250 bytes plaintext name + encrypted username/password. Master key (random 256-bit) is stored encrypted at flash address 0x000000; password entries occupy 0x001000–0x7FFFFF with randomized allocation.

### CLI Menu (`src/climenu.rs`)
Uses `embedded-cli` with derive-macro command definitions (`#[derive(Command)]`). `AppCtx` holds the password store and multi-step input state (password confirmation buffers). Commands: `status`, `init`, `open`, `close`, `store`, `fetch`, `drop`, `scan`, `list`, `search`, `flash` (with `read`/`write`/`erase` subcommands).

### Library (`src/lib.rs`)
Type aliases (`SerialTx`, `UsbdSerial`), `CliWriter`/`SerialAccess` (raw pointer wrappers for shared serial port access), `AppCli` type alias, state enums (`InitState`, `OpenState`, `StoreState`), and `hex_dump()` utility.

## Key Design Details

- Both `dev` and `release` profiles use identical optimization settings (`opt-level = 'z'`, LTO)
- `build.rs` embeds git commit, branch, timestamp, and rustc version into the binary
- `build.rs` selects the correct `memory-*.x` linker script based on the feature flag
- `spi_memory` is vendored in `src/spi_memory.rs`
- Encryption: `chacha20poly1305` (XChaCha20Poly1305 AEAD). Master password → SHA2-256 → decrypts master key → master key used for all entry encryption. Wire format: `[24-byte nonce][ciphertext + 16-byte tag]`
- `#![no_std]` with `alloc` — uses `alloc-cortex-m` heap allocator

## Hardware Pin Mapping

- **SPI1**: PA5 (SCK), PA6 (MISO), PA7 (MOSI), PA4 (CS) — 16MHz, flash chip
- **USB**: OTG_FS (PA11/PA12)
- **USART1**: PA9 (TX), PA10 (RX) — 115200 baud
- **LED**: PC13 (active low)
- **Button**: PA0 (active low)
