# blackpill-usb-pwdstore

This firmware makes your BlackPill board into a physical & private password stash.

* Using WeAct **BlackPill** with `STM32F411CEU6` cpu and 8MB W25Q64 SPI flash chip onboard
* Creates a USB CDC-ACM serial port with an interactive command line interface (powered by `embedded-cli`)
* Passwords are stored onto the SPI flash encrypted with the **XChaCha20Poly1305 AEAD** encryption algorithm
* Encryption key is a random 256-bit master key generated during initialization, itself stored encrypted using a master password set by the user
* **If master password is lost, there is no known way to recover any data!**
* Each password/secret has these values stored:
  * Flash location/address, example: `0x420`
  * **Name** (plaintext on flash and **searchable**) - max 250 bytes
  * **Username** (encrypted) - max 256 bytes
  * **Password** (encrypted) - max 256 bytes
* The location of new passwords is randomized to prevent premature flash wear
* No filesystem or database is used — the flash is accessed directly in `4KB` blocks, each holding one secret
* An 8 MB flash chip can store up to 2047 secrets/passwords, excluding the master key at address `0x000000`

## The hardware

![front side](https://raw.githubusercontent.com/sjm42/blackpill-usb-pwdstore/master/pics/blackpill1.jpg)

![flip side](https://raw.githubusercontent.com/sjm42/blackpill-usb-pwdstore/master/pics/blackpill2.jpg)

Obtain one of these, with the 8MB SPI flash chip:

[https://www.aliexpress.com/item/1005001456186625.html](https://www.aliexpress.com/item/1005001456186625.html)

(WeAct Studio official store is recommended)

Please refer to:

[https://github.com/WeActTC/MiniSTM32F4x1](https://github.com/WeActTC/MiniSTM32F4x1)

## How to build the firmware

These instructions are for Linux. The Rust installation and DFU programming steps may differ on macOS or Windows.

### Install Rust and necessary tools

First, install the Rust compiler:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs > rust-install.sh
```

**Check the script first.** I just hate instructions that blindly pipe curl output to shell.

```bash
more rust-install.sh
chmod 755 rust-install.sh
./rust-install.sh
```

Install the compile target for `Cortex-M4F`:

```bash
rustup target add thumbv7em-none-eabihf
```

Install tools for DFU programming:

```bash
sudo apt install -y dfu-util
rustup component add llvm-tools-preview
cargo install cargo-binutils
```

### Build it

Get the source:

```bash
git clone https://github.com/sjm42/blackpill-usb-pwdstore.git
cd blackpill-usb-pwdstore
```

Build using the `./build` script:

```bash
./build k b    # Build for BlackPill (default target)
./build k c    # Clippy lint check
./build k d    # DFU flash (interactive - prompts for DFU mode)
./build k f    # cargo-flash (requires a debug probe)
```

Hardware selector: `k` = BlackPill, `b` = Blue Pill, `n` = Nucleo F411

Or build manually:

```bash
cargo build --release --no-default-features --features black_pill --target thumbv7em-none-eabihf
```

### Upload the firmware (program the chip)

The STM32F411 has a built-in ROM bootloader that works over USB.

Please refer to **section 30**, *STM32F411xx devices bootloader* on page 30:

[https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)

Boot the BlackPill into **DFU** mode:

* Press both `NRST` and `BOOT0` buttons for one second
* Release `NRST` while still keeping `BOOT0` down for a second longer
* Release `BOOT0`

You should see something like this in kernel log (`dmesg -w`):

```text
usb 1-6: New USB device found, idVendor=0483, idProduct=df11, bcdDevice=22.00
usb 1-6: Product: STM32  BOOTLOADER
usb 1-6: Manufacturer: STMicroelectronics
```

Verify the DFU device:

```bash
dfu-util -l
```

Now program the chip:

```bash
./build k d
```

To start the new firmware, press `NRST` briefly and you should see:

```text
usb 1-6: New USB device found, idVendor=16c0, idProduct=27dd, bcdDevice= 0.10
usb 1-6: Product: Password Trove
usb 1-6: SerialNumber: 4242
```

You can now use `minicom` or any other terminal program to access your new gadget.

## Configure a permanent device path

You might want to configure your Linux to have the gadget always on the same device path.

```bash
cat >/etc/udev/rules.d/pwd-trove.rules <<'EOF'
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="27dd", ATTRS{product}=="Password Trove", MODE="666", GROUP="plugdev", SYMLINK="PwdTrove"
EOF
sudo service udev restart
sudo udevadm control --reload-rules
```

Now your Password Trove is always at `/dev/PwdTrove` when plugged in.

## Internal implementation

### RTIC 2 real-time framework

The firmware is built on [RTIC 2](https://rtic.rs/) (Real-Time Interrupt-driven Concurrency) with async software tasks. RTIC provides:

- **Priority-based preemptive scheduling** — higher-priority tasks (like USB interrupts at priority 5) preempt lower-priority ones (like CLI processing at priority 2)
- **Compile-time deadlock-free resource sharing** — shared resources are accessed through RTIC's lock mechanism, eliminating runtime data races
- **Async tasks with channels** — `rtic_sync` channels pass data between the USB interrupt handler and the async CLI processor
- **SysTick monotonic timer** at 10 kHz for delays, debouncing, and timestamps

Task priority layout:

| Priority | Task | Role |
|----------|------|------|
| 5 | `usb_fs` | USB OTG_FS interrupt handler — reads serial bytes, handles password input state machines |
| 4 | `button` | EXTI0 interrupt for PA0 button press with 500ms debounce |
| 3 | `led_blink`, `button_debounce` | LED blink duration control, button debounce timer |
| 2 | `feed_runner`, `task_init`, `task_open`, `task_store` | CLI command processing and crypto operations |
| 1 | `periodic` | Watchdog feed every 200ms |
| 0 | `idle` | WFI (wait for interrupt) — CPU sleeps when idle |

### Encryption scheme

Two layers of XChaCha20Poly1305 AEAD encryption protect the stored credentials:

1. **Master key generation** (`init`): A random 256-bit master key is generated. The user's master password is hashed with SHA2-256 to derive a wrapping key. The master key is encrypted with this wrapping key and stored at flash address `0x000000`.

2. **Credential encryption** (`store`): Each credential's username and password are individually encrypted with the master key. Each encrypted blob has the format `[24-byte nonce][ciphertext + 16-byte Poly1305 tag]` (40 bytes overhead per field).

3. **Opening the store** (`open`): The user's master password is hashed with SHA2-256, used to decrypt the master key from flash, and the master key is held in RAM for the session.

4. **Closing** (`close`): The cipher and RNG state are zeroed — the master key is no longer in RAM.

The RNG (`StdRng` / ChaCha12) is seeded from a hash of the master key combined with the monotonic timer value, ensuring unique nonces across sessions.

### Flash memory layout

The onboard W25Q64 8MB SPI flash is organized as 2048 sectors of 4KB each:

```
Address range        Sector    Purpose
0x000000–0x000FFF    0x000     Master key record (encrypted)
0x001000–0x001FFF    0x001     First password slot
  ...                 ...
0x7FF000–0x7FFFFF    0x7FF     Last password slot (2047 total)
```

Each 4KB `PwdRepr` block is structured as:

```
Offset  Size     Field
0       2        len_name (u16, little-endian)
2       2        len_username (u16)
4       2        len_password (u16)
6       250      name (plaintext, enables scanning/searching)
256     3840     data (encrypted username + encrypted password, concatenated)
```

A sector is considered "free" when its length fields are invalid (e.g., all `0xFF` from erase). New entries are placed at a random sector to distribute flash wear.

### SPI flash driver

The `spi_memory` driver is vendored in `src/spi_memory.rs` (originally from an external crate). It communicates with the W25Q64 over SPI1 at 16 MHz, supporting:
- JEDEC ID read for chip identification
- Byte-level reads, page-level writes (with automatic page-boundary handling)
- 4KB sector erase with busy-wait polling

### CLI system

The command-line interface uses `embedded-cli 0.2` with derive-macro command definitions. Commands are defined as a Rust enum with `#[derive(Command)]` and dispatched through pattern matching. The CLI supports:

- Command history (128-byte ring buffer)
- Tab completion
- Automatic `\n` → `\r\n` conversion for terminal compatibility

Available commands:

| Command | Description |
|---------|-------------|
| `status` | Show firmware version, build timestamp, compiler, open/closed state |
| `init` | Initialize a new password store (generates master key, prompts for master password) |
| `open` | Open existing store (prompts for master password, decrypts master key) |
| `close` | Close store (clears master key from RAM) |
| `store <name>` | Store a new credential (prompts for username and password with confirmation) |
| `fetch <loc>` | Retrieve and decrypt a credential by its hex location |
| `drop <loc>` | Erase a credential from flash |
| `scan` | Full flash scan showing all stored entries with encrypted data sizes |
| `list` | List all credential names and locations |
| `search <str>` | Search credential names by substring |
| `flash read/write/erase` | Low-level flash operations (for debugging) |

### Hardware watchdog

An independent hardware watchdog (IWDG) is configured with a 500ms timeout. The `periodic` task feeds it every 200ms. Long-running operations (flash scans, bulk erase) feed the watchdog inline to prevent resets. On panic, the firmware resets via `panic-reset`.

### Build-time metadata

`build.rs` embeds the following into the binary via environment variables: git commit hash, git branch, source timestamp, and rustc version. These are displayed by the `status` command.

### Pin mapping

| Peripheral | Pins | Configuration |
|------------|------|---------------|
| SPI1 (flash) | PA4 (CS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) | 16 MHz, Mode 3 |
| USB OTG_FS | PA11 (DM), PA12 (DP) | Full-speed USB |
| USART1 | PA9 (TX), PA10 (RX) | 115200 baud, 8N1 (debug output) |
| LED | PC13 | Active low |
| Button | PA0 | Active low, pull-up, falling-edge interrupt |

## Key dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| `rtic` | 2.x | Real-time task framework |
| `rtic-monotonics` | 2.x | SysTick monotonic timer |
| `rtic-sync` | 1.x | Async channels for inter-task communication |
| `stm32f4xx-hal` | 0.23 | STM32F4 hardware abstraction layer |
| `chacha20poly1305` | 0.10 | XChaCha20Poly1305 AEAD encryption |
| `sha2` | 0.10 | SHA-256 password hashing |
| `rand` | 0.9 | StdRng (ChaCha12) for nonce generation and randomized allocation |
| `embedded-cli` | 0.2 | Interactive CLI with history and tab completion |
| `usb-device` | 0.3 | USB device stack |
| `usbd-serial` | 0.2 | USB CDC-ACM serial port class |
| `alloc-cortex-m` | 0.4 | 32KB heap allocator |

## Other useful links

[https://github.com/WeActTC/MiniSTM32F4x1/tree/master/SDK/CMSIS-DAP](https://github.com/WeActTC/MiniSTM32F4x1/tree/master/SDK/CMSIS-DAP)

[https://github.com/koendv/blackmagic-blackpill](https://github.com/koendv/blackmagic-blackpill)

[https://github.com/blacksphere/blackmagic/wiki/Useful-GDB-commands](https://github.com/blacksphere/blackmagic/wiki/Useful-GDB-commands)

[https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)


## Example session

```text

minicom -D /dev/PwdTrove

> help
AVAILABLE ITEMS:
  status
  init
  open
  close
  store <name>
  fetch <loc>
  drop <loc>
  scan
  list
  search <srch>
  wait
  flash
  help [ <command> ]

> status
*** blackpill-usb-pwdstore ***
Version: 0.2.0
Source timestamp: 2025-01-15T13:28:53Z
Compiler: rustc 1.84.0 (9fc6b4312 2025-01-07)
Status: CLOSED

> init
master password:
> *********************
repeat password:
> *********************
New master key generated & saved successfully. Password store opened.

> list
loc   name
----------
0x000 MASTER

> store account1
username:
> foo1@example.com
password:
> ******************
repeat password:
> ******************
Stored to loc 0x0d5

> store account2
username:
> foo2@example.com
password:
> ******************
repeat password:
> ******************
Stored to loc 0x703

> store xyzzy123
username:
> xyzzy@example.com
password:
> ******************
repeat password:
> ******************
Stored to loc 0x0e8

> list
loc   name
----------
0x000 MASTER
0x0d5 account1
0x0e8 xyzzy123
0x703 account2

> fetch 42
No valid data in this location.

> fetch 0x703
name: account2
user: foo2@example.com
pass: verysecretpassword

> drop 420
No valid data in this location.

> drop 0x0d5
Dropped loc 0x0d5

> list
loc   name
----------
0x000 MASTER
0x0e8 xyzzy123
0x703 account2

> store another1
username:
> another@example.com
password:
> ******************
repeat password:
> ******************
Stored to loc 0x113

> list
loc   name
----------
0x000 MASTER
0x0e8 xyzzy123
0x113 another1
0x703 account2

> search 2
loc   name
----------
0x0e8 xyzzy123
0x703 account2

> search xyz
loc   name
----------
0x0e8 xyzzy123

> close
Password store closed.

> list
Error: password store is not open.

> open
master password:
> *********************
Master key decrypted successfully. Password store opened.

> list
loc   name
----------
0x000 MASTER
0x0e8 xyzzy123
0x113 another1
0x703 account2

> close
Password store closed.

```
