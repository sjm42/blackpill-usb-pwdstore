# blackpill-usb-pwdstore

This firmware makes your Black pill card into a physical & private password stash.

* Using **Black pill** with `STM32F411` with 8MB spi flash chip onboard
* Creates a USB ACM serial port with a simple command line interface
* Passwords are stored onto the spi flash encrypted with the RustCrypto **XChaCha20Poly1305 AEAD** encryption algorithm.
* Encryption key is random 256 bits that is generated in the initialization, and stored in encrypted form using a master password set by the user.
* **If master password is lost, there is no known way to recover any data!**
* Each password/secret has these values stored:
  * Flash location/address, example: `0x420`
  * **Name** (plaintext, searchable) - max 250 bytes
  * **Username** (encrypted) - max 256 bytes
  * **Password** (encrypted) - max 256 bytes
* The location of new passwords is randomized to prevent premature flash wear.
* No filesystem or database is used, we are just brutally accessing the flash in `4KB` blocks, each holding one secret.
* An 8 MB flash chip can thus store up to 2047 secrets/passwords, excluding the master key at address `0x000000`.

## The hardware?


Obtain one of these, with the 8MB spi flash chip:

[https://www.aliexpress.com/item/1005001456186625.html](https://www.aliexpress.com/item/1005001456186625.html)

(WeAct Studio official store is recommended)

Please refer to:

[https://github.com/WeActTC/MiniSTM32F4x1](https://github.com/WeActTC/MiniSTM32F4x1)

## How to build

Please note: these instructions are only applicable as-is for **Ubuntu 20.04 LTS** Linux, but it should not be that hard to adjust for MacOS or Windows.

The Rust installation part would be different, and dfu programming probably needs driver installation and different tools.

### Install rust and necessary tools

First, Rust compiler itself

`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs > rust-install.sh`

**Check the script first.** I just hate instructions that blindly pipe curl output to shell.

`more rust-install.sh`

`chmod 755 rust-install.sh`

`./rust-install.sh`

Install nightly toolchain and compile target for `Cortex-M4F`

`rustup toolchain install nightly`

`rustup target add thumbv7em-none-eabihf`

Install tools for dfu programming

`sudo apt install -y dfu-util`

`rustup component add llvm-tools-preview`

`cargo install cargo-binutils`

### Build it

Get the source

`git clone https://github.com/sjm42/blackpill-usb-pwdstore.git`

Build it:

`cd blackpill-usb-pwdstore`

`./build build`

### Upload the firmware i.e. program the chip

Boot the blackpill into dfu mode:

* press both `NRST` and `BOOT0` buttons for one second,
* release `NRST` while still keeping `BOOT0` down for a second longer. With a little practice, this is easy with one finger.
* release `BOOT0` button.

You should see something like this in kernel log:

`sudo tail -F /var/log/kern.log`

```text
Jan 15 15:22:18 bad kernel: [ 4643.224900] usb 1-6: New USB device found, idVendor=0483, idProduct=df11, bcdDevice=22.00
Jan 15 15:22:18 bad kernel: [ 4643.224910] usb 1-6: New USB device strings: Mfr=1, Product=2, SerialNumber=3
Jan 15 15:22:18 bad kernel: [ 4643.224914] usb 1-6: Product: STM32  BOOTLOADER
Jan 15 15:22:18 bad kernel: [ 4643.224917] usb 1-6: Manufacturer: STMicroelectronics
Jan 15 15:22:18 bad kernel: [ 4643.224920] usb 1-6: SerialNumber: 337938943430
```

You can also check the dfu device:

`dfu-util -l`

```text
dfu-util 0.9
...
Found DFU: [0483:df11] ver=2200, devnum=7, cfg=1, intf=0, path="1-6", alt=3, name="@Device Feature/0xFFFF0000/01*004 e", serial="337938943430"
Found DFU: [0483:df11] ver=2200, devnum=7, cfg=1, intf=0, path="1-6", alt=2, name="@OTP Memory /0x1FFF7800/01*512 e,01*016 e", serial="337938943430"
Found DFU: [0483:df11] ver=2200, devnum=7, cfg=1, intf=0, path="1-6", alt=1, name="@Option Bytes  /0x1FFFC000/01*016 e", serial="337938943430"
Found DFU: [0483:df11] ver=2200, devnum=7, cfg=1, intf=0, path="1-6", alt=0, name="@Internal Flash  /0x08000000/04*016Kg,01*064Kg,03*128Kg", serial="337938943430"
```

Program the chip:

```text
./build dfu
+ cargo objcopy --target thumbv7em-none-eabihf --release -- -O binary target/out.bin
    Finished release [optimized] target(s) in 0.03s
+ set +x
Boot the card in DFU mode and press Enter.

+ dfu-util -a0 -s 0x08000000 -D target/out.bin
dfu-util 0.9

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2016 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

dfu-util: Invalid DFU suffix signature
dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
Opening DFU capable USB device...
ID 0483:df11
Run-time device DFU version 011a
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuERROR, status = 10
dfuERROR, clearing status
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 011a
Device returned transfer size 2048
DfuSe interface name: "Internal Flash  "
Downloading to address = 0x08000000, size = 71792
Download	[=========================] 100%        71792 bytes
Download done.
File downloaded successfully
+ set +x
```

Now reset the chip i.e. press `NRST` and you should see something like this in kernel messages:

```text
Jan 15 15:33:12 bad kernel: [ 5297.255112] usb 1-6: new full-speed USB device number 8 using xhci_hcd
Jan 15 15:33:12 bad kernel: [ 5297.410722] usb 1-6: New USB device found, idVendor=16c0, idProduct=27dd, bcdDevice= 0.10
Jan 15 15:33:12 bad kernel: [ 5297.410742] usb 1-6: Product: Password Store
Jan 15 15:33:12 bad kernel: [ 5297.410751] usb 1-6: SerialNumber: 4242
```

You can use `minicom` or any other terminal program to access your new gadget.

```text
minicom -D /dev/ttyACM0
(press enter for a few times to check connection)

> status
*** blackpill-usb-pwdstore ***
Version: 0.1.0
Source timestamp: 2022-01-15T13:28:53Z
Compiler: rustc 1.60.0-nightly (ad46af247 2022-01-14)
Status: CLOSED

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

> 
```

## Other useful links

[https://github.com/WeActTC/MiniSTM32F4x1/tree/master/SDK/CMSIS-DAP](https://github.com/WeActTC/MiniSTM32F4x1/tree/master/SDK/CMSIS-DAP)

[https://github.com/koendv/blackmagic-blackpill](https://github.com/koendv/blackmagic-blackpill)

[https://github.com/blacksphere/blackmagic/wiki/Useful-GDB-commands](https://github.com/blacksphere/blackmagic/wiki/Useful-GDB-commands)

[https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)
