// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![feature(alloc_error_handler)]
// #![feature(associated_type_bounds)]
// #![deny(warnings)]

extern crate alloc;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use panic_halt as _;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[allow(clippy::empty_loop)]
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true,
    dispatchers = [DMA2_STREAM2, DMA2_STREAM3, DMA2_STREAM4, DMA2_STREAM5, DMA2_STREAM6, DMA2_STREAM7])]
mod app {
    // use alloc::vec::Vec;
    use core::fmt::{self, Write};
    use cortex_m::asm;
    use privatebox::PrivateBox;
    use rand::prelude::*;
    use rand::SeedableRng;
    use spi_memory::{prelude::*, series25::Flash};
    use systick_monotonic::*;
    use usb_device::prelude::*;

    use stm32f4xx_hal as hal;

    use hal::otg_fs::{UsbBus, UsbBusType, USB};
    use hal::pac::{SPI1, USART1};
    use hal::watchdog::IndependentWatchdog;
    use hal::{delay::Delay, gpio::*, prelude::*, serial, spi::*};
    // use embedded_hal::digital::v2::OutputPin;

    #[monotonic(binds=SysTick, default=true)]
    type SysMono = Systick<10_000>; // 10 kHz / 100 µs granularity

    type SerialTx = serial::Tx<USART1, u8>;

    type MyFlash = Flash<
        Spi<
            SPI1,
            (
                Pin<Alternate<PushPull, 5>, 'A', 5>,
                Pin<Alternate<PushPull, 5>, 'A', 6>,
                Pin<Alternate<PushPull, 5>, 'A', 7>,
            ),
            TransferModeNormal,
        >,
        ErasedPin<Output<PushPull>>,
    >;

    type UsbdSerial = usbd_serial::SerialPort<'static, UsbBusType>;

    pub struct MyMenuCtx {
        serial: UsbdSerial,
        flash: MyFlash,
        watchdog: IndependentWatchdog,
    }
    impl fmt::Write for MyMenuCtx {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            s.bytes()
                .try_for_each(|c| {
                    if c == b'\n' {
                        self.serial.write(&[b'\r'])?;
                    }
                    self.serial.write(&[c]).map(|_| ())
                })
                .map_err(|_| fmt::Error)
        }
    }

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        menu_runner: menu::Runner<'static, MyMenuCtx>,
        ser_tx: SerialTx,
        addr: u32,
        pin_button: ErasedPin<Input<PullUp>>,
        pin_led: ErasedPin<Output<PushPull>>,
        led_on: bool,
        button_down: bool,
    }

    #[local]
    struct Local {}

    const FLASH_SZ: u32 = 8 * 1024 * 1024;
    const BLOCK_SZ: u32 = 4096;
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let start = cortex_m_rt::heap_start() as usize;
        let size = 32 * 1024; // in bytes
        unsafe { crate::ALLOCATOR.init(start, size) }

        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let dp = cx.device;
        let rcc = dp.RCC.constrain();

        // Setup system clocks
        let hse = 25.mhz();
        let sysclk = 84.mhz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        // Initialize the monotonic
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().0);

        let mut syscfg = dp.SYSCFG.constrain();
        let gpioa = dp.GPIOA.split();
        let _gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        let ser_tx_pin = gpioa.pa9.into_alternate::<7>();
        let _ser_rx_pin = gpioa.pa10.into_alternate::<7>();

        // default is 115200 bps, 8N1
        let ser_cfg = serial::config::Config::default().wordlength_8();
        let mut ser_tx = serial::Serial::tx(dp.USART1, ser_tx_pin, ser_cfg, &clocks).unwrap();
        write!(ser_tx, "\r\n\nStarting up...\r\n").ok();

        write!(ser_tx, "* Clocks:\r\n").ok();
        write!(
            ser_tx,
            "  requested sysclk {:?} with hse at {:?}\r\n",
            sysclk, hse
        )
        .ok();
        write!(
            ser_tx,
            "  sysclk: {:?}\r\n  hclk: {:?}\r\n",
            clocks.sysclk(),
            clocks.hclk()
        )
        .ok();
        write!(
            ser_tx,
            "  pclk1: {:?}\r\n  pclk2: {:?}\r\n",
            clocks.pclk1(),
            clocks.pclk2()
        )
        .ok();
        write!(
            ser_tx,
            "  pll48clk: {:?}\r\n  ppre1: {:?}\r\n  ppre2: {:?}\r\n",
            clocks.pll48clk(),
            clocks.ppre1(),
            clocks.ppre2()
        )
        .ok();

        // On Blackpill stm32f411 user led is on PC13, active low
        let mut pin_led = gpioc.pc13.into_push_pull_output().erase();
        pin_led.set_high();

        // User button on Blackpill is on PA0, active low
        // - and make it generate interrupts when pressed
        let mut exti = dp.EXTI;
        let mut pin_button = gpioa.pa0.into_pull_up_input().erase();
        pin_button.make_interrupt_source(&mut syscfg);
        pin_button.trigger_on_edge(&mut exti, Edge::Falling);
        pin_button.enable_interrupt(&mut exti);

        // https://docs.zephyrproject.org/2.6.0/boards/arm/blackpill_f411ce/doc/index.html
        // https://cdn-shop.adafruit.com/product-files/4877/4877_schematic-STM32F411CEU6_WeAct_Black_Pill_V2.0.pdf
        // SPI1 CS/SCK/MISO/MOSI : PA4/PA5/PA6/PA7 (Routed to footprint for external flash)
        let cs = {
            let mut cs = gpioa.pa4.into_push_pull_output().erase();
            cs.set_high(); // deselect
            cs
        };

        let spi = {
            let sck = gpioa.pa5.into_alternate::<5>();
            let miso = gpioa.pa6.into_alternate::<5>();
            let mosi = gpioa.pa7.into_alternate::<5>();

            Spi::new(
                dp.SPI1,
                (sck, miso, mosi),
                Mode {
                    polarity: Polarity::IdleHigh,
                    phase: Phase::CaptureOnSecondTransition,
                },
                16.mhz(),
                &clocks,
            )
        };

        // Create a delay abstraction based on general-purpose 32-bit timer TIM5
        let delay = Delay::tim5(dp.TIM5, &clocks);
        let mut flash = Flash::init(spi, cs, delay).unwrap();

        // *** Begin USB setup ***
        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };
        unsafe {
            USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
        }

        let serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Siuro Hacklab")
        .product("Mystery Gadget")
        .serial_number("1234")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        write!(ser_tx, "Flash read JEDEC id...\r\n").ok();
        let jedec_id = flash.read_jedec_id().unwrap();
        write!(ser_tx, "Flash jedec id: {:?}\r\n\n", jedec_id).ok();

        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        watchdog.start(500.ms());

        static mut MENU_BUF: [u8; 64] = [0u8; 64];
        let menu_ctx = MyMenuCtx {
            serial,
            flash,
            watchdog,
        };
        let menu_runner = menu::Runner::new(&ROOT_MENU, unsafe { &mut MENU_BUF }, menu_ctx);

        // Start the hardware watchdog

        // feed the watchdog
        periodic::spawn().ok();

        (
            Shared {
                usb_dev,
                menu_runner,
                ser_tx,
                addr: 0,
                pin_button,
                pin_led,
                led_on: false,
                button_down: false,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // Wait for interrupt...
            asm::wfi();
        }
    }

    // Feed the watchdog to avoid hardware reset.
    #[task(priority=1, shared=[menu_runner])]
    fn periodic(cx: periodic::Context) {
        let mut menu_runner = cx.shared.menu_runner;
        (&mut menu_runner).lock(|menu_runner| {
            menu_runner.context.watchdog.feed();
        });
        periodic::spawn_after(200u64.millis()).ok();
    }

    #[task(priority=3, capacity=8, shared=[led_on, pin_led])]
    fn led_blink(cx: led_blink::Context, ms: u64) {
        let mut led_on = cx.shared.led_on;
        let mut led = cx.shared.pin_led;

        (&mut led, &mut led_on).lock(|led, led_on| {
            if !(*led_on) {
                led.set_low();
                *led_on = true;
                led_off::spawn_after(ms.millis()).ok();
            }
        });
    }

    #[task(priority=3, shared=[led_on, pin_led])]
    fn led_off(cx: led_off::Context) {
        let mut led = cx.shared.pin_led;
        let mut led_on = cx.shared.led_on;
        (&mut led, &mut led_on).lock(|led, led_on| {
            led.set_high();
            *led_on = false;
        });
    }

    #[task(priority=4, binds=EXTI0, shared=[pin_button, button_down, ser_tx])]
    fn button(cx: button::Context) {
        let mut button = cx.shared.pin_button;
        (&mut button).lock(|button| button.clear_interrupt_pending_bit());

        let mut button_down = cx.shared.button_down;
        (&mut button_down).lock(|button_down| {
            if *button_down {
                return;
            }
            *button_down = true;
        });

        let mut ser_tx = cx.shared.ser_tx;
        (&mut ser_tx).lock(|ser_tx| {
            write!(ser_tx, "\r\n# button #\r\n").ok();
        });
        led_blink::spawn(500).ok();

        let seed = monotonics::now().ticks();
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let mut key = [0u8; 32];
        rng.fill_bytes(&mut key);
        let mut msg = [0u8; 32];
        rng.fill_bytes(&mut msg);

        let mut pb = PrivateBox::new(&key, rng);
        let header = [];
        let metadata = [];
        let cont = pb.encrypt(&msg, &header, &metadata).unwrap();
        let (dec, _auth_h) = pb.decrypt(&cont, &metadata).unwrap();

        (&mut ser_tx).lock(|ser_tx| {
            write!(ser_tx, "key:\r\n").ok();
            hex_dump_tx(ser_tx, &key);

            write!(ser_tx, "msg:\r\n").ok();
            hex_dump_tx(ser_tx, &msg);

            write!(ser_tx, "container:\r\n").ok();
            hex_dump_tx(ser_tx, cont.as_slice());

            write!(ser_tx, "dec:\r\n").ok();
            hex_dump_tx(ser_tx, dec.as_slice());
        });

        // "debounce" (disable) button for 500ms
        button_up::spawn_after(500u64.millis()).ok();
    }

    #[task(priority=3, shared=[button_down])]
    fn button_up(cx: button_up::Context) {
        let mut button_down = cx.shared.button_down;
        (&mut button_down).lock(|button_down| {
            *button_down = false;
        });
    }

    #[task(priority=3, shared=[addr, ser_tx])]
    fn new_addr(cx: new_addr::Context) {
        let seed = monotonics::now().ticks();
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let addr_rand: u32 = rng.gen();

        let mut addr = cx.shared.addr;
        let mut ser_tx = cx.shared.ser_tx;
        (&mut addr, &mut ser_tx).lock(|addr, ser_tx| {
            write!(ser_tx, "\r\nRNG seed: {}\r\n", seed).ok();
            write!(ser_tx, "Old addr: {:06x}\r\n", *addr).ok();
            *addr = addr_rand & 0xffff_f000;
            write!(ser_tx, "New addr: {:06x}\r\n\n", *addr).ok();
        });
    }

    #[task(priority=5, binds=OTG_FS, shared=[usb_dev, menu_runner])]
    fn usb_fs(cx: usb_fs::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut menu_runner = cx.shared.menu_runner;

        (&mut usb_dev, &mut menu_runner).lock(|usb_dev, menu_runner| {
            let serial = &mut menu_runner.context.serial;
            if !usb_dev.poll(&mut [serial]) {
                return;
            }

            let mut buf = [0u8; 4];
            if let Ok(count) = serial.read(&mut buf) {
                if count < 1 {
                    return;
                }

                for c in buf[0..count].iter() {
                    // keep runner at lower priority
                    feed_runner::spawn(*c).ok();
                }
            }
        });

        (&mut menu_runner).lock(|menu_runner| {
            menu_runner.context.watchdog.feed();
        });

        led_blink::spawn(10).ok();
    }

    #[task(priority=2, capacity=5, shared=[menu_runner])]
    fn feed_runner(cx: feed_runner::Context, c: u8) {
        let mut menu_runner = cx.shared.menu_runner;
        (&mut menu_runner).lock(|menu_runner| {
            menu_runner.input_byte(c);
        });
    }

    const ROOT_MENU: menu::Menu<MyMenuCtx> = menu::Menu {
        label: "root",
        entry: None,
        exit: None,
        items: &[
            &menu::Item {
                command: "read",
                help: Some("read flash mem"),
                item_type: menu::ItemType::Callback {
                    function: cmd_read_write,
                    parameters: &[
                        menu::Parameter::Mandatory {
                            parameter_name: "addr",
                            help: Some("Address of flash read"),
                        },
                        menu::Parameter::Optional {
                            parameter_name: "len",
                            help: Some("Length of flash read"),
                        },
                    ],
                },
            },
            &menu::Item {
                command: "write",
                help: Some("write flash mem"),
                item_type: menu::ItemType::Callback {
                    function: cmd_read_write,
                    parameters: &[
                        menu::Parameter::Mandatory {
                            parameter_name: "addr",
                            help: Some("Address of flash write"),
                        },
                        menu::Parameter::Optional {
                            parameter_name: "len",
                            help: Some("Length of flash write"),
                        },
                    ],
                },
            },
            &menu::Item {
                command: "erase",
                help: Some("erase flash mem"),
                item_type: menu::ItemType::Callback {
                    function: cmd_erase,
                    parameters: &[
                        menu::Parameter::Mandatory {
                            parameter_name: "addr",
                            help: Some("Address of flash erase - must be multiple of 4KB (0x1000)"),
                        },
                        menu::Parameter::Optional {
                            parameter_name: "len",
                            help: Some("Length of flash erase - must be multiple of 4KB (0x1000)"),
                        },
                    ],
                },
            },
        ],
    };

    const FLASH_SIZE: usize = 8 * 1024 * 1024; // 8 MB
    const SECT_SIZE: usize = 4 * 1024; // 4 KB

    fn parse_radix(ctx: &mut MyMenuCtx, addr: &str, radix: u32) -> Option<usize> {
        match usize::from_str_radix(addr, radix) {
            Err(e) => {
                write!(ctx, "Address (radix {}) parse error: {:?}\r\n", radix, e).ok();
                None
            }
            Ok(a) => Some(a),
        }
    }

    fn parse_addr(ctx: &mut MyMenuCtx, addr: &str) -> Option<usize> {
        if let Some(hex) = addr.strip_prefix("0x") {
            // Hex parse
            parse_radix(ctx, hex, 16)
        } else if addr.starts_with('0') && addr.len() > 1 {
            // Octal parse
            parse_radix(ctx, &addr[1..], 8)
        } else {
            // Decimal parse
            parse_radix(ctx, addr, 10)
        }
    }

    const HEX_BUF_SZ: usize = 256;
    fn cmd_read_write(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let mode_write = item.command == "write";

        let addr = if let Ok(Some(aa)) = menu::argument_finder(item, args, "addr") {
            if let Some(a) = parse_addr(ctx, aa) {
                a
            } else {
                write!(ctx, "Could not parse addr: \"{}\".\r\n", aa).ok();
                return;
            }
        } else {
            write!(ctx, "Address not given.\r\n").ok();
            return;
        };
        if addr >= FLASH_SIZE {
            write!(
                ctx,
                "Error: addr {} (0x{:x}) is larger than flash size {} (0x{:x}).\r\n",
                addr, addr, FLASH_SIZE, FLASH_SIZE
            )
            .ok();
            return;
        }

        let mut len = HEX_BUF_SZ;
        if let Ok(Some(al)) = menu::argument_finder(item, args, "len") {
            if let Some(ret) = parse_addr(ctx, al) {
                len = ret;
            } else {
                write!(ctx, "Could not parse len: \"{}\".\r\n", al).ok();
                return;
            }
        }
        if addr + len > FLASH_SIZE {
            let new_len = FLASH_SIZE - addr;
            write!(
                ctx,
                "Warning: len {} (0x{:x}) reaches beyond flash size {} (0x{:x}) and was trucated to {} (0x{:x}).\r\n",
                len, len, FLASH_SIZE, FLASH_SIZE, new_len, new_len
            )
            .ok();
            len = new_len;
        }

        let mut buf = [0u8; HEX_BUF_SZ as usize];
        let mut rng = rand::rngs::StdRng::seed_from_u64(monotonics::now().ticks());

        let mut chunks = len / HEX_BUF_SZ;
        let last_sz = len % HEX_BUF_SZ;
        if last_sz != 0 {
            chunks += 1;
        }

        write!(
            ctx,
            "\r\n* {}ing {} bytes at 0x{:06x} in {} chunks, last {} bytes:\r\n",
            if mode_write { "Writ" } else { "Read" },
            len,
            addr,
            chunks,
            last_sz
        )
        .ok();

        for c in 0..chunks {
            let mem_addr = addr + c * HEX_BUF_SZ;
            let mem_len = if last_sz == 0 || c < chunks - 1 {
                HEX_BUF_SZ
            } else {
                last_sz
            };

            if mode_write {
                rng.fill_bytes(&mut buf[..mem_len]);
                hex_dump_usb(ctx, mem_addr, &buf[..mem_len]);
                match ctx.flash.write_bytes(mem_addr as u32, &mut buf[..mem_len]) {
                    Ok(w) => {
                        write!(ctx, "#wait {}\r\n", w).ok();
                    }
                    Err(e) => {
                        write!(
                            ctx,
                            "\r\n### Flash write failed at 0x{:06x} ({:?}) - abort.\r\n",
                            mem_addr, e
                        )
                        .ok();
                        return;
                    }
                }
            } else {
                if let Err(e) = ctx.flash.read(mem_addr as u32, &mut buf[..mem_len]) {
                    write!(
                        ctx,
                        "\r\n### Flash read failed at 0x{:06x} ({:?}) - abort.\r\n",
                        mem_addr, e
                    )
                    .ok();
                    return;
                }
                hex_dump_usb(ctx, mem_addr, &buf[..mem_len]);
            }
            // prevent hardware reset during lengthy flash ops
            ctx.watchdog.feed();
        }
        write!(ctx, "\r\n").ok();
    }

    fn cmd_erase(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let jedec_id = ctx.flash.read_jedec_id().unwrap();
        write!(ctx, "\r\nFlash jedec id: {:?}\r\n", jedec_id).ok();

        let addr = if let Ok(Some(aa)) = menu::argument_finder(item, args, "addr") {
            if let Some(a) = parse_addr(ctx, aa) {
                a
            } else {
                write!(ctx, "Could not parse addr: \"{}\".\r\n", aa).ok();
                return;
            }
        } else {
            write!(ctx, "Address not given.\r\n").ok();
            return;
        };
        if addr % SECT_SIZE != 0 {
            write!(
                ctx,
                "Error: addr {} (0x{:x}) is not multiple of {} (0x{:02x}).\r\n",
                addr, addr, SECT_SIZE, SECT_SIZE
            )
            .ok();
            return;
        }
        if addr >= FLASH_SIZE {
            write!(
                ctx,
                "Error: addr {} (0x{:x}) is larger than flash size {} (0x{:x}).\r\n",
                addr, addr, FLASH_SIZE, FLASH_SIZE
            )
            .ok();
            return;
        }

        let mut len = SECT_SIZE;
        if let Ok(Some(al)) = menu::argument_finder(item, args, "len") {
            if let Some(ret) = parse_addr(ctx, al) {
                len = ret;
            } else {
                write!(ctx, "Could not parse len: \"{}\".\r\n", al).ok();
                return;
            }
        }
        if len % SECT_SIZE != 0 {
            write!(
                ctx,
                "Error: len {} (0x{:x}) is not multiple of {} (0x{:02x}).\r\n",
                len, len, SECT_SIZE, SECT_SIZE
            )
            .ok();
            return;
        }
        if addr + len > FLASH_SIZE {
            let new_len = FLASH_SIZE - addr;
            write!(
                ctx,
                "Warning: len {} (0x{:x}) reaches beyond flash size {} (0x{:x}) and was trucated to {} (0x{:x}).\r\n",
                len, len, FLASH_SIZE, FLASH_SIZE, new_len, new_len
            )
            .ok();
            len = new_len;
        }

        let sectors = len / SECT_SIZE;
        write!(
            ctx,
            "* Erasing {} (0x{:x}) bytes at 0x{:06x} in {} sectors:\r\n",
            len, len, addr, sectors
        )
        .ok();

        for c in 0..sectors {
            let mem_addr = addr + c * SECT_SIZE;
            match ctx.flash.erase_sectors(mem_addr as u32, 1) {
                Ok(w) => {
                    write!(ctx, "\r#e 0x{:06x} #w {}      ", mem_addr, w).ok();
                }
                Err(e) => {
                    write!(
                        ctx,
                        "\r\n### Flash erase failed at 0x{:06x} ({:?}) - abort.\r\n",
                        mem_addr, e
                    )
                    .ok();
                    return;
                }
            }
            // prevent hardware reset during lengthy flash ops
            ctx.watchdog.feed();
        }
        write!(ctx, "\r\ndone.\r\n").ok();
    }

    const ROW_SZ: usize = 0x40; // 64
    fn hex_dump_usb(output: &mut MyMenuCtx, addr: usize, buf: &[u8]) {
        for (i, c) in buf.iter().enumerate() {
            if i % ROW_SZ == 0 {
                if i > 0 {
                    write!(output, "\r\n").ok();
                }
                write!(output, "#{:06x} ", addr + i).ok();
            }
            write!(output, "{:02x} ", c).ok();
        }
        write!(output, "\r\n").ok();
    }

    fn hex_dump_tx(serial: &mut SerialTx, buf: &[u8]) {
        let mut offset: usize = 0;
        let len = buf.len();
        let mut stop = false;
        while !stop {
            let mut end = offset + ROW_SZ;
            if end > len {
                stop = true;
                end = len;
            }
            let slice = &buf[offset..end];
            offset += ROW_SZ;
            for c in slice {
                write!(serial, "{:02x} ", c).ok();
            }
            write!(serial, "\r\n").ok();
        }
    }
}
// EOF
