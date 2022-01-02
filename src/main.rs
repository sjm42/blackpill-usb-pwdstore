// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
// #![feature(associated_type_bounds)]
#![feature(alloc_error_handler)]
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

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [DMA2_STREAM5, DMA2_STREAM6, DMA2_STREAM7])]
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
    use hal::pac::SPI1;
    use hal::watchdog::IndependentWatchdog;
    use hal::{delay::Delay, gpio::*, prelude::*, serial, spi::*};
    // use embedded_hal::digital::v2::OutputPin;

    #[monotonic(binds=SysTick, default=true)]
    type SysMono = Systick<10_000>; // 10 kHz / 100 µs granularity

    pub struct UsbSerial {
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
    }
    impl fmt::Write for UsbSerial {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            s.bytes()
                .try_for_each(|c| self.serial.write(&[c]).map(|_| ()))
                .map_err(|_| fmt::Error)
        }
    }

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

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: UsbSerial,
        flash: MyFlash,
        addr: u32,
        pin_button: ErasedPin<Input<PullUp>>,
        pin_led: ErasedPin<Output<PushPull>>,
        led_on: bool,
        button_down: bool,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
    }

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
        write!(ser_tx, "Flash jedec id: {:?}\r\n", jedec_id).ok();

        // Start the hardware watchdog
        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        watchdog.start(500.ms());

        // feed the watchdog
        periodic::spawn().ok();

        (
            Shared {
                usb_dev,
                serial: UsbSerial { serial },
                flash,
                addr: 0,
                pin_button,
                pin_led,
                led_on: false,
                button_down: false,
            },
            Local { watchdog },
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
    #[task(priority=1, local=[watchdog])]
    fn periodic(cx: periodic::Context) {
        cx.local.watchdog.feed();
        periodic::spawn_after(200u64.millis()).ok();
    }

    #[task(priority=2, capacity=2, shared=[led_on, pin_led])]
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

    #[task(priority=2, shared=[led_on, pin_led])]
    fn led_off(cx: led_off::Context) {
        let mut led = cx.shared.pin_led;
        let mut led_on = cx.shared.led_on;
        (&mut led, &mut led_on).lock(|led, led_on| {
            led.set_high();
            *led_on = false;
        });
    }

    #[task(priority=2, shared=[flash, addr, serial])]
    fn dump_flash(cx: dump_flash::Context) {
        let mut buf = [0u8; 256];
        let mut flash = cx.shared.flash;
        let mut addr = cx.shared.addr;
        let mut serial = cx.shared.serial;
        (&mut flash, &mut addr, &mut serial).lock(|flash, addr, serial| {
            write!(serial, "\r\nFlash read dump (0x{:06x}):\r\n", *addr).ok();
            flash.read(*addr, &mut buf).unwrap();
            hex_dump(serial, &buf);
        });
    }

    const ROW_SZ: usize = 32;
    fn hex_dump(serial: &mut UsbSerial, buf: &[u8]) {
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

    #[task(priority=4, binds=EXTI0, shared=[pin_button, button_down, serial])]
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

        let mut serial = cx.shared.serial;
        (&mut serial).lock(|serial| {
            write!(serial, "\r\n# button #\r\n").ok();
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

        (&mut serial).lock(|serial| {
            write!(serial, "key:\r\n").ok();
            hex_dump(serial, &key);

            write!(serial, "msg:\r\n").ok();
            hex_dump(serial, &msg);

            write!(serial, "container:\r\n").ok();
            hex_dump(serial, cont.as_slice());

            write!(serial, "dec:\r\n").ok();
            hex_dump(serial, dec.as_slice());
        });

        // "debounce" (disable) button for 100ms
        button_up::spawn_after(500u64.millis()).ok();
    }

    #[task(priority=2, shared=[button_down])]
    fn button_up(cx: button_up::Context) {
        let mut button_down = cx.shared.button_down;
        (&mut button_down).lock(|button_down| {
            *button_down = false;
        });
    }

    #[task(priority=2, shared=[flash, addr, serial])]
    fn erase_flash(cx: erase_flash::Context) {
        let mut flash = cx.shared.flash;
        let mut addr = cx.shared.addr;
        let mut serial = cx.shared.serial;
        (&mut flash, &mut addr, &mut serial).lock(|flash, addr, serial| {
            let jedec_id = flash.read_jedec_id().unwrap();
            write!(serial, "\r\nFlash jedec id: {:?}\r\n", jedec_id).ok();
            write!(serial, "### Flash erase (0x{:06x})...\r\n\n", *addr).ok();
            flash.erase_sectors(*addr, 1).ok();
        });
    }

    #[task(priority=2, shared=[flash, addr, serial])]
    fn write_flash(cx: write_flash::Context) {
        let seed = monotonics::now().ticks();
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let mut buf = [0u8; 256];
        buf.iter_mut().map(|b| *b = rng.gen()).count();

        let mut flash = cx.shared.flash;
        let mut addr = cx.shared.addr;
        let mut serial = cx.shared.serial;
        (&mut flash, &mut addr, &mut serial).lock(|flash, addr, serial| {
            write!(serial, "\r\nRNG seed: {}\r\n", seed).ok();
            write!(serial, "Write buffer dump:\r\n").ok();
            hex_dump(serial, &buf);
            let jedec_id = flash.read_jedec_id().unwrap();
            write!(serial, "Flash jedec id: {:?}\r\n", jedec_id).ok();
            write!(serial, "### Flash write (0x{:06x})...\r\n", *addr).ok();
            let w = flash.write_bytes(*addr, &mut buf).unwrap_or(0);
            write!(serial, "Wait cycles: {}\r\n\n", w).ok();
        });
    }

    #[task(priority=2, shared=[addr, serial])]
    fn new_addr(cx: new_addr::Context) {
        let seed = monotonics::now().ticks();
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let addr_rand: u32 = rng.gen();

        let mut addr = cx.shared.addr;
        let mut serial = cx.shared.serial;
        (&mut addr, &mut serial).lock(|addr, serial| {
            write!(serial, "\r\nRNG seed: {}\r\n", seed).ok();
            write!(serial, "Old addr: {:06x}\r\n", *addr).ok();
            *addr = addr_rand & 0xffff_f000;
            write!(serial, "New addr: {:06x}\r\n\n", *addr).ok();
        });
    }

    #[task(priority=5, binds=OTG_FS, shared=[usb_dev, serial])]
    fn usb_fs(cx: usb_fs::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            usb_poll(usb_dev, &mut serial.serial);
        });
        led_blink::spawn(10).ok();
    }

    fn usb_poll<B: usb_device::bus::UsbBus>(
        usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
        serial: &mut usbd_serial::SerialPort<'static, B>,
    ) {
        if !usb_dev.poll(&mut [serial]) {
            return;
        }

        let mut buf = [0u8; 4];
        if let Ok(count) = serial.read(&mut buf) {
            if count < 1 {
                return;
            }
            for c in buf[0..count].iter() {
                let x;
                if *c == b'?' {
                    x = *c;
                    dump_flash::spawn().ok();
                } else if *c == b'0' {
                    x = *c;
                    erase_flash::spawn().ok();
                } else if *c == b'!' {
                    x = *c;
                    write_flash::spawn().ok();
                } else if *c == b'#' {
                    x = *c;
                    new_addr::spawn().ok();
                } else if *c >= 0x41 && *c <= 0x5A {
                    // uppercase aka A-Z
                    x = b'X';
                } else if *c >= 0x61 && *c <= 0x7A {
                    // lowercase aka a-z
                    x = b'x';
                } else {
                    x = *c;
                }
                serial.write(&[x]).ok();
            }
        }
    }
}
// EOF
