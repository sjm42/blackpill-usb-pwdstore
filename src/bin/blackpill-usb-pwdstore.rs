// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![feature(alloc_error_handler)]
#![feature(associated_type_bounds)]
// #![deny(warnings)]

extern crate alloc;
extern crate no_std_compat as std;
use blackpill_usb_pwdstore::*;
use panic_halt as _;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;

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
    use blackpill_usb_pwdstore::*;

    use cortex_m::asm;
    use spi_memory::series25::Flash;
    use systick_monotonic::*;
    use usb_device::prelude::*;

    #[monotonic(binds=SysTick, default=true)]
    type SysMono = Systick<10_000>; // 10 kHz / 100 µs granularity

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        menu_runner: menu::Runner<'static, MyMenuCtx>,
        ser_tx: SerialTx,
        addr: u32,
        pin_button: ErasedPin<Input>,
        pin_led: ErasedPin<Output<PushPull>>,
        led_on: bool,
        button_down: bool,
    }

    #[local]
    struct Local {}

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
        let hse = 25.MHz();
        let sysclk = 84.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        // Initialize the monotonic
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().raw());

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
            "  requested sysclk {sysclk:?} with hse at {hse:?}\r\n",
        )
        .ok();
        write!(
            ser_tx,
            "  sysclk: {sysclk:?}\r\n  hclk: {hclk:?}\r\n",
            sysclk = clocks.sysclk(),
            hclk = clocks.hclk()
        )
        .ok();
        write!(
            ser_tx,
            "  pclk1: {pclk1:?}\r\n  pclk2: {pclk2:?}\r\n",
            pclk1 = clocks.pclk1(),
            pclk2 = clocks.pclk2()
        )
        .ok();
        write!(
            ser_tx,
            "  pll48clk: {pll48clk:?}\r\n  ppre1: {ppre1:?}\r\n  ppre2: {ppre2:?}\r\n",
            pll48clk = clocks.pll48clk(),
            ppre1 = clocks.ppre1(),
            ppre2 = clocks.ppre2()
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
                16.MHz(),
                &clocks,
            )
        };

        // Create a delay abstraction based on general-purpose 32-bit timer TIM5
        let delay = hal::timer::FTimerUs::new(dp.TIM5, &clocks).delay();
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
        .product("Password Trove")
        .serial_number("4242")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        write!(ser_tx, "Flash read JEDEC id...\r\n").ok();
        let jedec_id = flash.read_jedec_id().unwrap();
        write!(ser_tx, "Flash jedec id: {jedec_id:?}\r\n\n").ok();

        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        // Start the hardware watchdog
        watchdog.start(500u32.millis());

        let pwd_store = PwdStore::new(flash, watchdog);

        let menu_ctx = MyMenuCtx {
            now: monotonics::now().ticks(),
            serial: MyUsbSerial { serial },
            pwd_store,
            init: InitState::Idle,
            open: OpenState::Idle,
            store: StoreState::Idle,
            opt_str: None,
            idx1: 0,
            buf1: [0; NOECHO_BUF_SIZE],
            idx2: 0,
            buf2: [0; NOECHO_BUF_SIZE],
            idx3: 0,
            buf3: [0; NOECHO_BUF_SIZE],
        };
        static mut MENU_BUF: [u8; 80] = [0u8; 80];
        let menu_runner = menu::Runner::new(&ROOT_MENU, unsafe { &mut MENU_BUF }, menu_ctx);

        // feed the watchdog
        periodic::spawn().unwrap();

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
        menu_runner.lock(|menu_runner| {
            menu_runner.context.pwd_store.watchdog.feed();
        });
        periodic::spawn_after(200u64.millis()).unwrap();
    }

    #[task(priority=3, capacity=8, shared=[led_on, pin_led])]
    fn led_blink(ctx: led_blink::Context, ms: u64) {
        let led_blink::SharedResources {
            mut led_on,
            mut pin_led,
        } = ctx.shared;

        (&mut led_on, &mut pin_led).lock(|led_on, pin_led| {
            if !(*led_on) {
                pin_led.set_low();
                *led_on = true;
                led_off::spawn_after(ms.millis()).unwrap();
            }
        });
    }

    #[task(priority=3, shared=[led_on, pin_led])]
    fn led_off(ctx: led_off::Context) {
        let led_off::SharedResources {
            mut led_on,
            mut pin_led,
        } = ctx.shared;

        (&mut led_on, &mut pin_led).lock(|led_on, pin_led| {
            pin_led.set_high();
            *led_on = false;
        });
    }

    #[task(priority=4, binds=EXTI0, shared=[pin_button, button_down, menu_runner])]
    fn button(ctx: button::Context) {
        let button::SharedResources {
            mut pin_button,
            mut button_down,
            mut menu_runner,
        } = ctx.shared;

        pin_button.lock(|button| button.clear_interrupt_pending_bit());

        button_down.lock(|button_down| {
            if *button_down {
                // debounce?
                return;
            }
            *button_down = true;
        });

        menu_runner.lock(|menu_runner| {
            let ser = &mut menu_runner.context;
            write!(ser, "\r\n# button #\r\n").ok();
        });
        led_blink::spawn(500).unwrap();

        // disable button for 500ms (debounce)
        button_up::spawn_after(500u64.millis()).unwrap();
    }

    #[task(priority=3, shared=[button_down])]
    fn button_up(ctx: button_up::Context) {
        let button_up::SharedResources { mut button_down } = ctx.shared;
        button_down.lock(|button_down| {
            *button_down = false;
        });
    }

    #[task(priority=5, binds=OTG_FS, shared=[usb_dev, menu_runner])]
    fn usb_fs(cx: usb_fs::Context) {
        let usb_fs::SharedResources {
            mut usb_dev,
            mut menu_runner,
        } = cx.shared;

        (&mut usb_dev, &mut menu_runner).lock(|usb_dev, menu_runner| {
            let ctx = &mut menu_runner.context;
            let serial = &mut ctx.serial.serial;

            if !usb_dev.poll(&mut [serial]) {
                return;
            }

            let mut buf = [0u8; 4];
            if let Ok(count) = serial.read(&mut buf) {
                if count < 1 {
                    return;
                }

                for c in buf[0..count].iter() {
                    if ctx.init != InitState::Idle {
                        match ctx.init {
                            InitState::AskPass1 => {
                                if *c != b'\r' && ctx.idx1 < NOECHO_BUF_SIZE {
                                    ctx.buf1[ctx.idx1] = *c;
                                    ctx.idx1 += 1;
                                    serial.write(b"*").ok();
                                } else {
                                    serial.write(b"\r\nrepeat password:\r\n> ").ok();
                                    ctx.init = InitState::AskPass2;
                                }
                            }
                            InitState::AskPass2 => {
                                if *c != b'\r' && ctx.idx2 < NOECHO_BUF_SIZE {
                                    ctx.buf2[ctx.idx2] = *c;
                                    ctx.idx2 += 1;
                                    serial.write(b"*").ok();
                                } else {
                                    serial.write(b"\r\n").ok();
                                    ctx.init = InitState::Idle;
                                    task_init::spawn().unwrap();
                                }
                            }
                            _ => {}
                        }
                    } else if ctx.open != OpenState::Idle {
                        if *c != b'\r' && ctx.idx1 < NOECHO_BUF_SIZE {
                            ctx.buf1[ctx.idx1] = *c;
                            ctx.idx1 += 1;
                            serial.write(b"*").ok();
                        } else {
                            serial.write(b"\r\n").ok();
                            ctx.open = OpenState::Idle;
                            task_open::spawn().unwrap();
                        }
                    } else if ctx.store != StoreState::Idle {
                        match ctx.store {
                            StoreState::AskUser => {
                                if *c != b'\r' && ctx.idx1 < NOECHO_BUF_SIZE {
                                    ctx.buf1[ctx.idx1] = *c;
                                    ctx.idx1 += 1;
                                    serial.write(&[*c]).ok();
                                } else {
                                    serial.write(b"\r\npassword:\r\n> ").ok();
                                    ctx.store = StoreState::AskPass1;
                                }
                            }
                            StoreState::AskPass1 => {
                                if *c != b'\r' && ctx.idx2 < NOECHO_BUF_SIZE {
                                    ctx.buf2[ctx.idx2] = *c;
                                    ctx.idx2 += 1;
                                    serial.write(b"*").ok();
                                } else {
                                    serial.write(b"\r\nrepeat password:\r\n> ").ok();
                                    ctx.store = StoreState::AskPass2;
                                }
                            }
                            StoreState::AskPass2 => {
                                if *c != b'\r' && ctx.idx3 < NOECHO_BUF_SIZE {
                                    ctx.buf3[ctx.idx3] = *c;
                                    ctx.idx3 += 1;
                                    serial.write(b"*").ok();
                                } else {
                                    serial.write(b"\r\n").ok();
                                    ctx.store = StoreState::Idle;
                                    task_store::spawn().unwrap();
                                }
                            }
                            _ => {}
                        }
                    } else {
                        // keep runner at lower priority
                        feed_runner::spawn(*c).unwrap();
                    }
                }
            }
        });

        menu_runner.lock(|menu_runner| {
            menu_runner.context.pwd_store.watchdog.feed();
        });

        led_blink::spawn(10).unwrap();
    }

    #[task(priority=2, capacity=5, shared=[menu_runner])]
    fn feed_runner(ctx: feed_runner::Context, c: u8) {
        let feed_runner::SharedResources { mut menu_runner } = ctx.shared;

        menu_runner.lock(|menu_runner| {
            menu_runner.context.now = monotonics::now().ticks();
            menu_runner.input_byte(c);
        });
    }

    #[task(priority=2, shared=[menu_runner])]
    fn task_init(ctx: task_init::Context) {
        let task_init::SharedResources { mut menu_runner } = ctx.shared;

        menu_runner.lock(|menu_runner| {
            let ctx = &mut menu_runner.context;
            let pass1 = String::from_utf8_lossy(&ctx.buf1[..ctx.idx1]).to_string();
            let pass2 = String::from_utf8_lossy(&ctx.buf2[..ctx.idx2]).to_string();
            if pass1 != pass2 {
                write!(ctx, "Error: passwords are not equal.\r\n").ok();
                return;
            }
            ctx.pwd_store
                .init(&mut ctx.serial, &pass1, monotonics::now().ticks());
            menu_runner.prompt(true);
        });
    }

    #[task(priority=2, shared=[menu_runner])]
    fn task_open(ctx: task_open::Context) {
        let task_open::SharedResources { mut menu_runner } = ctx.shared;

        menu_runner.lock(|menu_runner| {
            let ctx = &mut menu_runner.context;
            let pass = String::from_utf8_lossy(&ctx.buf1[..ctx.idx1]).to_string();
            ctx.pwd_store
                .open(&mut ctx.serial, &pass, monotonics::now().ticks());
            menu_runner.prompt(true);
        });
    }

    #[task(priority=2, shared=[menu_runner])]
    fn task_store(ctx: task_store::Context) {
        let task_store::SharedResources { mut menu_runner } = ctx.shared;

        menu_runner.lock(|menu_runner| {
            let ctx = &mut menu_runner.context;
            let name = ctx.opt_str.take().unwrap();
            let user = String::from_utf8_lossy(&ctx.buf1[..ctx.idx1]).to_string();
            let pass1 = String::from_utf8_lossy(&ctx.buf2[..ctx.idx2]).to_string();
            let pass2 = String::from_utf8_lossy(&ctx.buf3[..ctx.idx3]).to_string();
            if pass1 != pass2 {
                write!(ctx, "Error: passwords are not equal.\r\n").ok();
                return;
            }
            ctx.pwd_store.store(&mut ctx.serial, &name, &user, &pass1);
            menu_runner.prompt(true);
        });
    }
}
// EOF
