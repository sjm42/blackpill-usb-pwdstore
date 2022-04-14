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

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use panic_halt as _;
use std::prelude::v1::*;

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

    use alloc::string::*;
    use core::fmt::{self, Write};
    use cortex_m::asm;
    use rand::prelude::*;
    use rand::SeedableRng;
    use spi_memory::{prelude::*, series25::Flash};
    use systick_monotonic::*;
    use usb_device::prelude::*;

    use stm32f4xx_hal as hal;

    use hal::otg_fs::{UsbBus, UsbBusType, USB};
    use hal::pac::USART1;
    use hal::watchdog::IndependentWatchdog;
    use hal::{gpio::*, prelude::*, serial, spi::*};
    // use embedded_hal::digital::v2::OutputPin;

    const NOECHO_BUF_SIZE: usize = 256;

    #[monotonic(binds=SysTick, default=true)]
    type SysMono = Systick<10_000>; // 10 kHz / 100 µs granularity

    type SerialTx = serial::Tx<USART1, u8>;
    type UsbdSerial = usbd_serial::SerialPort<'static, UsbBusType>;

    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum InitState {
        Idle,
        AskPass1,
        AskPass2,
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum OpenState {
        Idle,
        AskPass,
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum StoreState {
        Idle,
        AskUser,
        AskPass1,
        AskPass2,
    }

    pub struct MyUsbSerial {
        serial: UsbdSerial,
    }

    pub struct MyMenuCtx {
        serial: MyUsbSerial,
        pwd_store: PwdStore,
        init: InitState,
        open: OpenState,
        store: StoreState,
        opt_str: Option<String>,
        idx1: usize,
        buf1: [u8; NOECHO_BUF_SIZE],
        idx2: usize,
        buf2: [u8; NOECHO_BUF_SIZE],
        idx3: usize,
        buf3: [u8; NOECHO_BUF_SIZE],
    }

    impl fmt::Write for MyMenuCtx {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            s.bytes()
                .try_for_each(|c| {
                    if c == b'\n' {
                        self.serial.serial.write(&[b'\r'])?;
                    }
                    self.serial.serial.write(&[c]).map(|_| ())
                })
                .map_err(|_| fmt::Error)
        }
    }

    impl fmt::Write for MyUsbSerial {
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
        static mut MENU_BUF: [u8; 64] = [0u8; 64];
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
        (&mut menu_runner).lock(|menu_runner| {
            menu_runner.context.pwd_store.watchdog.feed();
        });
        periodic::spawn_after(200u64.millis()).unwrap();
    }

    #[task(priority=3, capacity=8, shared=[led_on, pin_led])]
    fn led_blink(cx: led_blink::Context, ms: u64) {
        let mut led_on = cx.shared.led_on;
        let mut led = cx.shared.pin_led;

        (&mut led, &mut led_on).lock(|led, led_on| {
            if !(*led_on) {
                led.set_low();
                *led_on = true;
                led_off::spawn_after(ms.millis()).unwrap();
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

    #[task(priority=4, binds=EXTI0, shared=[pin_button, button_down, menu_runner])]
    fn button(cx: button::Context) {
        let mut button = cx.shared.pin_button;
        (&mut button).lock(|button| button.clear_interrupt_pending_bit());

        let mut button_down = cx.shared.button_down;
        (&mut button_down).lock(|button_down| {
            if *button_down {
                // debounce?
                return;
            }
            *button_down = true;
        });

        let mut menu_runner = cx.shared.menu_runner;
        (&mut menu_runner).lock(|menu_runner| {
            let ser = &mut menu_runner.context;
            write!(ser, "\r\n# button #\r\n").ok();
        });
        led_blink::spawn(500).unwrap();

        // disable button for 500ms (debounce)
        button_up::spawn_after(500u64.millis()).unwrap();
    }

    #[task(priority=3, shared=[button_down])]
    fn button_up(cx: button_up::Context) {
        let mut button_down = cx.shared.button_down;
        (&mut button_down).lock(|button_down| {
            *button_down = false;
        });
    }

    #[task(priority=5, binds=OTG_FS, shared=[usb_dev, menu_runner])]
    fn usb_fs(cx: usb_fs::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut menu_runner = cx.shared.menu_runner;

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

        (&mut menu_runner).lock(|menu_runner| {
            menu_runner.context.pwd_store.watchdog.feed();
        });

        led_blink::spawn(10).unwrap();
    }

    #[task(priority=2, capacity=5, shared=[menu_runner])]
    fn feed_runner(cx: feed_runner::Context, c: u8) {
        let mut menu_runner = cx.shared.menu_runner;
        (&mut menu_runner).lock(|menu_runner| {
            menu_runner.input_byte(c);
        });
    }

    #[task(priority=2, shared=[menu_runner])]
    fn task_init(cx: task_init::Context) {
        let mut menu_runner = cx.shared.menu_runner;
        (&mut menu_runner).lock(|menu_runner| {
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
    fn task_open(cx: task_open::Context) {
        let mut menu_runner = cx.shared.menu_runner;
        (&mut menu_runner).lock(|menu_runner| {
            let ctx = &mut menu_runner.context;
            let pass = String::from_utf8_lossy(&ctx.buf1[..ctx.idx1]).to_string();
            ctx.pwd_store
                .open(&mut ctx.serial, &pass, monotonics::now().ticks());
            menu_runner.prompt(true);
        });
    }

    #[task(priority=2, shared=[menu_runner])]
    fn task_store(cx: task_store::Context) {
        let mut menu_runner = cx.shared.menu_runner;
        (&mut menu_runner).lock(|menu_runner| {
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

    const ROOT_MENU: menu::Menu<MyMenuCtx> = menu::Menu {
        label: "root",
        entry: None,
        exit: None,
        items: &[
            &menu::Item {
                command: "status",
                help: Some("Show status"),
                item_type: menu::ItemType::Callback {
                    function: cmd_status,
                    parameters: &[],
                },
            },

            &menu::Item {
                command: "init",
                help: Some("Initialize password store"),
                item_type: menu::ItemType::Callback {
                    function: cmd_init,
                    parameters: &[],
                },
            },

            &menu::Item {
                command: "open",
                help: Some("Open password store"),
                item_type: menu::ItemType::Callback {
                    function: cmd_open,
                    parameters: &[],
                },
            },

            &menu::Item {
                command: "close",
                help: Some("Close password store"),
                item_type: menu::ItemType::Callback {
                    function: cmd_close,
                    parameters: &[],
                },
            },

            &menu::Item {
                command: "store",
                help: Some("Store secret to flash"),
                item_type: menu::ItemType::Callback {
                    function: cmd_store,
                    parameters: &[
                        menu::Parameter::Mandatory {
                        parameter_name: "name",
                        help: Some("Name of secret"),
                        },
                    ],
                },
            },

            &menu::Item {
                command: "fetch",
                help: Some("Fetch secret from flash"),
                item_type: menu::ItemType::Callback {
                    function: cmd_fetch,
                    parameters: &[
                        menu::Parameter::Mandatory {
                        parameter_name: "loc",
                        help: Some("Location to fetch"),
                    }],
                },
            },

            &menu::Item {
                command: "drop",
                help: Some("Drop secret from flash"),
                item_type: menu::ItemType::Callback {
                    function: cmd_drop,
                    parameters: &[
                        menu::Parameter::Mandatory {
                        parameter_name: "loc",
                        help: Some("Location to drop"),
                    }],
                },
            },

            &menu::Item {
                command: "scan",
                help: Some("Scan flash for secrets"),
                item_type: menu::ItemType::Callback {
                    function: cmd_scan,
                    parameters: &[],
                },
            },

            &menu::Item {
                command: "list",
                help: Some("List secrets"),
                item_type: menu::ItemType::Callback {
                    function: cmd_list,
                    parameters: &[],
                },
            },

            &menu::Item {
                command: "search",
                help: Some("Search secret names with a string"),
                item_type: menu::ItemType::Callback {
                    function: cmd_search,
                    parameters: &[
                        menu::Parameter::Mandatory {
                        parameter_name: "srch",
                        help: Some("String to search"),
                    }],
                },
            },

            &menu::Item {
                command: "wait",
                help: Some("List secrets"),
                item_type: menu::ItemType::Callback {
                    function: cmd_wait,
                    parameters: &[],
                },
            },

            &menu::Item {
                command: "flash",
                help: Some("Flash functions"),
                item_type: menu::ItemType::Menu(
                    &menu::Menu {
                        label: "flash",
                        entry: None,
                        exit: None,
                        items: &[
                            &menu::Item {
                                command: "read",
                                help: Some("Read flash mem"),
                                item_type: menu::ItemType::Callback {
                                    function: cmd_flash_read_write,
                                    parameters: &[
                                        menu::Parameter::Mandatory {
                                            parameter_name: "addr",
                                            help: Some("Start address of flash read"),
                                        },
                                        menu::Parameter::Optional {
                                            parameter_name: "len",
                                            help: Some("Read length in bytes"),
                                        },
                                    ],
                                },
                            },

                            &menu::Item {
                                command: "write",
                                help: Some("Write flash mem"),
                                item_type: menu::ItemType::Callback {
                                    function: cmd_flash_read_write,
                                    parameters: &[
                                        menu::Parameter::Mandatory {
                                            parameter_name: "addr",
                                            help: Some("Start address of flash write"),
                                        },
                                        menu::Parameter::Optional {
                                            parameter_name: "len",
                                            help: Some("Write length in bytes"),
                                        },
                                        menu::Parameter::Optional {
                                            parameter_name: "data",
                                            help: Some("Fill byte, default is randomized"),
                                        },
                                    ],
                                },
                            },

                            &menu::Item {
                                command: "erase",
                                help: Some("Erase flash mem"),
                                item_type: menu::ItemType::Callback {
                                    function: cmd_flash_erase,
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
                    },
                ),
            }
        ],
    };

    fn cmd_status(
        _menu: &menu::Menu<MyMenuCtx>,
        _item: &menu::Item<MyMenuCtx>,
        _args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let pws = &mut ctx.pwd_store;
        pws.status(&mut ctx.serial);
    }

    fn cmd_init(
        _menu: &menu::Menu<MyMenuCtx>,
        _item: &menu::Item<MyMenuCtx>,
        _args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        if ctx.pwd_store.is_initialized() {
            write!(
                ctx,
                "Error: will not overwrite existing master key.\r\n\
                If you want a fresh start, issue these commands from main menu:\r\n\
                flash\r\n\
                erase 0 0x800000\r\n\
                exit\r\n"
            )
            .ok();
            return;
        }
        ctx.idx1 = 0;
        ctx.idx2 = 0;
        ctx.idx3 = 0;
        write!(ctx, "master password:").ok();
        ctx.init = InitState::AskPass1;
    }

    fn cmd_open(
        _menu: &menu::Menu<MyMenuCtx>,
        _item: &menu::Item<MyMenuCtx>,
        _args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        ctx.idx1 = 0;
        ctx.idx2 = 0;
        ctx.idx3 = 0;
        write!(ctx, "master password:").ok();
        ctx.open = OpenState::AskPass;
    }

    fn cmd_close(
        _menu: &menu::Menu<MyMenuCtx>,
        _item: &menu::Item<MyMenuCtx>,
        _args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let pws = &mut ctx.pwd_store;
        pws.close(&mut ctx.serial);
    }

    fn cmd_store(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let name = if let Ok(Some(aa)) = menu::argument_finder(item, args, "name") {
            aa
        } else {
            write!(ctx, "Name not given.\r\n").ok();
            return;
        };

        ctx.opt_str = Some(name.to_string());
        ctx.idx1 = 0;
        ctx.idx2 = 0;
        ctx.idx3 = 0;
        write!(ctx, "username: ").ok();
        ctx.store = StoreState::AskUser;
    }

    fn cmd_fetch(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let loc = if let Ok(Some(aa)) = menu::argument_finder(item, args, "loc") {
            if let Some(a) = parse_num(ctx, aa) {
                a
            } else {
                write!(ctx, "Could not parse loc: \"{aa}\".\r\n").ok();
                return;
            }
        } else {
            write!(ctx, "Location not given.\r\n").ok();
            return;
        };

        let pws = &mut ctx.pwd_store;
        pws.fetch(&mut ctx.serial, loc);
    }

    fn cmd_drop(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let loc = if let Ok(Some(aa)) = menu::argument_finder(item, args, "loc") {
            if let Some(a) = parse_num(ctx, aa) {
                a
            } else {
                write!(ctx, "Could not parse loc: \"{aa}\".\r\n").ok();
                return;
            }
        } else {
            write!(ctx, "Location not given.\r\n").ok();
            return;
        };

        let pws = &mut ctx.pwd_store;
        pws.drop(&mut ctx.serial, loc);
    }

    fn cmd_scan(
        _menu: &menu::Menu<MyMenuCtx>,
        _item: &menu::Item<MyMenuCtx>,
        _args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let pws = &mut ctx.pwd_store;
        pws.scan(&mut ctx.serial);
    }

    fn cmd_list(
        _menu: &menu::Menu<MyMenuCtx>,
        _item: &menu::Item<MyMenuCtx>,
        _args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let pws = &mut ctx.pwd_store;
        pws.list(&mut ctx.serial);
    }

    fn cmd_search(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let srch = if let Ok(Some(term)) = menu::argument_finder(item, args, "srch") {
            term
        } else {
            write!(ctx, "Search string not given.\r\n").ok();
            return;
        };

        let pws = &mut ctx.pwd_store;
        pws.search(&mut ctx.serial, srch.as_bytes());
    }

    fn cmd_wait(
        _menu: &menu::Menu<MyMenuCtx>,
        _item: &menu::Item<MyMenuCtx>,
        _args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        ctx.idx1 = 0;
        ctx.idx2 = 0;
        ctx.idx3 = 0;
        write!(ctx, "username: ").ok();
        ctx.store = StoreState::AskUser;
    }

    fn parse_radix(ctx: &mut MyMenuCtx, addr: &str, radix: u32) -> Option<usize> {
        match usize::from_str_radix(addr, radix) {
            Err(e) => {
                write!(ctx, "Address (radix {radix}) parse error: {e:?}\r\n").ok();
                None
            }
            Ok(a) => Some(a),
        }
    }

    fn parse_num(ctx: &mut MyMenuCtx, num: &str) -> Option<usize> {
        if let Some(hex) = num.strip_prefix("0x") {
            // Hex parse
            parse_radix(ctx, hex, 16)
        } else if num.starts_with('0') && num.len() > 1 {
            // Octal parse
            parse_radix(ctx, &num[1..], 8)
        } else {
            // Decimal parse
            parse_radix(ctx, num, 10)
        }
    }

    const HEX_BUF_SZ: usize = 256;
    fn cmd_flash_read_write(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let mode_write = item.command == "write";

        let addr = if let Ok(Some(aa)) = menu::argument_finder(item, args, "addr") {
            if let Some(a) = parse_num(ctx, aa) {
                a
            } else {
                write!(ctx, "Could not parse addr: \"{aa}\".\r\n").ok();
                return;
            }
        } else {
            write!(ctx, "Address not given.\r\n").ok();
            return;
        };
        if addr >= FLASH_SIZE {
            write!(
                ctx,
                "Error: addr {addr} (0x{addr:x}) is larger than flash size {FLASH_SIZE} (0x{FLASH_SIZE:x}).\r\n",
            )
            .ok();
            return;
        }

        let mut len = HEX_BUF_SZ;
        if let Ok(Some(al)) = menu::argument_finder(item, args, "len") {
            if let Some(ret) = parse_num(ctx, al) {
                len = ret;
            } else {
                write!(ctx, "Could not parse len: \"{al}\".\r\n").ok();
                return;
            }
        }
        if addr + len > FLASH_SIZE {
            let new_len = FLASH_SIZE - addr;
            write!(
                ctx,
                "Warning: len {len} (0x{len:x}) reaches beyond flash size {FLASH_SIZE} (0x{FLASH_SIZE:x}) \
                and was trucated to {new_len} (0x{new_len:x}).\r\n",
            )
            .ok();
            len = new_len;
        }

        let mut fill_byte: Option<u8> = None;
        if let Ok(Some(data_s)) = menu::argument_finder(item, args, "data") {
            if let Some(ret) = parse_num(ctx, data_s) {
                fill_byte = Some(ret as u8);
            } else {
                write!(ctx, "Could not parse data: \"{data_s}\".\r\n").ok();
                return;
            }
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
            "\r\n* {}ing {len} bytes at 0x{addr:06x} in {chunks} chunks, last {last_sz} bytes:\r\n",
            if mode_write { "Writ" } else { "Read" },
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
                if let Some(fill_b) = fill_byte {
                    // data byte to use for filling was given
                    buf.iter_mut().for_each(|b| *b = fill_b);
                } else {
                    // fill buf with random bytes
                    rng.fill_bytes(&mut buf[..mem_len]);
                }

                hex_dump(ctx, mem_addr, &buf[..mem_len]);
                match ctx
                    .pwd_store
                    .flash
                    .write_bytes(mem_addr as u32, &mut buf[..mem_len])
                {
                    Ok(w) => {
                        write!(ctx, "#wait {w}\r\n").ok();
                    }
                    Err(e) => {
                        write!(
                            ctx,
                            "\r\n### Flash write failed at 0x{mem_addr:06x} ({e:?}) - abort.\r\n",
                        )
                        .ok();
                        return;
                    }
                }
            } else {
                if let Err(e) = ctx
                    .pwd_store
                    .flash
                    .read(mem_addr as u32, &mut buf[..mem_len])
                {
                    write!(
                        ctx,
                        "\r\n### Flash read failed at 0x{mem_addr:06x} ({e:?}) - abort.\r\n",
                    )
                    .ok();
                    return;
                }
                hex_dump(ctx, mem_addr, &buf[..mem_len]);
            }
            // prevent hardware reset during lengthy flash ops
            ctx.pwd_store.watchdog.feed();
        }
        write!(ctx, "\r\n").ok();
    }

    fn cmd_flash_erase(
        _menu: &menu::Menu<MyMenuCtx>,
        item: &menu::Item<MyMenuCtx>,
        args: &[&str],
        ctx: &mut MyMenuCtx,
    ) {
        let jedec_id = ctx.pwd_store.flash.read_jedec_id().unwrap();
        write!(ctx, "\r\nFlash jedec id: {jedec_id:?}\r\n").ok();

        let addr = if let Ok(Some(aa)) = menu::argument_finder(item, args, "addr") {
            if let Some(a) = parse_num(ctx, aa) {
                a
            } else {
                write!(ctx, "Could not parse addr: \"{aa}\".\r\n").ok();
                return;
            }
        } else {
            write!(ctx, "Address not given.\r\n").ok();
            return;
        };
        if addr % FLASH_BLOCK_SIZE != 0 {
            write!(
                ctx,
                "Error: addr {addr} (0x{addr:x}) is not multiple of \
                {FLASH_BLOCK_SIZE} (0x{FLASH_BLOCK_SIZE:02x}).\r\n",
            )
            .ok();
            return;
        }
        if addr >= FLASH_SIZE {
            write!(
                ctx,
                "Error: addr {addr} (0x{addr:x}) is larger than flash size \
                {FLASH_SIZE} (0x{FLASH_SIZE:x}).\r\n",
            )
            .ok();
            return;
        }

        let mut len = FLASH_BLOCK_SIZE;
        if let Ok(Some(al)) = menu::argument_finder(item, args, "len") {
            if let Some(ret) = parse_num(ctx, al) {
                len = ret;
            } else {
                write!(ctx, "Could not parse len: \"{al}\".\r\n").ok();
                return;
            }
        }
        if len % FLASH_BLOCK_SIZE != 0 {
            write!(
                ctx,
                "Error: len {len} (0x{len:x}) is not multiple of \
                {FLASH_BLOCK_SIZE} (0x{FLASH_BLOCK_SIZE:x}).\r\n",
            )
            .ok();
            return;
        }
        if addr + len > FLASH_SIZE {
            let new_len = FLASH_SIZE - addr;
            write!(
                ctx,
                "Warning: len {len} (0x{len:x}) reaches beyond flash size \
                {FLASH_SIZE} (0x{FLASH_SIZE:x}) and was trucated to \
                {new_len} (0x{new_len:x}).\r\n",
            )
            .ok();
            len = new_len;
        }

        let sectors = len / FLASH_BLOCK_SIZE;
        write!(
            ctx,
            "* Erasing {len} (0x{len:x}) bytes at 0x{addr:06x} in {sectors} sectors:\r\n",
        )
        .ok();

        for c in 0..sectors {
            let mem_addr = addr + c * FLASH_BLOCK_SIZE;
            match ctx.pwd_store.flash.erase_sectors(mem_addr as u32, 1) {
                Ok(w) => {
                    write!(ctx, "\r#e 0x{mem_addr:06x} #w {w}      ").ok();
                }
                Err(e) => {
                    write!(
                        ctx,
                        "\r\n### Flash erase failed at 0x{mem_addr:06x} ({e:?}) - abort.\r\n",
                    )
                    .ok();
                    return;
                }
            }
            // prevent hardware reset during lengthy flash ops
            ctx.pwd_store.watchdog.feed();
        }
        write!(ctx, "\r\ndone.\r\n").ok();
    }
}
// EOF
