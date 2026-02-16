// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(dead_code)]
// #![deny(warnings)]

extern crate alloc;
extern crate no_std_compat as std;
use blackpill_usb_pwdstore::*;
use panic_reset as _;

use alloc_cortex_m::CortexMHeap;

use rtic_monotonics::systick::prelude::*;
systick_monotonic!(Mono, 10_000);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true,
    dispatchers = [DMA2_STREAM2, DMA2_STREAM3, DMA2_STREAM4, DMA2_STREAM5, DMA2_STREAM6, DMA2_STREAM7])]
mod app {
    use super::*;
    use blackpill_usb_pwdstore::spi_memory::Flash;
    use cortex_m::asm;
    use rtic_monotonics::fugit::ExtU32;
    use rtic_sync::channel::{Receiver, Sender};
    use usb_device::prelude::*;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        cli: AppCli,
        serial_access: SerialAccess,
        app_ctx: AppCtx,
        ser_tx: SerialTx,
        addr: u32,
        pin_button: ErasedPin<Input>,
        pin_led: ErasedPin<Output<PushPull>>,
        led_on: bool,
        button_down: bool,
    }

    #[local]
    struct Local {
        feed_sender: Sender<'static, u8, 8>,
        feed_receiver: Receiver<'static, u8, 8>,
        led_sender_usb: Sender<'static, u32, 8>,
        led_sender_btn: Sender<'static, u32, 8>,
        led_receiver: Receiver<'static, u32, 8>,
    }

    #[init(local = [
        ep_memory: [u32; 1024] = [0; 1024],
        usb_bus: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None,
        serial_port: Option<UsbdSerial> = None,
        cmd_buf: [u8; 128] = [0; 128],
        hist_buf: [u8; 128] = [0; 128],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let start = cortex_m_rt::heap_start() as usize;
        let size = 32 * 1024; // in bytes
        unsafe { crate::ALLOCATOR.init(start, size) }

        let dp = cx.device;
        let rcc = dp.RCC.constrain();

        // Setup system clocks
        let hse = 25.MHz();
        let sysclk = 84.MHz();
        let mut rcc = rcc.freeze(
            hal::rcc::Config::hse(hse)
                .sysclk(sysclk)
                .require_pll48clk(),
        );

        // Initialize the monotonic
        Mono::start(cx.core.SYST, rcc.clocks.sysclk().raw());

        let mut syscfg = dp.SYSCFG.constrain(&mut rcc);
        let gpioa = dp.GPIOA.split(&mut rcc);
        let _gpiob = dp.GPIOB.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);

        let ser_tx_pin = gpioa.pa9.into_alternate::<7>();
        let _ser_rx_pin = gpioa.pa10.into_alternate::<7>();

        // default is 115200 bps, 8N1
        let ser_cfg = serial::config::Config::default().wordlength_8();
        let mut ser_tx = serial::Serial::tx(dp.USART1, ser_tx_pin, ser_cfg, &mut rcc).unwrap();
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
            sysclk = rcc.clocks.sysclk(),
            hclk = rcc.clocks.hclk()
        )
        .ok();
        write!(
            ser_tx,
            "  pclk1: {pclk1:?}\r\n  pclk2: {pclk2:?}\r\n",
            pclk1 = rcc.clocks.pclk1(),
            pclk2 = rcc.clocks.pclk2()
        )
        .ok();
        write!(
            ser_tx,
            "  pll48clk: {pll48clk:?}\r\n",
            pll48clk = rcc.clocks.pll48clk(),
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
                (Some(sck), Some(miso), Some(mosi)),
                Mode {
                    polarity: Polarity::IdleHigh,
                    phase: Phase::CaptureOnSecondTransition,
                },
                16.MHz(),
                &mut rcc,
            )
        };

        // Create a delay abstraction based on general-purpose 32-bit timer TIM5
        let delay = hal::timer::FTimerUs::new(dp.TIM5, &mut rcc).delay();
        let mut flash = Flash::init(spi, cs, delay).unwrap();

        // *** Begin USB setup ***
        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: rcc.clocks.hclk(),
        };

        let usb_bus = cx.local.usb_bus;
        usb_bus.replace(UsbBus::new(usb, cx.local.ep_memory));

        // Create serial port in init-local for 'static lifetime
        let serial_port = cx.local.serial_port;
        *serial_port = Some(usbd_serial::SerialPort::new(usb_bus.as_ref().unwrap()));
        let serial_ptr = serial_port.as_mut().unwrap() as *mut UsbdSerial;

        let usb_dev = UsbDeviceBuilder::new(
            usb_bus.as_ref().unwrap(),
            UsbVidPid(0x16c0, 0x27dd),
        )
        .strings(&[usb_device::device::StringDescriptors::default()
            .manufacturer("Siuro Hacklab")
            .product("Password Trove")
            .serial_number("4242")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        write!(ser_tx, "Flash read JEDEC id...\r\n").ok();
        let jedec_id = flash.read_jedec_id().unwrap();
        write!(ser_tx, "Flash jedec id: {jedec_id:?}\r\n\n").ok();

        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        // Start the hardware watchdog
        watchdog.start(500u32.millis());

        let pwd_store = PwdStore::new(flash, watchdog);

        // Build the embedded-cli
        let cli_writer = CliWriter { serial: serial_ptr };
        let cli = embedded_cli::cli::CliBuilder::default()
            .writer(cli_writer)
            .command_buffer(cx.local.cmd_buf.as_mut_slice())
            .history_buffer(cx.local.hist_buf.as_mut_slice())
            .build()
            .unwrap();

        let app_ctx = AppCtx {
            now: Mono::now().ticks() as u64,
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

        let serial_access = SerialAccess(serial_ptr);

        // Create channels
        let (feed_sender, feed_receiver) = rtic_sync::make_channel!(u8, 8);
        let (led_sender, led_receiver) = rtic_sync::make_channel!(u32, 8);
        let led_sender_btn = led_sender.clone();

        // Spawn async looping tasks
        periodic::spawn().unwrap();
        led_blink::spawn().unwrap();
        feed_runner::spawn().unwrap();

        (
            Shared {
                usb_dev,
                cli,
                serial_access,
                app_ctx,
                ser_tx,
                addr: 0,
                pin_button,
                pin_led,
                led_on: false,
                button_down: false,
            },
            Local {
                feed_sender,
                feed_receiver,
                led_sender_usb: led_sender,
                led_sender_btn,
                led_receiver,
            },
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
    #[task(priority = 1, shared = [app_ctx])]
    async fn periodic(mut cx: periodic::Context) {
        loop {
            cx.shared.app_ctx.lock(|ctx| {
                ctx.pwd_store.watchdog.feed();
            });
            Mono::delay(200u32.millis()).await;
        }
    }

    #[task(priority = 3, shared = [led_on, pin_led], local = [led_receiver])]
    async fn led_blink(mut ctx: led_blink::Context) {
        loop {
            if let Ok(ms) = ctx.local.led_receiver.recv().await {
                (&mut ctx.shared.led_on, &mut ctx.shared.pin_led).lock(|led_on, pin_led| {
                    pin_led.set_low();
                    *led_on = true;
                });
                Mono::delay(ms.millis()).await;
                (&mut ctx.shared.led_on, &mut ctx.shared.pin_led).lock(|led_on, pin_led| {
                    pin_led.set_high();
                    *led_on = false;
                });
            }
        }
    }

    #[task(priority = 4, binds = EXTI0, shared = [pin_button, button_down, cli, app_ctx], local = [led_sender_btn])]
    fn button(ctx: button::Context) {
        let led_sender_btn = ctx.local.led_sender_btn;

        let button::SharedResources {
            mut pin_button,
            mut button_down,
            mut cli,
            mut app_ctx,
            ..
        } = ctx.shared;

        pin_button.lock(|button| button.clear_interrupt_pending_bit());

        let debouncing = button_down.lock(|button_down| {
            if *button_down {
                return true;
            }
            *button_down = true;
            false
        });
        if debouncing {
            return;
        }

        (&mut cli, &mut app_ctx).lock(|cli, _ctx| {
            let _ = cli.write(|writer| {
                write!(writer, "\n# button #\n").ok();
                Ok(())
            });
        });
        led_sender_btn.try_send(500).ok();

        // disable button for 500ms (debounce)
        button_debounce::spawn().ok();
    }

    #[task(priority = 3, shared = [button_down])]
    async fn button_debounce(mut ctx: button_debounce::Context) {
        Mono::delay(500u32.millis()).await;
        ctx.shared.button_down.lock(|bd| *bd = false);
    }

    #[task(priority = 5, binds = OTG_FS, shared = [usb_dev, serial_access, app_ctx], local = [feed_sender, led_sender_usb])]
    fn usb_fs(cx: usb_fs::Context) {
        let feed_sender = cx.local.feed_sender;
        let led_sender_usb = cx.local.led_sender_usb;

        let usb_fs::SharedResources {
            mut usb_dev,
            mut serial_access,
            mut app_ctx,
            ..
        } = cx.shared;

        (&mut usb_dev, &mut serial_access, &mut app_ctx).lock(|usb_dev, sa, ctx| {
            let serial = unsafe { &mut *sa.0 };

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
                        feed_sender.try_send(*c).ok();
                    }
                }
            }
        });

        app_ctx.lock(|ctx| {
            ctx.pwd_store.watchdog.feed();
        });

        led_sender_usb.try_send(10).ok();
    }

    #[task(priority = 2, shared = [cli, app_ctx], local = [feed_receiver])]
    async fn feed_runner(mut ctx: feed_runner::Context) {
        loop {
            if let Ok(c) = ctx.local.feed_receiver.recv().await {
                (&mut ctx.shared.cli, &mut ctx.shared.app_ctx).lock(|cli, app_ctx| {
                    app_ctx.now = Mono::now().ticks() as u64;
                    process_cli_byte(cli, app_ctx, c);
                });
            }
        }
    }

    #[task(priority = 2, shared = [cli, app_ctx])]
    async fn task_init(ctx: task_init::Context) {
        let task_init::SharedResources {
            mut cli,
            mut app_ctx,
            ..
        } = ctx.shared;

        (&mut cli, &mut app_ctx).lock(|cli, ctx| {
            let pass1 = String::from_utf8_lossy(&ctx.buf1[..ctx.idx1]).to_string();
            let pass2 = String::from_utf8_lossy(&ctx.buf2[..ctx.idx2]).to_string();
            let _ = cli.write(|writer| {
                if pass1 != pass2 {
                    writeln!(writer, "Error: passwords are not equal.").ok();
                } else {
                    ctx.pwd_store
                        .init(writer, &pass1, Mono::now().ticks() as u64);
                }
                Ok(())
            });
        });
    }

    #[task(priority = 2, shared = [cli, app_ctx])]
    async fn task_open(ctx: task_open::Context) {
        let task_open::SharedResources {
            mut cli,
            mut app_ctx,
            ..
        } = ctx.shared;

        (&mut cli, &mut app_ctx).lock(|cli, ctx| {
            let pass = String::from_utf8_lossy(&ctx.buf1[..ctx.idx1]).to_string();
            let _ = cli.write(|writer| {
                ctx.pwd_store
                    .open(writer, &pass, Mono::now().ticks() as u64);
                Ok(())
            });
        });
    }

    #[task(priority = 2, shared = [cli, app_ctx])]
    async fn task_store(ctx: task_store::Context) {
        let task_store::SharedResources {
            mut cli,
            mut app_ctx,
            ..
        } = ctx.shared;

        (&mut cli, &mut app_ctx).lock(|cli, ctx| {
            let name = ctx.opt_str.take().unwrap();
            let user = String::from_utf8_lossy(&ctx.buf1[..ctx.idx1]).to_string();
            let pass1 = String::from_utf8_lossy(&ctx.buf2[..ctx.idx2]).to_string();
            let pass2 = String::from_utf8_lossy(&ctx.buf3[..ctx.idx3]).to_string();
            let _ = cli.write(|writer| {
                if pass1 != pass2 {
                    writeln!(writer, "Error: passwords are not equal.").ok();
                } else {
                    ctx.pwd_store.store(writer, &name, &user, &pass1);
                }
                Ok(())
            });
        });
    }
}
// EOF
