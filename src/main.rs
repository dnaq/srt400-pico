#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

#[rtic::app(device = rp_pico::pac)]
mod app {
    use core::fmt::{self, Write};
    use embedded_hal::digital::v2::OutputPin;
    use rp_pico::hal::{
        self,
        clocks::Clock,
        gpio::{FunctionPio0, Pin},
        pio::{PIOExt, PinState},
    };

    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::hid_class::HIDClass;
    use usbd_serial::SerialPort;

    /// Wrapper around a usb-cdc SerialPort
    /// to be able to use the `write!()` macro with it
    pub struct DebugPort<'a>(SerialPort<'a, hal::usb::UsbBus>);
    impl<'a> Write for DebugPort<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let _ = self.0.write(s.as_bytes());
            Ok(())
        }
    }

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        debug_port: DebugPort<'static>,
        hid_device: HIDClass<'static, hal::usb::UsbBus>,
    }

    #[local]
    struct Local {
        /// The RX fifo of the PIO state machine
        rx: rp_pico::hal::pio::Rx<(rp_pico::pac::PIO0, rp_pico::hal::pio::SM0)>,
    }

    #[init(local = [
        // any local for `init()` will have a static lifetime
        usb_bus: Option<usb_device::bus::UsbBusAllocator<hal::usb::UsbBus>> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {

        // rtic exposes the pac as `cx.device`
        let mut pac = cx.device;

        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
        // The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Set up the USB driver
        let usb_bus = cx
            .local
            .usb_bus
            .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
                pac.USBCTRL_REGS,
                pac.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut pac.RESETS,
            )));

        // Set up the USB Communications Class Device driver for debugging
        let debug_port = DebugPort(SerialPort::new(usb_bus));

        // This is the hid-descriptor for the plover-hid protocol
        // see https://github.com/dnaq/plover-machine-hid
        const DESCRIPTOR: &[u8] = &[
            0x06, 0x50, 0xff, // UsagePage (65360)
            0x0a, 0x56, 0x4c, // Usage (19542)
            0xa1, 0x02, // Collection (Logical)
            0x85, 0x50, //     ReportID (80)
            0x25, 0x01, //     LogicalMaximum (1)
            0x75, 0x01, //     ReportSize (1)
            0x95, 0x40, //     ReportCount (64)
            0x05, 0x0a, //     UsagePage (ordinal)
            0x19, 0x00, //     UsageMinimum (Ordinal(0))
            0x29, 0x3f, //     UsageMaximum (Ordinal(63))
            0x81, 0x02, //     Input (Variable)
            0xc0, // EndCollection
        ];
        let hid_device = HIDClass::new(usb_bus, DESCRIPTOR, 20);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Stentura")
            .product("SRT400")
            .serial_number("TEST")
            .device_class(0)
            .build();

        // set up the pins in the right state for the pio
        // state machine
        let sio = hal::Sio::new(pac.SIO);
        let pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        let _clk: Pin<_, FunctionPio0> = pins.gpio0.into_mode();
        let clk_pin_id = 0;
        let _data_in: Pin<_, FunctionPio0> = pins.gpio1.into_mode();
        let data_in_pin_id = 1;
        let _latch_en: Pin<_, FunctionPio0> = pins.gpio2.into_mode();
        let latch_en_pin_id = 2;
        let mut test_en = pins.gpio3.into_push_pull_output();
        test_en.set_low().unwrap();

        // assemble the pio program for scanning the SRT400 state machine
        let keyscan = pio_proc::pio_file!("./src/keyscan.pio", select_program("keyscan"));

        // and install it to PIO0
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let installed = pio.install(&keyscan.program).unwrap();
        // we want to run the statemachine at 30MHz
        // since that's the max clock speed of the SRT400
        // shift registers
        let divisor = f64::from(clocks.reference_clock.freq().0) / 30.0;
        let (mut sm, rx, _tx) = hal::pio::PIOBuilder::from_program(installed)
            .side_set_pin_base(clk_pin_id)
            .in_pin_base(data_in_pin_id)
            .set_pins(latch_en_pin_id, 1)
            .clock_divisor(divisor as f32)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .build(sm0);
        sm.set_pindirs([
            (clk_pin_id, hal::pio::PinDir::Output),
            (data_in_pin_id, hal::pio::PinDir::Input),
            (latch_en_pin_id, hal::pio::PinDir::Output),
        ]);
        sm.set_pins([
            (clk_pin_id, PinState::Low),
            (latch_en_pin_id, PinState::Low),
        ]);
        // and we want the pio to trigger interrupt 0
        // whenever there is data in the rx fifo
        pio.interrupts()[0].enable_rx_not_empty_interrupt(0);
        let _sm = sm.start();

        (
            Shared {
                usb_device,
                debug_port,
                hid_device,
            },
            Local { rx },
            init::Monotonics(),
        )
    }

    /// This task is reponsible for polling for USB events
    /// whenever the USBCTRL_IRQ is raised
    #[task(binds = USBCTRL_IRQ, shared = [usb_device, debug_port, hid_device])]
    fn usbctrl_irq(cx: usbctrl_irq::Context) {
        (
            cx.shared.usb_device,
            cx.shared.debug_port,
            cx.shared.hid_device,
        )
            .lock(|usb_device, debug_port, hid_device| {
                if usb_device.poll(&mut [&mut debug_port.0, hid_device]) {
                    // DEBUG functionality, if "b" is received
                    // over the debug serial port we reser to bootloader
                    let mut buf = [0u8; 64];
                    if let Ok(x) = debug_port.0.read(&mut buf) {
                        let bytes = &buf[..x];
                        if bytes == b"b" {
                            hal::rom_data::reset_to_usb_boot(0, 0);
                        }
                    }
                }
            });
    }

    /// This task is reponsible for reading data from the rx fifo
    /// of pio0, and sending HID reports to the connected computer
    #[task(binds = PIO0_IRQ_0, shared = [debug_port, hid_device], local = [
        rx,
        report: [u8; 9] = [80, 0, 0, 0, 0, 0, 0, 0, 0],
    ])]
    fn pio0_irq_0(mut cx: pio0_irq_0::Context) {
        use byteorder::{BigEndian, ByteOrder};
        while let Some(pio_report) = cx.local.rx.read() {
            let _ = cx
                .shared
                .debug_port
                .lock(|dp| write!(dp, "got report: 0b{pio_report:032b}\r\n"));
            BigEndian::write_u32(&mut cx.local.report[1..], !pio_report);
            cx.local.report[3] &= 0xfe;
            cx.local.report[4] = 0;
            // FIXME: error handling when pushing reports
            cx.shared.hid_device.lock(|hid| {
                if let Err(e) = hid.push_raw_input(cx.local.report) {
                    let _ = cx
                        .shared
                        .debug_port
                        .lock(|dp| write!(dp, "got error when pushing hid: {e:?}\r\n"));
                }
            });
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}

// End of file
