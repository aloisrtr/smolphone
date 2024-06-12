//! # SmolPhone butler code
//! This is the firmware that runs on the RP2040-based micro-controller of the
//! SmolPhone project.

#![no_std]
#![no_main]

use hal::pll::{Locked, PhaseLockedLoop};
// General HAL
use hal::Clock;
use rp2040_hal as hal;
use rp2040_pac::{PLL_SYS, RESETS};
use usbd_serial::embedded_io::Write;

use core::sync::atomic::{AtomicBool, Ordering};
use core::{option::*, result::*};

use hal::gpio;

use fugit::{HertzU32, RateExtU32};

use embedded_hal::digital::OutputPin;

// Useful for the display and Slint
use embedded_graphics::{
    mono_font::{ascii::FONT_9X15, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use sharp_memory_display::*;

// Debugging
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

/// Boot section
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const CRISTAL_OSCILLATOR_FREQUENCY: HertzU32 = HertzU32::Hz(12_000_000u32);

struct SharedState {
    // Pins
    led_pin: gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioOutput, gpio::PullDown>,
    performance_core_on_pin:
        gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionSioOutput, gpio::PullDown>,
    waker_pin: gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullDown>,

    // General clocks
    xosc: Option<hal::xosc::CrystalOscillator<hal::xosc::Stable>>,
    delay: cortex_m::delay::Delay,
    pll_sys: Option<PhaseLockedLoop<Locked, PLL_SYS>>,
    rtc: hal::rtc::RealTimeClock,
    resets: RESETS,

    // Display
    display: MemoryDisplay<
        hal::spi::Spi<
            hal::spi::Enabled,
            rp2040_pac::SPI0,
            (
                gpio::Pin<gpio::bank0::Gpio3, gpio::FunctionSpi, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSpi, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionSpi, gpio::PullDown>,
            ),
            8,
        >,
        gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionSioOutput, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionSioOutput, gpio::PullDown>,
    >,
    line: u8,
}
impl SharedState {
    /// Flashes the onboard LED for a given amount of time.
    pub fn flash_led(&mut self, ms: u32) {
        self.led_pin.set_high().unwrap();
        self.wait(ms);
        self.led_pin.set_low().unwrap();
    }

    /// Active wait for the given amount of time.
    pub fn wait(&mut self, ms: u32) {
        self.delay.delay_ms(ms)
    }

    /// Sleeps until interrupt.
    pub fn sleep(&mut self) {
        cortex_m::asm::wfi();
    }

    /// Sleeps for a given amount of seconds or until interrupt.
    pub fn sleep_seconds(&mut self, seconds: u8) {
        // Set up our alarm and interrupt
        let now = self.rtc.now().unwrap();
        let schedule = hal::rtc::DateTimeFilter::default().second((now.second + seconds) % 60);
        self.rtc.schedule_alarm(schedule);
        self.rtc.enable_interrupt();
        unsafe { hal::pac::NVIC::unmask(hal::pac::Interrupt::RTC_IRQ) };

        // Actually enter sleep
        cortex_m::asm::wfi();

        // Reset everything back to normal
        hal::pac::NVIC::mask(hal::pac::Interrupt::RTC_IRQ);
        self.rtc.disable_alarm();
    }

    /// Enters the deep sleep (DORMANT) state.
    pub fn deep_sleep(&mut self) {
        // Unmask our interrupt
        unsafe {
            hal::pac::NVIC::unmask(hal::pac::interrupt::IO_IRQ_BANK0);
        }

        // Set up the clocks and PLLs for dormancy
        // let dormant_pll_sys = if let Some(pll_sys) = self.pll_sys.take() {
        //     pll_sys.disable()
        // } else {
        //     panic!()
        // };
        // let dormant_pll_usb = if let Some(pll_usb) = self.pll_usb.take() {
        //     pll_usb.disable()
        // } else {
        //     panic!()
        // };

        // Stop the crystal oscillator
        let dormant_xosc = unsafe { self.xosc.take().unwrap().dormant() };

        // Once we're back from the interrupt, restart everything and clear the interrupt
        hal::pac::NVIC::mask(hal::pac::interrupt::IO_IRQ_BANK0);
        let stable_xosc_token = dormant_xosc.await_stabilization().unwrap();
        self.xosc = Some(dormant_xosc.get_stable(stable_xosc_token));
        // self.pll_sys =
        //     Some(hal::pll::start_pll_blocking(dormant_pll_sys, &mut self.resets).unwrap());
        // self.pll_usb =
        //     Some(hal::pll::start_pll_blocking(dormant_pll_usb, &mut self.resets).unwrap());
    }

    /// Overclocks the RP2040 from its nominal frequency given a multiplier.
    pub fn overclock(&mut self, freq: HertzU32) {
        let pll_device = if let Some(pll) = self.pll_sys.take() {
            pll.free()
        } else {
            defmt::panic!()
        };

        //self.pll_sys = Some(hal::pll::setup_pll_blocking(, ))
    }

    /// Turns on the performance core.
    pub fn turn_on_performance_core(&mut self) {
        self.performance_core_on_pin.set_high().unwrap();
    }
    /// Turns off the performance core.
    pub fn turn_off_performance_core(&mut self) {
        self.performance_core_on_pin.set_low().unwrap();
    }

    /// Writes text to the display.
    pub fn write_text(&mut self, text: &str) {
        let text_style = MonoTextStyle::new(&FONT_9X15, BinaryColor::Off);
        Text::new(text, Point::new(5, 20 * (self.line as i32)), text_style)
            .draw(&mut self.display)
            .unwrap();
        self.display.flush_buffer();

        self.line += 1;
        if self.line >= 10 {
            self.line = 1;
            self.display.clear_buffer();
        }
    }
}

/// Operating modes that the SmolPhone can be in at any time.
#[derive(Format)]
enum OperatingMode {
    Main,
    Butler,
    #[allow(dead_code)]
    Sleep,
    #[allow(dead_code)]
    DeepSleep,
}

// General singleton definitions.
static mut USB_DEVICE: Option<usb_device::device::UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<usb_device::class_prelude::UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<hal::usb::UsbBus>> = None;
static mut BUTLER: Option<SharedState> = None;

/// Initialization code for the MCU, needed by all modes.
///
/// This part of the code also implements [trampolining](https://en.wikipedia.org/wiki/Tail_call#Through_trampolining)
/// to switch between modes effectively without using a huge "match" statement,
/// making it easier to separate code into manageable chunks.
#[rp2040_hal::entry]
fn init() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut core = hal::pac::CorePeripherals::take().unwrap();
    core.SCB.set_sleepdeep();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks and PLLs
    let mut clocks = hal::clocks::ClocksManager::new(pac.CLOCKS);
    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, CRISTAL_OSCILLATOR_FREQUENCY).unwrap();
    watchdog.enable_tick_generation((CRISTAL_OSCILLATOR_FREQUENCY.to_Hz() / 1_000_000) as u8);

    let pll_sys = hal::pll::setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        hal::pll::common_configs::PLL_SYS_125MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();
    let pll_usb = hal::pll::setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
        hal::pll::common_configs::PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    let mut deep_sleep_clocks_config = hal::clocks::ClockGate::default();
    deep_sleep_clocks_config.set_rtc_rtc(true);
    deep_sleep_clocks_config.set_usb_usbctrl(true);
    deep_sleep_clocks_config.set_sys_usbctrl(true);
    clocks.configure_sleep_enable(deep_sleep_clocks_config);

    // Pins configuration
    let sio = hal::Sio::new(pac.SIO);
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let led_pin = pins.gpio13.into_push_pull_output();
    let performance_core_on_pin = pins.gpio15.into_push_pull_output();
    let waker_pin = pins.gpio14.reconfigure();
    waker_pin.set_dormant_wake_enabled(gpio::Interrupt::EdgeLow, true);
    waker_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Display configuration
    let sclk = pins.gpio2.into_function::<gpio::FunctionSpi>();
    let mosi = pins.gpio3.into_function::<gpio::FunctionSpi>();
    let miso = pins.gpio4.into_function::<gpio::FunctionSpi>();
    let cs = pins.gpio5.into_push_pull_output();
    let disp = pins.gpio6.into_push_pull_output();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        500.kHz(),
        sharp_memory_display::MODE,
    );

    let mut display = MemoryDisplay::new(spi, cs, disp);
    display.enable();
    display.clear();

    // Intialization wrap-up
    unsafe {
        USB_BUS = Some(usb_device::class_prelude::UsbBusAllocator::new(
            hal::usb::UsbBus::new(pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, true, &mut pac.RESETS),
        ));
        USB_SERIAL = Some(usbd_serial::SerialPort::new(USB_BUS.as_ref().unwrap()));
        USB_DEVICE = Some(
            usb_device::device::UsbDeviceBuilder::new(
                USB_BUS.as_ref().unwrap(),
                usb_device::device::UsbVidPid(0x2e8a, 0x0005),
            )
            .strings(&[usb_device::device::StringDescriptors::default()
                .manufacturer("NotAStartup Inc.")
                .product("SmolPhone Butler")])
            .unwrap()
            .device_class(2)
            .build(),
        );
        USB_DEVICE.as_mut().unwrap().set_self_powered(false);

        BUTLER = Some(SharedState {
            led_pin,
            performance_core_on_pin,
            waker_pin,
            delay,
            pll_sys: Some(pll_sys),
            rtc: hal::rtc::RealTimeClock::new(
                pac.RTC,
                clocks.rtc_clock,
                &mut pac.RESETS,
                // TODO: find a way to configure this accurately
                hal::rtc::DateTime {
                    year: 0,
                    month: 1,
                    day: 1,
                    day_of_week: hal::rtc::DayOfWeek::Monday,
                    hour: 0,
                    minute: 0,
                    second: 0,
                },
            )
            .unwrap(),
            xosc: Some(xosc),
            display,
            line: 1,
            resets: pac.RESETS,
        });
    }

    // Welcome!
    unsafe {
        BUTLER.as_mut().unwrap().write_text("Hello, SmolPhone!");
    }

    unsafe {
        BUTLER.as_mut().unwrap().flash_led(1000);
    }

    // Modes are switched here. We always boot in main mode.
    let mut current_mode = OperatingMode::Main;
    loop {
        info!("entering mode {:?}", current_mode);
        current_mode = match current_mode {
            OperatingMode::Main => matmul_main(),
            OperatingMode::Butler => butler(),
            OperatingMode::Sleep => sleep(),
            OperatingMode::DeepSleep => deep_sleep(),
        }
    }
}

fn sleep() -> OperatingMode {
    unsafe {
        BUTLER.as_mut().unwrap().sleep_seconds(10);
    }
    OperatingMode::Main
}

fn deep_sleep() -> OperatingMode {
    unsafe {
        BUTLER.as_mut().unwrap().deep_sleep();
    }
    OperatingMode::Main
}

/// Main mode runs a simple program.
fn matmul_main() -> OperatingMode {
    // Safe because interrupts are masked
    let butler = unsafe { BUTLER.as_mut().unwrap() };

    // Enter deep sleep until woken by pin, then flash the LED.
    // butler.write_text("deep sleep on main mode");
    // butler.deep_sleep();
    // butler.flash_led(500);

    // Run the butler on idle for 100 ms, then flash the LED.
    butler.write_text("idle on main mode");
    butler.sleep_seconds(1);
    butler.flash_led(500);

    // Then run a matrix multiplication of size 128x128 before flashing the LED.
    butler.write_text("matmul on main mode");
    const SIZE: usize = 128;
    const A: [u8; SIZE * SIZE] = [2u8; SIZE * SIZE];
    const B: [u8; SIZE * SIZE] = [3u8; SIZE * SIZE];
    let mut c = [0u8; SIZE * SIZE];
    matmul(&A, &B, &mut c, SIZE).unwrap();
    butler.flash_led(500);

    // Go in Butler mode
    OperatingMode::Butler
}

fn matmul(a: &[u8], b: &[u8], c: &mut [u8], size: usize) -> Result<(), ()> {
    for i in 0..size {
        for k in 0..size {
            for j in 0..size {
                c[i * size + j] =
                    c[i * size + j].wrapping_add(a[i * size + k].wrapping_mul(b[k * size + j]));
            }
        }
    }
    Ok(())
}

/// Main function that runs a Mepo instance on the performance core while
/// controlling it using the MCU (local offloading example)
// fn mepo_main() -> OperatingMode {
//     // Safe because interrupts are masked
//     let led_indicator = unsafe { LED_INDICATOR.as_mut().unwrap() };

//     let zoom = 14;

//     let mut lat = 40f32;
//     let mut lon = -70f32;

//     let mut max_lat = (lat + 1f32) % 90f32;
//     let mut max_lon = (lon + 1f32) % 180f32;
//     let mut min_lat = (lat - 1f32) % -90f32;
//     let mut min_lon = (lon - 1f32) % -180f32;

//     let mut rendered = false;
//     led_indicator.flash_led(500);

//     loop {
//         led_indicator.flash_led(100);
//         led_indicator.wait(100);
//         lat += 0.01f32;
//         lon -= 0.002f32;

//         if !rendered || lat > max_lat || lon > max_lon || lat < min_lat || lon < min_lon {
//             led_indicator.flash_led(1000);
//             // TODO: Wake performance core
//             // TODO: Wait to be enumerated

//             // Send command to update lat and lon
//             unsafe {
//                 hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
//             }
//             let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
//             let mut command: ArrayString<255> = ArrayString::new();
//             write!(
//                 &mut command,
//                 "prefset_n lat {lat:.4}; prefset_n lon {lon:.4}; prefset_n zoom {zoom};"
//             )
//             .unwrap();
//             let mut command = command.as_bytes();

//             led_indicator.flash_led(1000);
//             while !command.is_empty() {
//                 serial
//                     .write(command)
//                     .map(|len| command = &command[len..])
//                     .unwrap()
//             }
//             hal::pac::NVIC::mask(hal::pac::Interrupt::USBCTRL_IRQ);

//             // TODO: Receive result from performance core and display it
//             // TODO: Shut down performance core

//             // Update max and min longitude/latitude
//             max_lat = (lat + 1f32) % 90f32;
//             max_lon = (lon + 1f32) % 180f32;
//             min_lat = (lat - 1f32) % -90f32;
//             min_lon = (lon - 1f32) % -180f32;
//             rendered = true;
//         }
//     }

//     OperatingMode::Main
// }

/// Butler mode sets up a USB stack, wakes up the performance core,
/// then waits to be recognized.
fn butler() -> OperatingMode {
    // Unmask the USB stack's interrupts.
    unsafe {
        hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }
    let butler = unsafe { BUTLER.as_mut().unwrap() };

    BUTLER_MODE_ON.store(true, Ordering::Relaxed);

    // Wake up the performance core.
    butler.write_text("waiting for performance core");
    butler.flash_led(500);
    butler.turn_on_performance_core();

    // Keep reading as long as we don't get any stop signal
    while BUTLER_MODE_ON.load(Ordering::Relaxed) {
        butler.sleep()
    }
    // Mask the USB stack's interrupts.
    hal::pac::NVIC::mask(hal::pac::Interrupt::USBCTRL_IRQ);

    butler.write_text("end of butler mode");
    butler.flash_led(500);

    butler.turn_off_performance_core();
    butler.write_text("waiting for performance core to be unplugged (bad design soz)");
    butler.sleep_seconds(5);
    butler.write_text("assuming it has been unplugged");

    OperatingMode::Main
}

/*
 * INTERRUPTS
 */

use hal::pac::interrupt;

/// Waker pin interrupt
#[allow(non_snake_case)]
#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    info!("received IO_IRQ_BANK0 interrupt");
    BUTLER
        .as_mut()
        .unwrap()
        .waker_pin
        .clear_interrupt(gpio::Interrupt::EdgeLow);
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn RTC_IRQ() {
    info!("received RC_IRQ interrupt");
    BUTLER.as_mut().unwrap().rtc.clear_interrupt();
}

static BUTLER_MODE_ON: AtomicBool = AtomicBool::new(false);
/// USB data in interrupt.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {}
            Ok(bytes) => {
                let received = core::str::from_utf8(&buf[..bytes]).unwrap().trim();
                match received {
                    "ready" => {
                        BUTLER_MODE_ON.store(true, Ordering::Relaxed);
                        info!("entering butler mode");
                        BUTLER
                            .as_mut()
                            .unwrap()
                            .write_text("performance core ready");
                        serial.write_all(b"ready").unwrap();
                    }
                    "exit" => {
                        BUTLER_MODE_ON.store(false, Ordering::Relaxed);
                        info!("quitting butler mode");
                    }
                    "flash" => BUTLER.as_mut().unwrap().flash_led(500),
                    _ => {}
                }
            }
        }
    }
}
