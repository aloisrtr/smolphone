#![no_std]
#![no_main]
use core::sync::atomic::{AtomicBool, Ordering};

use panic_halt as _;

use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::{
    self,
    gpio::{bank0::Gpio25, Output, PushPull},
    Clock,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_serial::SerialPort;

struct Butler {
    led_pin: hal::gpio::Pin<Gpio25, Output<PushPull>>,
    delay: cortex_m::delay::Delay,
    rosc: hal::pac::ROSC,
}
impl Butler {
    pub fn flash_led(&mut self, ms: u32) {
        self.led_pin.set_high().unwrap();
        self.wait(ms);
        self.led_pin.set_low().unwrap();
    }

    pub fn wait(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    pub fn sleep(&mut self) {
        self.rosc.dormant.write(|w| unsafe { w.bits(0x636f6d61) })
    }
}

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;
static mut LED_INDICATOR: Option<Butler> = None;

/// Initialization code for the MCU, needed by all modes.
#[rp_pico::entry]
fn init() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let led_pin = pins.led.into_push_pull_output();
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let rosc = pac.ROSC;

    unsafe {
        LED_INDICATOR = Some(Butler {
            led_pin,
            delay,
            rosc,
        });

        USB_BUS = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));
        USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));
        USB_DEVICE = Some(
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x2e8a, 0x0005))
                .manufacturer("NotAStartup Inc.")
                .product("SmolPhone Butler")
                .serial_number("PQRR")
                .device_class(2)
                .build(),
        );
    }
    // We still mask interrupts since we're not currently using the stack
    hal::pac::NVIC::mask(hal::pac::Interrupt::USBCTRL_IRQ);

    loop {
        // LED flashing for 1.5 seconds means start of bench.
        unsafe { LED_INDICATOR.as_mut().unwrap().flash_led(1500) }
        main();
        // LED flashing for .5 seconds means start of butler mode bench.
        unsafe { LED_INDICATOR.as_mut().unwrap().flash_led(500) }
        butler();
    }
}

/// Main mode runs a simple program.
fn main() {
    // Safe because interrupts are masked
    let led_indicator = unsafe { LED_INDICATOR.as_mut().unwrap() };

    const SIZE: usize = 256;
    let a = [2u8; SIZE * SIZE];
    let b = [3u8; SIZE * SIZE];
    let mut c = [0u8; SIZE * SIZE];

    // TODO: Enter deep sleep

    // We first run the butler on idle for 100 ms, then flash the LED.
    led_indicator.wait(100);
    led_indicator.flash_led(100);

    // Then run a matrix multiplication of size 256x256 before flashing the LED.
    matmul(&a, &b, &mut c, SIZE).unwrap();
    led_indicator.flash_led(100);
}

fn matmul(a: &[u8], b: &[u8], c: &mut [u8], size: usize) -> Result<(), ()> {
    for i in 0..size {
        for k in 0..size {
            for j in 0..size {
                c[i * size + j] += a[i * size + k] * b[k * size + j];
            }
        }
    }
    Ok(())
}

/// Butler mode sets up a USB stack, wakes up the performance core,
/// then waits to be recognized.
fn butler() {
    // Unmask the USB stack's interrupts.
    unsafe {
        hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    // Wake up the performance core.
    // TODO

    // Wait for the performance core to recognize the device
    while !BUTLER_MODE_ON.load(Ordering::Relaxed) {}

    // Keep reading as long as we don't get any stop signal
    while BUTLER_MODE_ON.load(Ordering::Relaxed) {
        unsafe { LED_INDICATOR.as_mut().unwrap().sleep() }
    }

    // Mask the USB stack's interrupts.
    hal::pac::NVIC::mask(hal::pac::Interrupt::USBCTRL_IRQ);
}

use rp_pico::hal::pac::interrupt;

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
            Err(UsbError::WouldBlock) => {
                BUTLER_MODE_ON.store(true, Ordering::Relaxed);
            }
            Err(_e) => {}
            Ok(bytes) => {
                let received = core::str::from_utf8(&buf[..bytes]).unwrap().trim();
                match received {
                    "exit" => BUTLER_MODE_ON.store(false, Ordering::Relaxed),
                    "flash" => LED_INDICATOR.as_mut().unwrap().flash_led(100),
                    // Echo back for debugging purposes
                    _ => {
                        let mut write_ptr = received.as_bytes();
                        while !write_ptr.is_empty() {
                            serial
                                .write(write_ptr)
                                .map(|len| write_ptr = &write_ptr[len..])
                                .unwrap()
                        }
                        serial.write(b"\r\n").unwrap();
                    }
                }
            }
        }
    }
}
