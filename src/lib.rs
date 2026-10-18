#![no_std]

use esp_hal::delay::Delay;
use esp_hal::gpio::interconnect::PeripheralOutput;

use esp_hal::uart::{Config, DataBits, Instance, Parity, StopBits, Uart};
use esp_hal::{Blocking, DriverMode};

#[cfg(feature = "async")]
use esp_hal::Async;

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs;

const BREAK_BITS: u32 = 25;
const MAB_US: u32 = 20; // mark after break

#[derive(Debug)]
pub enum DmxError {
    ChannelOutOfRange,
}

pub struct DMX<'d, Dm: DriverMode> {
    uart: Uart<'d, Dm>,
    universe: [u8; 513],
}

impl<'d> DMX<'d, Blocking> {
    pub fn new(uart: impl Instance + 'd, tx: impl PeripheralOutput<'d>) -> Self {
        let uart = Uart::new(
            uart,
            Config::default()
                .with_baudrate(250_000)
                .with_data_bits(DataBits::_8)
                .with_parity(Parity::None)
                .with_stop_bits(StopBits::_2),
        )
        .unwrap()
        .with_tx(tx);

        let universe = [0u8; 513];

        Self { uart, universe }
    }

    #[cfg(feature = "async")]
    pub fn into_async(self) -> DMX<'d, Async> {
        DMX {
            uart: self.uart.into_async(),
            universe: self.universe,
        }
    }

    pub fn send_frame(&mut self, delay: &Delay) {
        // BREAK: for 100 µs
        self.uart.send_break(BREAK_BITS);

        delay.delay_micros(MAB_US);

        // --- DMX data ---
        self.uart.write(&self.universe).unwrap();
        self.uart.flush().unwrap();
    }
}

#[cfg(feature = "async")]
impl<'d> DMX<'d, Async> {
    pub async fn send_frame_async<D: DelayNs>(&mut self, delay: &mut D) {
        // BREAK: for 100 µs
        self.uart.send_break_async(delay, BREAK_BITS).await;

        delay.delay_us(MAB_US).await;

        // --- DMX data ---
        self.uart.write_async(&self.universe).await.unwrap();
        self.uart.flush_async().await.unwrap();
    }
}

impl<'d, Dm> DMX<'d, Dm>
where
    Dm: DriverMode,
{
    pub fn set_universe(&mut self, universe: [u8; 513]) {
        self.universe = universe;
    }

    pub fn set_channel(&mut self, channel: usize, value: u8) -> Result<(), DmxError> {
        if channel == 0 || channel > 512 {
            return Err(DmxError::ChannelOutOfRange);
        }
        self.universe[channel] = value;
        Ok(())
    }
}
