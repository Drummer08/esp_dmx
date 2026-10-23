#![no_std]

use core::num::NonZeroU16;

use esp_hal::delay::Delay;
use esp_hal::gpio::interconnect::PeripheralOutput;

use esp_hal::uart::{Config, DataBits, Instance, Parity, StopBits, Uart};
use esp_hal::{Blocking, DriverMode};

#[cfg(feature = "async")]
use esp_hal::Async;

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs;

const DEFAULT_BREAK_BITS: u32 = 25;
const DEFAULT_MAB_US: u32 = 20;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmxError {
    ChannelOutOfRange,
}

pub struct Dmx<'d, Dm: DriverMode> {
    uart: Uart<'d, Dm>,
    universe: [u8; Channel::MAX as usize + 1],
    break_bits: u32,
    mab_us: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel(NonZeroU16);

impl Channel {
    pub const MIN: u16 = 1;
    pub const MAX: u16 = 512;

    pub fn new(value: u16) -> Result<Self, DmxError> {
        Self::try_from(value)
    }
}

impl From<Channel> for usize {
    fn from(channel: Channel) -> Self {
        channel.0.get() as usize
    }
}

impl TryFrom<u16> for Channel {
    type Error = DmxError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let value = NonZeroU16::new(value).ok_or(DmxError::ChannelOutOfRange)?;
        if value.get() > Channel::MAX {
            return Err(DmxError::ChannelOutOfRange);
        }
        Ok(Channel(value))
    }
}

impl<'d> Dmx<'d, Blocking> {
    pub fn new(
        uart: impl Instance + 'd,
        tx: impl PeripheralOutput<'d>,
        break_bits: u32,
        mab_us: u32,
    ) -> Self {
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

        let universe = [0u8; Channel::MAX as usize + 1];

        Self {
            uart,
            universe,
            break_bits,
            mab_us,
        }
    }

    pub fn with_default_timing(uart: impl Instance + 'd, tx: impl PeripheralOutput<'d>) -> Self {
        Self::new(uart, tx, DEFAULT_BREAK_BITS, DEFAULT_MAB_US)
    }

    #[cfg(feature = "async")]
    pub fn into_async(self) -> Dmx<'d, Async> {
        Dmx {
            uart: self.uart.into_async(),
            universe: self.universe,
            break_bits: self.break_bits,
            mab_us: self.mab_us,
        }
    }

    pub fn send_frame(&mut self, delay: &Delay) {
        // BREAK: for 100 µs
        self.uart.send_break(self.break_bits);

        delay.delay_micros(self.mab_us);

        // --- DMX data ---
        self.uart.write(&self.universe).unwrap();
        self.uart.flush().unwrap();
    }
}

#[cfg(feature = "async")]
impl<'d> Dmx<'d, Async> {
    pub async fn send_frame_async<D: DelayNs>(&mut self, delay: &mut D) {
        // BREAK: for 100 µs
        self.uart.send_break_async(delay, self.break_bits).await;

        delay.delay_us(self.mab_us).await;

        // --- DMX data ---
        self.uart.write_async(&self.universe).await.unwrap();
        self.uart.flush_async().await.unwrap();
    }
}

impl<'d, Dm> Dmx<'d, Dm>
where
    Dm: DriverMode,
{
    pub fn set_universe(&mut self, universe: [u8; Channel::MAX as usize]) {
        self.universe[0] = 0;
        self.universe[1..=Channel::MAX as usize].copy_from_slice(&universe);
    }

    pub fn clear(&mut self) {
        self.universe[1..].fill(0);
    }

    pub fn fill(&mut self, value: u8) {
        self.universe[1..=Channel::MAX as usize].fill(value);
    }

    pub fn channels(&self) -> &[u8] {
        &self.universe[1..]
    }

    pub fn channels_mut(&mut self) -> &mut [u8] {
        &mut self.universe[1..]
    }

    pub fn set_channel(&mut self, channel: Channel, value: u8) {
        self.universe[usize::from(channel)] = value;
    }

    pub fn get_channel(&self, channel: Channel) -> u8 {
        self.universe[usize::from(channel)]
    }

    pub fn set_channels(&mut self, start_channel: Channel, values: &[u8]) -> Result<(), DmxError> {
        let start: usize = usize::from(start_channel);
        let end = start + values.len();
        if end > Channel::MAX as usize + 1 {
            return Err(DmxError::ChannelOutOfRange);
        }
        self.universe[start..end].copy_from_slice(values);
        Ok(())
    }

    pub fn set_rgb(&mut self, start_channel: Channel, r: u8, g: u8, b: u8) -> Result<(), DmxError> {
        self.set_channels(start_channel, &[r, g, b])
    }

    pub fn set_break_bits(&mut self, break_bits: u32) {
        self.break_bits = break_bits;
    }

    pub fn break_bits(&self) -> u32 {
        self.break_bits
    }

    pub fn set_mab_us(&mut self, mab_us: u32) {
        self.mab_us = mab_us;
    }

    pub fn mab_us(&self) -> u32 {
        self.mab_us
    }
}
