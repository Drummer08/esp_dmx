#![no_std]

use core::num::NonZeroU16;

use esp_hal::delay::Delay;
use esp_hal::gpio::interconnect::PeripheralOutput;

use esp_hal::uart::{Config, ConfigError, DataBits, Instance, Parity, StopBits, TxError, Uart};
use esp_hal::{Blocking, DriverMode};

#[cfg(feature = "async")]
use esp_hal::Async;

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs;

/// DMX512 baud rate.
const BAUD: u32 = 250_000;
/// Duration of one UART bit in microseconds (4 µs at 250 kbaud).
const BIT_US: u32 = 1_000_000 / BAUD;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum DmxError {
    ChannelOutOfRange,
    /// The UART configuration was rejected.
    Config(ConfigError),
    /// The UART reported an error while transmitting.
    Uart(TxError),
}

impl From<ConfigError> for DmxError {
    fn from(e: ConfigError) -> Self {
        DmxError::Config(e)
    }
}

impl From<TxError> for DmxError {
    fn from(e: TxError) -> Self {
        DmxError::Uart(e)
    }
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Break(u32); // microseconds

impl Break {
    pub const MIN_US: u32 = 92;
    pub fn from_micros(us: u32) -> Self {
        if us < Self::MIN_US {
            Self(Self::MIN_US)
        } else {
            Self(us)
        }
    }

    pub(crate) fn bits(self) -> u32 {
        self.0.div_ceil(BIT_US)
    }

    pub fn as_micros(self) -> u32 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Mab(u32); // microseconds

impl Mab {
    pub const MIN_US: u32 = 12;
    pub fn from_micros(us: u32) -> Self {
        if us < Self::MIN_US {
            Self(Self::MIN_US)
        } else {
            Self(us)
        }
    }

    pub fn as_micros(self) -> u32 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    brk: Break,
    mab: Mab,
}

impl Timing {
    pub fn new(brk: Break, mab: Mab) -> Self {
        Self { brk, mab }
    }

    pub fn brk(&self) -> Break {
        self.brk
    }

    pub fn mab(&self) -> Mab {
        self.mab
    }
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            brk: Break::from_micros(176),
            mab: Mab::from_micros(12),
        }
    }
}

// ---------------------------------------------------------------------------
// Channel
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Dmx<'d, Dm: DriverMode> {
    uart: Uart<'d, Dm>,
    universe: [u8; Channel::MAX as usize + 1],
    timing: Timing,
}

impl<'d> Dmx<'d, Blocking> {
    pub fn new(
        uart: impl Instance + 'd,
        tx: impl PeripheralOutput<'d>,
        timing: Timing,
    ) -> Result<Self, DmxError> {
        let uart = Uart::new(
            uart,
            Config::default()
                .with_baudrate(BAUD)
                .with_data_bits(DataBits::_8)
                .with_parity(Parity::None)
                .with_stop_bits(StopBits::_2),
        )?
        .with_tx(tx);

        let universe = [0u8; Channel::MAX as usize + 1];

        Ok(Self {
            uart,
            universe,
            timing,
        })
    }

    pub fn with_defaults(
        uart: impl Instance + 'd,
        tx: impl PeripheralOutput<'d>,
    ) -> Result<Self, DmxError> {
        Self::new(uart, tx, Timing::default())
    }

    #[cfg(feature = "async")]
    pub fn into_async(self) -> Dmx<'d, Async> {
        Dmx {
            uart: self.uart.into_async(),
            universe: self.universe,
            timing: self.timing,
        }
    }

    pub fn send_frame(&mut self, delay: &Delay) -> Result<(), DmxError> {
        self.uart.send_break(self.timing.brk().bits());

        delay.delay_micros(self.timing.mab().as_micros());

        // --- DMX data ---
        self.uart.write(&self.universe)?;
        self.uart.flush()?;
        Ok(())
    }
}

#[cfg(feature = "async")]
impl<'d> Dmx<'d, Async> {
    pub async fn send_frame_async<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), DmxError> {
        self.uart
            .send_break_async(delay, self.timing.brk().bits())
            .await;

        delay.delay_us(self.timing.mab().as_micros()).await;

        // --- DMX data ---
        self.uart.write_async(&self.universe).await?;
        self.uart.flush_async().await?;
        Ok(())
    }
}

impl<'d, Dm> Dmx<'d, Dm>
where
    Dm: DriverMode,
{
    pub fn timing(&self) -> Timing {
        self.timing
    }

    pub fn set_timing(&mut self, timing: Timing) {
        self.timing = timing;
    }

    pub fn set_universe(&mut self, universe: [u8; Channel::MAX as usize]) {
        self.universe[1..].copy_from_slice(&universe);
    }

    pub fn clear(&mut self) {
        self.universe[1..].fill(0);
    }

    pub fn fill(&mut self, value: u8) {
        self.universe[1..].fill(value);
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
}
