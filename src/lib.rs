#![doc = include_str!("../README.md")]
#![no_std]
#![warn(missing_docs)]
#![cfg_attr(docsrs, feature(doc_cfg))]

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

/// Errors returned by this crate.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum DmxError {
    /// A channel number outside `1..=512`, or a write that would run past channel 512.
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

/// Length of the break: the low period that marks the start of a frame.
///
/// Stored in microseconds. Values below [`Break::MIN_US`] are raised to that minimum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Break(u32);

impl Break {
    /// Minimum break a transmitter should send, in microseconds (DMX512 spec).
    pub const MIN_US: u32 = 92;

    /// Creates a break of `us` microseconds, raised to [`Break::MIN_US`] if shorter.
    pub fn from_micros(us: u32) -> Self {
        if us < Self::MIN_US {
            Self(Self::MIN_US)
        } else {
            Self(us)
        }
    }

    /// Break length in UART bit times, rounded up so it is never shorter than requested.
    pub(crate) fn bits(self) -> u32 {
        self.0.div_ceil(BIT_US)
    }

    /// Returns the break length in microseconds.
    pub fn as_micros(self) -> u32 {
        self.0
    }
}

/// Length of the mark after break (MAB): the high period between the break and the start code.
///
/// Stored in microseconds. Values below [`Mab::MIN_US`] are raised to that minimum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Mab(u32);

impl Mab {
    /// Minimum mark after break a transmitter should send, in microseconds (DMX512 spec).
    pub const MIN_US: u32 = 12;

    /// Creates a mark after break of `us` microseconds, raised to [`Mab::MIN_US`] if shorter.
    pub fn from_micros(us: u32) -> Self {
        if us < Self::MIN_US {
            Self(Self::MIN_US)
        } else {
            Self(us)
        }
    }

    /// Returns the mark after break in microseconds.
    pub fn as_micros(self) -> u32 {
        self.0
    }
}

/// Break and mark-after-break timing of a frame.
///
/// A DMX frame starts with:
///
/// - [`Break`] (low)
/// - [`Mab`] (high)
/// - Start code
/// - Channel data
///
/// [`Timing::default`] uses a 176 µs break and a 12 µs mark after break.
///
/// # Examples
///
/// ```
/// use esp_dmx::{Timing, Break, Mab};
///
/// let timing = Timing::default();
/// assert_eq!(timing.brk().as_micros(), 176);
/// assert_eq!(timing.mab().as_micros(), 12);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    brk: Break,
    mab: Mab,
}

impl Timing {
    /// Creates a timing from a break and a mark after break.
    pub fn new(brk: Break, mab: Mab) -> Self {
        Self { brk, mab }
    }

    /// Returns the break.
    pub fn brk(&self) -> Break {
        self.brk
    }

    /// Returns the mark after break.
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

/// A DMX channel number in the range `1..=512`.
///
/// DMX channels start at [`Channel::MIN`], not 0 and go up to [`Channel::MAX`].
///
/// Slot 0 of a DMX frame is the start code and is not represented by
/// [`Channel`]. This crate always transmits a start code of `0x00`.
///
/// # Examples
///
/// ```
/// use esp_dmx::Channel;
///
/// let channel = Channel::new(1).unwrap();
/// assert_eq!(usize::from(channel), 1);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel(NonZeroU16);

impl Channel {
    /// Lowest valid channel number.
    pub const MIN: u16 = 1;
    /// Highest valid channel number.
    pub const MAX: u16 = 512;

    /// Creates a channel from a number.
    ///
    /// Returns [`DmxError::ChannelOutOfRange`] if `value` is 0 or greater than 512.
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

/// DMX512 Transmitter.
///
/// Holds one universe (512 channels plus start code) and sends it with [`Dmx::send_frame`] or,
/// with the `async` feature, [`Dmx::send_frame_async`].
///
/// Internally, one complete DMX universe is stored:
///
/// - slot 0: start code (`0x00`)
/// - slots 1..=512: DMX channels
///
/// The start code is managed by the driver and cannot be modified.
pub struct Dmx<'d, Dm: DriverMode> {
    uart: Uart<'d, Dm>,
    /// Index 0 is the start code (always 0), 1..=512 are the channels.
    universe: [u8; Channel::MAX as usize + 1],
    timing: Timing,
}

impl<'d> Dmx<'d, Blocking> {
    /// Creates a transmitter on the given UART and TX pin using the supplied [`Timing`] configuration.
    ///
    /// All channels start at 0. Returns [`DmxError::Config`] if the UART rejects the
    /// 250 kbaud 8N2 configuration.
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

    /// Creates a transmitter with the default [`Timing`].
    pub fn with_defaults(
        uart: impl Instance + 'd,
        tx: impl PeripheralOutput<'d>,
    ) -> Result<Self, DmxError> {
        Self::new(uart, tx, Timing::default())
    }

    /// Converts the transmitter to async mode.
    ///
    /// Available with the `async` feature.
    #[cfg(feature = "async")]
    #[cfg_attr(docsrs, doc(cfg(feature = "async")))]
    pub fn into_async(self) -> Dmx<'d, Async> {
        Dmx {
            uart: self.uart.into_async(),
            universe: self.universe,
            timing: self.timing,
        }
    }

    /// Sends one frame: break, mark after break, start code and 512 channels.
    ///
    /// Blocks until the last byte has left the UART. The break and the mark after break are
    /// timed with busy waits, so the CPU is occupied for the whole frame (about 23 ms).
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
    /// Sends one frame: break, mark after break, start code and 512 channels.
    ///
    /// Waits are done with `.await`, so other tasks can run in between. The break and mark
    /// after break are only as accurate as the supplied `delay` and the executor.
    ///
    /// Available with the `async` feature.
    #[cfg_attr(docsrs, doc(cfg(feature = "async")))]
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
    /// Returns the current [`Timing`].
    pub fn timing(&self) -> Timing {
        self.timing
    }

    /// Sets the [`Timing`] used for following frames.
    pub fn set_timing(&mut self, timing: Timing) {
        self.timing = timing;
    }

    /// Replaces all 512 channel values. Index 0 of the array is channel 1.
    pub fn set_universe(&mut self, universe: [u8; Channel::MAX as usize]) {
        self.universe[1..].copy_from_slice(&universe);
    }

    /// Sets all channels to 0.
    pub fn clear(&mut self) {
        self.universe[1..].fill(0);
    }

    /// Sets all channels to `value`.
    pub fn fill(&mut self, value: u8) {
        self.universe[1..].fill(value);
    }

    /// Returns the 512 channel values. Index 0 of the slice is channel 1.
    pub fn channels(&self) -> &[u8] {
        &self.universe[1..]
    }

    /// Returns the 512 channel values mutably. Index 0 of the slice is channel 1.
    pub fn channels_mut(&mut self) -> &mut [u8] {
        &mut self.universe[1..]
    }

    /// Sets a single [`Channel`] to the given value.
    pub fn set_channel(&mut self, channel: Channel, value: u8) {
        self.universe[usize::from(channel)] = value;
    }

    /// Returns the value of a single [`Channel`].
    pub fn get_channel(&self, channel: Channel) -> u8 {
        self.universe[usize::from(channel)]
    }

    /// Writes consecutive values starting at `start_channel`.
    ///
    /// Returns [`DmxError::ChannelOutOfRange`] if the values would run past [`Channel::MAX`].
    /// In that case nothing is written.
    pub fn set_channels(&mut self, start_channel: Channel, values: &[u8]) -> Result<(), DmxError> {
        let start: usize = usize::from(start_channel);
        let end = start + values.len();
        if end > Channel::MAX as usize + 1 {
            return Err(DmxError::ChannelOutOfRange);
        }
        self.universe[start..end].copy_from_slice(values);
        Ok(())
    }

    /// Writes red, green and blue to three consecutive [`Channel`]s starting at `start_channel`.
    ///
    /// Returns [`DmxError::ChannelOutOfRange`] if the three channels do not fit.
    pub fn set_rgb(&mut self, start_channel: Channel, r: u8, g: u8, b: u8) -> Result<(), DmxError> {
        self.set_channels(start_channel, &[r, g, b])
    }
}
