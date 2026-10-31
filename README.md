# esp-dmx

DMX512 transmitter for ESP microcontrollers using esp-hal.  

esp-dmx lets ESP microcontrollers transmit DMX512 lighting data using UART and an RS-485 transceiver.  
The crate manages a complete DMX universe (512 channels), generates the required DMX timing, and supports both blocking and asynchronous transmission.

## Hardware
DMX512 uses RS-485 signalling. Therefore an RS-485 transceiver such as a
MAX3485 or compatible device must be connected between the ESP chip and the DMX socket.

This is a basic wiring with a MAX3485 chip ([detailed schematic](https://raw.githubusercontent.com/Drummer08/esp_dmx/refs/heads/main/docs/images/Schematic.svg)).

![Example wiring](https://raw.githubusercontent.com/Drummer08/esp_dmx/refs/heads/main/docs/images/esp_dmx_bb.svg)

| ESP | MAX3485 | DMX Socket |
|-----|---------|------------|
|3.3V |RE||
|3.3V |DE||
| TX  | DI ||
| 3.3V | VCC ||
|| B | 2 (Data-) |
|| A | 3 (Data+) |
| GND | GND | 1 (GND) |

## Quick Start

Blocking
```rust,ignore
use esp_dmx::{Channel, Dmx};

let mut dmx = Dmx::with_defaults(peripherals.UART1, peripherals.GPIO4).unwrap();
dmx.set_channel(Channel::new(1).unwrap(), 255);

let delay = esp_hal::delay::Delay::new();
loop {
    dmx.send_frame(&delay).unwrap();
}
```
Async using Embassy
```rust,ignore
use esp_dmx::{Channel, Dmx};
use embassy_time::Delay;

let mut dmx = Dmx::with_defaults(peripherals.UART1, peripherals.GPIO4).unwrap().into_async();
dmx.set_channel(Channel::new(1).unwrap(), 255);

let mut delay = Delay;
loop {
    dmx.send_frame_async(&mut delay).await.unwrap();
}
```


## Features

* `async`: enables `Dmx::into_async` and `Dmx::send_frame_async`.
* `esp32c3`, `esp32c6`: select a chip. Only needed when building this crate on its
  own. In an application, enable the chip feature on `esp-hal` instead.
* `unstable`: forwards to `esp-hal/unstable`. Only needed when building this crate on its
  own. In an application, the unstable feature should be enabled on `esp-hal` instead.

## Examples

- [Blocking esp32c6 project setup](https://github.com/Drummer08/esp_dmx_example)  
- [Async esp32c6 project setup](https://github.com/Drummer08/esp_dmx_example_async)

## Key Types

- `Dmx` - DMX transmitter and universe storage.
- `Channel` - A validated DMX channel number (`1..=512`).
- `Timing` - Break and mark-after-break timing configuration.
- `Break` - DMX break duration.
- `Mab` - DMX mark-after-break duration.

## Currently Tested ESP Chips

- ESP32-C3
- ESP32-C6
