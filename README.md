# si473x

Async, `no_std` Rust driver for the Si473x AM/FM/SW/LW radio receivers. Built on `embedded-hal-async` for I2C and a generic `OutputPin` for the reset line.

## Features
- `no_std` compatible
- Async I2C command interface
- FM/AM tune and seek helpers plus tune status parsing
- Volume and mute control via device properties
- Optional `defmt` formatting support (`defmt` feature)

## Getting started
Add the crate to your project:

```toml
[dependencies]
si473x = { git = "https://github.com/alexey-papkovskiy/si473x-rs.git" }
```

The driver expects an async I2C implementation and a reset `OutputPin`. The I2C address is configurable via the const generic `A` on `Si47xxDevice` (defaults to `0x11`).

```rust
use si473x::{Si47xxDevice, Error};

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> Result<(), Error> {
    // Provide an async I2C bus and a reset pin from your HAL.
    let i2c = /* impl embedded_hal_async::i2c::I2c */;
    let reset = /* impl embedded_hal::digital::OutputPin */;

    let mut radio = Si47xxDevice::new(i2c, reset);
    radio.reset().await;
    radio.init_fm().await?;

    let status = radio.fm_tune_frequency(101.1).await?;
    // Use `status.frequency`, `status.rssi`, `status.snr`, etc.

    radio.volume_set(50).await?;
    Ok(())
}
```

AM support follows the same pattern using `init_am()` and `am()`. See `Si47xxDevice` methods for property access, tune status, and seek helpers.

## License
Licensed under either of

- Apache License, Version 2.0, (`LICENSE-APACHE` or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license (`LICENSE-MIT` or <http://opensource.org/licenses/MIT>)

at your option.
