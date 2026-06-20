#![no_std]
#![allow(async_fn_in_trait)]

use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::Error as I2cErrorTrait;
use embedded_hal_async::i2c::I2c;

mod band;
mod property;

pub use band::RadioBand;
use property::{AM_ONLY_PROPERTIES, FM_ONLY_PROPERTIES, SHARED_PROPERTIES};
pub use property::{Si47xxProperty, Volume};

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// I2C communication error
    I2c(embedded_hal_async::i2c::ErrorKind),
    /// Invalid parameter provided
    InvalidParameter,
    /// Device not responding
    NoResponse,
    /// Initialization error
    InitError,
    /// Device is powered down
    PoweredDown,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Si47xxRevision {
    pub pn: u8,
    pub firmware_major: u8,
    pub firmware_minor: u8,
    pub patch_h: u8,
    pub patch_l: u8,
    pub component_major: u8,
    pub component_minor: u8,
    pub chip_revision: u8,
}

impl Si47xxRevision {
    pub fn from_bytes(data: &[u8]) -> Self {
        Self {
            pn: data[0],
            firmware_major: data[1],
            firmware_minor: data[2],
            patch_h: data[3],
            patch_l: data[4],
            component_major: data[5],
            component_minor: data[6],
            chip_revision: data[7],
        }
    }
}

#[derive(Debug, Default, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Si47xxTuneStatus {
    pub valid: bool,
    pub frequency: f32,
    pub rssi: u8,
    pub snr: u8,
    pub multipath: u8,
}

pub enum Si47xxTuneResolution {
    AmModeLsbKhz = 1,
    FmModeLsbKhz = 10,
}

impl Si47xxTuneStatus {
    const KHZ_TO_MHZ: f32 = 1000.0;
    pub const fn new() -> Self {
        Self {
            valid: false,
            frequency: 0.0,
            rssi: 0,
            snr: 0,
            multipath: 0,
        }
    }
    pub fn from_bytes(data: &[u8], resolution: Si47xxTuneResolution) -> Self {
        let freq_h: u16 = data[1] as u16;
        let freq_l: u16 = data[2] as u16;
        let freq: u16 = (freq_h << 8) | freq_l;
        Self {
            valid: (data[0] & 0x01) != 0,
            frequency: (freq as f32 * (resolution as i32 as f32)) / Self::KHZ_TO_MHZ,
            rssi: data[3],
            snr: data[4],
            multipath: data[5],
        }
    }
}

pub struct Si47xxCmd;
impl Si47xxCmd {
    // Power Up command and arguments
    pub const PWRUP_CMD: u8 = 0x01;
    pub const PWRUP_CTSIEN: u8 = 0x80;
    pub const PWRUP_GPO2OEN: u8 = 0x40;
    pub const PWRUP_PATCH: u8 = 0x20;
    pub const PWRUP_XOSCEN: u8 = 0x10;
    pub const PWRUP_FUNC_FMRECEIVE: u8 = 0x00;
    pub const PWRUP_FUNC_AM_SW_LW: u8 = 0x01;
    pub const PWRUP_OPMODE_ANALOGOUT: u8 = 0x05;
    pub const PWRUP_OPMODE_DIGITALOUT: u8 = 0x0B;
    pub const PWRUP_OPMODE_DIGITALOUT_STEREO: u8 = 0xB0;
    // Get Revision command
    pub const REVISION_GET_CMD: u8 = 0x10;
    pub const REVISION_GET_RSP_LEN: usize = 8;
    // Power Down command
    pub const PWRDOWN_CMD: u8 = 0x11;
    // Set Property command
    pub const PROPERTY_SET_CMD: u8 = 0x12;
    // Get Property command
    pub const PROPERTY_GET_CMD: u8 = 0x13;
    // FM Tune Frequency command
    pub const FM_TUNE_FREQ_CMD: u8 = 0x20;
    pub const FM_SEEK_START_CMD: u8 = 0x21;
    pub const FM_TUNE_STATUS_CMD: u8 = 0x22;
    // AM Tune Frequency command
    pub const AM_TUNE_FREQ_CMD: u8 = 0x40;
    pub const AM_SEEK_START_CMD: u8 = 0x41;
    pub const AM_TUNE_STATUS_CMD: u8 = 0x42;
    // AM Seek command arguments
    pub const SEEK_UP: u8 = 0x08;
    pub const SEEK_DOWN: u8 = 0x00;
    pub const SEEK_WRAP: u8 = 0x04;
    // Volume command
    pub const VOLUME_MIN: u8 = 0;
    pub const VOLUME_MAX: u8 = 63;
    pub const VOLUME_VALUE_MUTE: u16 = 0x03;
    pub const VOLUME_VALUE_UNMUTE: u16 = 0x00;
    // Get Int Status command
    pub const GET_INT_STATUS_CMD: u8 = 0x14;
    // Status
    pub const STATUS_RSP_SIZE: usize = 1;
    pub const STATUS_RSP_CTS: u8 = 0x80;
    pub const STATUS_RSP_ERROR: u8 = 0x40;
    pub const STATUS_RSP_RSQINT: u8 = 0x08;
    pub const STATUS_RSP_RDSINT: u8 = 0x04;
    pub const STATUS_RSP_STCINT: u8 = 0x01;
}

/// Si47xx device driver
#[derive(Debug)]
pub struct Si47xxDevice<T: I2c, R: OutputPin, const A: u8 = 0x11> {
    pub(crate) i2c: T,
    reset_pin: R,
}

/// Asynchronous high-level interface for controlling Si47xx-based radio devices.
///
/// This trait mirrors the public API of [`Si47xxRadio`] and is implemented for it,
/// allowing applications to depend on this trait instead of the concrete
/// [`Si47xxRadio`] type. This is useful when you want to:
///
/// - Write code that is generic over different Si47xx driver implementations.
/// - Abstract over the concrete radio type in higher-level components.
///
/// Most users who work directly with the driver can use [`Si47xxRadio`] itself.
/// Use [`Si47xx`] when you need a trait object or a generic bound instead of a
/// concrete type.
pub trait Si47xx {
    /// Concrete device type returned when switching to a specific mode (AM/FM).
    type Device;

    /// Retrieve the silicon and firmware revision information from the device.
    async fn revision_get(&mut self) -> Result<Si47xxRevision, Error>;
    /// Power the device down into a low-power state.
    async fn power_down(self) -> Result<Self::Device, Error>;
    /// Perform a hardware reset of the device.
    async fn reset(self) -> Self::Device;
    /// Enable audio output from the device.
    async fn sound_on(&mut self) -> Result<(), Error>;
    /// Disable audio output from the device.
    async fn sound_off(&mut self) -> Result<(), Error>;
    /// Set the absolute audio volume level.
    /// The valid range of `volume` is 0 (mute) to 100 (max).
    async fn volume_set(&mut self, volume: Volume) -> Result<(), Error>;
    /// Increase the audio volume by a fixed step (e.g., 10%).
    async fn volume_up(&mut self) -> Result<Volume, Error>;
    /// Decrease the audio volume by a fixed step (e.g., 10%).
    async fn volume_down(&mut self) -> Result<Volume, Error>;
    /// Retrieve the current tuning status, including frequency, signal strength, and quality.
    async fn tune_status_get(&mut self) -> Result<Si47xxTuneStatus, Error>;
    /// Start seeking for the next valid station in the upward frequency direction.
    async fn seek_up<F>(&mut self, callback: F) -> Result<Si47xxTuneStatus, Error>
    where
        F: Fn(Si47xxTuneStatus);
    /// Set active mode seek band.
    async fn band_set(&mut self, band: RadioBand) -> Result<(), Error>;
    /// Get active mode seek band.
    async fn band_get(&mut self) -> Result<RadioBand, Error>;
    /// Tune to a specific frequency.
    async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error>;
    /// Iterate over all properties for the current mode and call `callback` for each pair.
    async fn property_for_each<F>(&mut self, callback: F) -> Result<(), Error>
    where
        F: FnMut(Si47xxProperty, u16);
    /// Switch the device to AM mode.
    async fn am(self) -> Result<Self::Device, Error>;
    /// Switch the device to FM mode.
    async fn fm(self) -> Result<Self::Device, Error>;
}

impl<T: I2c, R: OutputPin, const A: u8> Si47xx for Si47xxRadio<T, R, A> {
    type Device = Si47xxRadio<T, R, A>;
    async fn revision_get(&mut self) -> Result<Si47xxRevision, Error> {
        self.revision_get().await
    }
    async fn power_down(self) -> Result<Self::Device, Error> {
        self.power_down().await
    }
    async fn reset(self) -> Self::Device {
        self.reset().await
    }
    async fn sound_on(&mut self) -> Result<(), Error> {
        self.sound_on().await
    }
    async fn sound_off(&mut self) -> Result<(), Error> {
        self.sound_off().await
    }
    async fn volume_set(&mut self, volume: Volume) -> Result<(), Error> {
        self.volume_set(volume).await
    }
    async fn volume_up(&mut self) -> Result<Volume, Error> {
        self.volume_up().await
    }
    async fn volume_down(&mut self) -> Result<Volume, Error> {
        self.volume_down().await
    }
    async fn tune_status_get(&mut self) -> Result<Si47xxTuneStatus, Error> {
        self.tune_status_get().await
    }
    async fn seek_up<F>(&mut self, callback: F) -> Result<Si47xxTuneStatus, Error>
    where
        F: Fn(Si47xxTuneStatus),
    {
        self.seek_up(callback).await
    }
    async fn band_set(&mut self, band: RadioBand) -> Result<(), Error> {
        self.band_set(band).await
    }
    async fn band_get(&mut self) -> Result<RadioBand, Error> {
        self.band_get().await
    }
    async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error> {
        self.tune_frequency(frequency).await
    }
    async fn property_for_each<F>(&mut self, callback: F) -> Result<(), Error>
    where
        F: FnMut(Si47xxProperty, u16),
    {
        self.property_for_each(callback).await
    }
    async fn am(self) -> Result<Self::Device, Error> {
        self.am().await
    }
    async fn fm(self) -> Result<Self::Device, Error> {
        self.fm().await
    }
}

/// Si47xx device enum for AM, FM, and Off (powered-down) states
/// This enum encapsulates the Si47xxDevice for AM and FM modes, as well as an
/// Off state, allowing users to switch between modes while using the same interface.
#[derive(Debug)]
pub enum Si47xxRadio<T: I2c, R: OutputPin, const A: u8> {
    Am(Si47xxDevice<T, R, A>),
    Fm(Si47xxDevice<T, R, A>),
    Off(Si47xxDevice<T, R, A>),
}

impl<T: I2c, R: OutputPin, const A: u8> Si47xxRadio<T, R, A> {
    pub async fn revision_get(&mut self) -> Result<Si47xxRevision, Error> {
        match self {
            Si47xxRadio::Am(device) => device.revision_get().await,
            Si47xxRadio::Fm(device) => device.revision_get().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn power_down(self) -> Result<Self, Error> {
        match self {
            Si47xxRadio::Am(mut device) => {
                device.power_down().await?;
                Ok(Si47xxRadio::Off(device))
            }
            Si47xxRadio::Fm(mut device) => {
                device.power_down().await?;
                Ok(Si47xxRadio::Off(device))
            }
            Si47xxRadio::Off(device) => Ok(Si47xxRadio::Off(device)),
        }
    }
    pub async fn reset(self) -> Self {
        let mut device = match self {
            Si47xxRadio::Am(device) => device,
            Si47xxRadio::Fm(device) => device,
            Si47xxRadio::Off(device) => device,
        };
        device.reset().await;
        Si47xxRadio::Off(device)
    }
    pub async fn sound_on(&mut self) -> Result<(), Error> {
        match self {
            Si47xxRadio::Am(device) => device.sound_on().await,
            Si47xxRadio::Fm(device) => device.sound_on().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn sound_off(&mut self) -> Result<(), Error> {
        match self {
            Si47xxRadio::Am(device) => device.sound_off().await,
            Si47xxRadio::Fm(device) => device.sound_off().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn volume_set(&mut self, volume: Volume) -> Result<(), Error> {
        match self {
            Si47xxRadio::Am(device) => device.volume_set(volume).await,
            Si47xxRadio::Fm(device) => device.volume_set(volume).await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn volume_up(&mut self) -> Result<Volume, Error> {
        match self {
            Si47xxRadio::Am(device) => device.volume_up().await,
            Si47xxRadio::Fm(device) => device.volume_up().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn volume_down(&mut self) -> Result<Volume, Error> {
        match self {
            Si47xxRadio::Am(device) => device.volume_down().await,
            Si47xxRadio::Fm(device) => device.volume_down().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn tune_status_get(&mut self) -> Result<Si47xxTuneStatus, Error> {
        match self {
            Si47xxRadio::Am(device) => device.am_tune_status_get().await,
            Si47xxRadio::Fm(device) => device.fm_tune_status_get().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn seek_up<F>(&mut self, callback: F) -> Result<Si47xxTuneStatus, Error>
    where
        F: Fn(Si47xxTuneStatus),
    {
        match self {
            Si47xxRadio::Am(device) => device.am_seek_up(callback).await,
            Si47xxRadio::Fm(device) => device.fm_seek_up(callback).await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn band_set(&mut self, band: RadioBand) -> Result<(), Error> {
        match self {
            Si47xxRadio::Am(device) => device.am_band_set(band).await,
            Si47xxRadio::Fm(device) => device.fm_band_set(band).await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn band_get(&mut self) -> Result<RadioBand, Error> {
        match self {
            Si47xxRadio::Am(device) => device.am_band_get().await,
            Si47xxRadio::Fm(device) => device.fm_band_get().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error> {
        match self {
            Si47xxRadio::Am(device) => device.am_tune_frequency(frequency).await,
            Si47xxRadio::Fm(device) => device.fm_tune_frequency(frequency).await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn property_for_each<F>(&mut self, mut callback: F) -> Result<(), Error>
    where
        F: FnMut(Si47xxProperty, u16),
    {
        match self {
            Si47xxRadio::Am(device) => {
                for property in SHARED_PROPERTIES {
                    let value = device.property_get(property).await?;
                    callback(property, value);
                }
                for property in AM_ONLY_PROPERTIES {
                    let value = device.property_get(property).await?;
                    callback(property, value);
                }
                Ok(())
            }
            Si47xxRadio::Fm(device) => {
                for property in SHARED_PROPERTIES {
                    let value = device.property_get(property).await?;
                    callback(property, value);
                }
                for property in FM_ONLY_PROPERTIES {
                    let value = device.property_get(property).await?;
                    callback(property, value);
                }
                Ok(())
            }
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn am(self) -> Result<Self, Error> {
        match self {
            Si47xxRadio::Am(_) => Ok(self),
            Si47xxRadio::Fm(device) => device.am().await,
            Si47xxRadio::Off(device) => device.am().await,
        }
    }
    pub async fn fm(self) -> Result<Self, Error> {
        match self {
            Si47xxRadio::Am(device) => device.fm().await,
            Si47xxRadio::Fm(_) => Ok(self),
            Si47xxRadio::Off(device) => device.fm().await,
        }
    }
}

impl<T: I2c, R: OutputPin, const A: u8> Si47xxDevice<T, R, A> {
    const I2C_ADDRESS: u8 = A;
    const SEEK_TIMEOUT_MS: u64 = 30000;
    const SEEK_STEP_TIMEOUT_MS: u64 = 50;
    /// Create a new Si47xxDevice driver from the given I2C peripheral and reset pin
    pub fn new(i2c: T, reset_pin: R) -> Self {
        Self { i2c, reset_pin }
    }

    pub async fn am(mut self) -> Result<Si47xxRadio<T, R, A>, Error> {
        self.power_down().await?;
        self.init_am().await?;
        Ok(Si47xxRadio::Am(self))
    }

    pub async fn fm(mut self) -> Result<Si47xxRadio<T, R, A>, Error> {
        self.power_down().await?;
        self.init_fm().await?;
        Ok(Si47xxRadio::Fm(self))
    }

    /// Get device revision information
    /// Returns `Si47xxRevision` on success
    /// Returns `Error` on failure
    pub async fn revision_get(&mut self) -> Result<Si47xxRevision, Error> {
        let args: [u8; 1] = [Si47xxCmd::REVISION_GET_CMD];
        let mut resp = [0u8; Si47xxCmd::REVISION_GET_RSP_LEN + Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        Ok(Si47xxRevision::from_bytes(
            &resp[Si47xxCmd::STATUS_RSP_SIZE..],
        ))
    }

    /// Get interrupt status
    /// Returns status byte on success
    /// Returns `Error` on failure
    /// The status byte contains flags indicating various interrupt conditions
    /// - Bit 7 (0x80): CTS (Clear To Send)
    /// - Bit 6 (0x40): ERROR
    /// - Bit 3 (0x08): RSQINT (Received Signal Quality Interrupt)
    /// - Bit 2 (0x04): RDSINT (RDS Interrupt)
    /// - Bit 0 (0x01): STCINT (Seek/Tune Complete Interrupt)
    pub async fn get_int_status(&mut self) -> Result<u8, Error> {
        let args: [u8; 1] = [Si47xxCmd::GET_INT_STATUS_CMD];
        let mut resp = [0u8; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        Ok(resp[0])
    }

    /// Wait for specific interrupt status bits to be set
    /// `mask` specifies which bits to wait for
    /// `timeout_ms` specifies the timeout in milliseconds
    /// Returns `Ok(())` if all specified bits are set within the timeout
    /// Returns `Error::NoResponse` if the timeout is reached without the bits being set
    pub async fn wait_for_status(&mut self, mask: u8, mut timeout_ms: u32) -> Result<(), Error> {
        loop {
            let status = self.get_int_status().await?;
            if status & mask == mask {
                break;
            }
            Timer::after_millis(1).await;
            timeout_ms = timeout_ms.checked_sub(1).ok_or(Error::NoResponse)?;
        }
        Ok(())
    }

    /// Set a property value
    /// `property` specifies the property to set
    /// `value` specifies the value to set
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn property_set(
        &mut self,
        property: Si47xxProperty,
        value: u16,
    ) -> Result<(), Error> {
        let property: u16 = property as u16;
        let args: [u8; 6] = [
            Si47xxCmd::PROPERTY_SET_CMD,
            0,
            (property >> 8) as u8,
            (property) as u8,
            (value >> 8) as u8,
            (value) as u8,
        ];
        let mut resp: [u8; Si47xxCmd::STATUS_RSP_SIZE] = [0; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await
    }

    /// Get a property value
    /// `property` specifies the property to get
    /// Returns property value on success
    /// Returns `Error` on failure
    pub async fn property_get(&mut self, property: Si47xxProperty) -> Result<u16, Error> {
        let property: u16 = property as u16;
        let args: [u8; 4] = [
            Si47xxCmd::PROPERTY_GET_CMD,
            0,
            (property >> 8) as u8,
            (property) as u8,
        ];
        let mut resp: [u8; 3 + Si47xxCmd::STATUS_RSP_SIZE] = [0; 3 + Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        Ok(((resp[2] as u16) << 8) | (resp[3] as u16))
    }

    /// Unmute sound
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn sound_on(&mut self) -> Result<(), Error> {
        self.property_set(
            Si47xxProperty::AudioVolumeMute,
            Si47xxCmd::VOLUME_VALUE_UNMUTE,
        )
        .await
    }

    /// Mute sound
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn sound_off(&mut self) -> Result<(), Error> {
        self.property_set(
            Si47xxProperty::AudioVolumeMute,
            Si47xxCmd::VOLUME_VALUE_MUTE,
        )
        .await
    }

    /// Set audio volume
    /// `volume` specifies the volume level (0-100%)
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn volume_set(&mut self, volume: Volume) -> Result<(), Error> {
        let volume_value = volume.to_raw_value();
        self.property_set(Si47xxProperty::AudioVolume, volume_value)
            .await
    }

    /// Get audio volume
    /// Returns the volume level in percent (0-100)
    /// Returns `Error` on failure
    pub async fn volume_get(&mut self) -> Result<Volume, Error> {
        let volume_value = self.property_get(Si47xxProperty::AudioVolume).await?;
        Volume::from_raw_value(volume_value).ok_or(Error::InvalidParameter)
    }

    /// Volume up by 10%
    /// Returns the new volume level in percent (0-100) on success
    /// Returns `Error` on failure
    pub async fn volume_up(&mut self) -> Result<Volume, Error> {
        let current_raw = self.property_get(Si47xxProperty::AudioVolume).await?;
        let volume = Volume::from_raw_value(current_raw)
            .ok_or(Error::InvalidParameter)?
            .up();
        self.property_set(Si47xxProperty::AudioVolume, volume.to_raw_value())
            .await?;
        Ok(volume)
    }

    /// Volume down by 10%
    /// Returns the new volume level in percent (0-100) on success
    /// Returns `Error` on failure
    pub async fn volume_down(&mut self) -> Result<Volume, Error> {
        let current_raw = self.property_get(Si47xxProperty::AudioVolume).await?;
        let volume = Volume::from_raw_value(current_raw)
            .ok_or(Error::InvalidParameter)?
            .down();
        self.property_set(Si47xxProperty::AudioVolume, volume.to_raw_value())
            .await?;
        Ok(volume)
    }

    /// Set AM/SW/LW seek band limits from a predefined or custom radio band.
    ///
    /// The provided `band` must not be FM and must fit in the device AM seek
    /// property range (`149..=23000` kHz).
    pub async fn am_band_set(&mut self, band: RadioBand) -> Result<(), Error> {
        if band.is_fm() {
            return Err(Error::InvalidParameter);
        }

        let bottom_khz = band.bottom_khz();
        let top_khz = band.top_khz();

        if bottom_khz > top_khz
            || !(149..=23000).contains(&bottom_khz)
            || !(149..=23000).contains(&top_khz)
        {
            return Err(Error::InvalidParameter);
        }

        self.property_set(Si47xxProperty::AmSeekBandBottom, bottom_khz as u16)
            .await?;
        self.property_set(Si47xxProperty::AmSeekBandTop, top_khz as u16)
            .await
    }

    /// Read AM/SW/LW seek band limits and map them to a known radio band when possible.
    pub async fn am_band_get(&mut self) -> Result<RadioBand, Error> {
        let bottom_khz = self.property_get(Si47xxProperty::AmSeekBandBottom).await? as u32;
        let top_khz = self.property_get(Si47xxProperty::AmSeekBandTop).await? as u32;
        Ok(RadioBand::from_bottom_top_khz(bottom_khz, top_khz))
    }

    /// Set FM seek band limits from a predefined or custom FM radio band.
    ///
    /// The provided `band` must be FM and within the device FM seek property
    /// range (`64.0..=108.0` MHz encoded as `6400..=10800` in 10 kHz units).
    pub async fn fm_band_set(&mut self, band: RadioBand) -> Result<(), Error> {
        if !band.is_fm() {
            return Err(Error::InvalidParameter);
        }

        let bottom_10khz = band.bottom_khz() / 10;
        let top_10khz = band.top_khz() / 10;

        if bottom_10khz > top_10khz
            || !(6400..=10800).contains(&bottom_10khz)
            || !(6400..=10800).contains(&top_10khz)
        {
            return Err(Error::InvalidParameter);
        }

        self.property_set(Si47xxProperty::FmSeekBandBottom, bottom_10khz as u16)
            .await?;
        self.property_set(Si47xxProperty::FmSeekBandTop, top_10khz as u16)
            .await
    }

    /// Read FM seek band limits and map them to a known FM radio band when possible.
    pub async fn fm_band_get(&mut self) -> Result<RadioBand, Error> {
        let bottom_10khz = self.property_get(Si47xxProperty::FmSeekBandBottom).await? as u32;
        let top_10khz = self.property_get(Si47xxProperty::FmSeekBandTop).await? as u32;
        Ok(RadioBand::from_bottom_top_khz(
            bottom_10khz * 10,
            top_10khz * 10,
        ))
    }

    /// Get FM current tuned status
    /// Returns `Si47xxTuneStatus` on success
    /// Returns `Error` on failure
    pub async fn fm_tune_status_get(&mut self) -> Result<Si47xxTuneStatus, Error> {
        let args: [u8; 1] = [Si47xxCmd::FM_TUNE_STATUS_CMD];
        let mut resp: [u8; 7] = [0; 7];
        self.cmd_send(&args, &mut resp).await?;
        Ok(Si47xxTuneStatus::from_bytes(
            &resp[Si47xxCmd::STATUS_RSP_SIZE..],
            Si47xxTuneResolution::FmModeLsbKhz,
        ))
    }

    /// Start FM seek up
    /// Returns `Si47xxTuneStatus` on success
    /// Returns `Error` on failure
    pub async fn fm_seek_up<F>(&mut self, callback: F) -> Result<Si47xxTuneStatus, Error>
    where
        F: Fn(Si47xxTuneStatus),
    {
        let args: [u8; 2] = [
            Si47xxCmd::FM_SEEK_START_CMD,
            Si47xxCmd::SEEK_UP | Si47xxCmd::SEEK_WRAP,
        ];
        let mut resp: [u8; Si47xxCmd::STATUS_RSP_SIZE] = [0; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        let mut timeout_ms = Self::SEEK_TIMEOUT_MS;
        loop {
            let status = self.get_int_status().await?;
            if status & Si47xxCmd::STATUS_RSP_STCINT == Si47xxCmd::STATUS_RSP_STCINT {
                break;
            }
            Timer::after_millis(Self::SEEK_STEP_TIMEOUT_MS).await;
            timeout_ms = timeout_ms
                .checked_sub(Self::SEEK_STEP_TIMEOUT_MS)
                .ok_or(Error::NoResponse)?;
            let tune_status = self.fm_tune_status_get().await?;
            callback(tune_status);
        }
        self.fm_tune_status_get().await
    }

    /// Set FM tune frequency
    /// `frequency` specifies the frequency in MHz
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn fm_tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error> {
        if !(87.5..=108.0).contains(&frequency) {
            return Err(Error::InvalidParameter);
        }
        let freq_value: u16 = (frequency * 100.0) as u16;
        let args: [u8; 4] = [
            Si47xxCmd::FM_TUNE_FREQ_CMD,
            0,
            (freq_value >> 8) as u8,
            (freq_value) as u8,
        ];
        let mut resp: [u8; Si47xxCmd::STATUS_RSP_SIZE] = [0; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        self.wait_for_status(Si47xxCmd::STATUS_RSP_STCINT, 5000)
            .await?;
        self.fm_tune_status_get().await
    }

    /// Get AM current tuned status
    /// Returns `Si47xxTuneStatus` on success
    /// Returns `Error` on failure
    pub async fn am_tune_status_get(&mut self) -> Result<Si47xxTuneStatus, Error> {
        let args: [u8; 1] = [Si47xxCmd::AM_TUNE_STATUS_CMD];
        let mut resp: [u8; 7] = [0; 7];
        self.cmd_send(&args, &mut resp).await?;
        Ok(Si47xxTuneStatus::from_bytes(
            &resp[Si47xxCmd::STATUS_RSP_SIZE..],
            Si47xxTuneResolution::AmModeLsbKhz,
        ))
    }

    /// Start AM seek up
    /// Returns `Si47xxTuneStatus` on success
    /// Returns `Error` on failure
    pub async fn am_seek_up<F>(&mut self, callback: F) -> Result<Si47xxTuneStatus, Error>
    where
        F: Fn(Si47xxTuneStatus),
    {
        let args: [u8; 2] = [
            Si47xxCmd::AM_SEEK_START_CMD,
            Si47xxCmd::SEEK_UP | Si47xxCmd::SEEK_WRAP,
        ];
        let mut resp: [u8; Si47xxCmd::STATUS_RSP_SIZE] = [0; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        let mut timeout_ms = Self::SEEK_TIMEOUT_MS;
        loop {
            let status = self.get_int_status().await?;
            if status & Si47xxCmd::STATUS_RSP_STCINT == Si47xxCmd::STATUS_RSP_STCINT {
                break;
            }
            Timer::after_millis(Self::SEEK_STEP_TIMEOUT_MS).await;
            timeout_ms = timeout_ms
                .checked_sub(Self::SEEK_STEP_TIMEOUT_MS)
                .ok_or(Error::NoResponse)?;
            let tune_status = self.am_tune_status_get().await?;
            callback(tune_status);
        }
        self.am_tune_status_get().await
    }

    /// Set AM tune frequency
    /// `frequency` specifies the frequency in MHz
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn am_tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error> {
        if !(0.52..=28.0).contains(&frequency) {
            return Err(Error::InvalidParameter);
        }
        let freq_value: u16 = (frequency * 1000.0) as u16;
        let args: [u8; 4] = [
            Si47xxCmd::AM_TUNE_FREQ_CMD,
            0,
            (freq_value >> 8) as u8,
            (freq_value) as u8,
        ];
        let mut resp: [u8; Si47xxCmd::STATUS_RSP_SIZE] = [0; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        self.wait_for_status(Si47xxCmd::STATUS_RSP_STCINT, 5000)
            .await?;
        self.am_tune_status_get().await
    }

    pub async fn reset(&mut self) {
        self.reset_pin.set_low().ok();
        Timer::after_millis(300).await;
        self.reset_pin.set_high().ok();
    }

    pub async fn cmd_send(&mut self, cmd: &[u8], resp: &mut [u8]) -> Result<(), Error> {
        self.i2c
            .write(Self::I2C_ADDRESS, cmd)
            .await
            .map_err(|e| Error::I2c(e.kind()))?;
        // Wait for CTS
        loop {
            let mut status: [u8; 1] = [0];
            Timer::after_millis(1).await;
            self.i2c
                .read(Self::I2C_ADDRESS, &mut status)
                .await
                .map_err(|e| Error::I2c(e.kind()))?;
            if status[0] & Si47xxCmd::STATUS_RSP_CTS != 0 {
                break;
            }
        }
        // Read response
        self.i2c
            .read(Self::I2C_ADDRESS, resp)
            .await
            .map_err(|e| Error::I2c(e.kind()))
    }

    pub async fn init_fm(&mut self) -> Result<(), Error> {
        let args: [u8; 3] = [
            Si47xxCmd::PWRUP_CMD,
            Si47xxCmd::PWRUP_CTSIEN
                | Si47xxCmd::PWRUP_GPO2OEN
                | Si47xxCmd::PWRUP_XOSCEN
                | Si47xxCmd::PWRUP_FUNC_FMRECEIVE,
            Si47xxCmd::PWRUP_OPMODE_ANALOGOUT,
        ];
        let mut status: [u8; 1] = [0];
        self.cmd_send(&args, &mut status).await?;
        if status[0] & Si47xxCmd::STATUS_RSP_ERROR != 0 {
            return Err(Error::InitError);
        }
        Ok(())
    }

    pub async fn init_am(&mut self) -> Result<(), Error> {
        let args: [u8; 3] = [
            Si47xxCmd::PWRUP_CMD,
            Si47xxCmd::PWRUP_CTSIEN
                | Si47xxCmd::PWRUP_GPO2OEN
                | Si47xxCmd::PWRUP_XOSCEN
                | Si47xxCmd::PWRUP_FUNC_AM_SW_LW,
            Si47xxCmd::PWRUP_OPMODE_ANALOGOUT,
        ];
        let mut status: [u8; 1] = [0];
        self.cmd_send(&args, &mut status).await?;
        if status[0] & Si47xxCmd::STATUS_RSP_ERROR != 0 {
            return Err(Error::InitError);
        }
        Ok(())
    }

    pub async fn power_down(&mut self) -> Result<(), Error> {
        let args: [u8; 1] = [Si47xxCmd::PWRDOWN_CMD];
        let mut status: [u8; 1] = [0];
        self.cmd_send(&args, &mut status).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_frequency_eq(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 0.000_1);
    }

    #[test]
    fn tune_status_from_bytes_decodes_fm_resolution() {
        let status = Si47xxTuneStatus::from_bytes(
            &[0x01, 0x28, 0x96, 42, 31, 7],
            Si47xxTuneResolution::FmModeLsbKhz,
        );

        assert!(status.valid);
        assert_frequency_eq(status.frequency, 103.9);
        assert_eq!(status.rssi, 42);
        assert_eq!(status.snr, 31);
        assert_eq!(status.multipath, 7);
    }

    #[test]
    fn tune_status_from_bytes_decodes_am_resolution() {
        let status = Si47xxTuneStatus::from_bytes(
            &[0x00, 0x06, 0x03, 55, 18, 2],
            Si47xxTuneResolution::AmModeLsbKhz,
        );

        assert!(!status.valid);
        assert_frequency_eq(status.frequency, 1.539);
        assert_eq!(status.rssi, 55);
        assert_eq!(status.snr, 18);
        assert_eq!(status.multipath, 2);
    }

    #[test]
    fn tune_status_from_bytes_uses_only_bit_zero_for_valid_flag() {
        let status = Si47xxTuneStatus::from_bytes(
            &[0xfe, 0x00, 0x00, 0, 0, 0],
            Si47xxTuneResolution::FmModeLsbKhz,
        );

        assert!(!status.valid);
    }
}
