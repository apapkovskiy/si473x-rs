#![no_std]
#![allow(async_fn_in_trait)]

use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::Error as I2cErrorTrait;
use embedded_hal_async::i2c::I2c;

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
}

#[repr(u16)]
pub enum Si47xxProperty {
    /// Audio Volume property
    /// Range: 0 (min) to 63 (max)
    AudioVolume = 0x4000,
    /// Audio Volume Mute property
    AudioVolumeMute = 0x4001,
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

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Si47xxTuneStatus {
    pub valid: bool,
    pub frequency: f32,
    pub rssi: u8,
    pub snr: u8,
    pub multipath: u8,
}

impl Si47xxTuneStatus {
    pub fn from_bytes(data: &[u8]) -> Self {
        let freq_h: u16 = data[1] as u16;
        let freq_l: u16 = data[2] as u16;
        let freq: u16 = (freq_h << 8) | freq_l;
        Self {
            valid: (data[0] & 0x01) != 0,
            frequency: freq as f32 / 100.0,
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

pub trait Si47xx {
    // Define trait methods here if needed
    async fn seek_up(&mut self) -> Result<Si47xxTuneStatus, Error>;
    async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error>;
}

pub struct Si47xxFm<'d, T: I2c, R: OutputPin, const A: u8> {
    device: &'d mut Si47xxDevice<T, R, A>,
}

impl<'d, T: I2c, R: OutputPin, const A: u8> Si47xxFm<'d, T, R, A> {
    pub fn new(device: &'d mut Si47xxDevice<T, R, A>) -> Self {
        Self { device }
    }
}

impl<'d, T: I2c, R: OutputPin, const A: u8> Si47xx for Si47xxFm<'d, T, R, A> {
    async fn seek_up(&mut self) -> Result<Si47xxTuneStatus, Error> {
        self.device.fm_seek_up().await
    }

    async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error> {
        self.device.fm_tune_frequency(frequency).await
    }
}

pub struct Si47xxAm<'d, T: I2c, R: OutputPin, const A: u8> {
    device: &'d mut Si47xxDevice<T, R, A>,
}

impl<'d, T: I2c, R: OutputPin, const A: u8> Si47xxAm<'d, T, R, A> {
    pub fn new(device: &'d mut Si47xxDevice<T, R, A>) -> Self {
        Self { device }
    }
}

impl<'d, T: I2c, R: OutputPin, const A: u8> Si47xx for Si47xxAm<'d, T, R, A> {
    async fn seek_up(&mut self) -> Result<Si47xxTuneStatus, Error> {
        self.device.am_seek_up().await
    }

    async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error> {
        self.device.am_tune_frequency(frequency).await
    }
}

impl<T: I2c, R: OutputPin, const A: u8> Si47xxDevice<T, R, A> {
    const I2C_ADDRESS: u8 = A;
    /// Create a new Si47xxDevice driver from the given I2C peripheral and reset pin
    pub fn new(i2c: T, reset_pin: R) -> Self {
        Self { i2c, reset_pin }
    }

    pub fn am(&mut self) -> impl Si47xx {
        Si47xxAm::new(self)
    }

    pub fn fm(&mut self) -> impl Si47xx {
        Si47xxFm::new(self)
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
    pub async fn volume_set(&mut self, volume: u8) -> Result<(), Error> {
        if volume > 100 {
            return Err(Error::InvalidParameter);
        }
        let volume_value: u8 = ((volume as u16 * Si47xxCmd::VOLUME_MAX as u16) / 100) as u8;
        self.property_set(Si47xxProperty::AudioVolume, volume_value as u16)
            .await
    }

    /// Volume up by 10%
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn volume_up(&mut self) -> Result<(), Error> {
        let current_volume = self.property_get(Si47xxProperty::AudioVolume).await?;
        let new_volume = if current_volume
            > (Si47xxCmd::VOLUME_MAX as u16 - Si47xxCmd::VOLUME_MAX as u16 / 10)
        {
            Si47xxCmd::VOLUME_MAX as u16
        } else {
            current_volume + (Si47xxCmd::VOLUME_MAX as u16 / 10)
        };
        self.property_set(Si47xxProperty::AudioVolume, new_volume)
            .await
    }

    /// Volume down by 10%
    /// Returns `Ok(())` on success
    /// Returns `Error` on failure
    pub async fn volume_down(&mut self) -> Result<(), Error> {
        let current_volume = self.property_get(Si47xxProperty::AudioVolume).await?;
        let new_volume = current_volume.saturating_sub(Si47xxCmd::VOLUME_MAX as u16 / 10);
        self.property_set(Si47xxProperty::AudioVolume, new_volume)
            .await
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
        ))
    }

    /// Start FM seek up
    /// Returns `Si47xxTuneStatus` on success
    /// Returns `Error` on failure
    pub async fn fm_seek_up(&mut self) -> Result<Si47xxTuneStatus, Error> {
        let args: [u8; 2] = [
            Si47xxCmd::FM_SEEK_START_CMD,
            Si47xxCmd::SEEK_UP | Si47xxCmd::SEEK_WRAP,
        ];
        let mut resp: [u8; Si47xxCmd::STATUS_RSP_SIZE] = [0; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        self.wait_for_status(Si47xxCmd::STATUS_RSP_STCINT, 5000)
            .await?;
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
        ))
    }

    /// Start AM seek up
    /// Returns `Si47xxTuneStatus` on success
    /// Returns `Error` on failure
    pub async fn am_seek_up(&mut self) -> Result<Si47xxTuneStatus, Error> {
        let args: [u8; 2] = [
            Si47xxCmd::AM_SEEK_START_CMD,
            Si47xxCmd::SEEK_UP | Si47xxCmd::SEEK_WRAP,
        ];
        let mut resp: [u8; Si47xxCmd::STATUS_RSP_SIZE] = [0; Si47xxCmd::STATUS_RSP_SIZE];
        self.cmd_send(&args, &mut resp).await?;
        self.wait_for_status(Si47xxCmd::STATUS_RSP_STCINT, 5000)
            .await?;
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
        let freq_value: u16 = (frequency * 100.0) as u16;
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
}
