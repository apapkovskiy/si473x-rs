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
    /// Device is powered down
    PoweredDown,
}

/// Si47xx device property identifiers used with `SET_PROPERTY` / `GET_PROPERTY`.
///
/// Property groups in this enum:
/// - Shared: available in both FM and AM/SW/LW receiver components.
/// - FM-only: available only in FM/RDS receiver component.
/// - AM-only: available only in AM/SW/LW receiver component.
///
/// Quick AN332 references:
/// - FM properties summary: AN332, section `5.1`, Table `5`
/// - AM/SW/LW properties summary: AN332, section `5.2`, Table `9`
/// - FM detailed property bitfields: AN332, section `5.1.2`
/// - AM/SW/LW detailed property bitfields: AN332, section `5.2.2`
#[repr(u16)]
pub enum Si47xxProperty {
    /// Interrupt source mask for `GPO2/INT`.
    GpoIen = 0x0001,
    /// Digital audio format (I2S/left-justified/DSP, mono/stereo, sample bits).
    DigitalOutputFormat = 0x0102,
    /// Digital audio sample rate.
    ///
    /// Units: samples/s
    /// Range: `0` (disable), `32_000..=48_000`
    DigitalOutputSampleRate = 0x0104,
    /// Reference clock frequency used by AFC.
    ///
    /// Units: Hz
    /// Range: `0` (disable AFC) or `31130..=34406`
    RefclkFreq = 0x0201,
    /// Reference clock prescaler divider.
    ///
    /// Range: `1..=4095`
    RefclkPrescale = 0x0202,

    // FM-only properties (AN332 section 5.1 / 5.1.2)
    /// FM de-emphasis time constant (typically 50 us or 75 us).
    FmDeemphasis = 0x1100,
    /// FM demodulation channel filter bandwidth selection.
    ///
    /// Range: `0..=4`
    FmChannelFilter = 0x1102,
    /// Legacy FM stereo blend threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    FmBlendStereoThreshold = 0x1105,
    /// Legacy FM mono blend threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    FmBlendMonoThreshold = 0x1106,
    /// FM antenna input selection (long-wire vs TXO/LPI short antenna).
    FmAntennaInput = 0x1107,
    /// Maximum FM tune error before AFCRL is set.
    ///
    /// Units: kHz
    /// Range: `0..=255`
    FmMaxTuneError = 0x1108,
    /// FM RSQ interrupt source enables.
    FmRsqIntSource = 0x1200,
    /// FM RSQ SNR high threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    FmRsqSnrHiThreshold = 0x1201,
    /// FM RSQ SNR low threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    FmRsqSnrLoThreshold = 0x1202,
    /// FM RSQ RSSI high threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    FmRsqRssiHiThreshold = 0x1203,
    /// FM RSQ RSSI low threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    FmRsqRssiLoThreshold = 0x1204,
    /// FM RSQ multipath high threshold.
    ///
    /// Range: `0..=127` (`127` can be used to disable)
    FmRsqMultipathHiThreshold = 0x1205,
    /// FM RSQ multipath low threshold.
    ///
    /// Range: `0..=127`
    FmRsqMultipathLoThreshold = 0x1206,
    /// FM RSQ blend boundary threshold.
    ///
    /// Units: %
    /// Range: `0..=100`
    FmRsqBlendThreshold = 0x1207,
    /// FM soft-mute attack/decay rate (legacy combined control).
    ///
    /// Range: `1..=255`
    FmSoftMuteRate = 0x1300,
    /// FM soft-mute slope.
    ///
    /// Range: `0..=63`
    FmSoftMuteSlope = 0x1301,
    /// FM soft-mute max attenuation.
    ///
    /// Units: dB
    /// Range: `0..=31`
    FmSoftMuteMaxAttenuation = 0x1302,
    /// FM soft-mute SNR threshold.
    ///
    /// Units: dB
    /// Range: `0..=15`
    FmSoftMuteSnrThreshold = 0x1303,
    /// FM soft-mute release rate.
    ///
    /// Range: `1..=32767`
    FmSoftMuteReleaseRate = 0x1304,
    /// FM soft-mute attack rate.
    ///
    /// Range: `1..=32767`
    FmSoftMuteAttackRate = 0x1305,
    /// FM seek lower band edge.
    ///
    /// Units: 10 kHz
    /// Range: `6400..=10800` (64.0..=108.0 MHz)
    FmSeekBandBottom = 0x1400,
    /// FM seek upper band edge.
    ///
    /// Units: 10 kHz
    /// Range: `6400..=10800` (64.0..=108.0 MHz)
    FmSeekBandTop = 0x1401,
    /// FM seek channel spacing.
    ///
    /// Units: 10 kHz
    /// Valid values: `5`, `10`, `20`
    FmSeekFreqSpacing = 0x1402,
    /// FM seek/tune SNR validity threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    FmSeekTuneSnrThreshold = 0x1403,
    /// FM seek/tune RSSI validity threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    FmSeekTuneRssiThreshold = 0x1404,
    /// FM RDS interrupt source enables.
    FmRdsIntSource = 0x1500,
    /// FM RDS FIFO count threshold for interrupt.
    ///
    /// Range: typically `0..=25` (device/firmware dependent)
    FmRdsIntFifoCount = 0x1501,
    /// FM RDS enable and block error threshold configuration.
    FmRdsConfig = 0x1502,
    /// FM RDS decoder confidence configuration.
    FmRdsConfidence = 0x1503,
    /// FM blend RSSI stereo threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    FmBlendRssiStereoThreshold = 0x1800,
    /// FM blend RSSI mono threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    FmBlendRssiMonoThreshold = 0x1801,
    /// FM blend RSSI attack rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmBlendRssiAttackRate = 0x1802,
    /// FM blend RSSI release rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmBlendRssiReleaseRate = 0x1803,
    /// FM blend SNR stereo threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    FmBlendSnrStereoThreshold = 0x1804,
    /// FM blend SNR mono threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    FmBlendSnrMonoThreshold = 0x1805,
    /// FM blend SNR attack rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmBlendSnrAttackRate = 0x1806,
    /// FM blend SNR release rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmBlendSnrReleaseRate = 0x1807,
    /// FM blend multipath stereo threshold.
    ///
    /// Units: %
    /// Range: `0..=100`
    FmBlendMultipathStereoThreshold = 0x1808,
    /// FM blend multipath mono threshold.
    ///
    /// Units: %
    /// Range: `0..=100`
    FmBlendMultipathMonoThreshold = 0x1809,
    /// FM blend multipath attack rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmBlendMultipathAttackRate = 0x180A,
    /// FM blend multipath release rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmBlendMultipathReleaseRate = 0x180B,
    /// FM hi-cut SNR high threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    FmHicutSnrHighThreshold = 0x1A00,
    /// FM hi-cut SNR low threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    FmHicutSnrLowThreshold = 0x1A01,
    /// FM hi-cut attack rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmHicutAttackRate = 0x1A02,
    /// FM hi-cut release rate.
    ///
    /// Range: `0` (disable) or `1..=32767`
    FmHicutReleaseRate = 0x1A03,
    /// FM hi-cut multipath trigger threshold.
    ///
    /// Units: %
    /// Range: `0..=100`
    FmHicutMultipathTriggerThreshold = 0x1A04,
    /// FM hi-cut multipath end threshold.
    ///
    /// Units: %
    /// Range: `0..=100`
    FmHicutMultipathEndThreshold = 0x1A05,
    /// FM hi-cut and max audio cutoff selector.
    ///
    /// Range: encoded fields in `0..=7`
    FmHicutCutoffFrequency = 0x1A06,

    // AM/SW/LW-only properties (AN332 section 5.2 / 5.2.2)
    /// AM de-emphasis enable (50 us when enabled).
    AmDeemphasis = 0x3100,
    /// AM channel filter configuration.
    ///
    /// Range: encoded bandwidth selector (typically `0..=6`)
    AmChannelFilter = 0x3102,
    /// AM automatic volume control max gain.
    ///
    /// Range: encoded gain value (commonly `0x1000..=0x7800`)
    AmAutomaticVolumeControlMaxGain = 0x3103,
    /// SW AFC pull-in range.
    ///
    /// Units: encoded inverse-ppm value
    AmModeAfcSwPullInRange = 0x3104,
    /// SW AFC lock-in range.
    ///
    /// Units: encoded inverse-ppm value
    AmModeAfcSwLockInRange = 0x3105,
    /// AM RSQ interrupt source enables.
    AmRsqIntSource = 0x3200,
    /// AM RSQ SNR high threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    AmRsqSnrHiThreshold = 0x3201,
    /// AM RSQ SNR low threshold.
    ///
    /// Units: dB
    /// Range: `0..=127`
    AmRsqSnrLoThreshold = 0x3202,
    /// AM RSQ RSSI high threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    AmRsqRssiHiThreshold = 0x3203,
    /// AM RSQ RSSI low threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=127`
    AmRsqRssiLoThreshold = 0x3204,
    /// AM soft-mute rate.
    ///
    /// Units: encoded, actual rate is proportional to field value
    /// Range: `1..=255`
    AmSoftMuteRate = 0x3300,
    /// AM soft-mute slope.
    AmSoftMuteSlope = 0x3301,
    /// AM soft-mute max attenuation.
    ///
    /// Units: dB
    /// Typical range: up to `31` (device-dependent)
    AmSoftMuteMaxAttenuation = 0x3302,
    /// AM soft-mute SNR threshold.
    ///
    /// Units: dB
    /// Typical range: up to `15` (device-dependent)
    AmSoftMuteSnrThreshold = 0x3303,
    /// AM/SW/LW seek lower band edge.
    ///
    /// Units: kHz
    /// Range: `149..=23000`
    AmSeekBandBottom = 0x3400,
    /// AM/SW/LW seek upper band edge.
    ///
    /// Units: kHz
    /// Range: `149..=23000`
    AmSeekBandTop = 0x3401,
    /// AM/SW/LW seek channel spacing.
    ///
    /// Units: kHz
    /// Valid values: `1`, `5`, `9`, `10`
    AmSeekFreqSpacing = 0x3402,
    /// AM/SW/LW seek/tune SNR validity threshold.
    ///
    /// Units: dB
    /// Range: `0..=63`
    AmSeekTuneSnrThreshold = 0x3403,
    /// AM/SW/LW seek/tune RSSI validity threshold.
    ///
    /// Units: dBuV
    /// Range: `0..=63`
    AmSeekTuneRssiThreshold = 0x3404,

    /// Audio output volume.
    ///
    /// Range: `0..=63`
    AudioVolume = 0x4000,
    /// Audio hard mute (left/right channels).
    AudioVolumeMute = 0x4001,
}

impl Si47xxProperty {
    pub const fn is_shared(self) -> bool {
        matches!(
            self,
            Self::GpoIen
                | Self::DigitalOutputFormat
                | Self::DigitalOutputSampleRate
                | Self::RefclkFreq
                | Self::RefclkPrescale
                | Self::AudioVolume
                | Self::AudioVolumeMute
        )
    }

    pub const fn is_fm_only(self) -> bool {
        matches!(
            self,
            Self::FmDeemphasis
                | Self::FmChannelFilter
                | Self::FmBlendStereoThreshold
                | Self::FmBlendMonoThreshold
                | Self::FmAntennaInput
                | Self::FmMaxTuneError
                | Self::FmRsqIntSource
                | Self::FmRsqSnrHiThreshold
                | Self::FmRsqSnrLoThreshold
                | Self::FmRsqRssiHiThreshold
                | Self::FmRsqRssiLoThreshold
                | Self::FmRsqMultipathHiThreshold
                | Self::FmRsqMultipathLoThreshold
                | Self::FmRsqBlendThreshold
                | Self::FmSoftMuteRate
                | Self::FmSoftMuteSlope
                | Self::FmSoftMuteMaxAttenuation
                | Self::FmSoftMuteSnrThreshold
                | Self::FmSoftMuteReleaseRate
                | Self::FmSoftMuteAttackRate
                | Self::FmSeekBandBottom
                | Self::FmSeekBandTop
                | Self::FmSeekFreqSpacing
                | Self::FmSeekTuneSnrThreshold
                | Self::FmSeekTuneRssiThreshold
                | Self::FmRdsIntSource
                | Self::FmRdsIntFifoCount
                | Self::FmRdsConfig
                | Self::FmRdsConfidence
                | Self::FmBlendRssiStereoThreshold
                | Self::FmBlendRssiMonoThreshold
                | Self::FmBlendRssiAttackRate
                | Self::FmBlendRssiReleaseRate
                | Self::FmBlendSnrStereoThreshold
                | Self::FmBlendSnrMonoThreshold
                | Self::FmBlendSnrAttackRate
                | Self::FmBlendSnrReleaseRate
                | Self::FmBlendMultipathStereoThreshold
                | Self::FmBlendMultipathMonoThreshold
                | Self::FmBlendMultipathAttackRate
                | Self::FmBlendMultipathReleaseRate
                | Self::FmHicutSnrHighThreshold
                | Self::FmHicutSnrLowThreshold
                | Self::FmHicutAttackRate
                | Self::FmHicutReleaseRate
                | Self::FmHicutMultipathTriggerThreshold
                | Self::FmHicutMultipathEndThreshold
                | Self::FmHicutCutoffFrequency
        )
    }

    pub const fn is_am_only(self) -> bool {
        matches!(
            self,
            Self::AmDeemphasis
                | Self::AmChannelFilter
                | Self::AmAutomaticVolumeControlMaxGain
                | Self::AmModeAfcSwPullInRange
                | Self::AmModeAfcSwLockInRange
                | Self::AmRsqIntSource
                | Self::AmRsqSnrHiThreshold
                | Self::AmRsqSnrLoThreshold
                | Self::AmRsqRssiHiThreshold
                | Self::AmRsqRssiLoThreshold
                | Self::AmSoftMuteRate
                | Self::AmSoftMuteSlope
                | Self::AmSoftMuteMaxAttenuation
                | Self::AmSoftMuteSnrThreshold
                | Self::AmSeekBandBottom
                | Self::AmSeekBandTop
                | Self::AmSeekFreqSpacing
                | Self::AmSeekTuneSnrThreshold
                | Self::AmSeekTuneRssiThreshold
        )
    }
}

pub enum AmLwSwBands {}

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

pub enum Si47xxTuneResolution {
    AmModeLsbKhz = 1,
    FmModeLsbKhz = 10,
}

impl Si47xxTuneStatus {
    const KHZ_TO_MHZ: f32 = 1000.0;
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
    async fn volume_set(&mut self, volume: u8) -> Result<(), Error>;
    /// Increase the audio volume by a fixed step (e.g., 10%).
    async fn volume_up(&mut self) -> Result<(), Error>;
    /// Decrease the audio volume by a fixed step (e.g., 10%).
    async fn volume_down(&mut self) -> Result<(), Error>;
    /// Retrieve the current tuning status, including frequency, signal strength, and quality.
    async fn tune_status_get(&mut self) -> Result<Si47xxTuneStatus, Error>;
    /// Start seeking for the next valid station in the upward frequency direction.
    async fn seek_up(&mut self) -> Result<Si47xxTuneStatus, Error>;
    /// Tune to a specific frequency.
    async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error>;
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
    async fn volume_set(&mut self, volume: u8) -> Result<(), Error> {
        self.volume_set(volume).await
    }
    async fn volume_up(&mut self) -> Result<(), Error> {
        self.volume_up().await
    }
    async fn volume_down(&mut self) -> Result<(), Error> {
        self.volume_down().await
    }
    async fn tune_status_get(&mut self) -> Result<Si47xxTuneStatus, Error> {
        self.tune_status_get().await
    }
    async fn seek_up(&mut self) -> Result<Si47xxTuneStatus, Error> {
        self.seek_up().await
    }
    async fn tune_frequency(&mut self, frequency: f32) -> Result<Si47xxTuneStatus, Error> {
        self.tune_frequency(frequency).await
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
    pub async fn volume_set(&mut self, volume: u8) -> Result<(), Error> {
        match self {
            Si47xxRadio::Am(device) => device.volume_set(volume).await,
            Si47xxRadio::Fm(device) => device.volume_set(volume).await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn volume_up(&mut self) -> Result<(), Error> {
        match self {
            Si47xxRadio::Am(device) => device.volume_up().await,
            Si47xxRadio::Fm(device) => device.volume_up().await,
            Si47xxRadio::Off(_) => Err(Error::PoweredDown),
        }
    }
    pub async fn volume_down(&mut self) -> Result<(), Error> {
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
    pub async fn seek_up(&mut self) -> Result<Si47xxTuneStatus, Error> {
        match self {
            Si47xxRadio::Am(device) => device.am_seek_up().await,
            Si47xxRadio::Fm(device) => device.fm_seek_up().await,
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
            Si47xxTuneResolution::FmModeLsbKhz,
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
            Si47xxTuneResolution::AmModeLsbKhz,
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
