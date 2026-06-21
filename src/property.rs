#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Volume(u8);

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VolumeOutOfRange;

impl Volume {
    pub const MIN: u8 = 0;
    pub const MAX: u8 = 100;
    pub(crate) const DEVICE_MAX: u16 = 63;

    pub const fn new(value: u8) -> Option<Self> {
        if value <= Self::MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn get(self) -> u8 {
        self.0
    }

    pub(crate) const fn from_raw_value(value: u16) -> Option<Self> {
        if value <= Self::DEVICE_MAX {
            Self::new(((value * 100) / Self::DEVICE_MAX) as u8)
        } else {
            None
        }
    }

    pub(crate) const fn to_raw_value(self) -> u16 {
        (self.0 as u16 * Self::DEVICE_MAX) / 100
    }

    /// Increase volume by 10 percentage points.
    ///
    /// Saturates at `100%`.
    pub const fn up(self) -> Self {
        let increased = self.0.saturating_add(10);
        if increased > Self::MAX {
            Self(Self::MAX)
        } else {
            Self(increased)
        }
    }

    /// Decrease volume by 10 percentage points.
    ///
    /// Saturates at `0%`.
    pub const fn down(self) -> Self {
        Self(self.0.saturating_sub(10))
    }
}

impl TryFrom<u8> for Volume {
    type Error = VolumeOutOfRange;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::new(value).ok_or(VolumeOutOfRange)
    }
}

impl From<Volume> for u8 {
    fn from(value: Volume) -> Self {
        value.get()
    }
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
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
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

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UnknownSi47xxProperty(pub u16);

impl TryFrom<u16> for Si47xxProperty {
    type Error = UnknownSi47xxProperty;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0x0001 => Ok(Self::GpoIen),
            0x0102 => Ok(Self::DigitalOutputFormat),
            0x0104 => Ok(Self::DigitalOutputSampleRate),
            0x0201 => Ok(Self::RefclkFreq),
            0x0202 => Ok(Self::RefclkPrescale),
            0x1100 => Ok(Self::FmDeemphasis),
            0x1102 => Ok(Self::FmChannelFilter),
            0x1105 => Ok(Self::FmBlendStereoThreshold),
            0x1106 => Ok(Self::FmBlendMonoThreshold),
            0x1107 => Ok(Self::FmAntennaInput),
            0x1108 => Ok(Self::FmMaxTuneError),
            0x1200 => Ok(Self::FmRsqIntSource),
            0x1201 => Ok(Self::FmRsqSnrHiThreshold),
            0x1202 => Ok(Self::FmRsqSnrLoThreshold),
            0x1203 => Ok(Self::FmRsqRssiHiThreshold),
            0x1204 => Ok(Self::FmRsqRssiLoThreshold),
            0x1205 => Ok(Self::FmRsqMultipathHiThreshold),
            0x1206 => Ok(Self::FmRsqMultipathLoThreshold),
            0x1207 => Ok(Self::FmRsqBlendThreshold),
            0x1300 => Ok(Self::FmSoftMuteRate),
            0x1301 => Ok(Self::FmSoftMuteSlope),
            0x1302 => Ok(Self::FmSoftMuteMaxAttenuation),
            0x1303 => Ok(Self::FmSoftMuteSnrThreshold),
            0x1304 => Ok(Self::FmSoftMuteReleaseRate),
            0x1305 => Ok(Self::FmSoftMuteAttackRate),
            0x1400 => Ok(Self::FmSeekBandBottom),
            0x1401 => Ok(Self::FmSeekBandTop),
            0x1402 => Ok(Self::FmSeekFreqSpacing),
            0x1403 => Ok(Self::FmSeekTuneSnrThreshold),
            0x1404 => Ok(Self::FmSeekTuneRssiThreshold),
            0x1500 => Ok(Self::FmRdsIntSource),
            0x1501 => Ok(Self::FmRdsIntFifoCount),
            0x1502 => Ok(Self::FmRdsConfig),
            0x1503 => Ok(Self::FmRdsConfidence),
            0x1800 => Ok(Self::FmBlendRssiStereoThreshold),
            0x1801 => Ok(Self::FmBlendRssiMonoThreshold),
            0x1802 => Ok(Self::FmBlendRssiAttackRate),
            0x1803 => Ok(Self::FmBlendRssiReleaseRate),
            0x1804 => Ok(Self::FmBlendSnrStereoThreshold),
            0x1805 => Ok(Self::FmBlendSnrMonoThreshold),
            0x1806 => Ok(Self::FmBlendSnrAttackRate),
            0x1807 => Ok(Self::FmBlendSnrReleaseRate),
            0x1808 => Ok(Self::FmBlendMultipathStereoThreshold),
            0x1809 => Ok(Self::FmBlendMultipathMonoThreshold),
            0x180A => Ok(Self::FmBlendMultipathAttackRate),
            0x180B => Ok(Self::FmBlendMultipathReleaseRate),
            0x1A00 => Ok(Self::FmHicutSnrHighThreshold),
            0x1A01 => Ok(Self::FmHicutSnrLowThreshold),
            0x1A02 => Ok(Self::FmHicutAttackRate),
            0x1A03 => Ok(Self::FmHicutReleaseRate),
            0x1A04 => Ok(Self::FmHicutMultipathTriggerThreshold),
            0x1A05 => Ok(Self::FmHicutMultipathEndThreshold),
            0x1A06 => Ok(Self::FmHicutCutoffFrequency),
            0x3100 => Ok(Self::AmDeemphasis),
            0x3102 => Ok(Self::AmChannelFilter),
            0x3103 => Ok(Self::AmAutomaticVolumeControlMaxGain),
            0x3104 => Ok(Self::AmModeAfcSwPullInRange),
            0x3105 => Ok(Self::AmModeAfcSwLockInRange),
            0x3200 => Ok(Self::AmRsqIntSource),
            0x3201 => Ok(Self::AmRsqSnrHiThreshold),
            0x3202 => Ok(Self::AmRsqSnrLoThreshold),
            0x3203 => Ok(Self::AmRsqRssiHiThreshold),
            0x3204 => Ok(Self::AmRsqRssiLoThreshold),
            0x3300 => Ok(Self::AmSoftMuteRate),
            0x3301 => Ok(Self::AmSoftMuteSlope),
            0x3302 => Ok(Self::AmSoftMuteMaxAttenuation),
            0x3303 => Ok(Self::AmSoftMuteSnrThreshold),
            0x3400 => Ok(Self::AmSeekBandBottom),
            0x3401 => Ok(Self::AmSeekBandTop),
            0x3402 => Ok(Self::AmSeekFreqSpacing),
            0x3403 => Ok(Self::AmSeekTuneSnrThreshold),
            0x3404 => Ok(Self::AmSeekTuneRssiThreshold),
            0x4000 => Ok(Self::AudioVolume),
            0x4001 => Ok(Self::AudioVolumeMute),
            unknown => Err(UnknownSi47xxProperty(unknown)),
        }
    }
}

impl From<Si47xxProperty> for u16 {
    fn from(value: Si47xxProperty) -> Self {
        value as Self
    }
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

pub(crate) const SHARED_PROPERTIES: [Si47xxProperty; 7] = [
    Si47xxProperty::GpoIen,
    Si47xxProperty::DigitalOutputFormat,
    Si47xxProperty::DigitalOutputSampleRate,
    Si47xxProperty::RefclkFreq,
    Si47xxProperty::RefclkPrescale,
    Si47xxProperty::AudioVolume,
    Si47xxProperty::AudioVolumeMute,
];

pub(crate) const FM_ONLY_PROPERTIES: [Si47xxProperty; 48] = [
    Si47xxProperty::FmDeemphasis,
    Si47xxProperty::FmChannelFilter,
    Si47xxProperty::FmBlendStereoThreshold,
    Si47xxProperty::FmBlendMonoThreshold,
    Si47xxProperty::FmAntennaInput,
    Si47xxProperty::FmMaxTuneError,
    Si47xxProperty::FmRsqIntSource,
    Si47xxProperty::FmRsqSnrHiThreshold,
    Si47xxProperty::FmRsqSnrLoThreshold,
    Si47xxProperty::FmRsqRssiHiThreshold,
    Si47xxProperty::FmRsqRssiLoThreshold,
    Si47xxProperty::FmRsqMultipathHiThreshold,
    Si47xxProperty::FmRsqMultipathLoThreshold,
    Si47xxProperty::FmRsqBlendThreshold,
    Si47xxProperty::FmSoftMuteRate,
    Si47xxProperty::FmSoftMuteSlope,
    Si47xxProperty::FmSoftMuteMaxAttenuation,
    Si47xxProperty::FmSoftMuteSnrThreshold,
    Si47xxProperty::FmSoftMuteReleaseRate,
    Si47xxProperty::FmSoftMuteAttackRate,
    Si47xxProperty::FmSeekBandBottom,
    Si47xxProperty::FmSeekBandTop,
    Si47xxProperty::FmSeekFreqSpacing,
    Si47xxProperty::FmSeekTuneSnrThreshold,
    Si47xxProperty::FmSeekTuneRssiThreshold,
    Si47xxProperty::FmRdsIntSource,
    Si47xxProperty::FmRdsIntFifoCount,
    Si47xxProperty::FmRdsConfig,
    Si47xxProperty::FmRdsConfidence,
    Si47xxProperty::FmBlendRssiStereoThreshold,
    Si47xxProperty::FmBlendRssiMonoThreshold,
    Si47xxProperty::FmBlendRssiAttackRate,
    Si47xxProperty::FmBlendRssiReleaseRate,
    Si47xxProperty::FmBlendSnrStereoThreshold,
    Si47xxProperty::FmBlendSnrMonoThreshold,
    Si47xxProperty::FmBlendSnrAttackRate,
    Si47xxProperty::FmBlendSnrReleaseRate,
    Si47xxProperty::FmBlendMultipathStereoThreshold,
    Si47xxProperty::FmBlendMultipathMonoThreshold,
    Si47xxProperty::FmBlendMultipathAttackRate,
    Si47xxProperty::FmBlendMultipathReleaseRate,
    Si47xxProperty::FmHicutSnrHighThreshold,
    Si47xxProperty::FmHicutSnrLowThreshold,
    Si47xxProperty::FmHicutAttackRate,
    Si47xxProperty::FmHicutReleaseRate,
    Si47xxProperty::FmHicutMultipathTriggerThreshold,
    Si47xxProperty::FmHicutMultipathEndThreshold,
    Si47xxProperty::FmHicutCutoffFrequency,
];

pub(crate) const AM_ONLY_PROPERTIES: [Si47xxProperty; 19] = [
    Si47xxProperty::AmDeemphasis,
    Si47xxProperty::AmChannelFilter,
    Si47xxProperty::AmAutomaticVolumeControlMaxGain,
    Si47xxProperty::AmModeAfcSwPullInRange,
    Si47xxProperty::AmModeAfcSwLockInRange,
    Si47xxProperty::AmRsqIntSource,
    Si47xxProperty::AmRsqSnrHiThreshold,
    Si47xxProperty::AmRsqSnrLoThreshold,
    Si47xxProperty::AmRsqRssiHiThreshold,
    Si47xxProperty::AmRsqRssiLoThreshold,
    Si47xxProperty::AmSoftMuteRate,
    Si47xxProperty::AmSoftMuteSlope,
    Si47xxProperty::AmSoftMuteMaxAttenuation,
    Si47xxProperty::AmSoftMuteSnrThreshold,
    Si47xxProperty::AmSeekBandBottom,
    Si47xxProperty::AmSeekBandTop,
    Si47xxProperty::AmSeekFreqSpacing,
    Si47xxProperty::AmSeekTuneSnrThreshold,
    Si47xxProperty::AmSeekTuneRssiThreshold,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn volume_accepts_bounds() {
        assert_eq!(Volume::new(Volume::MIN), Some(Volume(0)));
        assert_eq!(Volume::new(Volume::MAX), Some(Volume(100)));
    }

    #[test]
    fn volume_rejects_out_of_range_values() {
        assert_eq!(Volume::new(101), None);
        assert_eq!(Volume::try_from(255), Err(VolumeOutOfRange));
    }

    #[test]
    fn volume_from_raw_value_maps_volume_range() {
        assert_eq!(Volume::from_raw_value(0), Some(Volume(0)));
        assert_eq!(Volume::from_raw_value(31), Some(Volume(49)));
        assert_eq!(
            Volume::from_raw_value(Volume::DEVICE_MAX),
            Some(Volume(100))
        );
    }

    #[test]
    fn volume_from_raw_value_rejects_out_of_range_values() {
        assert_eq!(Volume::from_raw_value(Volume::DEVICE_MAX + 1), None);
    }

    #[test]
    fn volume_to_raw_value_maps_percent_range() {
        assert_eq!(Volume(0).to_raw_value(), 0);
        assert_eq!(Volume(50).to_raw_value(), 31);
        assert_eq!(Volume(Volume::MAX).to_raw_value(), Volume::DEVICE_MAX);
    }

    #[test]
    fn volume_up_saturates_at_max() {
        assert_eq!(Volume(0).up(), Volume(10));
        assert_eq!(Volume(90).up(), Volume(100));
        assert_eq!(Volume(100).up(), Volume(100));
    }

    #[test]
    fn volume_down_saturates_at_min() {
        assert_eq!(Volume(100).down(), Volume(90));
        assert_eq!(Volume(10).down(), Volume(0));
        assert_eq!(Volume(0).down(), Volume(0));
    }

    #[test]
    fn property_try_from_u16_accepts_known_property_ids() {
        assert_eq!(Si47xxProperty::try_from(0x0001), Ok(Si47xxProperty::GpoIen));
        assert_eq!(
            Si47xxProperty::try_from(0x180A),
            Ok(Si47xxProperty::FmBlendMultipathAttackRate)
        );
        assert_eq!(
            Si47xxProperty::try_from(0x4001),
            Ok(Si47xxProperty::AudioVolumeMute)
        );
    }

    #[test]
    fn property_try_from_u16_rejects_unknown_property_ids() {
        assert_eq!(
            Si47xxProperty::try_from(0xFFFF),
            Err(UnknownSi47xxProperty(0xFFFF))
        );
    }

    #[test]
    fn property_into_u16_returns_property_id() {
        assert_eq!(u16::from(Si47xxProperty::AmSeekBandTop), 0x3401);
    }
}
