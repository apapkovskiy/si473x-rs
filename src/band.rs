/// Predefined FM, AM, LW, SW, and HAM tuning bands for Si47xx.
///
/// Frequency ranges are in kilohertz and are based on common ITU broadcast
/// allocations plus commonly used HF amateur radio allocations.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BandRangeKhz {
    pub bottom_khz: u32,
    pub top_khz: u32,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioBand {
    /// Unknown or custom band placeholder.
    Unknown(BandRangeKhz),
    /// Full FM broadcast coverage used by common multi-region receivers.
    FmBroadcast(BandRangeKhz),
    /// FM broadcast band used in most of Europe and the Americas.
    FmUsEu(BandRangeKhz),
    /// FM broadcast band used in Japan.
    FmJapan(BandRangeKhz),
    /// Legacy OIRT FM broadcast band.
    FmOirt(BandRangeKhz),
    /// Longwave broadcast band (Region 1 nominal plan).
    LwBroadcast(BandRangeKhz),
    /// Medium-wave AM broadcast band (global superset).
    AmBroadcast(BandRangeKhz),
    /// Full shortwave broadcast coverage used by common receiver band plans.
    SwBroadcast(BandRangeKhz),
    /// 120 m tropical broadcast band.
    Sw120m(BandRangeKhz),
    /// 90 m tropical broadcast band.
    Sw90m(BandRangeKhz),
    /// 75 m tropical broadcast band.
    Sw75m(BandRangeKhz),
    /// 60 m tropical broadcast band.
    Sw60m(BandRangeKhz),
    /// 49 m broadcast band.
    Sw49m(BandRangeKhz),
    /// 41 m broadcast band.
    Sw41m(BandRangeKhz),
    /// 31 m broadcast band.
    Sw31m(BandRangeKhz),
    /// 25 m broadcast band.
    Sw25m(BandRangeKhz),
    /// 22 m broadcast band.
    Sw22m(BandRangeKhz),
    /// 19 m broadcast band.
    Sw19m(BandRangeKhz),
    /// 16 m broadcast band.
    Sw16m(BandRangeKhz),
    /// 15 m broadcast band.
    Sw15m(BandRangeKhz),
    /// 13 m broadcast band.
    Sw13m(BandRangeKhz),
    /// 11 m broadcast band.
    Sw11m(BandRangeKhz),
    /// Amateur 2200 m band.
    Ham2200m(BandRangeKhz),
    /// Amateur 630 m band.
    Ham630m(BandRangeKhz),
    /// Amateur 160 m band.
    Ham160m(BandRangeKhz),
    /// Amateur 80 m band.
    Ham80m(BandRangeKhz),
    /// Amateur 60 m band (WRC-15 allocation).
    Ham60m(BandRangeKhz),
    /// Amateur 40 m band.
    Ham40m(BandRangeKhz),
    /// Amateur 30 m band.
    Ham30m(BandRangeKhz),
    /// Amateur 20 m band.
    Ham20m(BandRangeKhz),
    /// Amateur 17 m band.
    Ham17m(BandRangeKhz),
    /// Amateur 15 m band.
    Ham15m(BandRangeKhz),
    /// Amateur 12 m band.
    Ham12m(BandRangeKhz),
    /// Amateur 10 m band.
    Ham10m(BandRangeKhz),
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParseRadioBandsError;

impl RadioBand {
    pub const UNKNOWN: Self = Self::Unknown(BandRangeKhz {
        bottom_khz: 0,
        top_khz: 0,
    });
    pub const FM_BROADCAST: Self = Self::FmBroadcast(BandRangeKhz {
        bottom_khz: 64000,
        top_khz: 108000,
    });
    pub const FM_US_EU: Self = Self::FmUsEu(BandRangeKhz {
        bottom_khz: 87500,
        top_khz: 108000,
    });
    pub const FM_JAPAN: Self = Self::FmJapan(BandRangeKhz {
        bottom_khz: 76000,
        top_khz: 95000,
    });
    pub const FM_OIRT: Self = Self::FmOirt(BandRangeKhz {
        bottom_khz: 65000,
        top_khz: 74000,
    });
    pub const LW_BROADCAST: Self = Self::LwBroadcast(BandRangeKhz {
        bottom_khz: 153,
        top_khz: 279,
    });
    pub const AM_BROADCAST: Self = Self::AmBroadcast(BandRangeKhz {
        bottom_khz: 520,
        top_khz: 1710,
    });
    pub const SW_BROADCAST: Self = Self::SwBroadcast(BandRangeKhz {
        bottom_khz: 2300,
        top_khz: 26100,
    });
    pub const SW_120M: Self = Self::Sw120m(BandRangeKhz {
        bottom_khz: 2300,
        top_khz: 2495,
    });
    pub const SW_90M: Self = Self::Sw90m(BandRangeKhz {
        bottom_khz: 3200,
        top_khz: 3400,
    });
    pub const SW_75M: Self = Self::Sw75m(BandRangeKhz {
        bottom_khz: 3900,
        top_khz: 4000,
    });
    pub const SW_60M: Self = Self::Sw60m(BandRangeKhz {
        bottom_khz: 4750,
        top_khz: 4995,
    });
    pub const SW_49M: Self = Self::Sw49m(BandRangeKhz {
        bottom_khz: 5900,
        top_khz: 6200,
    });
    pub const SW_41M: Self = Self::Sw41m(BandRangeKhz {
        bottom_khz: 7200,
        top_khz: 7450,
    });
    pub const SW_31M: Self = Self::Sw31m(BandRangeKhz {
        bottom_khz: 9400,
        top_khz: 9900,
    });
    pub const SW_25M: Self = Self::Sw25m(BandRangeKhz {
        bottom_khz: 11600,
        top_khz: 12100,
    });
    pub const SW_22M: Self = Self::Sw22m(BandRangeKhz {
        bottom_khz: 13570,
        top_khz: 13870,
    });
    pub const SW_19M: Self = Self::Sw19m(BandRangeKhz {
        bottom_khz: 15100,
        top_khz: 15830,
    });
    pub const SW_16M: Self = Self::Sw16m(BandRangeKhz {
        bottom_khz: 17480,
        top_khz: 17900,
    });
    pub const SW_15M: Self = Self::Sw15m(BandRangeKhz {
        bottom_khz: 18900,
        top_khz: 19020,
    });
    pub const SW_13M: Self = Self::Sw13m(BandRangeKhz {
        bottom_khz: 21450,
        top_khz: 21850,
    });
    pub const SW_11M: Self = Self::Sw11m(BandRangeKhz {
        bottom_khz: 25670,
        top_khz: 26100,
    });
    pub const HAM_2200M: Self = Self::Ham2200m(BandRangeKhz {
        bottom_khz: 136,
        top_khz: 138,
    });
    pub const HAM_630M: Self = Self::Ham630m(BandRangeKhz {
        bottom_khz: 472,
        top_khz: 479,
    });
    pub const HAM_160M: Self = Self::Ham160m(BandRangeKhz {
        bottom_khz: 1800,
        top_khz: 2000,
    });
    pub const HAM_80M: Self = Self::Ham80m(BandRangeKhz {
        bottom_khz: 3500,
        top_khz: 4000,
    });
    pub const HAM_60M: Self = Self::Ham60m(BandRangeKhz {
        bottom_khz: 5352,
        top_khz: 5367,
    });
    pub const HAM_40M: Self = Self::Ham40m(BandRangeKhz {
        bottom_khz: 7000,
        top_khz: 7300,
    });
    pub const HAM_30M: Self = Self::Ham30m(BandRangeKhz {
        bottom_khz: 10100,
        top_khz: 10150,
    });
    pub const HAM_20M: Self = Self::Ham20m(BandRangeKhz {
        bottom_khz: 14000,
        top_khz: 14350,
    });
    pub const HAM_17M: Self = Self::Ham17m(BandRangeKhz {
        bottom_khz: 18068,
        top_khz: 18168,
    });
    pub const HAM_15M: Self = Self::Ham15m(BandRangeKhz {
        bottom_khz: 21000,
        top_khz: 21450,
    });
    pub const HAM_12M: Self = Self::Ham12m(BandRangeKhz {
        bottom_khz: 24890,
        top_khz: 24990,
    });
    pub const HAM_10M: Self = Self::Ham10m(BandRangeKhz {
        bottom_khz: 28000,
        top_khz: 29700,
    });

    pub const ALL_KNOWN_BANDS: [Self; 33] = [
        Self::FM_BROADCAST,
        Self::FM_US_EU,
        Self::FM_JAPAN,
        Self::FM_OIRT,
        Self::LW_BROADCAST,
        Self::AM_BROADCAST,
        Self::SW_BROADCAST,
        Self::SW_120M,
        Self::SW_90M,
        Self::SW_75M,
        Self::SW_60M,
        Self::SW_49M,
        Self::SW_41M,
        Self::SW_31M,
        Self::SW_25M,
        Self::SW_22M,
        Self::SW_19M,
        Self::SW_16M,
        Self::SW_15M,
        Self::SW_13M,
        Self::SW_11M,
        Self::HAM_2200M,
        Self::HAM_630M,
        Self::HAM_160M,
        Self::HAM_80M,
        Self::HAM_60M,
        Self::HAM_40M,
        Self::HAM_30M,
        Self::HAM_20M,
        Self::HAM_17M,
        Self::HAM_15M,
        Self::HAM_12M,
        Self::HAM_10M,
    ];

    /// Call `callback` for each predefined radio band.
    pub fn for_each<F>(mut callback: F)
    where
        F: FnMut(Self),
    {
        let mut i = 0;
        while i < Self::ALL_KNOWN_BANDS.len() {
            callback(Self::ALL_KNOWN_BANDS[i]);
            i += 1;
        }
    }

    pub const fn range(self) -> BandRangeKhz {
        match self {
            Self::Unknown(range)
            | Self::FmBroadcast(range)
            | Self::FmUsEu(range)
            | Self::FmJapan(range)
            | Self::FmOirt(range)
            | Self::LwBroadcast(range)
            | Self::AmBroadcast(range)
            | Self::SwBroadcast(range)
            | Self::Sw120m(range)
            | Self::Sw90m(range)
            | Self::Sw75m(range)
            | Self::Sw60m(range)
            | Self::Sw49m(range)
            | Self::Sw41m(range)
            | Self::Sw31m(range)
            | Self::Sw25m(range)
            | Self::Sw22m(range)
            | Self::Sw19m(range)
            | Self::Sw16m(range)
            | Self::Sw15m(range)
            | Self::Sw13m(range)
            | Self::Sw11m(range)
            | Self::Ham2200m(range)
            | Self::Ham630m(range)
            | Self::Ham160m(range)
            | Self::Ham80m(range)
            | Self::Ham60m(range)
            | Self::Ham40m(range)
            | Self::Ham30m(range)
            | Self::Ham20m(range)
            | Self::Ham17m(range)
            | Self::Ham15m(range)
            | Self::Ham12m(range)
            | Self::Ham10m(range) => range,
        }
    }

    /// Lower edge of the band in kHz.
    pub const fn bottom_khz(self) -> u32 {
        self.range().bottom_khz
    }

    /// Upper edge of the band in kHz.
    pub const fn top_khz(self) -> u32 {
        self.range().top_khz
    }

    /// Lower edge of the band in MHz.
    pub const fn bottom_mhz(self) -> f32 {
        self.bottom_khz() as f32 / 1000.0
    }

    /// Upper edge of the band in MHz.
    pub const fn top_mhz(self) -> f32 {
        self.top_khz() as f32 / 1000.0
    }

    pub const fn is_fm(self) -> bool {
        matches!(
            self,
            Self::FmBroadcast(_) | Self::FmUsEu(_) | Self::FmJapan(_) | Self::FmOirt(_)
        )
    }

    pub const fn is_am(self) -> bool {
        !self.is_fm()
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Unknown(_) => "unknown",
            Self::FmBroadcast(_) => "fm_broadcast",
            Self::FmUsEu(_) => "fm_us_eu",
            Self::FmJapan(_) => "fm_japan",
            Self::FmOirt(_) => "fm_oirt",
            Self::LwBroadcast(_) => "lw_broadcast",
            Self::AmBroadcast(_) => "am_broadcast",
            Self::SwBroadcast(_) => "sw_broadcast",
            Self::Sw120m(_) => "sw_120m",
            Self::Sw90m(_) => "sw_90m",
            Self::Sw75m(_) => "sw_75m",
            Self::Sw60m(_) => "sw_60m",
            Self::Sw49m(_) => "sw_49m",
            Self::Sw41m(_) => "sw_41m",
            Self::Sw31m(_) => "sw_31m",
            Self::Sw25m(_) => "sw_25m",
            Self::Sw22m(_) => "sw_22m",
            Self::Sw19m(_) => "sw_19m",
            Self::Sw16m(_) => "sw_16m",
            Self::Sw15m(_) => "sw_15m",
            Self::Sw13m(_) => "sw_13m",
            Self::Sw11m(_) => "sw_11m",
            Self::Ham2200m(_) => "ham_2200m",
            Self::Ham630m(_) => "ham_630m",
            Self::Ham160m(_) => "ham_160m",
            Self::Ham80m(_) => "ham_80m",
            Self::Ham60m(_) => "ham_60m",
            Self::Ham40m(_) => "ham_40m",
            Self::Ham30m(_) => "ham_30m",
            Self::Ham20m(_) => "ham_20m",
            Self::Ham17m(_) => "ham_17m",
            Self::Ham15m(_) => "ham_15m",
            Self::Ham12m(_) => "ham_12m",
            Self::Ham10m(_) => "ham_10m",
        }
    }

    pub const fn unknown_with_range(bottom_khz: u32, top_khz: u32) -> Self {
        Self::Unknown(BandRangeKhz {
            bottom_khz,
            top_khz,
        })
    }

    pub const fn from_bottom_top_khz(bottom_khz: u32, top_khz: u32) -> Self {
        if bottom_khz > top_khz {
            return Self::unknown_with_range(bottom_khz, top_khz);
        }

        let mut i = 0;
        let mut best = Self::unknown_with_range(bottom_khz, top_khz);
        // Bands overlap (e.g. `sw_broadcast` contains many SW sub-bands), so we
        // keep the narrowest containing range to return the most specific band.
        let mut best_width = u32::MAX;
        while i < Self::ALL_KNOWN_BANDS.len() {
            let band = Self::ALL_KNOWN_BANDS[i];
            // `sw_broadcast` is intentionally excluded: it is a very wide umbrella
            // range and would hide more useful sub-band detection behavior.
            if let Self::SwBroadcast(_) = band {
                i += 1;
                continue;
            }
            let range = band.range();
            if bottom_khz >= range.bottom_khz && top_khz <= range.top_khz {
                let width = range.top_khz - range.bottom_khz;
                if width < best_width {
                    best = band;
                    best_width = width;
                }
            }
            i += 1;
        }

        best
    }
}

impl core::str::FromStr for RadioBand {
    type Err = ParseRadioBandsError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim() {
            "unknown" => Ok(Self::UNKNOWN),
            "fm_broadcast" => Ok(Self::FM_BROADCAST),
            "fm_us_eu" => Ok(Self::FM_US_EU),
            "fm_japan" => Ok(Self::FM_JAPAN),
            "fm_oirt" => Ok(Self::FM_OIRT),
            "lw_broadcast" => Ok(Self::LW_BROADCAST),
            "am_broadcast" => Ok(Self::AM_BROADCAST),
            "sw_broadcast" => Ok(Self::SW_BROADCAST),
            "sw_120m" => Ok(Self::SW_120M),
            "sw_90m" => Ok(Self::SW_90M),
            "sw_75m" => Ok(Self::SW_75M),
            "sw_60m" => Ok(Self::SW_60M),
            "sw_49m" => Ok(Self::SW_49M),
            "sw_41m" => Ok(Self::SW_41M),
            "sw_31m" => Ok(Self::SW_31M),
            "sw_25m" => Ok(Self::SW_25M),
            "sw_22m" => Ok(Self::SW_22M),
            "sw_19m" => Ok(Self::SW_19M),
            "sw_16m" => Ok(Self::SW_16M),
            "sw_15m" => Ok(Self::SW_15M),
            "sw_13m" => Ok(Self::SW_13M),
            "sw_11m" => Ok(Self::SW_11M),
            "ham_2200m" => Ok(Self::HAM_2200M),
            "ham_630m" => Ok(Self::HAM_630M),
            "ham_160m" => Ok(Self::HAM_160M),
            "ham_80m" => Ok(Self::HAM_80M),
            "ham_60m" => Ok(Self::HAM_60M),
            "ham_40m" => Ok(Self::HAM_40M),
            "ham_30m" => Ok(Self::HAM_30M),
            "ham_20m" => Ok(Self::HAM_20M),
            "ham_17m" => Ok(Self::HAM_17M),
            "ham_15m" => Ok(Self::HAM_15M),
            "ham_12m" => Ok(Self::HAM_12M),
            "ham_10m" => Ok(Self::HAM_10M),
            _ => Ok(Self::UNKNOWN),
        }
    }
}

impl core::fmt::Display for RadioBand {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{} ({:.3}-{:.3} MHz)",
            self.as_str(),
            self.bottom_mhz(),
            self.top_mhz()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::RadioBand;
    use core::fmt;
    use core::fmt::Write as _;
    use core::str::FromStr;

    struct FixedBuf {
        bytes: [u8; 64],
        len: usize,
    }

    impl FixedBuf {
        const fn new() -> Self {
            Self {
                bytes: [0; 64],
                len: 0,
            }
        }

        fn as_str(&self) -> &str {
            core::str::from_utf8(&self.bytes[..self.len]).unwrap()
        }
    }

    impl fmt::Write for FixedBuf {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let src = s.as_bytes();
            let end = self.len + src.len();
            if end > self.bytes.len() {
                return Err(fmt::Error);
            }
            self.bytes[self.len..end].copy_from_slice(src);
            self.len = end;
            Ok(())
        }
    }

    #[test]
    fn broadcast_band_edges_are_ordered() {
        assert!(RadioBand::FM_BROADCAST.bottom_khz() < RadioBand::FM_BROADCAST.top_khz());
        assert!(RadioBand::FM_US_EU.bottom_khz() < RadioBand::FM_US_EU.top_khz());
        assert!(RadioBand::FM_JAPAN.bottom_khz() < RadioBand::FM_JAPAN.top_khz());
        assert!(RadioBand::LW_BROADCAST.bottom_khz() < RadioBand::LW_BROADCAST.top_khz());
        assert!(RadioBand::AM_BROADCAST.bottom_khz() < RadioBand::AM_BROADCAST.top_khz());
        assert!(RadioBand::SW_BROADCAST.bottom_khz() < RadioBand::SW_BROADCAST.top_khz());
    }

    #[test]
    fn ham_band_edges_are_ordered() {
        assert!(RadioBand::HAM_160M.bottom_khz() < RadioBand::HAM_160M.top_khz());
        assert!(RadioBand::HAM_20M.bottom_khz() < RadioBand::HAM_20M.top_khz());
        assert!(RadioBand::HAM_10M.bottom_khz() < RadioBand::HAM_10M.top_khz());
    }

    #[test]
    fn parses_known_band_names() {
        assert_eq!(
            RadioBand::from_str("fm_us_eu").unwrap(),
            RadioBand::FM_US_EU
        );
        assert_eq!(
            RadioBand::from_str("lw_broadcast").unwrap(),
            RadioBand::LW_BROADCAST
        );
        assert_eq!(RadioBand::from_str("sw_31m").unwrap(), RadioBand::SW_31M);
        assert_eq!(RadioBand::from_str("ham_20m").unwrap(), RadioBand::HAM_20M);
        assert_eq!(RadioBand::from_str("  sw_49m ").unwrap(), RadioBand::SW_49M);
    }

    #[test]
    fn unknown_input_maps_to_unknown_band() {
        assert_eq!(RadioBand::from_str("fm").unwrap(), RadioBand::UNKNOWN);
        assert_eq!(RadioBand::from_str("sw_50m").unwrap(), RadioBand::UNKNOWN);
    }

    #[test]
    fn converts_band_to_str() {
        assert_eq!(RadioBand::UNKNOWN.as_str(), "unknown");
        assert_eq!(RadioBand::FM_US_EU.as_str(), "fm_us_eu");
        assert_eq!(RadioBand::LW_BROADCAST.as_str(), "lw_broadcast");
        assert_eq!(RadioBand::SW_31M.as_str(), "sw_31m");
        assert_eq!(RadioBand::HAM_20M.as_str(), "ham_20m");
    }

    #[test]
    fn display_includes_name_and_mhz_range() {
        let mut s = FixedBuf::new();
        write!(&mut s, "{}", RadioBand::SW_31M).unwrap();
        assert_eq!(s.as_str(), "sw_31m (9.400-9.900 MHz)");
    }

    #[test]
    fn classifies_fm_and_am_bands() {
        assert!(RadioBand::FM_US_EU.is_fm());
        assert!(!RadioBand::FM_US_EU.is_am());

        assert!(RadioBand::AM_BROADCAST.is_am());
        assert!(!RadioBand::AM_BROADCAST.is_fm());

        assert!(RadioBand::UNKNOWN.is_am());
        assert!(!RadioBand::UNKNOWN.is_fm());
    }

    #[test]
    fn detects_band_when_range_is_inside() {
        assert_eq!(
            RadioBand::from_bottom_top_khz(88100, 107900),
            RadioBand::FM_US_EU
        );
        assert_eq!(
            RadioBand::from_bottom_top_khz(9600, 9800),
            RadioBand::SW_31M
        );
        assert_eq!(
            RadioBand::from_bottom_top_khz(153, 279),
            RadioBand::LW_BROADCAST
        );
    }

    #[test]
    fn returns_unknown_when_range_is_outside() {
        assert_eq!(
            RadioBand::from_bottom_top_khz(100, 120),
            RadioBand::unknown_with_range(100, 120)
        );
        assert_eq!(
            RadioBand::from_bottom_top_khz(2300, 26100),
            RadioBand::unknown_with_range(2300, 26100)
        );
        assert_eq!(
            RadioBand::from_bottom_top_khz(30000, 31000),
            RadioBand::unknown_with_range(30000, 31000)
        );
        assert_eq!(
            RadioBand::from_bottom_top_khz(10000, 9000),
            RadioBand::unknown_with_range(10000, 9000)
        );
    }

    #[test]
    fn for_each_visits_all_known_bands() {
        let mut visited = 0;
        RadioBand::for_each(|band| {
            assert_ne!(band, RadioBand::UNKNOWN);
            visited += 1;
        });
        assert_eq!(visited, RadioBand::ALL_KNOWN_BANDS.len());
    }
}
