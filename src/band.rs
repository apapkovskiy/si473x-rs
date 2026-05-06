/// Predefined AM, LW, SW, and HAM tuning bands for Si47xx AM mode.
///
/// Frequency ranges are in kilohertz and are based on common ITU broadcast
/// allocations plus commonly used HF amateur radio allocations.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BandRangeKhz {
    pub bottom_khz: u16,
    pub top_khz: u16,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AmLwSwBand {
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

impl AmLwSwBand {
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

    pub const fn range(self) -> BandRangeKhz {
        match self {
            Self::LwBroadcast(range)
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
    pub const fn bottom_khz(self) -> u16 {
        self.range().bottom_khz
    }

    /// Upper edge of the band in kHz.
    pub const fn top_khz(self) -> u16 {
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
}

#[cfg(test)]
mod tests {
    use super::AmLwSwBand;

    #[test]
    fn broadcast_band_edges_are_ordered() {
        assert!(AmLwSwBand::LW_BROADCAST.bottom_khz() < AmLwSwBand::LW_BROADCAST.top_khz());
        assert!(AmLwSwBand::AM_BROADCAST.bottom_khz() < AmLwSwBand::AM_BROADCAST.top_khz());
        assert!(AmLwSwBand::SW_BROADCAST.bottom_khz() < AmLwSwBand::SW_BROADCAST.top_khz());
    }

    #[test]
    fn ham_band_edges_are_ordered() {
        assert!(AmLwSwBand::HAM_160M.bottom_khz() < AmLwSwBand::HAM_160M.top_khz());
        assert!(AmLwSwBand::HAM_20M.bottom_khz() < AmLwSwBand::HAM_20M.top_khz());
        assert!(AmLwSwBand::HAM_10M.bottom_khz() < AmLwSwBand::HAM_10M.top_khz());
    }
}
