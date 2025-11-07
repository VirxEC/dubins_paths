use core::{error::Error, fmt, result};

/// The three segment types in a Dubin's Path
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub enum SegmentType {
    /// Left-turning segment
    L,
    /// Straight segment
    S,
    /// Right-turning segment
    R,
}

impl SegmentType {
    /// Segments of a "Left Straight Left" path
    pub const LSL: [Self; 3] = [Self::L, Self::S, Self::L];
    /// Segments of a "Left Straight Right" path
    pub const LSR: [Self; 3] = [Self::L, Self::S, Self::R];
    /// Segments of a "Right Straight Left" path
    pub const RSL: [Self; 3] = [Self::R, Self::S, Self::L];
    /// Segments of a "Right Straight Right" path
    pub const RSR: [Self; 3] = [Self::R, Self::S, Self::R];
    /// Segments of a "Right Left Right" path
    pub const RLR: [Self; 3] = [Self::R, Self::L, Self::R];
    /// Segments of a "Left Right Left" path
    pub const LRL: [Self; 3] = [Self::L, Self::R, Self::L];
}

/// All the possible path types
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub enum PathType {
    #[default]
    /// A "Left Straight Left" Dubin's path
    LSL,
    /// A "Left Straight Right" Dubin's path
    LSR,
    /// A "Right Straight Left" Dubin's path
    RSL,
    /// A "Right Straight Right" Dubin's path
    RSR,
    /// A "Right Left Right" Dubin's path
    RLR,
    /// A "Left Right Left" Dubin's path
    LRL,
}

impl PathType {
    /// All of the "Corner Straight Corner" path types
    pub const CSC: [Self; 4] = [Self::LSL, Self::LSR, Self::RSL, Self::RSR];
    /// All of the "Corner Corner Corner" path types
    pub const CCC: [Self; 2] = [Self::RLR, Self::LRL];
    /// All of the path types
    pub const ALL: [Self; 6] = [
        Self::LSL,
        Self::LSR,
        Self::RSL,
        Self::RSR,
        Self::RLR,
        Self::LRL,
    ];

    /// Convert the path type an array of it's [`SegmentType`]s
    #[inline]
    #[must_use]
    pub const fn to_segment_types(&self) -> [SegmentType; 3] {
        match self {
            Self::LSL => SegmentType::LSL,
            Self::LSR => SegmentType::LSR,
            Self::RSL => SegmentType::RSL,
            Self::RSR => SegmentType::RSR,
            Self::RLR => SegmentType::RLR,
            Self::LRL => SegmentType::LRL,
        }
    }
}

/// The error returned when a path is not found
#[derive(Copy, Clone, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NoPathError;

impl fmt::Display for NoPathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "No path exists with given parameters")
    }
}

impl Error for NoPathError {}

/// A type that allows the function to return either
///
/// Ok(T) or Err([`NoPathError`])
pub type Result<T> = result::Result<T, NoPathError>;

/// [`glam`] is a crate that provides vector types, and used to provide a more ergonomic API
///
/// It requries the `glam` feature to be enabled in order to be used within this crate
#[cfg(feature = "glam")]
pub extern crate glam;
