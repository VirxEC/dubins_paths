use core::{error::Error, fmt, ops::Add, result};

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

#[cfg(not(feature = "f64"))]
mod float_type {
    /// Alias for the f32 type used
    pub type FloatType = f32;
    /// Appropriate constants for the f32 precision type
    pub use core::f32::consts;

    #[cfg(feature = "glam")]
    pub type Vec2 = glam::Vec2;
}

#[cfg(feature = "f64")]
mod float_type {
    /// Alias for the f64 type used
    pub type FloatType = f64;
    /// Appropriate constants for the f64 precision type
    pub use core::f64::consts;

    #[cfg(feature = "glam")]
    pub type Vec2 = glam::DVec2;
}

#[cfg(feature = "glam")]
use float_type::Vec2;
pub use float_type::{consts, FloatType};

/// The car's position and rotation in radians
#[cfg(not(feature = "glam"))]
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct PosRot([FloatType; 3]);

#[cfg(not(feature = "glam"))]
impl PosRot {
    /// Create a new `PosRot` from a position and rotation
    #[inline]
    #[must_use]
    pub const fn from_floats(x: FloatType, y: FloatType, rot: FloatType) -> Self {
        Self([x, y, rot])
    }

    /// Get the x position
    #[inline]
    #[must_use]
    pub const fn x(&self) -> FloatType {
        self.0[0]
    }

    /// Get the y position
    #[inline]
    #[must_use]
    pub const fn y(&self) -> FloatType {
        self.0[1]
    }

    /// Get the rotation
    #[inline]
    #[must_use]
    pub const fn rot(&self) -> FloatType {
        self.0[2]
    }
}

/// The car's position and rotation in radians
#[cfg(feature = "glam")]
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct PosRot(Vec2, FloatType);

#[cfg(feature = "glam")]
impl PosRot {
    /// Create a new `PosRot` from a `Vec2` and rotation
    #[inline]
    #[must_use]
    pub const fn new(pos: Vec2, rot: FloatType) -> Self {
        Self(pos, rot)
    }

    /// Create a new `PosRot` from a position and rotation
    #[inline]
    #[must_use]
    pub const fn from_floats(x: FloatType, y: FloatType, rot: FloatType) -> Self {
        Self(Vec2::new(x, y), rot)
    }

    /// Get the position
    #[inline]
    #[must_use]
    pub const fn pos(&self) -> Vec2 {
        self.0
    }

    /// Get the x position
    #[inline]
    #[must_use]
    pub const fn x(&self) -> FloatType {
        self.0.x
    }

    /// Get the y position
    #[inline]
    #[must_use]
    pub const fn y(&self) -> FloatType {
        self.0.y
    }

    /// Get the rotation
    #[inline]
    #[must_use]
    pub const fn rot(&self) -> FloatType {
        self.1
    }
}

impl PosRot {
    #[inline]
    #[must_use]
    pub(crate) const fn from_rot(rot: Self) -> Self {
        Self::from_floats(0., 0., rot.rot())
    }
}

impl Add<Self> for PosRot {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::from_floats(
            self.x() + rhs.x(),
            self.y() + rhs.y(),
            self.rot() + rhs.rot(),
        )
    }
}

impl From<[FloatType; 3]> for PosRot {
    #[inline]
    fn from(posrot: [FloatType; 3]) -> Self {
        Self::from_floats(posrot[0], posrot[1], posrot[2])
    }
}

/// The normalized lengths of the path's segments
pub type Params = [FloatType; 3];

/// Ensure the given number is between 0 and 2pi
///
/// # Arguments
///
/// * `theta`: The value to be modded
#[inline]
#[must_use]
pub fn mod2pi(theta: FloatType) -> FloatType {
    let r = theta % consts::TAU;
    if r < 0.0 {
        r + consts::TAU
    } else {
        r
    }
}
