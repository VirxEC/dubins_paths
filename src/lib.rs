#![warn(missing_docs, clippy::pedantic, clippy::all, clippy::nursery)]
#![allow(clippy::suboptimal_flops)]
#![forbid(unsafe_code)]
#![cfg_attr(not(feature = "std"), no_std)]

//! Calculates a path between two points in space with starting and ending rotation requirements.
//!
//! The car is assumed to be a dubin's car.
//! A dubin's car is a car that can only do 3 things: turn left, turn right, or go straight.
//!
//! ## Examples
//!
//! ### Basic usage
//!
//! This will calculate the path that connects the current position and rotation of the car to the desired position and rotation.
//!
//! ```
//! use dubins_paths::{DubinsPath, PI, PosRot, Result as DubinsResult};
//!
//! // PosRot represents the car's (Pos)ition and (Rot)ation
//! // Where x and y are the coordinates on a 2d plane
//! // and theta is the orientation of the car's front in radians
//!
//! // The starting position and rotation
//! // PosRot::from_floats can also be used for const contexts
//! const q0: PosRot = PosRot::from_floats(0., 0., PI / 4.);
//!
//! // The target end position and rotation
//! // PosRot implements From<[f32; 3]>
//! let q1 = [100., -100., PI * (3. / 4.)].into();
//!
//! // The car's turning radius (must be > 0)
//! // This can be calculated by taking a cars angular velocity and dividing it by the car's forward velocity
//! // `turn radius = ang_vel / forward_vel`
//! let rho = 11.6;
//!
//! // Calculate the shortest possible path between these two points with the given turning radius
//! let shortest_path_possible: DubinsResult<DubinsPath> = DubinsPath::shortest_from(q0, q1, rho);
//!
//! // Assert that the path was found!
//! assert!(shortest_path_possible.is_ok());
//! ```
//!
//! ### Sample path for points
//!
//! Calculating the path is very optimized, and does not include any points along the path.
//!
//! This means that if you want to get points along the path, extra work must be done.
//!
//! However, if this is not needed, lots of time is saved.
//!
//! Below, we calculate all points along a path spaced at a given interval. Use [`sample`] instead of [`sample_many`] to get only one point.
//!
//! ```
//! use dubins_paths::{DubinsPath, PI, PosRot};
//!
//! let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
//!
//! // The distance between each sample point
//! let step_distance = 5.;
//!
//! #[cfg(feature = "alloc")]
//! {
//!     let samples: Vec<PosRot> = shortest_path_possible.sample_many(step_distance);
//!
//!     // The path is just over 185 units long
//!     assert_eq!(shortest_path_possible.length().round(), 185.0);
//!
//!     // There are 37 points spaced 5 units apart (37 * 5 = 185), + 1 more for the endpoint
//!     assert_eq!(samples.len(), 38);
//! }
//! ```
//!
//! ## Features
//!
//! * `std` - (Default) Enables the use of the standard library
//! * `alloc` - (Default) Enables `sample` and `sample_many`
//! * `libm` - Enables the use of the `libm` crate for mathematical functions
//! * `glam` - Use a [`glam`](https://crates.io/crates/glam) compatible API
//! * `f64` - By default, the library uses `f32` precision and the equivalent `glam::f32` structs if that feature is enabled. Setting `f64` changes all numbers to 64-bit precision, and uses `glam::f64` vector types
//!
//! [`sample`]: DubinsPath::sample
//! [`sample_many`]: DubinsPath::sample_many

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!("Either the std or the libm feature is required");

#[cfg(all(not(feature = "std"), feature = "alloc"))]
extern crate alloc;

#[cfg(all(not(feature = "std"), feature = "alloc"))]
use alloc::vec::Vec;

/// [`glam`] is a crate that provides vector types, and used to provide a more ergonomic API
///
/// It requries the `glam` feature to be enabled in order to be used within this crate
#[cfg(feature = "glam")]
pub extern crate glam;

#[cfg(not(feature = "f64"))]
mod float_type {
    /// Alias for the f32 type used
    pub type FloatType = f32;
    /// Appropriate PI for the f32 precision type
    pub use core::f32::consts::{PI, TAU};

    #[cfg(feature = "glam")]
    pub type Vec2 = glam::Vec2;
}

#[cfg(feature = "f64")]
mod float_type {
    /// Alias for the f64 type used
    pub type FloatType = f64;
    /// Appropriate PI for the f64 precision type
    pub use core::f64::consts::{PI, TAU};

    #[cfg(feature = "glam")]
    pub type Vec2 = glam::DVec2;
}

#[cfg(feature = "glam")]
use float_type::Vec2;
pub use float_type::{FloatType, PI, TAU};

use core::{error::Error, fmt, ops::Add, result};

#[cfg(feature = "alloc")]
use core::ops::{Bound, RangeBounds};

#[cfg(not(feature = "libm"))]
type Math = FloatType;

#[cfg(feature = "libm")]
type Math = libm::Libm<FloatType>;

/// The three segment types in a Dubin's Path
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
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

/// The car's position and rotation in radians
#[cfg(not(feature = "glam"))]
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PosRot([FloatType; 3]);

#[cfg(not(feature = "glam"))]
impl PosRot {
    /// Create a new `PosRot` from a position and rotation
    #[cfg(not(feature = "f64"))]
    #[deprecated(since = "2.5.0", note = "please use `from_floats` instead")]
    #[must_use]
    pub const fn from_f32(x: FloatType, y: FloatType, rot: FloatType) -> Self {
        Self::from_floats(x, y, rot)
    }

    /// Create a new `PosRot` from a position and rotation
    #[must_use]
    pub const fn from_floats(x: FloatType, y: FloatType, rot: FloatType) -> Self {
        Self([x, y, rot])
    }

    /// Get the x position
    #[must_use]
    pub const fn x(&self) -> FloatType {
        self.0[0]
    }

    /// Get the y position
    #[must_use]
    pub const fn y(&self) -> FloatType {
        self.0[1]
    }

    /// Get the rotation
    #[must_use]
    pub const fn rot(&self) -> FloatType {
        self.0[2]
    }
}

/// The car's position and rotation in radians
#[cfg(feature = "glam")]
#[derive(Clone, Copy, Debug, Default)]
pub struct PosRot(Vec2, FloatType);

#[cfg(feature = "glam")]
impl PosRot {
    /// Create a new `PosRot` from a `Vec2` and rotation
    #[must_use]
    pub const fn new(pos: Vec2, rot: FloatType) -> Self {
        Self(pos, rot)
    }

    /// Create a new `PosRot` from a position and rotation
    #[cfg(not(feature = "f64"))]
    #[deprecated(since = "2.5.0", note = "please use `from_floats` instead")]
    #[must_use]
    pub const fn from_f32(x: FloatType, y: FloatType, rot: FloatType) -> Self {
        Self::from_floats(x, y, rot)
    }

    /// Create a new `PosRot` from a position and rotation
    #[must_use]
    pub const fn from_floats(x: FloatType, y: FloatType, rot: FloatType) -> Self {
        Self(Vec2::new(x, y), rot)
    }

    /// Get the position
    #[must_use]
    pub const fn pos(&self) -> Vec2 {
        self.0
    }

    /// Get the x position
    #[must_use]
    pub const fn x(&self) -> FloatType {
        self.0.x
    }

    /// Get the y position
    #[must_use]
    pub const fn y(&self) -> FloatType {
        self.0.y
    }

    /// Get the rotation
    #[must_use]
    pub const fn rot(&self) -> FloatType {
        self.1
    }
}

impl PosRot {
    #[must_use]
    const fn from_rot(rot: Self) -> Self {
        Self::from_floats(0., 0., rot.rot())
    }
}

impl Add<Self> for PosRot {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::from_floats(
            self.x() + rhs.x(),
            self.y() + rhs.y(),
            self.rot() + rhs.rot(),
        )
    }
}

impl From<[FloatType; 3]> for PosRot {
    fn from(posrot: [FloatType; 3]) -> Self {
        Self::from_floats(posrot[0], posrot[1], posrot[2])
    }
}

/// The normalized lengths of the path's segments
pub type Params = [FloatType; 3];

/// The pre-calculated information that applies to every path type
///
/// To construct this type, use [`Intermediate::new`]
#[derive(Copy, Clone, Debug, Default)]
pub struct Intermediate {
    alpha: FloatType,
    beta: FloatType,
    d: FloatType,
    sa: FloatType,
    sb: FloatType,
    ca: FloatType,
    cb: FloatType,
    c_ab: FloatType,
    d_sq: FloatType,
}

impl Intermediate {
    /// Pre-calculated values that are required by all of Dubin's Paths
    ///
    /// # Arguments
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{Intermediate, PI, PosRot};
    ///
    /// // The starting position
    /// let q0 = PosRot::from_floats(0., 0., PI / 4.);
    /// // The target end position
    /// let q1 = PosRot::from([100., -100., PI * (3. / 4.)]);
    /// // The car's turning radius (must be > 0)
    /// let rho = 11.6;
    ///
    /// let intermediate_results = Intermediate::new(q0, q1, rho);
    /// ```
    #[must_use]
    pub fn new(q0: PosRot, q1: PosRot, rho: FloatType) -> Self {
        debug_assert!(rho > 0.);

        let dx = q1.x() - q0.x();
        let dy = q1.y() - q0.y();
        let d = Math::hypot(dx, dy) / rho;

        // test required to prevent domain errors if dx=0 and dy=0
        let theta = mod2pi(Math::atan2(dy, dx));

        let alpha = mod2pi(q0.rot() - theta);
        let beta = mod2pi(q1.rot() - theta);

        Self {
            alpha,
            beta,
            d,
            sa: Math::sin(alpha),
            sb: Math::sin(beta),
            ca: Math::cos(alpha),
            cb: Math::cos(beta),
            c_ab: Math::cos(alpha - beta),
            d_sq: d * d,
        }
    }
}

impl Intermediate {
    /// Try to calculate a Left Straight Left path
    fn lsl(&self) -> Result<Params> {
        let p_sq = 2. * self.d * (self.sa - self.sb) + 2. + self.d_sq - 2. * self.c_ab;

        if p_sq >= 0. {
            let tmp0 = self.d + self.sa - self.sb;
            let tmp1 = Math::atan2(self.cb - self.ca, tmp0);

            Ok([
                mod2pi(tmp1 - self.alpha),
                Math::sqrt(p_sq),
                mod2pi(self.beta - tmp1),
            ])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Right Straight Right path
    fn rsr(&self) -> Result<Params> {
        let p_sq = 2. * self.d * (self.sb - self.sa) + 2. + self.d_sq - (2. * self.c_ab);

        if p_sq >= 0. {
            let tmp0 = self.d - self.sa + self.sb;
            let tmp1 = Math::atan2(self.ca - self.cb, tmp0);

            Ok([
                mod2pi(self.alpha - tmp1),
                Math::sqrt(p_sq),
                mod2pi(tmp1 - self.beta),
            ])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Left Straight Right path
    fn lsr(&self) -> Result<Params> {
        let p_sq = 2. * self.d * (self.sa + self.sb) + 2. * self.c_ab - 2. + self.d_sq;

        if p_sq >= 0. {
            let p = Math::sqrt(p_sq);
            let tmp0 =
                Math::atan2(-self.ca - self.cb, self.d + self.sa + self.sb) - Math::atan2(-2., p);

            Ok([
                mod2pi(tmp0 - self.alpha),
                p,
                mod2pi(tmp0 - mod2pi(self.beta)),
            ])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Right Straight Left path
    fn rsl(&self) -> Result<Params> {
        let p_sq = 2. * self.c_ab - 2. + self.d_sq - 2. * self.d * (self.sa + self.sb);

        if p_sq >= 0. {
            let p = Math::sqrt(p_sq);
            let tmp0 =
                Math::atan2(self.ca + self.cb, self.d - self.sa - self.sb) - Math::atan2(2., p);

            Ok([mod2pi(self.alpha - tmp0), p, mod2pi(self.beta - tmp0)])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Right Left Right path
    fn rlr(&self) -> Result<Params> {
        let tmp0 = (2. * self.d * (self.sa - self.sb) + 2. * self.c_ab + 6. - self.d_sq) / 8.;

        if tmp0.abs() <= 1. {
            let p = mod2pi(TAU - Math::acos(tmp0));
            let phi = Math::atan2(self.ca - self.cb, self.d - self.sa + self.sb);
            let t = mod2pi(self.alpha - phi + mod2pi(p / 2.));

            Ok([t, p, mod2pi(self.alpha - self.beta - t + mod2pi(p))])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Left Right Left path
    fn lrl(&self) -> Result<Params> {
        let tmp0 = (2. * self.d * (self.sb - self.sa) + 2. * self.c_ab + 6. - self.d_sq) / 8.;

        if tmp0.abs() <= 1. {
            let p = mod2pi(TAU - Math::acos(tmp0));
            let phi = Math::atan2(self.ca - self.cb, self.d + self.sa - self.sb);
            let t = mod2pi(-self.alpha - phi + p / 2.);

            Ok([t, p, mod2pi(mod2pi(self.beta) - self.alpha - t + mod2pi(p))])
        } else {
            Err(NoPathError)
        }
    }

    /// Calculate a specific Dubin's Path
    ///
    /// # Arguments
    ///
    /// * `path_type`: The Dubin's path type that's to be calculated.
    ///
    /// # Errors
    ///
    /// Will return a [`NoPathError`] if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{self, PI, Intermediate, PathType, Params};
    ///
    /// let intermediate_results = Intermediate::new([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6);
    ///
    /// let word: dubins_paths::Result<Params> = intermediate_results.word(PathType::LSR);
    ///
    /// assert!(word.is_ok());
    /// ```
    pub fn word(&self, path_type: PathType) -> Result<Params> {
        match path_type {
            PathType::LSL => self.lsl(),
            PathType::RSL => self.rsl(),
            PathType::LSR => self.lsr(),
            PathType::RSR => self.rsr(),
            PathType::LRL => self.lrl(),
            PathType::RLR => self.rlr(),
        }
    }
}

/// Ensure the given number is between 0 and 2pi
///
/// # Arguments
///
/// * `theta`: The value to be modded
#[must_use]
pub fn mod2pi(theta: FloatType) -> FloatType {
    let r = theta % TAU;
    if r < 0.0 {
        r + TAU
    } else {
        r
    }
}

/// All the basic information about Dubin's Paths
#[derive(Clone, Copy, Debug, Default)]
pub struct DubinsPath {
    /// The initial location (x, y, theta)
    pub qi: PosRot,
    /// The model's turn radius (forward velocity / angular velocity)
    pub rho: FloatType,
    /// The normalized lengths of the three segments
    pub param: Params,
    /// The type of the path
    pub path_type: PathType,
}

impl DubinsPath {
    /// Finds the `[x, y, theta]` along some distance of some type with some starting position
    ///
    /// If you're looking to find the position of the car along some distance of the path, use [`sample`] instead.
    ///
    /// # Arguments
    ///
    /// * `t`: Normalized distance along the segement (distance / rho).
    /// * `q0`: The starting location and orientation of the car.
    /// * `type_`: The segment type.
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PosRot, SegmentType};
    ///
    /// // Normalized distance along the segement (distance / rho)
    /// let t = 0.32158;
    /// // The starting position
    /// let qi = PosRot::from_floats(0., 0., PI / 4.);
    /// // The path type to be calculated, in this case it's Left Straight Right
    /// let type_: SegmentType = SegmentType::L;
    ///
    /// let position: PosRot = DubinsPath::segment(t, qi, type_);
    /// ```
    ///
    /// [`sample`]: DubinsPath::sample
    #[must_use]
    pub fn segment(t: FloatType, qi: PosRot, type_: SegmentType) -> PosRot {
        let rot = qi.rot();
        let st = Math::sin(rot);
        let ct = Math::cos(rot);

        let qt = match type_ {
            SegmentType::L => {
                PosRot::from_floats(Math::sin(rot + t) - st, -Math::cos(rot + t) + ct, t)
            }
            SegmentType::R => {
                PosRot::from_floats(-Math::sin(rot - t) + st, Math::cos(rot - t) - ct, -t)
            }
            SegmentType::S => PosRot::from_floats(ct * t, st * t, 0.),
        };

        qt + qi
    }

    /// Scale the target configuration, translate back to the original starting point
    fn offset(&self, q: PosRot) -> PosRot {
        PosRot::from_floats(
            q.x() * self.rho + self.qi.x(),
            q.y() * self.rho + self.qi.y(),
            mod2pi(q.rot()),
        )
    }

    /// Get car location and orientation long after some travel distance
    ///
    /// # Arguments
    ///
    /// * `t`: The travel distance - must be less than the total length of the path
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(PosRot::from_floats(0., 0., PI / 4.), PosRot::from_floats(100., -100., PI * (3. / 4.)), 11.6).unwrap();
    ///
    /// // Find the halfway point of the path
    /// let t = shortest_path_possible.length() / 2.;
    ///
    /// let position: PosRot = shortest_path_possible.sample(t);
    /// ```
    #[must_use]
    pub fn sample(&self, t: FloatType) -> PosRot {
        // tprime is the normalised variant of the parameter t
        let tprime = t / self.rho;
        let types = self.path_type.to_segment_types();

        // initial configuration
        let qi = PosRot::from_rot(self.qi);

        let q = if tprime < self.param[0] {
            Self::segment(tprime, qi, types[0])
        } else {
            let q1 = Self::segment(self.param[0], qi, types[0]); // end-of segment 1

            if tprime < self.param[0] + self.param[1] {
                Self::segment(tprime - self.param[0], q1, types[1])
            } else {
                let q2 = Self::segment(self.param[1], q1, types[1]); // end-of segment 2

                Self::segment(tprime - self.param[0] - self.param[1], q2, types[2])
            }
        };

        self.offset(q)
    }

    #[cfg(feature = "alloc")]
    fn sample_cached(
        &self,
        t: FloatType,
        types: [SegmentType; 3],
        qi: PosRot,
        q1: PosRot,
        q2: PosRot,
    ) -> PosRot {
        let tprime = t / self.rho;

        let q = if tprime < self.param[0] {
            Self::segment(tprime, qi, types[0])
        } else if tprime < self.param[0] + self.param[1] {
            Self::segment(tprime - self.param[0], q1, types[1])
        } else {
            Self::segment(tprime - self.param[0] - self.param[1], q2, types[2])
        };

        self.offset(q)
    }

    /// Find the shortest path out of the specified path types
    ///
    /// # Arguments
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    /// * `types`: A reference to a slice that contains the path types to be compared.
    ///
    /// # Errors
    ///
    /// Will return a [`NoPathError`] if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PathType, PosRot, Result as DubinsResult};
    ///
    /// // The starting position
    /// let q0: PosRot = [0., 0., PI / 4.].into();
    /// // The target end position
    /// let q1: PosRot = [100., -100., PI * (3. / 4.)].into();
    /// // The car's turning radius (must be > 0)
    /// let rho = 11.6;
    /// // A slice of the PathTypes that we should compare
    /// // Some are predefined, like CCC (Corner Corner Corner), CSC (Corner Straight Corner), and ALL
    /// // If you plan on using ALL, consider using the shortcut function `DubinsPath::shortest_from(q0, q1, rho)`
    /// let types: &[PathType] = &PathType::CSC;
    ///
    /// let shortest_path_in_selection: DubinsResult<DubinsPath> = DubinsPath::shortest_in(q0, q1, rho, types);
    ///
    /// assert!(shortest_path_in_selection.is_ok());
    /// ```
    pub fn shortest_in(q0: PosRot, q1: PosRot, rho: FloatType, types: &[PathType]) -> Result<Self> {
        let intermediate_results = Intermediate::new(q0, q1, rho);

        let params = types.iter().copied().flat_map(|path_type| {
            intermediate_results
                .word(path_type)
                .map(|param| (param, path_type))
        });

        let mut best = Err(NoPathError);
        let mut best_sum = FloatType::INFINITY;

        for (param, path_type) in params {
            let sum = param.iter().sum();

            if sum < best_sum {
                best = Ok((param, path_type));
                best_sum = sum;
            }
        }

        best.map(|(param, path_type)| Self {
            qi: q0,
            rho,
            param,
            path_type,
        })
    }

    /// Find the shortest path out of the 6 path types
    ///
    /// # Arguments
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    ///
    /// # Errors
    ///
    /// Will return a [`NoPathError`] if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PathType, PosRot, Result as DubinsResult};
    ///
    /// // The starting position
    /// let q0: PosRot = [0., 0., PI / 4.].into();
    /// // The target end position
    /// let q1: PosRot = [100., -100., PI * (3. / 4.)].into();
    /// // The car's turning radius (must be > 0)
    /// let rho = 11.6;
    ///
    /// let shortest_path_possible: DubinsResult<DubinsPath> = DubinsPath::shortest_from(q0, q1, rho);
    ///
    /// assert!(shortest_path_possible.is_ok());
    /// ```
    pub fn shortest_from(q0: PosRot, q1: PosRot, rho: FloatType) -> Result<Self> {
        Self::shortest_in(q0, q1, rho, &PathType::ALL)
    }

    /// Calculate the specified path type
    ///
    /// # Arguments
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    /// * `path_type`: The Dubin's path type that's to be calculated.
    ///
    /// # Errors
    ///
    /// Will return a [`NoPathError`] if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PathType, PosRot, Result as DubinsResult};
    ///
    /// // The starting position
    /// let q0: PosRot = [0., 0., PI / 4.].into();
    /// // The target end position
    /// let q1: PosRot = [100., -100., PI * (3. / 4.)].into();
    /// // The car's turning radius (must be > 0)
    /// let rho = 11.6;
    /// // The path type to be calculated, in this case it's Left Straight Right
    /// let path_type: PathType = PathType::LSR;
    ///
    /// let path: DubinsResult<DubinsPath> = DubinsPath::new(q0, q1, rho, path_type);
    ///
    /// assert!(path.is_ok());
    /// ```
    pub fn new(q0: PosRot, q1: PosRot, rho: FloatType, path_type: PathType) -> Result<Self> {
        Ok(Self {
            qi: q0,
            rho,
            param: Intermediate::new(q0, q1, rho).word(path_type)?,
            path_type,
        })
    }

    /// Calculate the total distance of any given path.
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// let total_path_length = shortest_path_possible.length();
    /// ```
    #[must_use]
    pub fn length(&self) -> FloatType {
        (self.param[0] + self.param[1] + self.param[2]) * self.rho
    }

    /// Calculate the total distance of the path segment
    ///
    /// # Arguments
    ///
    /// `i`: Index of the segment to get the length of in the range \[0, 2]
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// // Each path has 3 segments
    /// // i must be in the range [0, 2]
    /// let total_segment_length = shortest_path_possible.segment_length(1);
    /// ```
    #[must_use]
    pub fn segment_length(&self, i: usize) -> FloatType {
        self.param[i] * self.rho
    }

    /// Get a vec of all the points along the path
    ///
    /// # Arguments
    ///
    /// * `step_distance`: The distance between each point
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// // The distance between each sample point
    /// let step_distance = 5.;
    ///
    /// let samples: Vec<PosRot> = shortest_path_possible.sample_many(step_distance);
    /// ```
    #[must_use]
    #[cfg(feature = "alloc")]
    pub fn sample_many(&self, step_distance: FloatType) -> Vec<PosRot> {
        self.sample_many_range(step_distance, ..)
    }

    /// Get a vec of all the points along the path, within the specified range
    ///
    /// # Arguments
    ///
    /// * `step_distance`: The distance between each point
    /// * `range`: The start and end distance of the path to sample
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// // The distance between each sample point
    /// let step_distance = 5.;
    ///
    /// // Sample between start_distance..end_distance
    /// let distances = 40.0..120.0;
    ///
    /// let samples: Vec<PosRot> = shortest_path_possible.sample_many_range(step_distance, distances);
    /// assert_eq!(samples.len(), 16);
    /// ```
    #[must_use]
    #[cfg(feature = "alloc")]
    pub fn sample_many_range<T: RangeBounds<FloatType>>(
        &self,
        step_distance: FloatType,
        range: T,
    ) -> Vec<PosRot> {
        debug_assert!(step_distance > 0.);

        let types = self.path_type.to_segment_types();

        let qi = PosRot::from_rot(self.qi);
        let q1 = Self::segment(self.param[0], qi, types[0]);
        let q2 = Self::segment(self.param[1], q1, types[1]);

        let start = match range.start_bound() {
            Bound::Included(start) => *start,
            Bound::Excluded(start) => *start + step_distance,
            Bound::Unbounded => 0.0,
        };
        let (end, includes_end) = match range.end_bound() {
            Bound::Included(end) => (*end, true),
            Bound::Excluded(end) => (*end, false),
            Bound::Unbounded => (self.length(), true),
        };

        // Ignoring cast_sign_loss because we know step_distance should positive
        // Ignoring cast_possible_truncation because we rounded down using f32::floor()
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mut num_samples = Math::floor((end - start) / step_distance) as u32;

        // If the num of samples we have specified here floors to 0 for an Unbounded starting range limit - we intentionally up that to 1 to give us the starting point
        // This should cover full "interpolation" of curves with a distance close or under the sampling value
        if num_samples == 0 && range.start_bound() == Bound::Unbounded {
            num_samples = 1;
        }

        let mut samples: Vec<_> = (0..num_samples)
            .map(|i| {
                // Since the value originally comes from FloatType,
                // this should be fine
                #[allow(clippy::cast_precision_loss)]
                #[allow(clippy::cast_lossless)]
                (i as FloatType * step_distance + start)
            })
            .map(|t| self.sample_cached(t, types, qi, q1, q2))
            .collect();

        if includes_end {
            samples.push(self.sample_cached(end, types, qi, q1, q2));
        }

        samples
    }

    /// Get the endpoint of the path
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// let endpoint: PosRot = shortest_path_possible.endpoint();
    /// ```
    #[must_use]
    pub fn endpoint(&self) -> PosRot {
        self.sample(self.length())
    }

    /// Extract a subpath from a path
    ///
    /// # Arguments
    ///
    /// * `path`: The path take the subpath from
    /// * `t`: The length along the path to end the subpath
    ///
    /// # Examples
    ///
    /// ```
    /// use dubins_paths::{DubinsPath, PI};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// // End the path halfway through the real path
    /// let t = shortest_path_possible.length() / 2.;
    ///
    /// let subpath: DubinsPath = shortest_path_possible.extract_subpath(t);
    /// ```
    #[must_use]
    pub fn extract_subpath(&self, t: FloatType) -> Self {
        // calculate the true parameter
        let tprime = t / self.rho;

        // fix the parameters
        let param0 = self.param[0].min(tprime);
        let param1 = self.param[1].min(tprime - param0);
        let param2 = self.param[2].min(tprime - param0 - param1);

        // copy most of the data
        Self {
            qi: self.qi,
            rho: self.rho,
            param: [param0, param1, param2],
            path_type: self.path_type,
        }
    }
}

#[cfg(test)]
mod tests {

    use super::{mod2pi, DubinsPath, FloatType, NoPathError, PathType, PosRot, SegmentType, PI};
    use core::mem::size_of;
    use rand::Rng;

    const TURN_RADIUS: FloatType = 1. / 0.00076;

    // f32 seems to have more precision issues in this space. 64bit calcs work well enough with epsilon though
    #[cfg(feature = "f64")]
    const POSROT_EPSILON: FloatType = FloatType::EPSILON;
    #[cfg(not(feature = "f64"))]
    const POSROT_EPSILON: FloatType = 0.00001_f32;

    #[test]
    fn mod2pi_test() {
        assert!(mod2pi(-FloatType::from_bits(1)) >= 0.);
        assert!(mod2pi(2. * PI).abs() < FloatType::EPSILON);
    }

    #[cfg(all(feature = "glam", not(feature = "f64")))]
    mod glam_test_vec {
        pub type Vec = glam::Vec3A;
    }
    #[cfg(all(feature = "glam", feature = "f64"))]
    mod glam_test_vec {
        pub type Vec = glam::DVec3;
    }

    #[test]
    fn many_path_correctness() {
        #[cfg(feature = "glam")]
        fn angle_2d(vec1: FloatType, vec2: FloatType) -> FloatType {
            glam_test_vec::Vec::new(vec1.cos(), vec1.sin(), 0.)
                .dot(glam_test_vec::Vec::new(vec2.cos(), vec2.sin(), 0.))
                .clamp(-1., 1.)
                .acos()
        }

        #[cfg(not(feature = "glam"))]
        fn angle_2d(vec1: FloatType, vec2: FloatType) -> FloatType {
            (vec1.cos() * vec1.cos() + vec2.sin() * vec2.sin())
                .clamp(-1., 1.)
                .acos()
        }

        // Test that the path is correct for a number of random configurations.
        // If no path is found, just skip.
        // If the path is found the sampled endpoint is different from the specified endpoint, then fail.

        let runs = 50_000;
        let mut thread_rng = rand::rng();

        for _ in 0..runs {
            let q0 = PosRot::from_floats(
                thread_rng.random_range((-10000.0 as FloatType)..(10000.0 as FloatType)),
                thread_rng.random_range((-10000.0 as FloatType)..(10000.0 as FloatType)),
                thread_rng.random_range(((-2.0 as FloatType) * PI)..((2.0 as FloatType) * PI)),
            );
            let q1 = PosRot::from_floats(
                thread_rng.random_range((-10000.0 as FloatType)..(10000.0 as FloatType)),
                thread_rng.random_range((-10000.0 as FloatType)..(10000.0 as FloatType)),
                thread_rng.random_range(((-2.0 as FloatType) * PI)..((2.0 as FloatType) * PI)),
            );

            let Ok(path) = DubinsPath::shortest_from(q0, q1, TURN_RADIUS) else {
                continue;
            };

            let start = path.sample(0.);

            #[cfg(feature = "glam")]
            let dist_diff = q0.pos().distance(start.pos());

            #[cfg(not(feature = "glam"))]
            let dist_diff = (q0.x() - start.x()).hypot(q0.y() - start.y());

            let rot_diff = angle_2d(q0.rot(), start.rot());

            assert!(
                dist_diff <= 0.1 && rot_diff <= 0.1,
                "Start is different! {:?} | {q0:?} | {start:?} | {dist_diff}, {rot_diff}",
                path.path_type
            );

            let endpoint = path.endpoint();

            #[cfg(feature = "glam")]
            let dist_diff = q1.pos().distance(endpoint.pos());

            #[cfg(not(feature = "glam"))]
            let dist_diff = (q1.x() - endpoint.x()).hypot(q1.y() - endpoint.y());

            let rot_diff = angle_2d(q1.rot(), endpoint.rot());

            assert!(
                dist_diff <= 0.1 && rot_diff <= 0.1,
                "Endpoint is different! {:?} | {q1:?} | {endpoint:?} | {dist_diff}, {rot_diff}",
                path.path_type
            );
        }
    }

    #[test]
    fn size_of_items() {
        #[cfg(not(feature = "f64"))]
        assert_eq!(size_of::<PosRot>(), 12);
        #[cfg(feature = "f64")]
        assert_eq!(size_of::<PosRot>(), 24);
        #[cfg(not(feature = "f64"))]
        assert_eq!(size_of::<DubinsPath>(), 32);
        #[cfg(feature = "f64")]
        assert_eq!(size_of::<DubinsPath>(), 64);
        assert_eq!(size_of::<PathType>(), 1);
        assert_eq!(size_of::<SegmentType>(), 1);
        assert_eq!(size_of::<NoPathError>(), 0);
    }

    macro_rules! test_pos_rot_equivalence {
        ($a:expr, $b:expr, $epsilon_diff:expr) => {
            approx::assert_ulps_eq!($a.x(), $b.x(), max_ulps = 4, epsilon = $epsilon_diff);
            approx::assert_ulps_eq!($a.y(), $b.y(), max_ulps = 4, epsilon = $epsilon_diff);
            approx::assert_ulps_eq!($a.rot(), $b.rot(), max_ulps = 4, epsilon = $epsilon_diff);
        };
    }

    #[test]
    #[allow(clippy::approx_constant)]
    #[allow(clippy::excessive_precision)]
    fn sample_many_correctness() {
        let start_pose = PosRot::from_floats(0.0, 8.5, 1.570_796_326_794_896_6);
        let goal_pose = PosRot::from_floats(
            -3.521_126_760_563_390_7,
            23.779_342_723_004_696,
            2.793_089_315_952_38,
        );
        let rho = 8.0;

        let path = DubinsPath::shortest_from(start_pose, goal_pose, rho).expect("Path Error");

        test_pos_rot_equivalence!(&path.qi, &start_pose, POSROT_EPSILON);
        test_pos_rot_equivalence!(&path.endpoint(), &goal_pose, POSROT_EPSILON);

        #[cfg(feature = "alloc")]
        {
            let interpolated = path.sample_many(0.4);
            let first_point = interpolated.first().unwrap();
            let last_pose = interpolated.last().unwrap();
            test_pos_rot_equivalence!(first_point, &start_pose, POSROT_EPSILON);
            test_pos_rot_equivalence!(last_pose, &goal_pose, POSROT_EPSILON);
        }
    }

    #[test]
    #[cfg(feature = "alloc")]
    fn sample_many_ranges() {
        let path = DubinsPath::shortest_from(
            [0., 0., PI / 4.].into(),
            [100., -100., PI * (3. / 4.)].into(),
            11.6,
        )
        .unwrap();

        // The distance between each sample point
        let step_distance = 5.;

        let zero: FloatType = 0.0;

        // There are 37 points spaced 5 units apart (37 * 5 = 185), + 1 more for the endpoint
        assert_eq!(
            path.sample_many_range(step_distance, zero..=path.length())
                .len(),
            38
        );
        assert_eq!(
            path.sample_many_range(step_distance, zero..path.length())
                .len(),
            37
        );
        assert_eq!(
            path.sample_many_range(step_distance, ..path.length()).len(),
            37
        );
        assert_eq!(path.sample_many_range(step_distance, zero..).len(), 38);
        assert_eq!(path.sample_many_range(step_distance, ..).len(), 38);
    }

    #[test]
    #[cfg(feature = "alloc")]
    #[allow(clippy::excessive_precision)]
    fn sample_many_small_interpolation() {
        // Tests a small real case of a curve with a distance less than the interpolation/sampling distance
        // In this case - we should see a degenerate "straight line" as the result

        let qi = PosRot::from_floats(
            567_105.332_258_019_13,
            6_909_411.667_294_749_1,
            1.603_392_277_147_537_1,
        );

        let path = DubinsPath {
            qi,
            rho: 4.123_025_907_936_836_1,
            param: [
                0.053_869_577_187_711_57,
                0.232_106_506_469_036_27,
                0.433_657_285_200_080_34,
            ],
            path_type: PathType::LSL,
        };

        let sample_distance = 3.0;
        assert!(path.length() < sample_distance);
        let sampled = path.sample_many(sample_distance);
        assert!(sampled.len() == 2);

        // Check we have a degenerate line with just start and end positions
        test_pos_rot_equivalence!(sampled.first().unwrap(), qi, POSROT_EPSILON);
        test_pos_rot_equivalence!(sampled.last().unwrap(), path.endpoint(), POSROT_EPSILON);
    }
}
