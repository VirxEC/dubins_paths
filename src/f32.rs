use crate::{NoPathError, PathType, Result, SegmentType};
use core::{f32::consts::TAU, ops::Add};

#[cfg(feature = "glam")]
use glam::Vec2;

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

#[cfg(feature = "alloc")]
use core::ops::{Bound, RangeBounds};

#[cfg(not(feature = "libm"))]
type Math = f32;

#[cfg(feature = "libm")]
type Math = libm::Libm<f32>;

/// The car's position and rotation in radians
#[cfg(not(feature = "glam"))]
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct PosRot([f32; 3]);

#[cfg(not(feature = "glam"))]
impl PosRot {
    /// Create a new [`PosRot`] from a position and rotation
    #[inline]
    #[must_use]
    pub const fn from_floats(x: f32, y: f32, rot: f32) -> Self {
        Self([x, y, rot])
    }

    /// Get the x position
    #[inline]
    #[must_use]
    pub const fn x(&self) -> f32 {
        self.0[0]
    }

    /// Get the y position
    #[inline]
    #[must_use]
    pub const fn y(&self) -> f32 {
        self.0[1]
    }

    /// Get the rotation
    #[inline]
    #[must_use]
    pub const fn rot(&self) -> f32 {
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
pub struct PosRot(Vec2, f32);

#[cfg(feature = "glam")]
impl PosRot {
    /// Create a new [`PosRot`] from a [`Vec2`] and rotation
    #[inline]
    #[must_use]
    pub const fn new(pos: Vec2, rot: f32) -> Self {
        Self(pos, rot)
    }

    /// Create a new [`PosRot`] from a position and rotation
    #[inline]
    #[must_use]
    pub const fn from_floats(x: f32, y: f32, rot: f32) -> Self {
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
    pub const fn x(&self) -> f32 {
        self.0.x
    }

    /// Get the y position
    #[inline]
    #[must_use]
    pub const fn y(&self) -> f32 {
        self.0.y
    }

    /// Get the rotation
    #[inline]
    #[must_use]
    pub const fn rot(&self) -> f32 {
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

impl From<[f32; 3]> for PosRot {
    #[inline]
    fn from(posrot: [f32; 3]) -> Self {
        Self::from_floats(posrot[0], posrot[1], posrot[2])
    }
}

/// The normalized lengths of the path's segments
pub type Params = [f32; 3];

/// Ensure the given number is between 0 and 2pi
///
/// # Arguments
///
/// * `theta`: The value to be modded
#[inline]
#[must_use]
pub(crate) fn mod2pi(theta: f32) -> f32 {
    let r = theta % TAU;
    if r < 0.0 { r + TAU } else { r }
}

/// The pre-calculated information that applies to every path type
///
/// To construct this type, use [`Intermediate::new`]
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct Intermediate {
    alpha: f32,
    beta: f32,
    d: f32,
    sa: f32,
    sb: f32,
    ca: f32,
    cb: f32,
    c_ab: f32,
    d_sq: f32,
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::{Intermediate, PosRot};
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
    pub fn new(q0: PosRot, q1: PosRot, rho: f32) -> Self {
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::{
    ///     PathType, Result as DubinsResult,
    ///     f32::{Intermediate, Params},
    /// };
    ///
    /// let intermediate_results = Intermediate::new(
    ///     [0., 0., PI / 4.].into(),
    ///     [100., -100., PI * (3. / 4.)].into(),
    ///     11.6,
    /// );
    ///
    /// let word: DubinsResult<Params> = intermediate_results.word(PathType::LSR);
    ///
    /// assert!(word.is_ok());
    /// ```
    #[inline]
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

/// All the basic information about Dubin's Paths
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)
)]
pub struct DubinsPath {
    /// The initial location (x, y, theta)
    pub qi: PosRot,
    /// The model's turn radius (forward velocity / angular velocity)
    pub rho: f32,
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::{
    ///     SegmentType,
    ///     f32::{DubinsPath, PosRot},
    /// };
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
    pub fn segment(t: f32, qi: PosRot, type_: SegmentType) -> PosRot {
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::{DubinsPath, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(
    ///     PosRot::from_floats(0., 0., PI / 4.),
    ///     PosRot::from_floats(100., -100., PI * (3. / 4.)),
    ///     11.6,
    /// )
    /// .unwrap();
    ///
    /// // Find the halfway point of the path
    /// let t = shortest_path_possible.length() / 2.;
    ///
    /// let position: PosRot = shortest_path_possible.sample(t);
    /// ```
    #[must_use]
    pub fn sample(&self, t: f32) -> PosRot {
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
        tprime: f32,
        types: [SegmentType; 3],
        qi: PosRot,
        q1: PosRot,
        q2: PosRot,
    ) -> PosRot {
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::{f32::{DubinsPath, PosRot}, PathType, Result as DubinsResult};
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
    pub fn shortest_in(q0: PosRot, q1: PosRot, rho: f32, types: &[PathType]) -> Result<Self> {
        let intermediate_results = Intermediate::new(q0, q1, rho);

        let params = types.iter().copied().flat_map(|path_type| {
            intermediate_results
                .word(path_type)
                .map(|param| (param, path_type))
        });

        let mut best = Err(NoPathError);
        let mut best_sum = f32::INFINITY;

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
    /// use core::f32::consts::PI;
    /// use dubins_paths::{
    ///     PathType, Result as DubinsResult,
    ///     f32::{DubinsPath, PosRot},
    /// };
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
    pub fn shortest_from(q0: PosRot, q1: PosRot, rho: f32) -> Result<Self> {
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::{
    ///     PathType, Result as DubinsResult,
    ///     f32::{DubinsPath, PosRot},
    /// };
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
    pub fn new(q0: PosRot, q1: PosRot, rho: f32, path_type: PathType) -> Result<Self> {
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::DubinsPath;
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(
    ///     [0., 0., PI / 4.].into(),
    ///     [100., -100., PI * (3. / 4.)].into(),
    ///     11.6,
    /// )
    /// .unwrap();
    ///
    /// let total_path_length = shortest_path_possible.length();
    /// ```
    #[inline]
    #[must_use]
    pub fn length(&self) -> f32 {
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::DubinsPath;
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(
    ///     [0., 0., PI / 4.].into(),
    ///     [100., -100., PI * (3. / 4.)].into(),
    ///     11.6,
    /// )
    /// .unwrap();
    ///
    /// // Each path has 3 segments
    /// // i must be in the range [0, 2]
    /// let total_segment_length = shortest_path_possible.segment_length(1);
    /// ```
    #[inline]
    #[must_use]
    pub fn segment_length(&self, i: usize) -> f32 {
        self.param[i] * self.rho
    }

    /// Get a vec of all the points along the path,
    /// with the start and end being sampled regardless of `step_distance`
    ///
    /// # Arguments
    ///
    /// * `step_distance`: The distance between each point
    ///
    /// # Examples
    ///
    /// ```
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::{DubinsPath, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(
    ///     [0., 0., PI / 4.].into(),
    ///     [100., -100., PI * (3. / 4.)].into(),
    ///     11.6,
    /// )
    /// .unwrap();
    ///
    /// // The distance between each sample point
    /// let step_distance = 5.;
    ///
    /// let samples: Vec<PosRot> = shortest_path_possible.sample_many(step_distance);
    /// assert_eq!(
    ///     samples.len(),
    ///     (shortest_path_possible.length() / step_distance) as usize + 1
    /// );
    /// ```
    #[must_use]
    #[cfg(feature = "alloc")]
    pub fn sample_many(&self, step_distance: f32) -> Vec<PosRot> {
        // special case where we know to sample the whole range
        debug_assert!(step_distance > 0.);

        let types = self.path_type.to_segment_types();

        let qi = PosRot::from_rot(self.qi);
        let q1 = Self::segment(self.param[0], qi, types[0]);
        let q2 = Self::segment(self.param[1], q1, types[1]);
        let sample_step_distance = step_distance / self.rho;

        let end = self.param.iter().sum();

        // Ignoring cast_sign_loss because we know step_distance should positive
        // Ignoring cast_possible_truncation because rounding down is the correct behavior
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mut num_samples = (end / sample_step_distance) as usize;

        // If the num of samples we have specified here floors to 0 for an Unbounded starting range limit - we intentionally up that to 1 to give us the starting point
        // This should cover full "interpolation" of curves with a distance close or under the sampling value
        num_samples = num_samples.max(1);

        (0..num_samples)
            .map(|i| {
                // There's nothing we can do about the precision loss
                #[allow(clippy::cast_precision_loss)]
                (i as f32 * sample_step_distance)
            })
            .map(|t| self.sample_cached(t, types, qi, q1, q2))
            .chain(core::iter::once(self.sample_cached(end, types, qi, q1, q2)))
            .collect()
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::{DubinsPath, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(
    ///     [0., 0., PI / 4.].into(),
    ///     [100., -100., PI * (3. / 4.)].into(),
    ///     11.6,
    /// )
    /// .unwrap();
    ///
    /// // The distance between each sample point
    /// let step_distance = 5.;
    ///
    /// // Sample from start_distance to end_distance
    /// let distances = 40.0..=120.0;
    ///
    /// let samples: Vec<PosRot> = shortest_path_possible.sample_many_range(step_distance, distances);
    /// assert_eq!(samples.len(), 16);
    /// ```
    #[must_use]
    #[cfg(feature = "alloc")]
    pub fn sample_many_range<T: RangeBounds<f32>>(
        &self,
        step_distance: f32,
        range: T,
    ) -> Vec<PosRot> {
        debug_assert!(step_distance > 0.);

        let types = self.path_type.to_segment_types();

        let qi = PosRot::from_rot(self.qi);
        let q1 = Self::segment(self.param[0], qi, types[0]);
        let q2 = Self::segment(self.param[1], q1, types[1]);
        let sample_step_distance = step_distance / self.rho;

        let mut start = match range.start_bound() {
            Bound::Included(start) => *start,
            Bound::Excluded(start) => *start + step_distance,
            Bound::Unbounded => 0.0,
        };
        let (end, includes_end) = match range.end_bound() {
            Bound::Included(end) => (*end, true),
            Bound::Excluded(end) => (*end, false),
            Bound::Unbounded => (self.length(), true),
        };

        let sample_range = (end - start) / step_distance;
        start /= self.rho;

        // Ignoring cast_sign_loss because we know step_distance is positive
        // Ignoring cast_possible_truncation because that is the correct behavior
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mut num_samples = sample_range as usize;

        // If the num of samples we have specified here floors to 0 for an Unbounded starting range limit - we intentionally up that to 1 to give us the starting point
        // This should cover full "interpolation" of curves with a distance close or under the sampling value
        if num_samples == 0 && range.start_bound() == Bound::Unbounded {
            num_samples = 1;
        }

        let mut samples: Vec<_> = (0..num_samples)
            .map(|i| {
                // There's nothing we can do about the precision loss
                #[allow(clippy::cast_precision_loss)]
                (i as f32 * sample_step_distance + start)
            })
            .map(|t| self.sample_cached(t, types, qi, q1, q2))
            .collect();

        if includes_end {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let max_num_samples = Math::ceil(sample_range) as usize;
            if max_num_samples != num_samples {
                samples.push(self.sample_cached(end / self.rho, types, qi, q1, q2));
            }
        }

        samples
    }

    /// Get the endpoint of the path
    ///
    /// # Examples
    ///
    /// ```
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::{DubinsPath, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(
    ///     [0., 0., PI / 4.].into(),
    ///     [100., -100., PI * (3. / 4.)].into(),
    ///     11.6,
    /// )
    /// .unwrap();
    ///
    /// let endpoint: PosRot = shortest_path_possible.endpoint();
    /// ```
    #[inline]
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
    /// use core::f32::consts::PI;
    /// use dubins_paths::f32::DubinsPath;
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(
    ///     [0., 0., PI / 4.].into(),
    ///     [100., -100., PI * (3. / 4.)].into(),
    ///     11.6,
    /// )
    /// .unwrap();
    ///
    /// // End the path halfway through the real path
    /// let t = shortest_path_possible.length() / 2.;
    ///
    /// let subpath: DubinsPath = shortest_path_possible.extract_subpath(t);
    /// ```
    #[must_use]
    pub fn extract_subpath(&self, t: f32) -> Self {
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
