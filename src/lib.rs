#[cfg(feature = "glam")]
use glam::Vec3A;
use std::{error::Error, f32::consts::PI, fmt, result};

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
    pub const ALL: [Self; 6] = [Self::LSL, Self::LSR, Self::RSL, Self::RSR, Self::RLR, Self::LRL];

    /// Convert the path type an array of it's segment types
    #[must_use]
    pub const fn to_segment_types(&self) -> [SegmentType; 3] {
        match self {
            Self::LSL => [SegmentType::L, SegmentType::S, SegmentType::L],
            Self::LSR => [SegmentType::L, SegmentType::S, SegmentType::R],
            Self::RSL => [SegmentType::R, SegmentType::S, SegmentType::L],
            Self::RSR => [SegmentType::R, SegmentType::S, SegmentType::R],
            Self::RLR => [SegmentType::R, SegmentType::L, SegmentType::R],
            Self::LRL => [SegmentType::L, SegmentType::R, SegmentType::L],
        }
    }
}

/// The one and only error that can be returned by the library, when a path is not found
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
/// Ok(T) or Err(DubinsError)
pub type Result<T> = result::Result<T, NoPathError>;

/// The car's positon and rotation: \[x, y, theta]
#[cfg(not(feature = "glam"))]
pub type PosRot = [f32; 3];

/// The car's position and rotation: (\Vec3A(x, y, 0.), theta)
#[cfg(feature = "glam")]
#[derive(Clone, Copy, Debug, Default)]
pub struct PosRot {
    pub pos: Vec3A,
    pub rot: f32,
}

#[cfg(feature = "glam")]
impl PosRot {
    /// Create a new PosRot from a Vec3A and rotation
    pub const fn new(pos: Vec3A, rot: f32) -> Self {
        Self {
            pos,
            rot,
        }
    }

    /// Create a new PosRot from a position and rotation
    pub const fn from_f32(x: f32, y: f32, rot: f32) -> Self {
        Self {
            pos: Vec3A::new(x, y, 0.),
            rot,
        }
    }
}

#[cfg(feature = "glam")]
impl From<[f32; 3]> for PosRot {
    fn from(posrot: [f32; 3]) -> Self {
        Self::from_f32(posrot[0], posrot[1], posrot[2])
    }
}

/// The normalized lengths of the path's segments
pub type Params = [f32; 3];

/// The pre-calculated information that applies to every path type
///
/// To construct this type, use `Intermediate::from`
#[derive(Copy, Clone, Debug, Default)]
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

#[cfg(not(feature = "glam"))]
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
    /// use std::f32::consts::PI;
    /// use dubins_paths::{Intermediate, PosRot};
    ///
    /// // The starting position
    /// let q0: PosRot = [0., 0., PI / 4.];
    /// // The target end position
    /// let q1: PosRot = [100., -100., PI * (3. / 4.)];
    /// // The car's turning radius (must be > 0)
    /// let rho: f32 = 11.6;
    ///
    /// let intermediate_results = Intermediate::from(q0, q1, rho);
    /// ```
    #[must_use]
    pub fn from(q0: PosRot, q1: PosRot, rho: f32) -> Self {
        let dx = q1[0] - q0[0];
        let dy = q1[1] - q0[1];
        let d = dx.hypot(dy) / rho;

        // test required to prevent domain errors if dx=0 and dy=0
        let theta = if d > 0. {
            mod2pi(dy.atan2(dx))
        } else {
            0.
        };

        let alpha = mod2pi(q0[2] - theta);
        let beta = mod2pi(q1[2] - theta);

        Self {
            alpha,
            beta,
            d,
            sa: alpha.sin(),
            sb: beta.sin(),
            ca: alpha.cos(),
            cb: beta.cos(),
            c_ab: (alpha - beta).cos(),
            d_sq: d * d,
        }
    }
}

#[cfg(feature = "glam")]
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
    /// use glam::Vec3A;
    /// use std::f32::consts::PI;
    /// use dubins_paths::{Intermediate, PosRot};
    ///
    /// // The starting position
    /// let q0 = PosRot::new(Vec3A::ZERO, PI / 4.);
    /// // The target end position
    /// let q1 = PosRot::from_f32(100., -100., PI * (3. / 4.));
    /// // The car's turning radius (must be > 0)
    /// let rho: f32 = 11.6;
    ///
    /// let intermediate_results = Intermediate::from(q0, q1, rho);
    /// ```
    #[must_use]
    pub fn from(q0: PosRot, q1: PosRot, rho: f32) -> Self {
        let q = q1.pos - q0.pos;
        let d = q.x.hypot(q.y) / rho;

        // test required to prevent domain errors if q.x=0 and q.y=0
        let theta = if d > 0. {
            mod2pi(q.y.atan2(q.x))
        } else {
            0.
        };

        let alpha = mod2pi(q0.rot - theta);
        let beta = mod2pi(q1.rot - theta);

        Self {
            alpha,
            beta,
            d,
            sa: alpha.sin(),
            sb: beta.sin(),
            ca: alpha.cos(),
            cb: beta.cos(),
            c_ab: (alpha - beta).cos(),
            d_sq: d * d,
        }
    }
}

impl Intermediate {
    /// Try to calculate a Left Straight Left path
    fn lsl(&self) -> Result<Params> {
        let p_sq = (2. * self.d).mul_add(self.sa - self.sb, 2. + self.d_sq - (2. * self.c_ab));

        if p_sq >= 0. {
            let tmp0 = self.d + self.sa - self.sb;
            let tmp1 = (self.cb - self.ca).atan2(tmp0);

            Ok([mod2pi(tmp1 - self.alpha), p_sq.sqrt(), mod2pi(self.beta - tmp1)])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Right Straight Right path
    fn rsr(&self) -> Result<Params> {
        let p_sq = (2. * self.d).mul_add(self.sb - self.sa, 2. + self.d_sq - (2. * self.c_ab));

        if p_sq >= 0. {
            let tmp0 = self.d - self.sa + self.sb;
            let tmp1 = (self.ca - self.cb).atan2(tmp0);

            Ok([mod2pi(self.alpha - tmp1), p_sq.sqrt(), mod2pi(tmp1 - self.beta)])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Left Straight Right path
    fn lsr(&self) -> Result<Params> {
        let p_sq = (2. * self.d).mul_add(self.sa + self.sb, 2.0f32.mul_add(self.c_ab, -2. + (self.d_sq)));

        if p_sq >= 0. {
            let p = p_sq.sqrt();
            let tmp0 = (-self.ca - self.cb).atan2(self.d + self.sa + self.sb) - (-2_f32).atan2(p);

            Ok([mod2pi(tmp0 - self.alpha), p, mod2pi(tmp0 - mod2pi(self.beta))])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Right Straight Left path
    fn rsl(&self) -> Result<Params> {
        let p_sq = 2.0f32.mul_add(self.c_ab, -2. + self.d_sq) - (2. * self.d * (self.sa + self.sb));

        if p_sq >= 0. {
            let p = p_sq.sqrt();
            let tmp0 = (self.ca + self.cb).atan2(self.d - self.sa - self.sb) - (2_f32).atan2(p);

            Ok([mod2pi(self.alpha - tmp0), p, mod2pi(self.beta - tmp0)])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Right Left Right path
    fn rlr(&self) -> Result<Params> {
        let tmp0 = (2. * self.d).mul_add(self.sa - self.sb, 2.0f32.mul_add(self.c_ab, 6. - self.d_sq)) / 8.;
        let phi = (self.ca - self.cb).atan2(self.d - self.sa + self.sb);

        if tmp0.abs() <= 1. {
            let p = mod2pi((2. * PI) - tmp0.acos());
            let t = mod2pi(self.alpha - phi + mod2pi(p / 2.));

            Ok([t, p, mod2pi(self.alpha - self.beta - t + mod2pi(p))])
        } else {
            Err(NoPathError)
        }
    }

    /// Try to calculate a Left Right Left path
    fn lrl(&self) -> Result<Params> {
        let tmp0 = (2. * self.d).mul_add(self.sb - self.sa, 2.0f32.mul_add(self.c_ab, 6. - self.d_sq)) / 8.;
        let phi = (self.ca - self.cb).atan2(self.d + self.sa - self.sb);

        if tmp0.abs() <= 1. {
            let p = mod2pi(2. * PI - tmp0.acos());
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
    /// Will return a `NoPathError` if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::{self, Intermediate, PathType, Params};
    ///
    /// let intermediate_results = Intermediate::from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6);
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

/// Floating point modulus suitable for rings
///
/// # Arguments
///
/// * `x`: The value to be modded
/// * `y`: The modulus
fn fmodr(x: f32, y: f32) -> f32 {
    x - y * (x / y).floor()
}

/// Ensure the given number is between 0 and 2pi
///
/// # Arguments
///
/// * `theta`: The value to be modded
#[must_use]
pub fn mod2pi(theta: f32) -> f32 {
    fmodr(theta, 2. * PI)
}

/// All the basic information about Dubin's Paths
#[derive(Clone, Copy, Debug, Default)]
pub struct DubinsPath {
    /// The initial location (x, y, theta)
    pub qi: PosRot,
    /// The model's turn radius (forward velocity / angular velocity)
    pub rho: f32,
    /// The normalized lengths of the three segments
    pub param: Params,
    /// The type of the path
    pub type_: PathType,
}

#[cfg(not(feature = "glam"))]
impl DubinsPath {
    /// Finds the `[x, y, theta]` along some distance of some type with some starting position
    ///
    /// If you're looking to find the position of the car along some distance of the path, use `DubinsPath::sample` instead.
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
    /// use std::f32::consts::PI;
    /// use dubins_paths::{DubinsPath, PosRot, SegmentType};
    ///
    /// // Normalized distance along the segement (distance / rho)
    /// let t = 0.32158;
    /// // The starting position
    /// let qi: PosRot = [0., 0., PI / 4.];
    /// // The path type to be calculated, in this case it's Left Straight Right
    /// let type_: SegmentType = SegmentType::L;
    ///
    /// let position: PosRot = DubinsPath::segment(t, qi, type_);
    /// ```
    #[must_use]
    pub fn segment(t: f32, qi: PosRot, type_: SegmentType) -> PosRot {
        let st = qi[2].sin();
        let ct = qi[2].cos();

        let qt = match type_ {
            SegmentType::L => [(qi[2] + t).sin() - st, -(qi[2] + t).cos() + ct, t],
            SegmentType::R => [-(qi[2] - t).sin() + st, (qi[2] - t).cos() - ct, -t],
            SegmentType::S => [ct * t, st * t, 0.],
        };

        [qt[0] + qi[0], qt[1] + qi[1], qt[2] + qi[2]]
    }

    /// Get car location and orientation long after some travel distance
    ///
    /// # Arguments
    ///
    /// * `t`: The travel distance - must be less than the total length of the path
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::{DubinsPath, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.], [100., -100., PI * (3. / 4.)], 11.6).unwrap();
    ///
    /// // Find the halfway point of the path
    /// let t: f32 = shortest_path_possible.length() / 2.;
    ///
    /// let position: PosRot = shortest_path_possible.sample(t);
    /// ```
    #[must_use]
    pub fn sample(&self, t: f32) -> PosRot {
        // tprime is the normalised variant of the parameter t
        let tprime = t / self.rho;
        let types = self.type_.to_segment_types();

        // initial configuration
        let qi = [0., 0., self.qi[2]];

        // generate the target configuration
        let p1 = self.param[0];
        let p2 = self.param[1];

        let q1 = Self::segment(p1, qi, types[0]); // end-of segment 1
        let q2 = Self::segment(p2, q1, types[1]); // end-of segment 2

        let q = if tprime < p1 {
            Self::segment(tprime, qi, types[0])
        } else if tprime < p1 + p2 {
            Self::segment(tprime - p1, q1, types[1])
        } else {
            Self::segment(tprime - p1 - p2, q2, types[2])
        };

        // scale the target configuration, translate back to the original starting point
        [q[0].mul_add(self.rho, self.qi[0]), q[1].mul_add(self.rho, self.qi[1]), mod2pi(q[2])]
    }
}

#[cfg(feature = "glam")]
impl DubinsPath {
    /// Finds the `[x, y, theta]` along some distance of some type with some starting position
    ///
    /// If you're looking to find the position of the car along some distance of the path, use `DubinsPath::sample` instead.
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
    /// use std::f32::consts::PI;
    /// use dubins_paths::{DubinsPath, PosRot, SegmentType};
    ///
    /// // Normalized distance along the segement (distance / rho)
    /// let t = 0.32158;
    /// // The starting position
    /// let qi = PosRot::from_f32(0., 0., PI / 4.);
    /// // The path type to be calculated, in this case it's Left Straight Right
    /// let type_: SegmentType = SegmentType::L;
    ///
    /// let position: PosRot = DubinsPath::segment(t, qi, type_);
    /// ```
    #[must_use]
    pub fn segment(t: f32, qi: PosRot, type_: SegmentType) -> PosRot {
        let st = qi.rot.sin();
        let ct = qi.rot.cos();

        let qt = match type_ {
            SegmentType::L => PosRot::from_f32((qi.rot + t).sin() - st, -(qi.rot + t).cos() + ct, t),
            SegmentType::R => PosRot::from_f32(-(qi.rot - t).sin() + st, (qi.rot - t).cos() - ct, -t),
            SegmentType::S => PosRot::from_f32(ct * t, st * t, 0.),
        };

        PosRot::new(qt.pos + qi.pos, qt.rot + qi.rot)
    }

    /// Get car location and orientation long after some travel distance
    ///
    /// # Arguments
    ///
    /// * `t`: The travel distance - must be less than the total length of the path
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::{DubinsPath, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from(PosRot::from_f32(0., 0., PI / 4.), PosRot::from_f32(100., -100., PI * (3. / 4.)), 11.6).unwrap();
    ///
    /// // Find the halfway point of the path
    /// let t: f32 = shortest_path_possible.length() / 2.;
    ///
    /// let position: PosRot = shortest_path_possible.sample(t);
    /// ```
    #[must_use]
    pub fn sample(&self, t: f32) -> PosRot {
        // tprime is the normalised variant of the parameter t
        let tprime = t / self.rho;
        let types = self.type_.to_segment_types();

        // initial configuration
        let qi = [0., 0., self.qi.rot].into();

        // generate the target configuration
        let p1 = self.param[0];
        let p2 = self.param[1];

        let q1 = Self::segment(p1, qi, types[0]); // end-of segment 1
        let q2 = Self::segment(p2, q1, types[1]); // end-of segment 2

        let q = if tprime < p1 {
            Self::segment(tprime, qi, types[0])
        } else if tprime < p1 + p2 {
            Self::segment(tprime - p1, q1, types[1])
        } else {
            Self::segment(tprime - p1 - p2, q2, types[2])
        };

        // scale the target configuration, translate back to the original starting point
        PosRot::new((q.pos * self.rho) + self.qi.pos, mod2pi(q.rot))
    }
}

impl DubinsPath {
    /// Create a new path
    const fn new(qi: PosRot, rho: f32, param: Params, type_: PathType) -> Self {
        Self {
            qi,
            rho,
            param,
            type_,
        }
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
    /// Will return a `NoPathError` if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::{self, DubinsPath, PathType, PosRot};
    ///
    /// // The starting position
    /// let q0: PosRot = [0., 0., PI / 4.].into();
    /// // The target end position
    /// let q1: PosRot = [100., -100., PI * (3. / 4.)].into();
    /// // The car's turning radius (must be > 0)
    /// let rho: f32 = 11.6;
    /// // A slice of the PathTypes that we should compare
    /// // Some are predefined, like CCC (Corner Corner Corner), CSC (Corner Straight Corner), and ALL
    /// // If you plan on using ALL, consider using the shortcut function `DubinsPath::shortest_from(q0, q1, rho)`
    /// let types: &[PathType] = &PathType::CSC;
    ///
    /// let shortest_path_in_selection: dubins_paths::Result<DubinsPath> = DubinsPath::shortest_in(q0, q1, rho, types);
    ///
    /// assert!(shortest_path_in_selection.is_ok());
    /// ```
    pub fn shortest_in(q0: PosRot, q1: PosRot, rho: f32, types: &[PathType]) -> Result<Self> {
        let mut best: Option<(Params, PathType)> = None;

        let intermediate_results = Intermediate::from(q0, q1, rho);

        for path_type in types {
            if let Ok(param) = intermediate_results.word(*path_type) {
                if let Some((best_param, _)) = &best {
                    if param.iter().sum::<f32>() > best_param.iter().sum::<f32>() {
                        continue;
                    }
                }

                best = Some((param, *path_type));
            }
        }

        match best {
            Some((param, path_type)) => Ok(Self::new(q0, rho, param, path_type)),
            None => Err(NoPathError),
        }
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
    /// Will return a `NoPathError` if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::{self, DubinsPath, PathType, PosRot};
    ///
    /// // The starting position
    /// let q0: PosRot = [0., 0., PI / 4.].into();
    /// // The target end position
    /// let q1: PosRot = [100., -100., PI * (3. / 4.)].into();
    /// // The car's turning radius (must be > 0)
    /// let rho: f32 = 11.6;
    ///
    /// let shortest_path_possible: dubins_paths::Result<DubinsPath> = DubinsPath::shortest_from(q0, q1, rho);
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
    /// Will return a `NoPathError` if no path could be found.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::{self, DubinsPath, PathType, PosRot};
    ///
    /// // The starting position
    /// let q0: PosRot = [0., 0., PI / 4.].into();
    /// // The target end position
    /// let q1: PosRot = [100., -100., PI * (3. / 4.)].into();
    /// // The car's turning radius (must be > 0)
    /// let rho: f32 = 11.6;
    /// // The path type to be calculated, in this case it's Left Straight Right
    /// let path_type: PathType = PathType::LSR;
    ///
    /// let path: dubins_paths::Result<DubinsPath> = DubinsPath::from(q0, q1, rho, path_type);
    ///
    /// assert!(path.is_ok());
    /// ```
    pub fn from(q0: PosRot, q1: PosRot, rho: f32, path_type: PathType) -> Result<Self> {
        let in_ = Intermediate::from(q0, q1, rho);
        let params = in_.word(path_type)?;

        Ok(Self::new(q0, rho, params, path_type))
    }

    /// Calculate the total distance of any given path.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::DubinsPath;
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// let total_path_length = shortest_path_possible.length();
    /// ```
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
    /// use std::f32::consts::PI;
    /// use dubins_paths::DubinsPath;
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// // Each path has 3 segments
    /// // i must be in the range [0, 2]
    /// let total_segment_length: f32 = shortest_path_possible.segment_length(1);
    /// ```
    #[must_use]
    pub fn segment_length(&self, i: usize) -> f32 {
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
    /// use std::f32::consts::PI;
    /// use dubins_paths::{DubinsPath, PosRot};
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// // The distance between each sample point
    /// let step_distance: f32 = 5.;
    ///
    /// let samples: Vec<PosRot> = shortest_path_possible.sample_many(step_distance);
    /// ```
    #[must_use]
    pub fn sample_many(&self, step_distance: f32) -> Vec<PosRot> {
        let num_samples = (self.length() / step_distance).floor() as usize;
        let mut results: Vec<PosRot> = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            results.push(self.sample(i as f32 * step_distance));
        }

        results
    }

    /// Get the endpoint of the path
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f32::consts::PI;
    /// use dubins_paths::{DubinsPath, PosRot};
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
    /// use std::f32::consts::PI;
    /// use dubins_paths::DubinsPath;
    ///
    /// let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();
    ///
    /// // End the path halfway through the real path
    /// let t: f32 = shortest_path_possible.length() / 2.;
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
        Self::new(self.qi, self.rho, [param0, param1, param2], self.type_)
    }
}
