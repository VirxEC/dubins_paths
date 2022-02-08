use std::f32::{consts::PI, INFINITY};

/// The three segment types in a Dubin's Path
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentType {
    /// Left
    L,
    /// Straight
    S,
    /// Right
    R,
}

/// All the possible path types
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DubinsPathType {
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

impl DubinsPathType {
    /// All of the "Turn Straight Turn" path types
    pub const CSC: [DubinsPathType; 4] = [Self::LSL, Self::LSR, Self::RSL, Self::RSR];
    /// All of the "Turn Turn Turn" path types
    pub const CCC: [DubinsPathType; 2] = [Self::RLR, Self::LRL];
    /// All of the path types
    pub const ALL: [DubinsPathType; 6] = [Self::LSL, Self::LSR, Self::RSL, Self::RSR, Self::RLR, Self::LRL];

    /// Convert the path type an array of it's segment types
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

/// All the possible errors that may occur when generating the path
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DubinsError {
    /// No connection between configurations with this word
    NoPath,
}

/// A type that allows the function to return either
///
/// Ok(T) or Err(DubinsError)
pub type DubinsResult<T> = Result<T, DubinsError>;

/// The car's \[x, y, theta] position
pub type DubinsPos = [f32; 3];

/// The pre-calculated information that applies to every path type
#[derive(Clone, Copy, Debug, Default)]
pub struct DubinsIntermediateResults {
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

impl DubinsIntermediateResults {
    /// Pre-calculated values that are required by all of Dubin's Paths
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    pub fn from(q0: DubinsPos, q1: DubinsPos, rho: f32) -> Self {
        let dx = q1[0] - q0[0];
        let dy = q1[1] - q0[1];
        let d = (dx * dx + dy * dy).sqrt() / rho;

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

    fn lsl(&self) -> DubinsResult<DubinsPos> {
        let p_sq = 2. + self.d_sq - (2. * self.c_ab) + (2. * self.d * (self.sa - self.sb));

        if p_sq >= 0. {
            let tmp0 = self.d + self.sa - self.sb;
            let tmp1 = (self.cb - self.ca).atan2(tmp0);

            Ok([mod2pi(tmp1 - self.alpha), p_sq.sqrt(), mod2pi(self.beta - tmp1)])
        } else {
            Err(DubinsError::NoPath)
        }
    }

    fn rsr(&self) -> DubinsResult<DubinsPos> {
        let p_sq = 2. + self.d_sq - (2. * self.c_ab) + (2. * self.d * (self.sb - self.sa));

        if p_sq >= 0. {
            let tmp0 = self.d - self.sa + self.sb;
            let tmp1 = (self.ca - self.cb).atan2(tmp0);

            Ok([mod2pi(self.alpha - tmp1), p_sq.sqrt(), mod2pi(tmp1 - self.beta)])
        } else {
            Err(DubinsError::NoPath)
        }
    }

    fn lsr(&self) -> DubinsResult<DubinsPos> {
        let p_sq = -2. + (self.d_sq) + (2. * self.c_ab) + (2. * self.d * (self.sa + self.sb));

        if p_sq >= 0. {
            let p = p_sq.sqrt();
            let tmp0 = (-self.ca - self.cb).atan2(self.d + self.sa + self.sb) - (-2_f32).atan2(p);

            Ok([mod2pi(tmp0 - self.alpha), p, mod2pi(tmp0 - mod2pi(self.beta))])
        } else {
            Err(DubinsError::NoPath)
        }
    }

    fn rsl(&self) -> DubinsResult<DubinsPos> {
        let p_sq = -2. + self.d_sq + (2. * self.c_ab) - (2. * self.d * (self.sa + self.sb));

        if p_sq >= 0. {
            let p = p_sq.sqrt();
            let tmp0 = (self.ca + self.cb).atan2(self.d - self.sa - self.sb) - (2_f32).atan2(p);

            Ok([mod2pi(self.alpha - tmp0), p, mod2pi(self.beta - tmp0)])
        } else {
            Err(DubinsError::NoPath)
        }
    }

    fn rlr(&self) -> DubinsResult<DubinsPos> {
        let tmp0 = (6. - self.d_sq + 2. * self.c_ab + 2. * self.d * (self.sa - self.sb)) / 8.;
        let phi = (self.ca - self.cb).atan2(self.d - self.sa + self.sb);

        if tmp0.abs() <= 1. {
            let p = mod2pi((2. * PI) - tmp0.acos());
            let t = mod2pi(self.alpha - phi + mod2pi(p / 2.));

            Ok([t, p, mod2pi(self.alpha - self.beta - t + mod2pi(p))])
        } else {
            Err(DubinsError::NoPath)
        }
    }

    fn lrl(&self) -> DubinsResult<DubinsPos> {
        let tmp0 = (6. - self.d_sq + 2. * self.c_ab + 2. * self.d * (self.sb - self.sa)) / 8.;
        let phi = (self.ca - self.cb).atan2(self.d + self.sa - self.sb);

        if tmp0.abs() <= 1. {
            let p = mod2pi(2. * PI - tmp0.acos());
            let t = mod2pi(-self.alpha - phi + p / 2.);

            Ok([t, p, mod2pi(mod2pi(self.beta) - self.alpha - t + mod2pi(p))])
        } else {
            Err(DubinsError::NoPath)
        }
    }

    /// Calculate a specific Dubin's Path
    ///
    /// * `path_type`: The Dubin's path type that's to be calculated.
    pub fn word(&self, path_type: DubinsPathType) -> DubinsResult<DubinsPos> {
        match path_type {
            DubinsPathType::LSL => self.lsl(),
            DubinsPathType::RSL => self.rsl(),
            DubinsPathType::LSR => self.lsr(),
            DubinsPathType::RSR => self.rsr(),
            DubinsPathType::LRL => self.lrl(),
            DubinsPathType::RLR => self.rlr(),
        }
    }
}

/// Floating point modulus suitable for rings
pub fn fmodr(x: f32, y: f32) -> f32 {
    x - y * (x / y).floor()
}

/// Equivalent to fmodr(theta, 2. * PI)
pub fn mod2pi(theta: f32) -> f32 {
    fmodr(theta, 2. * PI)
}

/// All the basic information about Dubin's Paths
#[derive(Clone, Copy, Debug)]
pub struct DubinsPath {
    /// The initial location (x, y, theta)
    pub qi: DubinsPos,
    /// The normalized lengths of the three segments
    pub param: DubinsPos,
    /// The model's turn radius (forward velocity / angular velocity)
    pub rho: f32,
    /// The type of the path
    pub type_: DubinsPathType,
}

impl DubinsPath {
    /// Find the shortest path out of the specified path types
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    /// * `types`: A reference to a slice that contains the path types to be compared.
    pub fn shortest_in(q0: DubinsPos, q1: DubinsPos, rho: f32, types: &[DubinsPathType]) -> DubinsResult<Self> {
        let mut best_cost = INFINITY;
        let mut best_params = [0.; 3];
        let mut best_type = None;

        let intermediate_results = DubinsIntermediateResults::from(q0, q1, rho);

        for path_type in types {
            if let Ok(param) = intermediate_results.word(*path_type) {
                let cost = param[0] + param[1] + param[2];
                if cost < best_cost {
                    best_cost = cost;
                    best_params = param;
                    best_type = Some(*path_type);
                }
            }
        }

        match best_type {
            Some(type_) => Ok(Self {
                qi: q0,
                rho,
                param: best_params,
                type_,
            }),
            None => Err(DubinsError::NoPath),
        }
    }

    /// Find the shortest path out of the 6 path types
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    pub fn shortest_from(q0: DubinsPos, q1: DubinsPos, rho: f32) -> DubinsResult<Self> {
        Self::shortest_in(q0, q1, rho, &DubinsPathType::ALL)
    }

    /// Calculate the specified path type
    ///
    /// * `q0`: The starting location and orientation of the car.
    /// * `q1`: The ending location and orientation of the car.
    /// * `rho`: The turning radius of the car. Must be greater than 0.
    /// * `path_type`: The Dubin's path type that's to be calculated.
    pub fn from(q0: DubinsPos, q1: DubinsPos, rho: f32, path_type: DubinsPathType) -> DubinsResult<Self> {
        let in_ = DubinsIntermediateResults::from(q0, q1, rho);
        let params = in_.word(path_type)?;

        Ok(Self {
            param: params,
            qi: q0,
            rho,
            type_: path_type,
        })
    }

    /// Calculate the total distance of any given path.
    pub fn length(&self) -> f32 {
        (self.param[0] + self.param[1] + self.param[2]) * self.rho
    }

    /// Finds the `[x, y, theta]` along some distance of some type with some starting position
    ///
    /// * `t`: Normalized distance along the segement (distance / rho).
    /// * `q0`: The starting location and orientation of the car.
    /// * `type_`: The segment type.
    pub fn segment(t: f32, qi: DubinsPos, type_: SegmentType) -> DubinsPos {
        let st = qi[2].sin();
        let ct = qi[2].cos();

        let qt = match type_ {
            SegmentType::L => [(qi[2] + t).sin() - st, -(qi[2] + t).cos() + ct, t],
            SegmentType::R => [-(qi[2] - t).sin() + st, (qi[2] - t).cos() - ct, -t],
            SegmentType::S => [ct * t, st * t, 0.],
        };

        [qt[0] + qi[0], qt[1] + qi[1], qt[2] + qi[2]]
    }

    /// Calculate the total distance of the path segment
    ///
    /// `i`: Index of the segment to get the length of in the range \[0, 2]
    pub fn segment_length(&self, i: usize) -> f32 {
        self.param[i] * self.rho
    }

    /// Get car location and orientation long after some travel distance
    ///
    /// * `t`: The travel distance - must be less than the total length of the path
    pub fn sample(&self, t: f32) -> DubinsPos {
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
        [q[0] * self.rho + self.qi[0], q[1] * self.rho + self.qi[1], mod2pi(q[2])]
    }

    /// Get a vec of all the points along the path
    ///
    /// * `step_distance`: The distance between each point
    pub fn sample_many(&self, step_distance: f32) -> Vec<DubinsPos> {
        let num_samples = (self.length() / step_distance).floor() as usize;
        let mut results: Vec<DubinsPos> = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            results.push(self.sample(i as f32 * step_distance));
        }

        results
    }

    /// Get the endpoint of the path
    pub fn endpoint(&self) -> DubinsPos {
        self.sample(self.length())
    }

    /// Extract a subpath from a path
    ///
    /// * `path`: The path take the subpath from
    /// * `t`: The length along the path to end the subpath
    pub fn extract_subpath(&self, t: f32) -> DubinsPath {
        // calculate the true parameter
        let tprime = t / self.rho;

        // fix the parameters
        let param0 = self.param[0].min(tprime);
        let param1 = self.param[1].min(tprime - param0);
        let param2 = self.param[2].min(tprime - param0 - param1);

        // copy most of the data
        Self {
            qi: self.qi,
            param: [param0, param1, param2],
            rho: self.rho,
            type_: self.type_,
        }
    }
}
