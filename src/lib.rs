use std::f32::{consts::PI, INFINITY};

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

    /// Convert from usize to a path type
    pub fn from(value: usize) -> Self {
        match value {
            0 => Self::LSL,
            1 => Self::LSR,
            2 => Self::RSL,
            3 => Self::RSR,
            4 => Self::RLR,
            5 => Self::LRL,
            _ => panic!("Unknown value: {}", value),
        }
    }

    /// Convert from path type to usize
    pub fn to(self: &Self) -> usize {
        match self {
            Self::LSL => 0,
            Self::LSR => 1,
            Self::RSL => 2,
            Self::RSR => 3,
            Self::RLR => 4,
            Self::LRL => 5,
        }
    }
}

/// All the basic information about Dubin's Paths
#[derive(Clone, Copy, Debug)]
pub struct DubinsPath {
    /// The initial location (x, y, theta)
    pub qi: [f32; 3],
    /// The normalized lengths of the three segments
    pub param: [f32; 3],
    /// The model's turn radius (forward velocity / angular velocity)
    pub rho: f32,
    /// The type of the path
    pub type_: DubinsPathType,
}

/// All the possible errors that may occur when generating the path
#[derive(Clone, Copy, Debug)]
pub enum DubinsError {
    /// Colocated configurations
    CoConfigs,
    /// Path parameterisitation error
    Param,
    /// The rho value is invalid
    BadRho,
    /// No connection between configurations with this word
    NoPath,
}

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

/// The segment types for each of the path types
pub const DIRDATA: [[SegmentType; 3]; 6] = [[SegmentType::L, SegmentType::S, SegmentType::L], [SegmentType::L, SegmentType::S, SegmentType::R], [SegmentType::R, SegmentType::S, SegmentType::L], [SegmentType::R, SegmentType::S, SegmentType::R], [SegmentType::R, SegmentType::L, SegmentType::R], [SegmentType::L, SegmentType::R, SegmentType::L]];

/// The pre-calculated information that applies to every path type
#[derive(Clone, Copy, Debug)]
pub struct DubinsIntermediateResults {
    pub alpha: f32,
    pub beta: f32,
    pub d: f32,
    pub sa: f32,
    pub sb: f32,
    pub ca: f32,
    pub cb: f32,
    pub c_ab: f32,
    pub d_sq: f32,
}

impl Default for DubinsIntermediateResults {
    fn default() -> Self {
        Self {
            alpha: 0.,
            beta: 0.,
            d: 0.,
            sa: 0.,
            sb: 0.,
            ca: 0.,
            cb: 0.,
            c_ab: 0.,
            d_sq: 0.,
        }
    }
}

/// Floating point modulus suitable for rings
fn fmodr(x: f32, y: f32) -> f32 {
    x - y * (x / y).floor()
}

fn mod2pi(theta: f32) -> f32 {
    fmodr(theta, 2. * PI)
}

/// Find the shortest path out of the specified Dubin's paths
/// 
/// * `q0`: Three f32's in the format `[x, y, theta]`
///
/// Represents starting location and orientation of the car.
///
/// * `q1`: Three f32's in the format `[x, y, theta]`
///
/// Represents ending location and orientation of the car.
///
/// * `rho`: The turning radius of the car.
///
/// Can be can be calculated by taking the forward velocity of the car and dividing it by the car's angular velocity.
///
/// * `types`: A reference to a slice that contains the path types to be compared.
pub fn shortest_path_in(q0: [f32; 3], q1: [f32; 3], rho: f32, types: &[DubinsPathType]) -> Result<DubinsPath, DubinsError> {
    let mut best_cost = INFINITY;
    let mut best_params = [0.; 3];
    let mut best_type = None;

    let intermediate_results = intermediate_results(q0, q1, rho)?;

    for path_type in types {
        if let Ok(param) = word(&intermediate_results, *path_type) {
            let cost = param[0] + param[1] + param[2];
            if cost < best_cost {
                best_cost = cost;
                best_params = param;
                best_type = Some(*path_type);
            }
        }
    }

    match best_type {
        Some(type_) => Ok(DubinsPath {
            qi: q0,
            rho,
            param: best_params,
            type_,
        }),
        None => Err(DubinsError::NoPath),
    }
}

/// Find the shortest path out of the 6 Dubin's paths
/// 
/// * `q0`: Three f32's in the format `[x, y, theta]`
///
/// Represents starting location and orientation of the car.
///
/// * `q1`: Three f32's in the format `[x, y, theta]`
///
/// Represents ending location and orientation of the car.
///
/// * `rho`: The turning radius of the car.
///
/// Can be can be calculated by taking the forward velocity of the car and dividing it by the car's angular velocity.
pub fn shortest_path(q0: [f32; 3], q1: [f32; 3], rho: f32) -> Result<DubinsPath, DubinsError> {
    shortest_path_in(q0, q1, rho, &DubinsPathType::ALL)
}

/// Calculate a Dubin's path
/// Find the shortest path out of the specified Dubin's paths
/// 
/// * `q0`: Three f32's in the format `[x, y, theta]`
///
/// Represents starting location and orientation of the car.
///
/// * `q1`: Three f32's in the format `[x, y, theta]`
///
/// Represents ending location and orientation of the car.
///
/// * `rho`: The turning radius of the car.
///
/// Can be can be calculated by taking the forward velocity of the car and dividing it by the car's angular velocity.
///
/// * `path_type`: The Dubin's path type that's to be calculated.
pub fn path(q0: [f32; 3], q1: [f32; 3], rho: f32, path_type: DubinsPathType) -> Result<DubinsPath, DubinsError> {
    let in_ = intermediate_results(q0, q1, rho)?;
    let params = word(&in_, path_type)?;

    Ok(DubinsPath {
        param: params,
        qi: q0,
        rho,
        type_: path_type,
    })
}

/// Calculate the total distance of any given path.
pub fn path_length(path: &DubinsPath) -> f32 {
    (path.param[0] + path.param[1] + path.param[2]) * path.rho
}

/// Calculate the total distance of the path segment
/// 
/// `i`: Index of the segment to get the length of - [0, 2]
pub fn segment_length(path: &DubinsPath, i: usize) -> f32 {
    path.param[i] * path.rho
}

fn segment(t: f32, qi: [f32; 3], type_: SegmentType) -> [f32; 3] {
    let st = qi[2].sin();
    let ct = qi[2].cos();

    let mut qt = match type_ {
        SegmentType::L => [(qi[2] + t).sin() - st, -(qi[2] + t).cos() + ct, t],
        SegmentType::R => [-(qi[2] - t).sin() + st, (qi[2] - t).cos() - ct, -t],
        SegmentType::S => [ct * t, st * t, 0.],
    };

    qt[0] += qi[0];
    qt[1] += qi[1];
    qt[2] += qi[2];

    qt
}

/// Sample a path
///
/// * `path`: The path to sample
/// * `t`: The travel distance at which to sample the path
pub fn path_sample(path: &DubinsPath, t: f32) -> Result<[f32; 3], DubinsError> {
    /* tprime is the normalised variant of the parameter t */
    let tprime = t / path.rho;
    let types = DIRDATA[path.type_.to()];

    if t < 0. || t > path_length(path) {
        return Err(DubinsError::Param);
    }

    /* initial configuration */
    let qi = [0., 0., path.qi[2]];

    /* generate the target configuration */
    let p1 = path.param[0];
    let p2 = path.param[1];

    let q1 = segment(p1, qi, types[0]); // end-of segment 1
    let q2 = segment(p2, q1, types[1]); // end-of segment 2

    let mut q = if tprime < p1 {
        segment(tprime, qi, types[0])
    } else if tprime < p1 + p2 {
        segment(tprime - p1, q1, types[1])
    } else {
        segment(tprime - p1 - p2, q2, types[2])
    };

    /* scale the target configuration, translate back to the original starting point */
    q[0] = q[0] * path.rho + path.qi[0];
    q[1] = q[1] * path.rho + path.qi[1];
    q[2] = mod2pi(q[2]);

    Ok(q)
}

/// Get a vec of all the points along a path
///
/// * `path`: The path to sample
/// * `step_distance`: The distance between each point
pub fn path_sample_many(path: &DubinsPath, step_distance: f32) -> Result<Vec<[f32; 3]>, DubinsError> {
    let length = path_length(path);

    let mut x = 0.;
    let mut results: Vec<[f32; 3]> = Vec::new();

    while x < length {
        let q = path_sample(path, x)?;

        results.push(q);

        x += step_distance;
    }

    Ok(results)
}

/// Get the endpoint of the path
///
/// * `path`: The path to get the endpoint of
pub fn path_endpoint(path: &DubinsPath) -> Result<[f32; 3], DubinsError> {
    path_sample(path, path_length(path) - f32::EPSILON)
}

/// Extract a subpath from a path
///
/// * `path`: The path take the subpath from
/// * `t`: The length along the path to end the subpath
pub fn extract_subpath(path: &DubinsPath, t: f32) -> Result<DubinsPath, DubinsError> {
    if t < 0.|| t > path_length(path) {
        return Err(DubinsError::Param);
    }

    let mut new_path = path.clone();

    // calculate the true parameter
    let tprime = t / path.rho;

    // fix the parameters
    new_path.param[0] = path.param[0].min(tprime);
    new_path.param[1] = path.param[1].min(tprime - new_path.param[0]);
    new_path.param[2] = path.param[2].min(tprime - new_path.param[0] - new_path.param[1]);
    
    Ok(new_path)
}

/// Pre-calculated values that are required by all of Dubin's Paths
/// 
/// * `q0`: Three f32's in the format `[x, y, theta]`
///
/// Represents starting location and orientation of the car.
///
/// * `q1`: Three f32's in the format `[x, y, theta]`
///
/// Represents ending location and orientation of the car.
///
/// * `rho`: The turning radius of the car.
pub fn intermediate_results(q0: [f32; 3], q1: [f32; 3], rho: f32) -> Result<DubinsIntermediateResults, DubinsError> {
    if rho <= 0.0 {
        return Err(DubinsError::BadRho);
    }

    let dx = q1[0] - q0[0];
    let dy = q1[1] - q0[1];
    let d = (dx * dx + dy * dy).sqrt() / rho;

    /* test required to prevent domain errors if dx=0 and dy=0 */
    let theta = if d > 0. {
        mod2pi(dy.atan2(dx))
    } else {
        0.
    };

    let alpha = mod2pi(q0[2] - theta);
    let beta = mod2pi(q1[2] - theta);

    Ok(DubinsIntermediateResults {
        alpha,
        beta,
        d,
        sa: alpha.sin(),
        sb: beta.sin(),
        ca: alpha.cos(),
        cb: beta.cos(),
        c_ab: (alpha - beta).cos(),
        d_sq: d * d,
    })
}

fn lsl(in_: &DubinsIntermediateResults) -> Result<[f32; 3], DubinsError> {
    let p_sq = 2. + in_.d_sq - (2. * in_.c_ab) + (2. * in_.d * (in_.sa - in_.sb));

    if p_sq >= 0. {
        let tmp0 = in_.d + in_.sa - in_.sb;
        let tmp1 = (in_.cb - in_.ca).atan2(tmp0);

        return Ok([mod2pi(tmp1 - in_.alpha), p_sq.sqrt(), mod2pi(in_.beta - tmp1)]);
    }

    Err(DubinsError::NoPath)
}

fn rsr(in_: &DubinsIntermediateResults) -> Result<[f32; 3], DubinsError> {
    let p_sq = 2. + in_.d_sq - (2. * in_.c_ab) + (2. * in_.d * (in_.sb - in_.sa));

    if p_sq >= 0. {
        let tmp0 = in_.d - in_.sa + in_.sb;
        let tmp1 = (in_.ca - in_.cb).atan2(tmp0);

        return Ok([mod2pi(in_.alpha - tmp1), p_sq.sqrt(), mod2pi(tmp1 - in_.beta)]);
    }

    Err(DubinsError::NoPath)
}

fn lsr(in_: &DubinsIntermediateResults) -> Result<[f32; 3], DubinsError> {
    let p_sq = -2. + (in_.d_sq) + (2. * in_.c_ab) + (2. * in_.d * (in_.sa + in_.sb));

    if p_sq >= 0. {
        let p = p_sq.sqrt();
        let tmp0 = (-in_.ca - in_.cb).atan2(in_.d + in_.sa + in_.sb) - (-2_f32).atan2(p);

        return Ok([mod2pi(tmp0 - in_.alpha), p, mod2pi(tmp0 - mod2pi(in_.beta))]);
    }

    Err(DubinsError::NoPath)
}

fn rsl(in_: &DubinsIntermediateResults) -> Result<[f32; 3], DubinsError> {
    let p_sq = -2. + in_.d_sq + (2. * in_.c_ab) - (2. * in_.d * (in_.sa + in_.sb));

    if p_sq >= 0. {
        let p = p_sq.sqrt();
        let tmp0 = (in_.ca + in_.cb).atan2(in_.d - in_.sa - in_.sb) - (2_f32).atan2(p);

        return Ok([mod2pi(in_.alpha - tmp0), p, mod2pi(in_.beta - tmp0)]);
    }

    Err(DubinsError::NoPath)
}

fn rlr(in_: &DubinsIntermediateResults) -> Result<[f32; 3], DubinsError> {
    let tmp0 = (6. - in_.d_sq + 2. * in_.c_ab + 2. * in_.d * (in_.sa - in_.sb)) / 8.;
    let phi = (in_.ca - in_.cb).atan2(in_.d - in_.sa + in_.sb);

    if tmp0.abs() <= 1. {
        let p = mod2pi((2. * PI) - tmp0.acos());
        let t = mod2pi(in_.alpha - phi + mod2pi(p / 2.));

        return Ok([t, p, mod2pi(in_.alpha - in_.beta - t + mod2pi(p))]);
    }

    Err(DubinsError::NoPath)
}

fn lrl(in_: &DubinsIntermediateResults) -> Result<[f32; 3], DubinsError> {
    let tmp0 = (6. - in_.d_sq + 2. * in_.c_ab + 2. * in_.d * (in_.sb - in_.sa)) / 8.;
    let phi = (in_.ca - in_.cb).atan2(in_.d + in_.sa - in_.sb);

    if tmp0.abs() <= 1. {
        let p = mod2pi(2. * PI - tmp0.acos());
        let t = mod2pi(-in_.alpha - phi + p / 2.);

        return Ok([t, p, mod2pi(mod2pi(in_.beta) - in_.alpha - t + mod2pi(p))]);
    }

    Err(DubinsError::NoPath)
}

/// Calculate a specific Dubin's Path
/// 
/// * `path_type`: The Dubin's path type that's to be calculated.
pub fn word(in_: &DubinsIntermediateResults, path_type: DubinsPathType) -> Result<[f32; 3], DubinsError> {
    match path_type {
        DubinsPathType::LSL => lsl(in_),
        DubinsPathType::RSL => rsl(in_),
        DubinsPathType::LSR => lsr(in_),
        DubinsPathType::RSR => rsr(in_),
        DubinsPathType::LRL => lrl(in_),
        DubinsPathType::RLR => rlr(in_),
    }
}
