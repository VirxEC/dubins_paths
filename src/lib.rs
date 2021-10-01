use std::f64::{consts::PI, INFINITY};

const EPSILON: f64 = 10e-10;

#[derive(Clone, Copy, Debug)]
pub enum DubinsPathType {
    LSL,
    LSR,
    RSL,
    RSR,
    RLR,
    LRL,
}

impl DubinsPathType {
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

    pub fn csc() -> [DubinsPathType; 4] {
        [
            Self::LSL,
            Self::LSR,
            Self::RSL,
            Self::RSR,
        ]
    }

    pub fn ccc() -> [DubinsPathType; 2] {
        [
            Self::RLR,
            Self::LRL,
        ]
    }

    pub fn all() -> [DubinsPathType; 6] {
        [
            Self::LSL,
            Self::LSR,
            Self::RSL,
            Self::RSR,
            Self::RLR,
            Self::LRL,
        ]
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DubinsPath {
    /* the initial configuration */
    pub qi: [f64; 3],
    /* the lengths of the three segments */
    pub param: [f64; 3],
    /* model forward velocity / model angular velocity */
    pub rho: f64,
    /* the path type described */
    pub type_: DubinsPathType,
}

#[derive(Clone, Copy, Debug)]
pub enum DubErr {
    COCONFIGS, /* Colocated configurations */
    PARAM,     /* Path parameterisitation error */
    BADRHO,    /* the rho value is invalid */
    NOPATH,    /* no connection between configurations with this word */
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SegmentType {
    L,
    S,
    R,
}

/* The segment types for each of the Path types */
const DIRDATA: [[SegmentType; 3]; 6] = [[SegmentType::L, SegmentType::S, SegmentType::L], [SegmentType::L, SegmentType::S, SegmentType::R], [SegmentType::R, SegmentType::S, SegmentType::L], [SegmentType::R, SegmentType::S, SegmentType::R], [SegmentType::R, SegmentType::L, SegmentType::R], [SegmentType::L, SegmentType::R, SegmentType::L]];

#[derive(Clone, Copy, Debug)]
struct DubinsIntermediateResults {
    alpha: f64,
    beta: f64,
    d: f64,
    sa: f64,
    sb: f64,
    ca: f64,
    cb: f64,
    c_ab: f64,
    d_sq: f64,
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
fn fmodr(x: f64, y: f64) -> f64 {
    x - y * (x / y).floor()
}

fn mod2pi(theta: f64) -> f64 {
    fmodr(theta, 2. * PI)
}

/// Find the shortest path out of the specified Dubin's paths
pub fn shortest_path_in(q0: [f64; 3], q1: [f64; 3], rho: f64, types: &[DubinsPathType]) -> Result<DubinsPath, DubErr> {
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
        None => Err(DubErr::NOPATH),
    }
}

/// Find the shortest path out of the 6 Dubin's paths
pub fn shortest_path(q0: [f64; 3], q1: [f64; 3], rho: f64) -> Result<DubinsPath, DubErr> {
    shortest_path_in(q0, q1, rho, &DubinsPathType::all())
}

pub fn path(q0: [f64; 3], q1: [f64; 3], rho: f64, path_type: DubinsPathType) -> Result<DubinsPath, DubErr> {
    let in_ = intermediate_results(q0, q1, rho)?;
    let params = word(&in_, path_type)?;

    Ok(DubinsPath {
        param: params,
        qi: q0,
        rho,
        type_: path_type,
    })
}

pub fn path_length(path: &DubinsPath) -> f64 {
    (path.param[0] + path.param[1] + path.param[2]) * path.rho
}

// double segment_length( DubinsPath* path, int i )
// {
//     if( (i < 0) || (i > 2) )
//     {
//         return INFINITY;
//     }
//     return path->param[i] * path->rho;
// }

// double segment_length_normalized( DubinsPath* path, int i )
// {
//     if( (i < 0) || (i > 2) )
//     {
//         return INFINITY;
//     }
//     return path->param[i];
// }

fn segment(t: f64, qi: [f64; 3], type_: SegmentType) -> [f64; 3] {
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

fn path_sample(path: &DubinsPath, t: f64) -> Result<[f64; 3], DubErr> {
    /* tprime is the normalised variant of the parameter t */
    let tprime = t / path.rho;
    let types = DIRDATA[path.type_.to()];

    if t < 0. || t > path_length(path) {
        return Err(DubErr::PARAM);
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

/// Get a vec of all the points along the path
///
/// All points are equally spaced out
pub fn path_sample_many(path: &DubinsPath, step_distance: f64) -> Result<Vec<[f64; 3]>, DubErr> {
    let length = path_length(path);

    let mut x = 0.;
    let mut results: Vec<[f64; 3]> = Vec::with_capacity((length * step_distance + 1.) as usize);

    while x < length {
        let q = path_sample(path, x)?;

        results.push(q);

        x += step_distance;
    }

    Ok(results)
}

/// Get the endpoint of the path
pub fn path_endpoint(path: &DubinsPath) -> Result<[f64; 3], DubErr> {
    path_sample(path, path_length(path) - EPSILON)
}

// int extract_subpath( DubinsPath* path, double t, DubinsPath* newpath )
// {
//     /* calculate the true parameter */
//     double tprime = t / path->rho;

//     if((t < 0) || (t > path_length(path)))
//     {
//         return EDUBPARAM;
//     }

//     /* copy most of the data */
//     newpath->qi[0] = path->qi[0];
//     newpath->qi[1] = path->qi[1];
//     newpath->qi[2] = path->qi[2];
//     newpath->rho   = path->rho;
//     newpath->type  = path->type;

//     /* fix the parameters */
//     newpath->param[0] = fmin( path->param[0], tprime );
//     newpath->param[1] = fmin( path->param[1], tprime - newpath->param[0]);
//     newpath->param[2] = fmin( path->param[2], tprime - newpath->param[0] - newpath->param[1]);
//     return 0;
// }

fn intermediate_results(q0: [f64; 3], q1: [f64; 3], rho: f64) -> Result<DubinsIntermediateResults, DubErr> {
    if rho <= 0.0 {
        return Err(DubErr::BADRHO);
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

fn lsl(in_: &DubinsIntermediateResults) -> Result<[f64; 3], DubErr> {
    let p_sq = 2. + in_.d_sq - (2. * in_.c_ab) + (2. * in_.d * (in_.sa - in_.sb));

    if p_sq >= 0. {
        let tmp0 = in_.d + in_.sa - in_.sb;
        let tmp1 = (in_.cb - in_.ca).atan2(tmp0);

        return Ok([mod2pi(tmp1 - in_.alpha), p_sq.sqrt(), mod2pi(in_.beta - tmp1)]);
    }

    Err(DubErr::NOPATH)
}

fn rsr(in_: &DubinsIntermediateResults) -> Result<[f64; 3], DubErr> {
    let p_sq = 2. + in_.d_sq - (2. * in_.c_ab) + (2. * in_.d * (in_.sb - in_.sa));

    if p_sq >= 0. {
        let tmp0 = in_.d - in_.sa + in_.sb;
        let tmp1 = (in_.ca - in_.cb).atan2(tmp0);

        return Ok([mod2pi(in_.alpha - tmp1), p_sq.sqrt(), mod2pi(tmp1 - in_.beta)]);
    }

    Err(DubErr::NOPATH)
}

fn lsr(in_: &DubinsIntermediateResults) -> Result<[f64; 3], DubErr> {
    let p_sq = -2. + (in_.d_sq) + (2. * in_.c_ab) + (2. * in_.d * (in_.sa + in_.sb));

    if p_sq >= 0. {
        let p = p_sq.sqrt();
        let tmp0 = (-in_.ca - in_.cb).atan2(in_.d + in_.sa + in_.sb) - (-2_f64).atan2(p);

        return Ok([mod2pi(tmp0 - in_.alpha), p, mod2pi(tmp0 - mod2pi(in_.beta))]);
    }

    Err(DubErr::NOPATH)
}

fn rsl(in_: &DubinsIntermediateResults) -> Result<[f64; 3], DubErr> {
    let p_sq = -2. + in_.d_sq + (2. * in_.c_ab) - (2. * in_.d * (in_.sa + in_.sb));
    if p_sq >= 0. {
        let p = p_sq.sqrt();
        let tmp0 = (in_.ca + in_.cb).atan2(in_.d - in_.sa - in_.sb) - (2_f64).atan2(p);

        return Ok([mod2pi(in_.alpha - tmp0), p, mod2pi(in_.beta - tmp0)]);
    }

    Err(DubErr::NOPATH)
}

fn rlr(in_: &DubinsIntermediateResults) -> Result<[f64; 3], DubErr> {
    let tmp0 = (6. - in_.d_sq + 2. * in_.c_ab + 2. * in_.d * (in_.sa - in_.sb)) / 8.;
    let phi = (in_.ca - in_.cb).atan2(in_.d - in_.sa + in_.sb);

    if tmp0.abs() <= 1. {
        let p = mod2pi((2. * PI) - tmp0.acos());
        let t = mod2pi(in_.alpha - phi + mod2pi(p / 2.));

        return Ok([t, p, mod2pi(in_.alpha - in_.beta - t + mod2pi(p))]);
    }

    Err(DubErr::NOPATH)
}

fn lrl(in_: &DubinsIntermediateResults) -> Result<[f64; 3], DubErr> {
    let tmp0 = (6. - in_.d_sq + 2. * in_.c_ab + 2. * in_.d * (in_.sb - in_.sa)) / 8.;
    let phi = (in_.ca - in_.cb).atan2(in_.d + in_.sa - in_.sb);

    if tmp0.abs() <= 1. {
        let p = mod2pi(2. * PI - tmp0.acos());
        let t = mod2pi(-in_.alpha - phi + p / 2.);

        return Ok([t, p, mod2pi(mod2pi(in_.beta) - in_.alpha - t + mod2pi(p))]);
    }

    Err(DubErr::NOPATH)
}

fn word(in_: &DubinsIntermediateResults, path_type: DubinsPathType) -> Result<[f64; 3], DubErr> {
    match path_type {
        DubinsPathType::LSL => lsl(in_),
        DubinsPathType::RSL => rsl(in_),
        DubinsPathType::LSR => lsr(in_),
        DubinsPathType::RSR => rsr(in_),
        DubinsPathType::LRL => lrl(in_),
        DubinsPathType::RLR => rlr(in_),
    }
}
