#![warn(missing_docs, clippy::pedantic, clippy::all, clippy::nursery)]
#![allow(clippy::suboptimal_flops)]
#![forbid(unsafe_code)]
#![cfg_attr(any(not(feature = "std"), feature = "libm"), no_std)]

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
//! use dubins_paths::{consts::PI, DubinsPath, PosRot, Result as DubinsResult};
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
//! use dubins_paths::{consts::PI, DubinsPath, PosRot};
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
//! * `serde` - Implementations of `Deserialize` and `Serialize` for most types
//! * `rkyv` - Implementations of `Archive`, `Deserialize` and `Serialize` for most types
//!
//! [`sample`]: DubinsPath::sample
//! [`sample_many`]: DubinsPath::sample_many

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!("Either the std or the libm feature is required");

#[cfg(all(any(not(feature = "std"), feature = "libm"), feature = "alloc"))]
extern crate alloc;

/// [`glam`] is a crate that provides vector types, and used to provide a more ergonomic API
///
/// It requries the `glam` feature to be enabled in order to be used within this crate
#[cfg(feature = "glam")]
pub extern crate glam;

mod base;
mod path;

pub use base::*;
pub use path::*;

#[cfg(test)]
mod tests {

    use super::{
        consts::TAU, mod2pi, DubinsPath, FloatType, NoPathError, PathType, PosRot, SegmentType,
    };
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
        assert!(mod2pi(TAU).abs() < FloatType::EPSILON);
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
                thread_rng.random_range(-TAU..TAU),
            );
            let q1 = PosRot::from_floats(
                thread_rng.random_range((-10000.0 as FloatType)..(10000.0 as FloatType)),
                thread_rng.random_range((-10000.0 as FloatType)..(10000.0 as FloatType)),
                thread_rng.random_range(-TAU..TAU),
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
            [0., 0., TAU / 8.].into(),
            [100., -100., TAU * (3. / 8.)].into(),
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
