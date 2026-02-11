use core::f64::consts::TAU;

use dubins_paths::{
    PathType,
    f64::{DubinsPath, PosRot},
};
#[cfg(feature = "glam")]
use glam::DVec3;
use rand::RngExt;

const POSROT_EPSILON: f64 = 0.00001;
const TURN_RADIUS: f64 = 1. / 0.00076;

#[test]
fn many_path_correctness() {
    #[cfg(feature = "glam")]
    fn angle_2d(vec1: f64, vec2: f64) -> f64 {
        DVec3::new(vec1.cos(), vec1.sin(), 0.)
            .dot(DVec3::new(vec2.cos(), vec2.sin(), 0.))
            .clamp(-1., 1.)
            .acos()
    }

    #[cfg(not(feature = "glam"))]
    fn angle_2d(vec1: f64, vec2: f64) -> f64 {
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
            thread_rng.random_range(-10000.0..10000.0),
            thread_rng.random_range(-10000.0..10000.0),
            thread_rng.random_range(-TAU..TAU),
        );
        let q1 = PosRot::from_floats(
            thread_rng.random_range(-10000.0..10000.0),
            thread_rng.random_range(-10000.0..10000.0),
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

    let zero = 0.0;

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
