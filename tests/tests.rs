extern crate dubins_paths;

use core::f32::consts::PI;
use dubins_paths::{mod2pi, DubinsPath, NoPathError, PathType, PosRot, SegmentType};
use rand::Rng;
use std::{mem::size_of, panic::panic_any, time::Instant};

const TURN_RADIUS: f32 = 1. / 0.00076;

#[test]
fn mod2pi_test() {
    assert!(mod2pi(-f32::from_bits(1)) >= 0.);
    assert_eq!(mod2pi(2. * PI), 0.);
}

#[test]
fn fast_shortest_csc_path() {
    let runs = 100_000;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();

        let q0: PosRot = [2000., 2000., 0.].into();
        let q1: PosRot = [0., 0., PI].into();

        if let Err(err) = DubinsPath::shortest_in(q0, q1, TURN_RADIUS, &PathType::CCC) {
            panic_any(err);
        }

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {elapsed} seconds ({elapsed_ms}ms) - total {total_elapsed} seconds");
    assert!(elapsed_ms < 0.002);
}

#[test]
fn fast_shortest_ccc_path() {
    let runs = 100_000;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();

        let q0: PosRot = [2000., 2000., 0.].into();
        let q1: PosRot = [0., 0., PI].into();

        if let Err(err) = DubinsPath::shortest_in(q0, q1, TURN_RADIUS, &PathType::CSC) {
            panic_any(err);
        }

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {elapsed} seconds ({elapsed_ms}ms) - total {total_elapsed} seconds");
    assert!(elapsed_ms < 0.002);
}

#[test]
fn fast_shortest_path() {
    let runs = 100_000;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();

        let q0: PosRot = [2000., 2000., 0.].into();
        let q1: PosRot = [0., 0., PI].into();

        if let Err(err) = DubinsPath::shortest_from(q0, q1, TURN_RADIUS) {
            panic_any(err);
        }

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {elapsed} seconds ({elapsed_ms}ms) - total {total_elapsed} seconds");
    assert!(elapsed_ms < 0.002);
}

#[test]
fn fast_many_sample() {
    const STEP_DISTANCE: f32 = 10.;

    let runs = 50_000;
    let mut times = Vec::with_capacity(runs);

    let q0: PosRot = [2000., 2000., 0.].into();
    let q1: PosRot = [0., 0., PI].into();

    let path = match DubinsPath::shortest_from(q0, q1, TURN_RADIUS) {
        Ok(p) => p,
        Err(err) => panic_any(err),
    };

    let expected_samples = (path.length() / STEP_DISTANCE).floor() as usize;
    let num_samples = path.sample_many(STEP_DISTANCE).len();

    // it's ok for it to be one less than expected due to floating point errors
    assert!(expected_samples == num_samples || expected_samples + 1 == num_samples);

    for _ in 0..runs {
        let start = Instant::now();

        let _ = path.sample_many(STEP_DISTANCE);

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {elapsed} seconds ({elapsed_ms}ms) - total {total_elapsed} seconds");
    assert!(elapsed_ms < 0.1);
}

#[test]
fn many_path_correctness() {
    #[cfg(feature = "glam")]
    fn angle_2d(vec1: f32, vec2: f32) -> f32 {
        glam::Vec3A::new(vec1.cos(), vec1.sin(), 0.)
            .dot(glam::Vec3A::new(vec2.cos(), vec2.sin(), 0.))
            .clamp(-1., 1.)
            .acos()
    }

    #[cfg(not(feature = "glam"))]
    fn angle_2d(vec1: f32, vec2: f32) -> f32 {
        (vec1.cos() * vec1.cos() + vec2.sin() * vec2.sin()).clamp(-1., 1.).acos()
    }

    // Test that the path is correct for a number of random configurations.
    // If no path is found, just skip.
    // If the path is found the sampled endpoint is different from the specified endpoint, then fail.

    let runs = 50_000;
    let mut thread_rng = rand::thread_rng();
    let mut error = 0;

    for _ in 0..runs {
        let q0: PosRot = [
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range((-2. * PI)..(2. * PI)),
        ]
        .into();
        let q1: PosRot = [
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range((-2. * PI)..(2. * PI)),
        ]
        .into();

        let path = match DubinsPath::shortest_from(q0, q1, TURN_RADIUS) {
            Ok(p) => p,
            Err(_) => continue,
        };

        let endpoint = path.endpoint();

        #[cfg(feature = "glam")]
        if q1.pos().distance(endpoint.pos()) > 1. || angle_2d(q1.rot(), endpoint.rot()) > 0.1 {
            println!("Endpoint is different! {:?} | {q1:?} | {endpoint:?}", path.path_type);
            error += 1;
        }

        #[cfg(not(feature = "glam"))]
        if (q1.x() - endpoint.x()).abs() > 1.
            || (q1.x() - endpoint.x()).abs() > 1.
            || angle_2d(q1.rot(), endpoint.rot()) > 0.1
        {
            println!("Endpoint is different! {:?} | {q1:?} | {endpoint:?}", path.path_type);
            error += 1;
        }
    }

    assert_eq!(error, 0)
}

#[test]
fn size_of_items() {
    assert_eq!(size_of::<PosRot>(), 12);
    assert_eq!(size_of::<DubinsPath>(), 32);
    assert_eq!(size_of::<PathType>(), 1);
    assert_eq!(size_of::<SegmentType>(), 1);
    assert_eq!(size_of::<NoPathError>(), 0);
}
