use std::{f32::consts::PI, panic::panic_any, time::Instant};

extern crate dubins_paths;

use dubins_paths::{DubinsPath, DubinsPathType, DubinsPos};
const TURN_RADIUS: f32 = 1. / 0.00076;

#[test]
fn fast_shortest_csc_path() {
    let runs = 10000000;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();

        let q0: DubinsPos = [2000., 2000., 0.];
        let q1: DubinsPos = [0., 0., PI];

        if let Err(err) = DubinsPath::shortest_in(q0, q1, TURN_RADIUS, &DubinsPathType::CCC) {
            panic_any(err);
        }

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {} seconds ({}ms) - total {} seconds", elapsed, &elapsed_ms, total_elapsed);
    assert!(elapsed_ms < 0.001);
}

#[test]
fn fast_shortest_ccc_path() {
    let runs = 10000000;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();

        let q0: DubinsPos = [2000., 2000., 0.];
        let q1: DubinsPos = [0., 0., PI];

        if let Err(err) = DubinsPath::shortest_in(q0, q1, TURN_RADIUS, &DubinsPathType::CSC) {
            panic_any(err);
        }

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {} seconds ({}ms) - total {} seconds", elapsed, &elapsed_ms, total_elapsed);
    assert!(elapsed_ms < 0.001);
}

#[test]
fn fast_shortest_path() {
    let runs = 10000000;
    let mut times = Vec::with_capacity(runs);

    for _ in 0..runs {
        let start = Instant::now();

        let q0: DubinsPos = [2000., 2000., 0.];
        let q1: DubinsPos = [0., 0., PI];

        if let Err(err) = DubinsPath::shortest_from(q0, q1, TURN_RADIUS) {
            panic_any(err);
        }

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {} seconds ({}ms) - total {} seconds", elapsed, &elapsed_ms, total_elapsed);
    assert!(elapsed_ms < 0.001);
}

#[test]
fn fast_many_sample() {
    let runs = 100000;
    let mut times = Vec::with_capacity(runs);

    let q0: DubinsPos = [2000., 2000., 0.];
    let q1: DubinsPos = [0., 0., PI];

    let path = match DubinsPath::shortest_from(q0, q1, TURN_RADIUS) {
        Ok(p) => p,
        Err(err) => panic_any(err),
    };

    for _ in 0..runs {
        let start = Instant::now();

        path.sample_many(10.);

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {} seconds ({}ms) - total {} seconds", elapsed, &elapsed_ms, total_elapsed);
    assert!(elapsed_ms < 0.05);
}
