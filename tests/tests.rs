use std::{panic::panic_any, time::Instant};

extern crate dubins_paths;

use dubins_paths::{path_sample_many, shortest_path};
pub const MAX_TURN_RADIUS: f64 = 1. / 0.00076;

#[test]
fn fast_shortest_path() {
    let runs = 100000;
    let mut times = Vec::new();

    for _ in 0..runs {
        let start = Instant::now();

        let q0 = [2000., 2000., 0.];
        let q1 = [0., 0., 3.14];

        match shortest_path(q0, q1, MAX_TURN_RADIUS) {
            Ok(_) => (),
            Err(err) => panic_any(err),
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
    let mut times = Vec::new();

    let q0 = [2000., 2000., 0.];
    let q1 = [0., 0., 3.14];

    let path = match shortest_path(q0, q1, MAX_TURN_RADIUS) {
        Ok(p) => p,
        Err(err) => panic_any(err),
    };

    for _ in 0..runs {
        let start = Instant::now();

        match path_sample_many(&path, 20.) {
            Ok(_) => (),
            Err(err) => panic_any(err),
        }

        times.push(start.elapsed().as_secs_f32());
    }

    let total_elapsed = times.iter().sum::<f32>();
    let elapsed: f32 = total_elapsed / (runs as f32);
    let elapsed_ms = elapsed * 1000.;
    println!("Ran test in an average of {} seconds ({}ms) - total {} seconds", elapsed, &elapsed_ms, total_elapsed);
    assert!(elapsed_ms < 0.05);
}
