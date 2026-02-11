#![allow(clippy::incompatible_msrv)]

use core::{f32::consts::TAU, hint::black_box};

use dubins_paths::f32::DubinsPath;
use rand::RngExt as _;

fn main() {
    let runs = 1_000_000;
    let range = 10000.0;

    let mut thread_rng = rand::rng();

    for _ in 0..runs {
        let q0 = [
            thread_rng.random_range(-range..range),
            thread_rng.random_range(-range..range),
            thread_rng.random_range(-TAU..TAU),
        ]
        .into();
        let q1 = [
            thread_rng.random_range(-range..range),
            thread_rng.random_range(-range..range),
            thread_rng.random_range(-TAU..TAU),
        ]
        .into();

        let rho = thread_rng.random_range(600.0..3000.);

        let Ok(path) = DubinsPath::shortest_from(q0, q1, rho) else {
            continue;
        };

        let step_distance = thread_rng.random_range(5.0..(rho / 100.0));
        let _ = black_box(path.sample_many(step_distance));
    }
}
