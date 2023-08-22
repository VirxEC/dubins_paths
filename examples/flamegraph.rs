use core::{f32::consts::PI, hint::black_box};

use dubins_paths::DubinsPath;
use rand::Rng;

fn main() {
    let runs = 1_000_000;

    let mut thread_rng = rand::thread_rng();

    for _ in 0..runs {
        let q0 = [
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range((-2. * PI)..(2. * PI)),
        ]
        .into();
        let q1 = [
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range(-10000_f32..10000.),
            thread_rng.gen_range((-2. * PI)..(2. * PI)),
        ]
        .into();

        let rho = thread_rng.gen_range(600f32..3000.);

        let Ok(path) = DubinsPath::shortest_from(black_box(q0), q1, rho) else {
            continue;
        };

        let step_distance = thread_rng.gen_range(5f32..(rho / 100.));
        let _ = path.sample_many(black_box(step_distance));
    }
}
