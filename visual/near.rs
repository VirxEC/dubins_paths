use std::f32::consts::TAU;

use dubins_paths::f32::{DubinsPath, PosRot};
use plotters::prelude::*;
use rand::{RngExt, SeedableRng, distr::Uniform, rngs::SmallRng};
fn generate_start_end() -> (PosRot, PosRot) {
    // use rand::Rng;
    // let mut thread_rng = rand::rngs::ThreadRng::default();
    // let seed1 = thread_rng.next_u64();
    // let seed2 = thread_rng.next_u64();
    // dbg!(seed1, seed2);
    // let mut rng =
    //     SmallRng::seed_from_u64(seed1).sample_iter(Uniform::new(-1000.0, 1000.0).unwrap());
    // let mut rot_rng = SmallRng::seed_from_u64(seed2).sample_iter(Uniform::new(-TAU, TAU).unwrap());

    let mut rng = SmallRng::seed_from_u64(7805237080529646671)
        .sample_iter(Uniform::new(-1000.0, 1000.0).unwrap());
    let mut rot_rng =
        SmallRng::seed_from_u64(9624419770505363141).sample_iter(Uniform::new(-TAU, TAU).unwrap());

    let start = PosRot::from_floats(
        rng.next().unwrap(),
        rng.next().unwrap(),
        rot_rng.next().unwrap(),
    );

    let end = PosRot::from_floats(
        rng.next().unwrap(),
        rng.next().unwrap(),
        rot_rng.next().unwrap(),
    );

    (start, end)
}

const fn curvature(v: f32) -> f32 {
    if 0.0 <= v && v < 500.0 {
        0.006_900 - 5.84e-6 * v
    } else if 500.0 <= v && v < 1000.0 {
        0.005_610 - 3.26e-6 * v
    } else if 1000.0 <= v && v < 1500.0 {
        0.004_300 - 1.95e-6 * v
    } else if 1500.0 <= v && v < 1750.0 {
        0.003_025 - 1.1e-6 * v
    } else if 1750.0 <= v && v < 2500.0 {
        0.001_800 - 4e-7 * v
    } else {
        0.0
    }
}

const fn turn_radius(v: f32) -> f32 {
    if v == 0.0 { 0.0 } else { 1.0 / curvature(v) }
}

fn render(
    samples: Vec<PosRot>,
    extra_samples: Vec<PosRot>,
    rand_closest_point_pairs: Vec<([f32; 2], [f32; 2])>,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("0.png", (1080, 1080)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-2000f32..2000.0, -2000f32..2000.0)?;

    chart.configure_mesh().draw()?;

    chart.draw_series(samples.iter().enumerate().map(|(i, p)| {
        Circle::new(
            (p.x(), p.y()),
            2,
            RGBColor(
                (255.0 * (1.0 - i as f32 / samples.len() as f32)) as u8,
                (255.0 * (i as f32 / samples.len() as f32)) as u8,
                0,
            )
            .filled(),
        )
    }))?;

    chart.draw_series(
        rand_closest_point_pairs
            .iter()
            .map(|(a, b)| PathElement::new(vec![(a[0], a[1]), (b[0], b[1])], BLACK)),
    )?;

    chart.draw_series(
        rand_closest_point_pairs
            .iter()
            .map(|(a, _)| Circle::new((a[0], a[1]), 3, BLUE.filled())),
    )?;

    chart.draw_series(
        extra_samples
            .iter()
            .map(|p| Circle::new((p.x(), p.y()), 3, BLACK.filled())),
    )?;

    root.present()?;

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    const TURN_RADIUS: f32 = turn_radius(1410.0);

    let (start, end) = generate_start_end();
    let path = DubinsPath::shortest_from(start, end, TURN_RADIUS).unwrap();
    let path_info = path.get_path_info();

    let mut rng = SmallRng::seed_from_u64(0).sample_iter(Uniform::new(-2000.0, 2000.0).unwrap());
    let rand_closest_point_pairs: Vec<_> = (0..100)
        .map(|_| [rng.next().unwrap(), rng.next().unwrap()])
        .map(|point| {
            let est_dist = path_info.est_distance_traveled(point);
            let point_proj_path = path.sample(est_dist);

            (point, [point_proj_path.x(), point_proj_path.y()])
        })
        .collect();

    let path_samples = path.sample_many(10.0);
    let segment_samples = vec![
        path.sample(path.param[0] * path.rho),
        path.sample((path.param[0] + path.param[1]) * path.rho),
    ];

    render(path_samples, segment_samples, rand_closest_point_pairs)
}
