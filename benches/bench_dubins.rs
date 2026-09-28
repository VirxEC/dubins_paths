use core::{
    f32::consts::{PI, TAU},
    hint::black_box,
};

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use dubins_paths::{
    PathType,
    f32::{DubinsPath, PosRot},
};
use rand::{Rng, RngExt, SeedableRng, distr::Uniform, rngs::SmallRng};

const TURN_RADIUS: f32 = 1. / 0.00076;

fn generate_start_end(seed_rng: &mut SmallRng) -> (PosRot, PosRot) {
    let mut rng = SmallRng::seed_from_u64(seed_rng.next_u64())
        .sample_iter(Uniform::new(-1000.0, 1000.0).unwrap());
    let mut rot_rng =
        SmallRng::seed_from_u64(seed_rng.next_u64()).sample_iter(Uniform::new(-TAU, TAU).unwrap());

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

fn generate_random_points(seed_rng: &mut SmallRng) -> Vec<[f32; 2]> {
    let mut rng = SmallRng::seed_from_u64(seed_rng.next_u64())
        .sample_iter(Uniform::new(-2000.0, 2000.0).unwrap());

    (0..3 * 120)
        .map(|_| [rng.next().unwrap(), rng.next().unwrap()])
        .collect()
}

fn bench_est_distance_traveled(c: &mut Criterion) {
    let mut seed_rng = SmallRng::seed_from_u64(0);

    let paths = (0..100)
        .map(|_| {
            let (start, end) = generate_start_end(&mut seed_rng);

            (
                DubinsPath::shortest_from(start, end, TURN_RADIUS).unwrap(),
                generate_random_points(&mut seed_rng),
            )
        })
        .collect::<Vec<_>>();

    c.bench_with_input(
        BenchmarkId::new("est_distance_traveled", "100 paths with 200 points each"),
        &paths,
        |b, paths| {
            b.iter(|| {
                for (path, points) in paths {
                    let path_info = path.get_path_info();

                    for &point in points {
                        black_box(path_info.est_distance_traveled(point));
                    }
                }
            });
        },
    );
}

fn setup_benchmark() -> (PosRot, PosRot) {
    let q0: PosRot = [2000., 2000., 0.].into();
    let q1: PosRot = [0., 0., PI].into();
    (q0, q1)
}

fn bench_shortest_path_type(c: &mut Criterion, name: &str, path_types: &[PathType]) {
    let (q0, q1) = setup_benchmark();

    c.bench_function(name, |b| {
        b.iter(|| {
            DubinsPath::shortest_in(
                black_box(q0),
                black_box(q1),
                black_box(TURN_RADIUS),
                black_box(path_types),
            )
            .unwrap()
        });
    });
}

fn bench_shortest_csc_path(c: &mut Criterion) {
    bench_shortest_path_type(c, "shortest_csc_path", &PathType::CSC);
}

fn bench_shortest_ccc_path(c: &mut Criterion) {
    bench_shortest_path_type(c, "shortest_ccc_path", &PathType::CCC);
}

fn bench_shortest_path(c: &mut Criterion) {
    let (q0, q1) = setup_benchmark();

    c.bench_function("shortest_path", |b| {
        b.iter(|| {
            DubinsPath::shortest_from(black_box(q0), black_box(q1), black_box(TURN_RADIUS)).unwrap()
        });
    });
}

fn bench_many_sample(c: &mut Criterion) {
    const STEP_DISTANCE: f32 = 19.;
    let (q0, q1) = setup_benchmark();
    let path = DubinsPath::shortest_from(q0, q1, TURN_RADIUS).unwrap();

    c.bench_function("many_sample", |b| {
        b.iter(|| path.sample_many(black_box(STEP_DISTANCE)));
    });
}

criterion_group!(
    benches,
    bench_shortest_csc_path,
    bench_shortest_ccc_path,
    bench_shortest_path,
    bench_many_sample,
    bench_est_distance_traveled
);
criterion_main!(benches);
