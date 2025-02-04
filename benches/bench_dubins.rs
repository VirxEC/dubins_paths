use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dubins_paths::{DubinsPath, FloatType, PathType, PosRot, PI};

const TURN_RADIUS: FloatType = 1. / 0.00076;

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
    const STEP_DISTANCE: FloatType = 10.;
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
    bench_many_sample
);
criterion_main!(benches);
