use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dubins_paths::{DubinsPath, PathType, PosRot};
use std::time::Duration;

const TURN_RADIUS: f32 = 1. / 0.00076;

fn bench_shortest_csc_path(c: &mut Criterion) {
    let q0: PosRot = [2000., 2000., 0.].into();
    let q1: PosRot = [0., 0., std::f32::consts::PI].into();

    c.bench_function("shortest_csc_path", |b| {
        b.iter(|| {
            DubinsPath::shortest_in(
                black_box(q0),
                black_box(q1),
                black_box(TURN_RADIUS),
                black_box(&PathType::CSC),
            )
            .unwrap()
        })
    });
}

fn bench_shortest_ccc_path(c: &mut Criterion) {
    let q0: PosRot = [2000., 2000., 0.].into();
    let q1: PosRot = [0., 0., std::f32::consts::PI].into();

    c.bench_function("shortest_ccc_path", |b| {
        b.iter(|| {
            DubinsPath::shortest_in(
                black_box(q0),
                black_box(q1),
                black_box(TURN_RADIUS),
                black_box(&PathType::CCC),
            )
            .unwrap()
        })
    });
}

fn bench_shortest_path(c: &mut Criterion) {
    let q0: PosRot = [2000., 2000., 0.].into();
    let q1: PosRot = [0., 0., std::f32::consts::PI].into();

    c.bench_function("shortest_path", |b| {
        b.iter(|| DubinsPath::shortest_from(black_box(q0), black_box(q1), black_box(TURN_RADIUS)).unwrap())
    });
}

fn bench_many_sample(c: &mut Criterion) {
    const STEP_DISTANCE: f32 = 10.;
    let q0: PosRot = [2000., 2000., 0.].into();
    let q1: PosRot = [0., 0., std::f32::consts::PI].into();
    let path = DubinsPath::shortest_from(q0, q1, TURN_RADIUS).unwrap();

    c.bench_function("many_sample", |b| b.iter(|| path.sample_many(black_box(STEP_DISTANCE))));
}

criterion_group! {
    name = benches;
    config = Criterion::default().measurement_time(Duration::from_secs(10));
    targets = bench_shortest_csc_path, bench_shortest_ccc_path, bench_shortest_path, bench_many_sample
}
criterion_main!(benches);
