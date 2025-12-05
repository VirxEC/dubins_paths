# Dubin's Paths

[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Calculates a path between two points in space with starting and ending rotation requirements.

Credit to [Andrew Walker](https://github.com/AndrewWalker) for the [original C code](https://github.com/AndrewWalker/Dubins-Curves)
I've ported the code to Rust and documented everything that I could understand. Documentation in the original repository was minimal.

The car is assumed to be a dubin's car.
A dubin's car is a car that can only do 3 things: turn left, turn right, or go straight.

## Examples

### Basic usage

This will calculate the path that connects the current position and rotation of the car to the desired position and rotation.

```rust
use core::f32::consts::PI;
use dubins_paths::{f32::{DubinsPath, PosRot}, Result as DubinsResult};

// PosRot represents the car's (Pos)ition and (Rot)ation
// Where x and y are the coordinates on a 2d plane
// and theta is the orientation of the car's front in radians

// The starting position and rotation
// PosRot::from_floats can also be used for const contexts
const q0: PosRot = PosRot::from_floats(0., 0., PI / 4.);

// The target end position and rotation
// PosRot implements From<[f32; 3]>
let q1 = [100., -100., PI * (3. / 4.)].into();

// The car's turning radius (must be > 0)
// This can be calculated by taking a cars angular velocity and dividing it by the car's forward velocity
// `turn radius = ang_vel / forward_vel`
let rho = 11.6;

// Calculate the shortest possible path between these two points with the given turning radius
let shortest_path_possible: DubinsResult<DubinsPath> = DubinsPath::shortest_from(q0, q1, rho);

// Assert that the path was found!
assert!(shortest_path_possible.is_ok());
```

### Sample path for points

Calculating the path is very optimized, and does not include any points along the path.

This means that if you want to get points along the path, extra work must be done.

Below, we calculate all points along a path spaced at a given interval. Use [`sample`] instead of [`sample_many`] to get only one point.

```rust
use core::f32::consts::PI;
use dubins_paths::f32::{DubinsPath, PosRot};

let shortest_path_possible = DubinsPath::shortest_from([0., 0., PI / 4.].into(), [100., -100., PI * (3. / 4.)].into(), 11.6).unwrap();

// The distance between each sample point
let step_distance = 5.;

#[cfg(feature = "alloc")]
{
    let samples: Vec<PosRot> = shortest_path_possible.sample_many(step_distance);

    // The path is just over 185 units long
    assert_eq!(shortest_path_possible.length().round(), 185.0);

    // There are 37 points spaced 5 units apart (37 * 5 = 185), + 1 more for the endpoint
    assert_eq!(samples.len(), 38);
}
```

## Features

* `std` - (Default) Enables the use of the standard library
* `alloc` - (Default) Enables [`sample_many`] and [`sample_many_range`]
* `libm` - Use the [`libm`] crate for math operations
* `glam` - Use a [`glam`](https://crates.io/crates/glam) compatible API
* `serde` - Implementations of [`serde::Deserialize`] and [`serde::Serialize`] for most types
* `rkyv` - Implementations of [`rkyv::Archive`], [`rkyv::Deserialize`] and [`rkyv::Serialize`] for most types

[`sample`]: f32::DubinsPath::sample
[`sample_many`]: f32::DubinsPath::sample_many
[`sample_many_range`]: f32::DubinsPath::sample_many_range