# Dubin's Paths

[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Rust code for calculating Dubin's Paths

Credit to [Andrew Walker](https://github.com/AndrewWalker) for the [original C code](https://github.com/AndrewWalker/Dubins-Curves)

I've ported the code to Rust and documented everything that I could understand. Documentation in the original repository was minimal.

## Quick example

```rust
use core::f32::consts::PI;
use dubins_paths::{self, DubinsPath, PathType, PosRot};

// PosRot has the format [f32; 3]
// It represents the car's [x, y, theta]
// Where x and y are the coordinates on a 2d plane
// and theta is the orientation of the car's front in radians

// The starting position
let q0: PosRot = [0., 0., PI / 4.];

// The target end position
let q1: PosRot = [100., -100., PI * (3. / 4.)];

// The car's turning radius (must be > 0)
// This can be calculated by taking a cars angular velocity and dividing it by the car's forward velocity
let rho: f32 = 11.6;

// Calculate the shortest possible path between these two points with the given turning radius
let shortest_path_possible: dubins_paths::Result<DubinsPath> = DubinsPath::shortest_from(q0, q1, rho);

// Assert that the path was found!
assert!(shortest_path_possible.is_ok());
```

DubinsPath has many methods you should look into, such as length, extract_subpath, sample, and sample_many.

## Features

* `glam` - Use a [glam](https://crates.io/crates/glam) compatible API
* `fast-math` - Enable the fast-math feature in glam

## More documentation

Looking for some more detailed documentation? Head on over to the [docs.rs](https://docs.rs/dubins_paths/) page!
