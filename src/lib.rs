#![warn(missing_docs, clippy::pedantic, clippy::all, clippy::nursery)]
#![allow(clippy::suboptimal_flops)]
#![doc = include_str!("../README.md")]
#![no_std]

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!("Either the std or the libm feature is required");

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "std")]
extern crate std;

/// [`glam`] is a crate that provides vector types, and used to provide a more ergonomic API
///
/// It requries the `glam` feature to be enabled in order to be used within this crate
#[cfg(feature = "glam")]
pub extern crate glam;

mod base;
pub use base::*;

/// `f32`-based implementation of Dubins paths
pub mod f32;
/// `f64`-based implementation of Dubins paths
pub mod f64;

#[cfg(test)]
mod tests {
    use super::{NoPathError, PathType, SegmentType, f32, f64};

    #[test]
    fn size_of_items() {
        assert_eq!(size_of::<f32::PosRot>(), 12);
        assert_eq!(size_of::<f64::PosRot>(), 24);
        assert_eq!(size_of::<f32::DubinsPath>(), 32);
        assert_eq!(size_of::<f64::DubinsPath>(), 64);
        assert_eq!(size_of::<PathType>(), 1);
        assert_eq!(size_of::<SegmentType>(), 1);
        assert_eq!(size_of::<NoPathError>(), 0);
    }

    #[test]
    fn mod2pi_test() {
        assert!(f32::mod2pi(-f32::from_bits(1)) >= 0.);
        assert!(f32::mod2pi(core::f32::consts::TAU).abs() < f32::EPSILON);
        assert!(f64::mod2pi(-f64::from_bits(1)) >= 0.);
        assert!(f64::mod2pi(core::f64::consts::TAU).abs() < f64::EPSILON);
    }
}
