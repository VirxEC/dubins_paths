# Dubin's Paths

## DubinsPathType

### LSL

A "Left Straight Left" Dubin's path.

### LSR

A "Left Straight Right" Dubin's path.

### RSL

A "Right Straight Left" Dubin's path.

### RSR

A "Right Straight Right" Dubin's path.

### RLR

A "Right Left Right" Dubin's path.

### LRL

A "Left Right Left" Dubin's path.

### from(value: usize) -> Self

Convert from usize to a path type

### to(self: &Self) -> usize

Convert from path type to usize

### csc() -> [DubinsPathType; 4]

Get all of the "Turn Straight Turn" path types

Returns `[Self::LSL, Self::LSR, Self::RSL, Self::RSR]`

### ccc() -> [DubinsPathType; 2]

Get all of the "Turn Turn Turn" path types

Returns `[Self::RLR, Self::LRL]`

### all() -> [DubinsPathType; 6]

Get all of the path types

Returns `[Self::LSL, Self::LSR, Self::RSL, Self::RSR, Self::RLR, Self::LRL]`

## DubinsPath

### q0

Three f64's in the format: `[x, y, theta]`

Represents starting location and orientation of the car.

### q1

Three f64's in the format: `[x, y, theta]`

Represents ending location and orientation of the car.

### rho

The turning radius of the car.

Can be can be calculated by taking the forward velocity of the car and dividing it by the car's angular velocity.

### type_

The type of the Dubin's path.

## DubinsError

### CoConfigs

Colocated configurations

### Param

Path parameterisitation error

### BadRho

The rho value is invalid

### NoPath

No connection between configurations with this word

## shortest_path(q0: [f64; 3], q1: [f64; 3], rho: f64) -> Result<DubinsPath, DubinsError>

Find the shortest path out of the 6 Dubin's paths.

### q0

Three f64's in the format: `[x, y, theta]`

Represents starting location and orientation of the car.

### q1

Three f64's in the format: `[x, y, theta]`

Represents ending location and orientation of the car.

### rho

The turning radius of the car.

Can be can be calculated by taking the forward velocity of the car and dividing it by the car's angular velocity.

## shortest_path_in(q0: [f64; 3], q1: [f64; 3], rho: f64, types: &[DubinsPathType]) -> Result<DubinsPath, DubinsError>

Find the shortest path out of the specified Dubin's paths.

### q0

Three f64's in the format: `[x, y, theta]`

Represents starting location and orientation of the car.

### q1

Three f64's in the format: `[x, y, theta]`

Represents ending location and orientation of the car.

### rho

The turning radius of the car.

Can be can be calculated by taking the forward velocity of the car and dividing it by the car's angular velocity.

### types

A reference to a slice that contains the path types to be compared.

## path(q0: [f64; 3], q1: [f64; 3], rho: f64, path_type: DubinsPathType) -> Result<DubinsPath, DubinsError>

Calculate the Dubin's path that was specified.

### q0

Three f64's in the format: `[x, y, theta]`

Represents starting location and orientation of the car.

### q1

Three f64's in the format: `[x, y, theta]`

Represents ending location and orientation of the car.

### rho

The turning radius of the car.

Can be can be calculated by taking the forward velocity of the car and dividing it by the car's angular velocity.

### path_type

The Dubins path type that's to be calculated.
