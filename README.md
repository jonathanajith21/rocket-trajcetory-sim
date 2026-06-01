# Rocket Flight Trajectory Simulator

A 1-DOF rocket flight simulator written in MATLAB (base, no toolboxes). It models a
rocket's full vertical flight, launch, powered ascent, burnout, coast, apogee, and
parachute recovery, and includes a static-stability check and a motor-selection study.
Validated against OpenRocket.

## What it does

The simulator treats the rocket as a point mass moving vertically. At each instant it
computes the net force (thrust − weight − drag) and integrates the resulting acceleration
with `ode45` to track altitude, velocity, and mass over the flight. It includes:

- A thrust-curve model integrated to total impulse and mapped to a motor class (A, B, C…)
- An exponential atmosphere (air density falls with altitude)
- Velocity-opposing drag, with separate coefficients for ascent and parachute descent
- A two-phase integration that deploys the chute after apogee, using events to pin
  apogee and landing exactly
- A Barrowman static-stability estimate (center of pressure vs center of gravity,
  reported as static margin in calibers)
- Max-Q (peak dynamic pressure) for structural load reference

## Files

| File | Purpose |
|------|---------|
| `define_rocket.m` | Rocket, motor, and environment parameters |
| `simulate_flight.m` | The physics engine — runs a flight, returns a results struct |
| `rocket_trajectory_simulator.m` | Single-flight run: prints a summary and six plots |
| `motor_selection_study.m` | Sweeps several motors to find the best fit for a target apogee |
| `define_rocket_validation.m` | Parameters matched to an OpenRocket build (AeroTech H165R) |
| `run_validation.m` | Compares the simulator's output to OpenRocket |

## How to run

Keep all files in the same folder, set it as your MATLAB Current Folder, then:

- Single flight + plots: run `rocket_trajectory_simulator.m`
- Motor-selection study: run `motor_selection_study.m`
- Validation against OpenRocket: run `run_validation.m`

## Validation

I rebuilt the same rocket in OpenRocket with an AeroTech H165R motor and compared.
With total impulse matched to the real motor:

| Metric | This sim | OpenRocket | Difference |
|--------|----------|------------|------------|
| Apogee (m) | 542 | 445 | +21.7% |
| Max velocity (m/s) | 109 | 103 | +5.6% |
| Max acceleration (g) | 19.4 | 12.8 | +50.9% |
| Total impulse (N·s) | 165 | 161 | +2.5% |

Max velocity and total impulse match closely. Apogee and peak acceleration run higher,
consistently, this traces to the simplifications in the model: a fixed drag coefficient
(versus OpenRocket's Mach-dependent drag, which lets my rocket coast higher) and a
synthetic thrust curve that front-loads the burn more than the real motor.

## Limitations and next steps

- 1-DOF only (vertical flight, no wind or off-vertical launch)
- Fixed drag coefficient rather than Mach-dependent
- Synthetic thrust curve; real .eng motor data would improve the acceleration match
- Stability uses an estimated CG; real CAD mass properties would make it exact
  
