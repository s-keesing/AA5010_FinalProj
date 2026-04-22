# AA5010_FinalProj
Code for final project "Attitude Dynamics and Control of a Nano-Satellite Orbiting Mars" for AA5010 course at CU Boulder

## Overview

This repo contains MATLAB code developed for a project on attitude determination and control of a nano-satellite in low Mars orbit. The objective of the project is to simulate the spacecraft's ability to autonomously transition between multiple operational modes—sun-pointing (power), nadir-pointing (science), and GMO-pointing (communication)—while maintaining stable and accurate attitude control.

The implementation includes orbit modeling, reference frame generation, nonlinear rigid-body attitude dynamics, and a feedback control system based on Modified Rodrigues Parameters (MRPs).

---

## Features

* Circular orbit propagation for both LMO (nano-satellite) and GMO (mother spacecraft)
* Construction of multiple reference frames:

  * Inertial frame (N)
  * Hill frame (H)
  * Sun-pointing frame (Rs)
  * Nadir-pointing frame (Rn)
  * GMO-pointing frame (Rc)
* Attitude representation using Modified Rodrigues Parameters (MRPs)
* Nonlinear rigid-body dynamics simulation
* Fourth-order Runge-Kutta (RK4) numerical integrator with shadow-set switching
* Proportional-Derivative (PD) attitude control law
* Autonomous mission mode switching logic
* Full mission simulation over one orbital period

---

## Repo Structure

```
/src
    mission_simulation.m     % Runs full mission scenario
    RK4.m                    % RK4 propagation with MRP shadow switching
    tracking_error.m         % Computes attitude error states
    hill2inertial.m          % Computes inertial position and velocity vectors for a given spacecraft
    inertial2sunpointing.m   % Generates DCM rotating between sun-pointing frame and inertial frame
    Rc2dcm.m                 % Generates DCM rotating between GMO-pointing frame and inertial frame
    Rn2dcm.m                 % Generates DCM rotating between nadir-pointing frame and inertial frame
    lmo2dcm.m                % Generates DCM rotating between inertial frame and LMO-centered Hill frame
    RcNomega.m               % Computes angular velocity between GMO-pointing frame and inertial frame
    RnNomega.m               % Computes angular velocity between nadir-pointing frame and inertial frame
    attitude_integration.m   % Demonstrates integration of attitude states over time for arbitrary initial state
    sun_pointing_control.m   % Simulates attitude control from initial state to sun-pointing mode
    gmo_pointing_control.m   % Simulates attitude control from initial state to GMO-pointing mode
    nadir_pointing_control.m % Simulates attitude control from initial state to nadir-pointing mode
    control_law.m            % Computes control torque
    plot_attitude_results.m  % Constructs subplots of attitude states and tracking error over time

README.md
```

---

## How to Run

1. Open MATLAB and navigate to the repository root directory.
2. Ensure all subfolders are on the MATLAB path:

   ```matlab
   addpath(genpath(pwd));
   ```
3. Run the main simulation script:

   ```matlab
   mission_simulation
   ```
4. The script will:

   * Simulate attitude dynamics over one orbit (~6500 s)
   * Automatically switch between mission modes
   * Generate plots for:

     * Attitude states
     * Tracking error
     * Mission mode vs. time
     * Control torque

---

## Control Design

The spacecraft uses a proportional-derivative (PD) feedback control law based on attitude and angular velocity tracking error. Gains are selected using a linearized approximation of the error dynamics to meet the following design requirements:

* Maximum settling time: 120 seconds
* Critically damped or underdamped response
* Uniform gains across all principal axes

---

## Assumptions

* Spacecraft is a rigid body with known principal inertia
* Full three-axis control authority is available
* No external disturbances (e.g., gravity gradient, drag, SRP)
* Perfect state knowledge (no sensor noise)
* Circular, two-body orbits with constant angular rates

---

## Outputs

The simulation produces:

* Time histories of attitude (MRPs) and angular velocity
* Tracking error relative to active reference frame
* Mission mode transitions
* Control torque profiles

These outputs correspond directly to the figures presented in the project report.

---

## Author

Senna Keesing

---

## Notes

This code was developed for academic purposes as part of an aerospace engineering course project. It is intended to demonstrate fundamental concepts in spacecraft attitude dynamics and control rather than serve as flight-ready software.
