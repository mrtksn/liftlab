# Drone Force Bench

An interactive 3D simulator for a drone frame you can change while it flies. You attach motors, servo-tilted motors, rigid masses and masses on cables, and the controller works out the motor thrusts and servo angles needed to hold it steady. A flight-envelope check tells you whether the current layout can hover at all and how much control headroom is left on each axis.

## Run it

Open `index.html` in a browser. It is a single self-contained file. It needs an internet connection to load three.js (r128, from cdnjs) and the Google Fonts it uses.

## What it models

- One rigid body with 6 degrees of freedom, integrated at 2 kHz. The controller runs at 1 kHz.
- **Motor:** thrust along its axis, first-order spin-up lag, drag torque κ·T opposite to its spin, adjustable health.
- **Motor on servo:** thrust direction rotates about a hinge, with a servo angle limit and a maximum servo speed.
- **Rigid mass:** box, sphere or vertical cylinder, contributing mass, center-of-gravity shift and inertia.
- **Mass on cable:** a point mass on a tension-only spring-damper cable that can swing, go slack and touch the ground.
- Masses and cables can be hidden from the controller ("Controller knows" off), so it must absorb them with integral action.

## Control and allocation

- Position PID produces a desired force. Attitude uses geometric control with integral action.
- Gains are set in acceleration units and multiplied by the modeled mass and inertia, so they carry over to new geometry.
- Allocation is two-stage bounded weighted least squares. Stage 1 picks servo angles using virtual inputs (T·cos θ, T·sin θ). Stage 2 solves motor thrusts at the servos' actual angles.
- Two steering modes: "Tilt body" (4 controlled axes: climb, roll, pitch, yaw) and "Stay level" (all 6 axes, needs thrust vectoring).

## Flight envelope

The attainable set of accelerations is a zonotope built from each actuator's contribution, linearized over servo range. It is compared with what hover requires, including static cable loads, and reported as Flyable / Marginal / Cannot hover / Not controllable, plus per-axis headroom.

## Known simplifications

- The controller sees the true state (no sensor noise or estimator).
- A tilting motor's mass stays at its pivot. Gyroscopic torque from spinning props is ignored.
- No aerodynamic interaction between rotors, the frame and the payload beyond simple linear drag.
