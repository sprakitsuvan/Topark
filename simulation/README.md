# Simulation
[BallBeam_SMK.zip](https://github.com/user-attachments/files/35014556/BallBeam_SMK.zip)

Simulation

Software used: MATLAB R2026a, Simulink Multibody

Model files and scripts
- BallBeam_SM_v1.slx: Simulink and Simscape Multibody model
- params_ballbeam_v1.mat: parameters that the model loads automatically on opening

Solver settings
- ode23t (variable step), relative tolerance 1e-4
- Maximum step = tEnd / animationFrames = 8/260
- Simulation time 8 s, gravity 9.81 m/s² in the −Y direction
  
Assumptions
- The ball slides without rolling (acceleration is g·sin θ).
Ball–beam contact is not modeled; a prismatic joint constrains the ball to the beam surface.
- All parts are rigid; sensors are ideal, with no noise or delay.
- The actuator is a first-order system (τ = 0.08 s); the controller is a continuous PD controller (Kp = 7.2, Kd = 4.6, angle limited to ±12°).
  
Boundary and initial conditions
- The base plate, post, side plates, and pin are rigidly fixed to the world frame (no degrees of freedom).
- Beam: revolute joint about the +Z axis at the origin; initial angle 0 rad; the angle is set by the actuator.
- Ball: prismatic joint along the beam’s X axis; initial position 50 mm, initial velocity 0; target x = 0.
