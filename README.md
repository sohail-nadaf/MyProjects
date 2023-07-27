# Non linear 6 - DoF Simulation of a Fixed wing UAV

The aim of this project is to simulate the dynamics of the aircraft and implement control techniques for attitude regulation. The implementation of sub models is as follows:
- The dynamics of the aircraft propagate through the Runge-Kutta integration scheme(RK4), any computation with the orientation was performed in quaternion algebra and converted back to Euler angles for visualization.
- The atmospheric conditions are modeled using an approximation to the von Karman turbulence model given by the Dryden-Functions(low altitude, moderate turbulence) as a gain to the white noise input signal along with a steady wind in the inertial frame.
- Gravity is expressed in UAV body frame.
- The aerodynamic model uses stability and control derivatives along with some nonlinearity in the lift curve to imitate the effects of stall. 
- The propulsion is modeled as a function of propeller velocity which in turn is a function of voltage applied to the motor.

# Matlab Files
<details>
<summary>Main Loop</summary>

- mavsim.m
</details>
<details>
<summary>Model files</summary>
	
- mav_dynamics.m

- wind_simulation.m

- dryden.m
</details>

<details>
<summary>Model viewer</summary>

- data_viewer.m

- spacecraft_viewer.m
</details>

<details>
<summary>Parameters</summary>

- aerosonde.m

- simulation_parameters.m
</details>

<details>
<summary>State container</summary>

- msg_state.m
</details>
