# Intermediate Axis

Have you ever seen a [T-handle spinning in microgravity?](https://www.youtube.com/watch?v=1n-HMSCDYtM)

Most people's reaction: :open_mouth:

## Purpose of this repo

I first saw this video in 2015 when I was a 1st year Bachelor student.
I had learned enough to describe it mathematically and got a perturbative solution in 2016.
I then made an Octave script to solve it numerically in 2018.
This repo's purpose is to rewrite this code in Python and provide it to others who are interested in the phenomenology of the process.
I will also add mathematical description and perturbative description to be used for comparison with the numerical results!

## Organization

- **Code** contains all the code (written in Python) which simulates the phenomenon.
It is split into several .py files as follows:
  - **Parameters_and_Constrants** includes all the physical parameters and inertial properties of the body to be simulated;
  - **RK_Driver** contains several functions which compute the right-hand side of the dynamical differential equations and drives numerical integration through a classical Runge-Kutta (RK4) integrator;
  - **Visualizations** contains some different visualization functions (either as 2D plots or time-dependent 3D animation with a vector);
  - **Test_Environment** is used for general testing of different functions and for a driver of the integration + plotting;
  - **Perturbative_Test** is used to check between the analytical solution (small deviation) and the numerical results obtained by the integrator - it is a verification that the code works in small offset from the intermediate axis;
- **Physics** contains a LaTeX document with the explanation of some of the basic concepts, it contains a general mathematical description of the rotary motion of a body in 3 dimensions and solves the differential equations perturbatively in the case of a small deviation from the intermediate axis.

## Running the code for the T-Handle scenario
