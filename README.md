# Intermediate Axis

Have you ever seen a [T-handle spinning in microgravity?](https://www.youtube.com/watch?v=1n-HMSCDYtM)

Most people's reaction: :open_mouth:

## Purpose of this repo

I first saw this video in 2015 when I was a 1st year Bachelor student.
I had learned enough to describe it mathematically and got a perturbative solution in 2016.
I then made an Octave script to solve it numerically in 2018.
This repo's purpose is to rewrite this code in Python and provide it to others who are interested in the phenomenology of the process.
I will also add mathematical description and perturbative description to be used for comparison with the numerical results, this is mostly to practice my LaTeX :) !

## Organization

- **Code** contains all the code (written in Python) which simulates the phenomenon.
It is split into several .py files as follows:
  - **Parameters_and_Constrants.py** includes all the physical parameters and inertial properties of the body to be simulated;
  - **RK_Driver.py** contains several functions which compute the right-hand side of the dynamical differential equations and drives numerical integration through a classical Runge-Kutta (RK4) integrator;
  - **Visualizations.py** contains some different visualization functions (either as 2D plots or time-dependent 3D animation with a vector);
  - **Test_Environment.py** is used for general testing of different functions and for a driver of the integration + plotting, it currently holds the T-handle scenario run and generates a .gif in the Code folder;
  - **Perturbative_Test.py** is used to check between the analytical solution (small deviation) and the numerical results obtained by the integrator - it is a verification that the code works in small offset from the intermediate axis;
- **Physics** contains a LaTeX document with the explanation of some of the basic concepts, it contains a general mathematical description of the rotary motion of a body in 3 dimensions and solves the differential equations perturbatively in the case of a small deviation from the intermediate axis;
- **Results** contains resulting plots and .gif animations obtained from running the different scenarios.

## Running the code for the T-Handle scenario
The script **Perturbative_Test.py** has a configuration where the handle is rotating with initial rate of 0.16 Hz at the intermediate axis of inertia and with a very small perturbation (1/100th of the intermediate rate) about the smallest axis.
The realistic values of the angular rate in the video is most likely in the ranges 0.5-3 Hz so several .gifs were produced at 0.16, 0.5 and 1 Hz.
These can be found in the Results folder and provide a very similar behaviour to that seen in the video.
