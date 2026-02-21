# GWO-PID DC Motor Tuning
A bio-inspired optimization tool written in C for automated controller design.

## Overview
This project implements the **Grey Wolf Optimizer (GWO)** to find the optimal proportional, integral, and derivative gains ($K_p, K_i, K_d$) for a PID controller. The algorithm mimics the social hierarchy and hunting mechanism of grey wolves to navigate complex search spaces and identify global optima.

The objective of the optimization is to minimize the **ITAE** (Integral of Time-weighted Absolute Error) of a DC motor's speed, ensuring fast response times and zero steady-state error.

## Features
- **Dynamic Simulation:** Model-based DC motor dynamics integrated directly into the fitness function.
- **Nature-Inspired Search:** Efficient convergence using Alpha, Beta, and Delta leadership structures.
- **Interactive Configuration:** Users can define Pack Size, Iteration limits, and Target Setpoints at runtime.
- **Robustness:** Includes safety checks for numerical stability and boundary constraints for PID gains.

## How to Run
1. Compile the code using a C compiler (e.g., GCC):
   `gcc main.c -lm -o gwo_optimizer`
2. Execute the program:
   `./gwo_optimizer`
3. Enter your desired motor parameters and search constraints when prompted.
