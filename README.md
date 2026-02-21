# GWO-PID DC Motor Tuning
A bio-inspired optimization tool written in C.

## Overview
This project uses the **Grey Wolf Optimizer (GWO)** to find the optimal gains (Kp, Ki, Kd) for a PID controller. The objective is to minimize the **ITAE** (Integral of Time-weighted Absolute Error) of a DC motor's speed.

## Results
- **Optimized ITAE:** 0.0109
- **Set Point:** 50.0
- **Convergence:** ~10 Iterations
