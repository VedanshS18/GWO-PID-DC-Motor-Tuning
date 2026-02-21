/*
 * Project: Automated PID Tuning for DC Motor Speed Control
 * Algorithm: Grey Wolf Optimizer (GWO)
 * Developer: Vedansh Saxena
 * Institute: MIT Bengaluru
 *
 * Description: 
 * This program uses a bio-inspired metaheuristic (GWO) to optimize 
 * Proportional, Integral, and Derivative gains for a DC Motor simulation.
 * The objective is to minimize the ITAE (Integral of Time-weighted Absolute Error).
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// Structure to represent an individual solution (wolf) in the pack
typedef struct {
    double params[3]; // Index 0: Kp, 1: Ki, 2: Kd
    double fitness;   // Stores the ITAE score
} Wolf;

// --- DC Motor Simulation & Fitness Evaluation ---
double simulate_motor(double Kp, double Ki, double Kd, double setpoint) {
    double current_speed = 0.0, last_error = 0.0, integral = 0.0, itae = 0.0;
    double dt = 0.01; // Time step for simulation

    // Simulate for 2.0 seconds to observe convergence
    for (double t = 0; t < 2.0; t += dt) {
        double error = setpoint - current_speed;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        
        // PID Control Signal
        double u = (Kp * error) + (Ki * integral) + (Kd * derivative);
        
        // Linearized DC Motor Dynamics (Physics Simulation)
        double acceleration = (u - (current_speed * 0.1)) / 0.1;
        current_speed += acceleration * dt;
        
        // ITAE calculation starts after 0.2s to ignore initial rise time
        if (t > 0.2) { 
            itae += t * fabs(error) * dt;
        }
        last_error = error;
    }
    
    // Safety check to handle numerical instability or infinity
    if (isnan(itae) || isinf(itae)) return 1e15; 
    return itae;
}

int main() {
    srand(time(0)); // Seed random number generator
    int pack_size, max_iter;
    double setpoint, ub;

    // --- User Configuration ---
    printf("--- GWO-PID Optimizer Settings ---\n");
    printf("Enter the Pack Size: "); scanf("%d", &pack_size);
    printf("Enter Maximum Iterations: "); scanf("%d", &max_iter);
    printf("Enter Target Speed (Setpoint): "); scanf("%lf", &setpoint);
    printf("Enter K Parameter Upper Bound: "); scanf("%lf", &ub);
    printf("----------------------------------\n\n");

    // Allocate memory for the wolf pack
    Wolf *pack = (Wolf *)malloc(pack_size * sizeof(Wolf));
    Wolf alpha, beta, delta;
    
    // Initialize leaders with extremely high error
    alpha.fitness = beta.fitness = delta.fitness = 1e15;

    // --- Initialization Phase ---
    // Randomly place wolves within the search space
    for (int i = 0; i < pack_size; i++) {
        for(int j=0; j<3; j++) pack[i].params[j] = ((double)rand()/RAND_MAX) * ub;
        pack[i].fitness = simulate_motor(pack[i].params[0], pack[i].params[1], pack[i].params[2], setpoint);
    }

    // --- Optimization Loop (The Hunt) ---
    for (int iter = 0; iter < max_iter; iter++) {
        // Identify Alpha, Beta, and Delta leaders
        for (int i = 0; i < pack_size; i++) {
            if (pack[i].fitness < alpha.fitness) {
                delta = beta; beta = alpha; alpha = pack[i];
            } else if (pack[i].fitness < beta.fitness) {
                delta = beta; beta = pack[i];
            } else if (pack[i].fitness < delta.fitness) {
                delta = pack[i];
            }
        }

        // 'a' decreases linearly from 2.0 to 0.0 to control convergence
        double a = 2.0 * (1.0 - (double)iter / max_iter);

        // Update the position of all Omega wolves
        for (int i = 0; i < pack_size; i++) {
            for (int j = 0; j < 3; j++) {
                double r1 = (double)rand()/RAND_MAX, r2 = (double)rand()/RAND_MAX;
                double A = 2.0 * a * r1 - a; 
                double C = 2.0 * r2;

                // Mathematics to encircle the prey based on leaders
                double X1 = alpha.params[j] - A * fabs(C * alpha.params[j] - pack[i].params[j]);
                double X2 = beta.params[j] - A * fabs(C * beta.params[j] - pack[i].params[j]);
                double X3 = delta.params[j] - A * fabs(C * delta.params[j] - pack[i].params[j]);

                // Update position to the mean of the three leaders
                pack[i].params[j] = (X1 + X2 + X3) / 3.0;

                // Boundary constraints: Keep K-parameters within [0, UB]
                if (pack[i].params[j] < 0) pack[i].params[j] = 0;
                if (pack[i].params[j] > ub) pack[i].params[j] = ub;
            }
            // Re-evaluate fitness of the updated wolf
            pack[i].fitness = simulate_motor(pack[i].params[0], pack[i].params[1], pack[i].params[2], setpoint);
        }
        
        // Print progress every 10 iterations
        if (iter % 10 == 0) printf("Iteration %d: Best ITAE = %.4f\n", iter, alpha.fitness);
    }

    // --- Final Output ---
    printf("\n--- GWO Optimization Success ---\n");
    printf("Best PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", alpha.params[0], alpha.params[1], alpha.params[2]);
    printf("Final ITAE: %.4f\n", alpha.fitness);

    free(pack); // Clean up allocated memory
    return 0;
}