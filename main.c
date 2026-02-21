/* * GWO-PID DC Motor Control Project
 * Vedansh Saxena - MIT Bengaluru
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// Individual wolf in the pack
typedef struct {
    double params[3]; // [0]=Kp, [1]=Ki, [2]=Kd
    double fitness;
} Wolf;

// Motor simulation function - calculates ITAE error
double simulate_motor(double Kp, double Ki, double Kd, double setpoint) {
    double current_speed = 0.0;
    double last_error = 0.0;
    double integral = 0.0;
    double itae = 0.0;
    double dt = 0.01; 

    for (double t = 0; t < 2.0; t += dt) {
        double error = setpoint - current_speed;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        
        // PID Formula
        double u = (Kp * error) + (Ki * integral) + (Kd * derivative);
        
        // Motor physics
        double acceleration = (u - (current_speed * 0.1)) / 0.1;
        current_speed += acceleration * dt;
        
        // Calculate ITAE (start after 0.2s to skip initial rise)
        if (t > 0.2) { 
            itae += t * fabs(error) * dt;
        }
        last_error = error;
    }
    
    // Check for bad values
    if (isnan(itae) || isinf(itae)) return 1e15; 
    return itae;
}

int main() {
    srand(time(0)); 
    
    int pack_size, max_iter;
    double setpoint, ub;

    printf("--- GWO-PID Optimizer Settings ---\n");
    printf("Enter Pack Size: "); scanf("%d", &pack_size);
    printf("Enter Max Iterations: "); scanf("%d", &max_iter);
    printf("Enter Target Speed: "); scanf("%lf", &setpoint);
    printf("Enter Upper Bound: "); scanf("%lf", &ub);
    printf("----------------------------------\n\n");

    // Init pack
    Wolf *pack = (Wolf *)malloc(pack_size * sizeof(Wolf));
    Wolf alpha, beta, delta;
    
    alpha.fitness = beta.fitness = delta.fitness = 1e15;

    // Random initialization of wolves
    for (int i = 0; i < pack_size; i++) {
        for(int j=0; j<3; j++) {
            pack[i].params[j] = ((double)rand()/RAND_MAX) * ub;
        }
        pack[i].fitness = simulate_motor(pack[i].params[0], pack[i].params[1], pack[i].params[2], setpoint);
    }

    // Main optimization loop
    for (int iter = 0; iter < max_iter; iter++) {
        
        // Sort leaders
        for (int i = 0; i < pack_size; i++) {
            if (pack[i].fitness < alpha.fitness) {
                delta = beta; 
                beta = alpha; 
                alpha = pack[i];
            } else if (pack[i].fitness < beta.fitness) {
                delta = beta; 
                beta = pack[i];
            } else if (pack[i].fitness < delta.fitness) {
                delta = pack[i];
            }
        }

        // Decay constant 'a'
        double a = 2.0 * (1.0 - (double)iter / max_iter);

        // Update omega wolves
        for (int i = 0; i < pack_size; i++) {
            for (int j = 0; j < 3; j++) {
                double r1 = (double)rand()/RAND_MAX;
                double r2 = (double)rand()/RAND_MAX;
                
                double A = 2.0 * a * r1 - a; 
                double C = 2.0 * r2;

                // Position calculation relative to leaders
                double X1 = alpha.params[j] - A * fabs(C * alpha.params[j] - pack[i].params[j]);
                double X2 = beta.params[j] - A * fabs(C * beta.params[j] - pack[i].params[j]);
                double X3 = delta.params[j] - A * fabs(C * delta.params[j] - pack[i].params[j]);

                // Final position update
                pack[i].params[j] = (X1 + X2 + X3) / 3.0;

                // Bound check
                if (pack[i].params[j] < 0) pack[i].params[j] = 0;
                if (pack[i].params[j] > ub) pack[i].params[j] = ub;
            }
            // Update fitness
            pack[i].fitness = simulate_motor(pack[i].params[0], pack[i].params[1], pack[i].params[2], setpoint);
        }
        
        if (iter % 10 == 0) {
            printf("Iteration %d: Best ITAE = %.4f\n", iter, alpha.fitness);
        }
    }

    printf("\n--- Optimization Results ---\n");
    printf("Final PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", alpha.params[0], alpha.params[1], alpha.params[2]);
    printf("Best ITAE: %.4f\n", alpha.fitness);

    free(pack); 
    return 0;
}
