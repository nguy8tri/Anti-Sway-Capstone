#ifndef _DISCRETE_LIB_H_
#define _DISCRETE_LIB_H_

/* Discrete-Time Data Structures */

/**
 * A struct representing a biquad
*/
typedef struct {
    double numerator[3];  // The numerator coefficients, in decreasing order of time delays (z^0, z^-1, z^-2)
    double denominator[3]; // The denominator coefficients, in decreasing order of time delays (z^0, z^-1, z^-2)
    double prev_input[2]; // The previous inputs, in increasing time delays (z^-1, z^-2)
    double prev_output[2]; // The previous outputs, in increasing time delays (z^-1, z^-2)
} Biquad;

/**
 * A proportional constant
*/
typedef float Proportional;

/**
 * A struct representing an integrator
*/
typedef struct {
    Proportional gain; // Accounts for timestep
    Biquad integral = {{1.0, 1.0, 0.0}, {2.0, -2.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
} Integrator;

/**
 * A struct representing a derivative term
*/
typedef struct {
    Proportional gain; // Accounts for timestep
    Biquad derivative = {{2.0, -2.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
} Differentiator;

/* Initialization Functions */

/**
 * Initializes an Integrator
 * 
 * @param gain The gain to assign the integrator
 * @param timestep The timestep to approximate the integrator
 * @param result A return parameter, which becomes the integrator
 * with the gain and timestep
 * 
 * @return result, which will be an integrator with a gain gain, and
 * the timestep
*/
void IntegratorInit(Proportional gain, double timestep, Integrator *result);

/**
 * Initializes a Differentiator
 * 
 * @param gain The gain to assign the differentiator
 * @param timestep The timestep to approximate the differentiator
 * @param result A return parameter, which becomes the differentiator
 * with the gain and timestep
 * 
 * @return result, which will be an differentiator with a gain gain, and
 * the timestep
*/
void DifferentiatorInit(Proportional gain, double timestep, Differentiator *result);


/* Time-Stepping Functions */

/**
 * Executes a dynamic, discrete time system by using
 * its biquad decomposition.
 * 
 * @param input The input to the system
 * @param sys The system, as an array of biquads
 * @param size The size of sys
 * @param lower_lim The lower saturation limit of the system
 * @param upper_lim The upper saturation limit of the system
 * 
 * @return The output of the system given the input
 * 
 * @pre The input is the next sampled value of the input to
 * the system 
 * @post The system is updated with current/past calculated values
*/
inline double Cascade(double input, Biquad sys[], int size, double lower_lim, double upper_lim);

/**
 * Timesteps an Integration
 * 
 * @param input The input to the integrator
 * @param term A pointer to an integrator term
 * @param lower_lim The lower saturation limit of the system
 * @param upper_lim The upper saturation limit of the system
 * 
 * @return The output of the integrator given the input
 * 
 * @pre The input is the next sampled value of the input to
 * the system
 * @post term is updated with current/past calculated values
*/
inline double Integrate(double input, Integrator *term, double lower_lim, double upper_lim);

/**
 * Timesteps a Differentiation
 * 
 * @param input The input to the differentiator
 * @param term A pointer to an differentiator term
 * @param lower_lim The lower saturation limit of the system
 * @param upper_lim The upper saturation limit of the system
 * 
 * @return The output of the differentiator given the input
 * 
 * @pre The input is the next sampled value of the input to
 * the system
 * @post term is updated with current/past calculated values
*/
inline double Differentiate(double input, Differentiator *term, double lower_lim, double upper_lim);

/**
 * Timesteps a PID Controller
 * 
 * @param input The input to the PID Controller
 * @param p A pointer to the proportional term
 * @param i A pointer to the integrator term
 * @param d A pointer to the differentiator term
 * @param lower_lim The lower saturation limit of the system
 * @param upper_lim The upper saturation limit of the system
 * 
 * @return The output of the PID Controller given the input
 * 
 * @pre The input is the next sampled value of the input to
 * the system
 * @post i and d are updated with current/past calculated values
*/
inline double PID(double input, Proportional *p, Integrator *i, Differentiator *d, double lower_lim, double upper_lim);

#endif  // _DISCRETE_LIB_H_