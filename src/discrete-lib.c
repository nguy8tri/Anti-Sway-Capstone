// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include "discrete-lib.h"

// Saturates a value (val) given a low (lo) and high (hi) value
#define SATURATE(val, lo, hi) val < lo ? lo : (val > hi ? hi : val)


/* Cascade Helper Function */

/**
 * Evaluates a singular dynamic distrete time biquad 
 * within a system, which itself is a system.
 * 
 * @param sys The system
 * @param input The system's input
 * 
 * @return The output of the system
 * 
 * @pre The input is the next sampled value of the input to
 * the system 
 * @post The system is updated with current/past calculated values
*/
static inline double EvaluateBiquad(Biquad *sys, double input);


/* Initialization Functions */


void IntegratorInit(Proportional gain, double timestep, Integrator *result) {
    result->gain = gain * timestep / 2.0;
    result->prev_input = 0.0;
    result->prev_output = 0.0;
}

void DifferentiatorInit(Proportional gain,
                        double timestep,
                        Differentiator *result) {
    result->gain = gain * 2.0 / timestep;
    result->prev_input = 0.0;
    result->prev_output = 0.0;
}


/* Time-Stepping Functions */


inline double Cascade(double input,
                      Biquad sys[],
                      int size,
                      double lower_lim,
                      double upper_lim) {
    Biquad *ptr;

    double output;
    for (ptr = sys; ptr < sys + size; ptr++) {
        output = EvaluateBiquad(ptr, input);
        input = output;
    }

    // Let the system act naturally, and simply saturate its result
    // (So don't reset the last value of the last biquad to a saturated value)
    return SATURATE(output, lower_lim, upper_lim);
}

inline double Integrate(double input,
                        Integrator *term,
                        double lower_lim,
                        double upper_lim) {
    // s^-1 = prop * (1+z^-1)/(1-z^-1) = out/in
    // out = out_prev + prop * (in + in_prev)
    double result = term->prev_output + term->gain * (input + term->prev_input);
    term->prev_input = input;
    term->prev_output = result;
    return result;
}

inline double Differentiate(double input,
                            Differentiator *term,
                            double lower_lim,
                            double upper_lim) {
    // s = prop * (1-z^-1)/(1+z^-1) = out/in
    // out = -out_prev + prop * (in - in_prev)
    double result = -term->prev_output +
        term->gain * (input - term->prev_input);
    term->prev_input = input;
    term->prev_output = result;
    return SATURATE(result, lower_lim, upper_lim);
}

inline double PID(double input,
                  Proportional *p,
                  Integrator *i,
                  Differentiator *d,
                  double lower_lim,
                  double upper_lim) {
    result = 0.0;
    if (p != NULL) result += *p * input;
    if (i != NULL) result += Integrate(input, i, NEG_INF, POS_INF);
    if (d != NULL) result += Differentiate(input, d, NEG_INF, POS_INF);

    return SATURATE(result, lower_lim, upper_lim);
}

static inline double EvaluateBiquad(Biquad *sys, double input) {
    // Here's the most efficient way to handle this:
    double *ptr = (double *) sys;

    // If you're curious, I encourage you to figure out what's going on
    // here before looking at the Lab Report
    double output = *ptr * input +
                    ptr[1] * ptr[6] +
                    ptr[2] * ptr[7] -
                    ptr[4] * ptr[8] -
                    ptr[5] * ptr[9];

    output /= ptr[3];

    ptr[7] = ptr[6];
    ptr[6] = input;

    ptr[9] = ptr[8];
    ptr[8] = output;

    return output;
}
