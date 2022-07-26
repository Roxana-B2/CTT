/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  k_p = Kpi;
  k_d = Kdi;
  k_i = Kii;
  output_min = output_lim_mini;
  output_max = output_lim_maxi;
  p_err = 0;
  d_err = 0;
  i_err = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  d_err = dt>0 ? (cte - p_err) / dt : 0;
  p_err = cte;
  i_err += cte * dt;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    //double control = -k_p * p_err - k_d * d_err - k_i * i_err;
    double control = k_p * p_err + k_d * d_err + k_i * i_err;
  
    control = control < output_min ? output_min : control;
    control = control > output_max ? output_max : control;
      
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  dt = new_delta_time;
  return dt;
}