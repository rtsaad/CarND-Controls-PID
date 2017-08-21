#include "PID.h"
#include <limits>
#include <chrono>
#include <iostream>
#include <numeric>
#include <list>
#include <ctime>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  //init errors
  p_error = 0;
  i_error = 0;  
  d_error = 0;
  error = 0;

  //setpoint
  setpoint = 0;

  //number or readings
  n = 0;

  //set PID gains
  Kp = kp;
  Ki = ki;
  Kd = kd;

  
  
}

void PID::Change(double kp, double ki, double kd) {
  //set PID gains
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void PID::UpdateError(double cte) {  
  if(setpoint!=0){
    cte -= setpoint;
  }

  if(p_error == 0){
    time_stamp = clock();
  }

  time_t current_time = clock();

  double delta_t = (double(current_time - time_stamp) / CLOCKS_PER_SEC);
  time_stamp = current_time;  

  if(delta_t > 0){
    d_error  = (cte - p_error)/delta_t;
  } else {
    d_error = 0;
  }
  p_error  = cte;
  i_error += cte*delta_t;
  error += cte*cte;
  n++;
}

double PID::ComputeControlResponse(){
  return -((Kp*p_error) + (Ki*i_error) + (Kd*d_error));
}

void PID::SetPoint(double s){
  setpoint = s;
}

double PID::TotalError() {
  if(n==0){
    return std::numeric_limits<long int>::max();
  }
  return (error/n);
}

