#ifndef PID_H
#define PID_H

#include <chrono>
#include <list>
#include <time.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  //Total error
  double error;

  //track time
  time_t time_stamp;

  //setpoint
  double setpoint;
  long int n;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
   */
  void Change(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   *
   */
  double ComputeControlResponse();

  /*
   *
   */
  void SetPoint(double s);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
