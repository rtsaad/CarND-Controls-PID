#ifndef PID_H
#define PID_H

#include <chrono>
#include <list>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  std::list<double> i_error_list;
  double d_error;
  //Previous error
  double prev_error;
  //Total error
  double error;

  //setpoint
  double setpoint;
  long int n;

  //last timestamp  
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  

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
