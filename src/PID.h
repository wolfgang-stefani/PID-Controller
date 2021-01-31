#ifndef PID_H
#define PID_H
#include <vector>
#include <iostream>
using std::vector;

enum TwiddleState {INIT, INCREMENT,DECREMENT};

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, vector<double> input_dp);
  
  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the steering value (total PID error).
   * @output The steering value (total PID error)
   */
  double TotalError();
  void twiddle(double current_err);
  void move_index();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  vector<double> p;
  vector<double> dp;
  double tol; // tolerance
  double best_err;
  TwiddleState state;    
  int index = 0;
  int iter = 0;
};

#endif  // PID_H