#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients
   */
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

}

void PID::Init_p(vector<double> input_dp){
    
    // Only initialize errors once and keep them for next round of calculations
    p = {Kp,Ki,Kd};
    p_error = 0;
    i_error = 0;
    d_error = 0;
    dp = input_dp;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  
  d_error = cte - p_error; // difference between actual and previous error. Denominator (delta t) equals 1 --> not needed
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
  /**
   * Calculate and return the steering value
   */
  return -p_error * Kp -i_error * Ki - d_error * Kd;
}

void PID::twiddle(double current_err){
    // Calculate sum of dp:
    //https://stackoverflow.com/questions/3221812/how-to-sum-up-elements-of-a-c-vector
    double sum = 0.0;
    for (auto& n : dp)
        sum += n;
    
    if (sum > tol) {
        switch (state) {
            // Only enter once to init the best error with first error
            case INIT:
                best_err = current_err;
                p[index] += dp[index];
                state = INCREMENT;
                break;
                
            case INCREMENT:
                if (current_err < best_err){
                    best_err = current_err;
                    // Increase this parameter and save it for next iteration
                    dp[index] *= 1.1;
                    
                    //move to next parameter
                    move_index();
                    //Update value for next parameter
                    p[index] += dp[index];
                    // Stay at INCREMENT
                }
                else{
                  // If the current error is big, move 1 dp[index] to the other direction and run again
                  // *2 because we already add 1 from previous loop
                    p[index] -= 2* dp[index];
                    // Jump state to DECREMENT next loop
                    state = DECREMENT;
                }
                break;
                
            case DECREMENT:
                // Similar logic but if current error is still too big, using smaller decrement steps
                if (current_err < best_err){
                    best_err = current_err;
                    // Increase this parameter and save it for next iteration
                    dp[index] *= 1.1;
                }
                else{
                    // Reset to originial P value
                    p[index] += dp[index];
                    // Decrease dp by a small step
                    dp[index] *= 0.9;
                }
                
                //move to next parameter
                move_index();
                
                //Update value for next parameter
                p[index] += dp[index];
                
                // Move back to INCREMENT to start over
                state = INCREMENT;
                
                break;
            
        }
        Init(p[0], p[1], p[2]);
    }
    

}

void PID::print_output(double current_err){
    std::cout
    << "\n***********************************************************\nIteration " << iter
    << " Best Error: "<< best_err <<" Current error: " << current_err
    << " Current state: " << state << " Index " << index
    << "\nNext P:    " << Kp <<","<< Ki<< ","<< Kd
    << "\ndp:        " << dp[0] <<","<< dp[1]<< ","<< dp[2]
    << "\n*************************************************************\n"<<std::endl;
}
                  
void PID::move_index(){
    do {
        index = (index + 1) % p.size();
    }while(dp[index] == 0);
    
    // Count for iteration whenever index hit 0
    if (index == 0) ++iter;
}