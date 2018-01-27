#include "PID.h"
#include <iostream>

using namespace std;
const double DP = 1;
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_i, double Ki_i, double Kd_i) {
  p[0] = Kp_i;
  p[1] = Kd_i;
  p[2] = Ki_i;
  prev_time = clock();
  /* If any one of the coefficients is initialized to a non-zero value from main
     Assume that hyper parameter optimization is not required
  */
  if(Kp_i || Ki_i || Kd_i){ // Any one is non zero
    inTwiddle = false;
    //cout<< p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
  }
  // Else, initialize the twiddle algorithm
  else{ 
    inTwiddle = true;
    twiddle_index = 0;
    twiddle_dir = 1;
    // Allow for the twiddle algorithm to start from unequal values for the parameters.
    p[0] = 0.135;p[1]=1.12;p[2]=0.01;
    // Allow for the algorithm to use unequal step sizes.
    dp[0] = DP/10;dp[1] = DP;dp[2] = DP/100.0;
    p[0] += dp[0];
    best_err = 1000;
  }
  pid_resetmeas();
}

void PID::UpdateError(double cte) {
  clock_t cur_time = clock();
  clock_t diff_time = cur_time - prev_time;
  prev_time = cur_time;
  double delta_t  = (1000/3.75)*((double)diff_time)/CLOCKS_PER_SEC; // In units of 3.75 ms
  d_error = (cte - p_error)/delta_t;
  p_error = cte;
  i_error += (cte*delta_t);
  if(!inTwiddle){
    //cout << p_error<<" "<<d_error<<" "<<i_error <<endl;
  }
}

double PID::TotalError() {
  double totError = Kp*p_error + Kd*d_error + Ki*i_error;
  return totError;
}

void PID::pid_resetmeas(){
  cum_err = 0;
  iStep = 0;
  Kp = p[0];
  Kd = p[1];
  Ki = p[2];
  i_error = 0;
  d_error = 0;
  p_error = 0;
}

bool PID::twiddle(){
  const int N_TWIDDLE_STEPS = 2200;
  iStep++;
  static int threshCount=0;
  if((p_error > 3.2)||(p_error<-3.2)){
    //If car runs off th track, start a counter
    threshCount++;
  }
  else{
    threshCount=0;
  }
  //Run the algorithm and accumulate the error
  if(iStep > 200){
    double temp = p_error;
    cum_err += (temp*temp);
  }
  int num_steps = iStep;
  if(threshCount > 5){
    //If car is off the track for 5 time steps, stop this iteration
    cum_err = 10000*(N_TWIDDLE_STEPS-200)/iStep; //Override with a large error proportional to early failure
    iStep = N_TWIDDLE_STEPS; //Stop this iteration
    threshCount = 0; // Reset count
  }
  if(iStep == N_TWIDDLE_STEPS){
    // Apply the twiddle algorithm within this code block.
    double this_error = cum_err/(N_TWIDDLE_STEPS-200);
    cout << p[0]<<" "<<p[1]<<" "<<p[2]<<" "<<p_error<<" "<<d_error<<" "<<i_error<<" "<<this_error<<" "<<best_err<<" "<<num_steps<<endl;
    if(this_error < best_err){
      best_err = this_error;
      dp[twiddle_index] *= 1.25;
      twiddle_index = (twiddle_index+1)%3;
      twiddle_dir = 1;
      p[twiddle_index] += dp[twiddle_index];
    }
    else{
      if(twiddle_dir==0){
	p[twiddle_index] += dp[twiddle_index]; // Add back
	dp[twiddle_index] *= 0.75; // Scale down
	twiddle_index++; // Increment the parameter index
	twiddle_dir = 1; // Start with forward direction for next index.
	if(twiddle_index == 3){ // All three parameters are done for this iteration
	  double sum_param = 10*dp[0] + dp[1] + 100*dp[2];
	  if(sum_param <= DP/5.0){ // Params are less than tolerance
	    inTwiddle = false;
	  }
	  else{
	    twiddle_index = 0; //Continue with iterations
	    p[twiddle_index] += dp[twiddle_index];
	  }
	}
	else{
	  p[twiddle_index] += dp[twiddle_index];
	}
	
      } //if(forward_dir==0)
      else{ // Try the negative direction
	p[twiddle_index] -= (2*dp[twiddle_index]); // decrease 
	twiddle_dir = 0; // Resets the direction to backward (0)
      } // else of if(forward_dir==0)
    } // else of if(this_error < best_err)
    return true; // So that the simulator and measurements are reset.
  } // if(iStep == N_TWIDDLE_STEPS)
  return false; // Continue with measurements for this iteration of twiddle
}
