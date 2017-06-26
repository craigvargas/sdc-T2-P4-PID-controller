#ifndef PID_H
#define PID_H

#include <vector>
#include <fstream>
using namespace std;

class PID {
  
private:
  // Delta values for twiddle algorithm
  double dKp;
  double dKi;
  double dKd;
  // Cross track error
  double cteSum;
  double lastCte;
  double cteSumSquared;
  /*
   Variables to keep track of twiddle algorithm.  The twiddle algo is 
   implmented in a unique way since the simulator runs independent of our code.
   Because of this we cannot implement the loop in our code so instead we use
   indices to keep track of the state of the twiddle algorithm so that we
   can perform the correct conditional checks everytime a new message
   to evaluate a trial comes in.
   */
  unsigned short paramIndex;
  unsigned short twiddleIndex;
  /*
   Variable to keep track of the number of control updates the algorithm is asked for
   used as a proxy for the distance traveled by the car during a trial
   */
  unsigned int numUpdates;
  unsigned int numUpdatesBest;
  unsigned int numUpdatesPrev;
  // store control variables and delta variables in vectors for east of access via an index
  vector<double> kList;
  vector<double> dkList;
  // Variable to keeep track of the number of times we alter a variable in the twiddle algo
  unsigned int counter;
  // boolean to keep track of the first trial or car run
  bool isFirstTrial;
  /*
   Variable to hold the value of a utility function that will ultimately be used
   to grade how well a trial did. Used as the function to maximize
   */
  double utility;
  double utilityBest;
  double utilityPrev;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

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
  void Init(double Kp_, double Ki_, double Kd_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
   * Set private parameters dKp, dKi, dKd.
   */
  void SetDeltas(double dKp_, double dKi_, double dKd_);
  
  /*
   * Calculate the next steeting instruction.
   */
  double GetSteering(double lastSteering, double cte);
  
  /*
   Increments the twiddleIndex
   */
  void IncTwiddleIndex();
  
  /*
   Increments the paramIndex
   */
  void IncParamIndex();
  
  /*
   convenience print function to display the current state of
   control and twiddle variables
   */
  void Print();

//  void LogToFile(ofstream &myfile);
  /*
   Evaluates the most recent car trial and updates the twiddle variables
   */
  void NextCycle();
  
  /*
   Resets many of the twiddle variables so a new trial can be run
   */
  void ResetState();

};

#endif /* PID_H */
