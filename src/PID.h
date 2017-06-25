#ifndef PID_H
#define PID_H

#include <vector>
#include <fstream>
using namespace std;

class PID {
  
private:
  
  double dKp;
  double dKi;
  double dKd;
  //double lastError;
  double cteSum;
  double lastCte;
  unsigned short paramIndex;
  unsigned short twiddleIndex;
  unsigned int numUpdates;
  unsigned int numUpdatesBest;
  unsigned int numUpdatesPrev;
//  unsigned int numUpdatesUp;
//  unsigned int numUpdatesDown;
  vector<double> kList;
  vector<double> dkList;
  unsigned int counter;
  bool isFirstTrial;
  double cteSumSquared;
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
   * Update the P varibale in PID.
   */
  //void UpdateP();
  
  /*
   * Update the I varibale in PID.
   */
  //void UpdateI();
  
  /*
   * Update the D varibale in PID.
   */
  //void UpdateD();
  
  /*
   * Return an update factor based on the twiddle algo.
   */
  //double GetFactor();
  
  void IncTwiddleIndex();
  
  void IncParamIndex();
  
  void Print();

  void LogToFile(ofstream &myfile);
  
  void NextCycle();
  
  void ResetState();

};

#endif /* PID_H */
