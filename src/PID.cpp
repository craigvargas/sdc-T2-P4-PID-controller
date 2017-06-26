#include "PID.h"
#include <iostream>
#include <cfloat>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  // Cross track error
  lastCte = DBL_MAX;
  cteSum = 0;
  cteSumSquared = 0;
  // Variables to keep track of twiddle algorithm
  paramIndex = 0;
  twiddleIndex = 0;
  /*
   Variable to keep track of the number of control updates the algorithm is asked for
   used as a proxy for the distance traveled by the car during a trial
   */
  numUpdates = 0;
  numUpdatesBest = 0;
  numUpdatesPrev = 0;
  // Variable to keeep track of the number of times we alter a variable in the twiddle algo
  counter = 0;
  /*
   Variable to hold the value of a utility function that will ultimately be used
   to grade how well a trial did. Used as the function to maximize
   */
  utility = 0;
  utilityBest = 0;
  utilityPrev = 0;
  // Tells weather this this is the car's initial run
  isFirstTrial = true;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  kList.push_back(Kp);
  kList.push_back(Ki);
  kList.push_back(Kd);
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
  return cteSum;
}

void PID::SetDeltas(double dKp_, double dKi_, double dKd_){
  dKp = dKp_;
  dKi = dKi_;
  dKd = dKd_;
  dkList.push_back(dKp);
  dkList.push_back(dKi);
  dkList.push_back(dKd);
}

double PID::GetSteering(double lastSteering, double cte){
  cteSum += cte;
  double cteDiff;
  if(lastCte == DBL_MAX){
    cteDiff = 0;
  }else{
    cteDiff = cte - lastCte;
  }
  double P = kList[0];
  double I = kList[1];
  double D = kList[2];
  
  /* Debugging
  cout<<endl;
  cout<<"=========================================\n";
  cout<<"Before steering calc:\n\n";
  cout<<"cte: " << cte << "  cteDiff: " << cteDiff << "  cteSum: " << cteSum << endl;
  cout<<"=========================================\n";
   */
  
  double steering = -P*cte - D*cteDiff - I*cteSum;
  lastCte = cte;
  numUpdates++;
  cteSumSquared += (cte*cte);
  return steering;
}

void PID::IncTwiddleIndex(){
  twiddleIndex++;
  if (twiddleIndex>3){
    twiddleIndex = 0;
  }
}

void PID::IncParamIndex(){
  paramIndex++;
  if (paramIndex>2){
    paramIndex = 0;
  }
  twiddleIndex = 0;
}

void PID::NextCycle(){
  
  /* 
   help adjust for a nuance in how messages come in from the simulator.
   After a reset the simulator sends a message that makes the algorithm
   thinks its time to evaluate a trial but in reality the car hasn't
   moved yet.  This condition helps prevent twiddle from updating 
   variables when no new information is available to update and evaluate.
   */
  if(numUpdates<=1){
    return;
  }
  
  /*
   The condtional structure below is used to onnly give credibility to
   trials where the car drove a minimum distance.  This was needed when
   the controller was evaluated based on minimum cte_squared.  Whacky 
   controls that drove straight off of the road were considered good
   by the algorithm because the cte_squared was low, but it was only 
   low because the conrols were so bad the car never had a chance to 
   drive anywhere.  The function to evaluate the goodness of the 
   controller was later altreed so that this
   */
//  if(numUpdates > 900){
//    utility = numUpdates - cteSumSquared; // numUpdates - fabs(cteSum);
//  }else{
//    utility = -DBL_MAX;
//  }
  
  /*
   If we evaluating the first trial then force its value to be the best
   value so far.
   */
  if(isFirstTrial){
    numUpdatesBest = numUpdates;
    utilityBest = utility;
    isFirstTrial = false;
  }
  
  cout<<endl;
  cout<<"=============================================================="<<endl;
  cout<<"PID Update:" << endl;
  cout<<"Cycle idices -> twiddleIndex: " << twiddleIndex << "  paramIndex: " << paramIndex << endl;
  cout<<"=============================================================="<<endl;
  cout<<"Before Update: "<<endl;
  Print();
  cout<<endl;
  
  switch (twiddleIndex) {
      
    case 0:
      //Try adjusting the varible by the current delta value: p += dp
      kList[paramIndex] += dkList[paramIndex];
      IncTwiddleIndex();
      break;
    case 1:
      /*
       if(better outcome) save outcome, dp *= 1.1, move to next param
       else try adjusting the varibe in the other direciton: p -= 2*dp
       */
      if(utility > utilityBest){
        cout<<"*** Utility increased. New variable value saved ***"<<endl;
        numUpdatesBest = numUpdates;
        utilityBest = utility;
        dkList[paramIndex] *= 1.1;
        IncParamIndex();
      }else{
        kList[paramIndex] -= 2*dkList[paramIndex];
        IncTwiddleIndex();
      }
      break;
    case 2:
      /*
       if(better outcome) save outcome, dp *= 1.1, move to next param
       else p += dp (p <- original_value), dp *= 0.9, move to next param
       */
      if(utility > utilityBest){
        cout<<"*** Utility increased. New variable value saved ***"<<endl;
        numUpdatesBest = numUpdates;
        utilityBest = utility;
        dkList[paramIndex] *= 1.1;
        IncParamIndex();
      }else{
        kList[paramIndex] += dkList[paramIndex];
        dkList[paramIndex] *= 0.9;
        IncParamIndex();
      }
      break;
    default:
      cout<<endl;
      cout<<"=============================================================="<<endl;
      cout<<"WARNING: Unhandled value in switch statemet in GetFactor. twiddleIndex: ";
      cout<< twiddleIndex << endl;
      cout<<"=============================================================="<<endl;
      break;
  }
  
  ResetState();

  cout<<"After Update: "<<endl;
  Print();
  cout<<"=============================================================="<<endl;
  cout<<"=============================================================="<<endl<<endl;
  
  if(twiddleIndex == 0){
    /*
     When twiddleIndex == 0 we adjust the variable back to its old value.  
     This means the state of the control variables are back to the values
     that produced the last best result.  We've already seen this trial so
     we should force the algo to evaluate the next variable
     */
    cout<<"\n\n********** SKIP CYCLE **********\n\n";
    //p += dp
    kList[paramIndex] += dkList[paramIndex];
    IncTwiddleIndex();
  }
}

void PID::ResetState(){
  numUpdatesPrev = numUpdates;
  utilityPrev = utility;
  numUpdates = 0;
  utility = 0;
  cteSum = 0;
  lastCte = DBL_MAX;
  cteSumSquared = 0;
  counter++;
}

void PID::Print(){
  double P = kList[0];
  double I = kList[1];
  double D = kList[2];
  double dP = dkList[0];
  double dI = dkList[1];
  double dD = dkList[2];
  cout<<"P: " << P << "  I: " << I << "  D: " << D << endl;
  cout<<"dP: " << dP << "  dI: " << dI << "  dD: " << dD << endl;
  cout<< "Last numUpdates: " << numUpdatesPrev << "  Best numUpdates: "<< numUpdatesBest << endl;
  cout<< "Last utility: " << utilityPrev << "  Best utility: "<< utilityBest << endl;
  cout<<"Count: " << counter << endl;
}

//void PID::LogToFile(ofstream &myfile){
//  double P = kList[0];
//  double I = kList[1];
//  double D = kList[2];
//  myfile<<"P: " << P << "  I: " << I << "  D: " << D << endl;
//}

