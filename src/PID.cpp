#include "PID.h"
#include <iostream>
#include <cfloat>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  //lastError = 0;
  lastCte = -1;
  cteSum = 0;
  paramIndex = 0;
  twiddleIndex = 0;
  numUpdates = 0;
  numUpdatesBest = 0;
  numUpdatesPrev = 0;
  counter = 0;
  
  cteSumSquared = 0;
  utility = 0;
  utilityBest = 0;
  utilityPrev = 0;
  
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
  
  /*
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
  
  if(numUpdates<=1){
    return;
  }
  
  if(numUpdates > 900){
    utility = -cteSumSquared; // numUpdates - fabs(cteSum);
  }else{
    utility = -DBL_MAX;
  }
  
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
      //p += dp
      kList[paramIndex] += dkList[paramIndex];
      IncTwiddleIndex();
      break;
    case 1:
      //if(better outcome) save outcome, dp *= 1.1, move to next param
      //else p -= 2*dp
//      if(numUpdates > numUpdatesBest){
      if(utility > utilityBest){
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
      //if(better outcome) save outcome, dp *= 1.1, move to next param
      //else p += dp (p <- original_value), dp *= 0.9, move to next param
//      if(numUpdates > numUpdatesBest){
      if(utility > utilityBest){
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
      cout<<"WARNING: Unhandled case statemet in GetFactor. twiddleIndex: " << twiddleIndex << endl;
      cout<<"=============================================================="<<endl;
      break;
  }
  
  ResetState();

  cout<<"After Update: "<<endl;
  Print();
  cout<<"=============================================================="<<endl;
  cout<<"=============================================================="<<endl<<endl;
  
  if(twiddleIndex == 0){
    //Don't repeat a trial that you've already seen
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

void PID::LogToFile(ofstream &myfile){
  double P = kList[0];
  double I = kList[1];
  double D = kList[2];
  myfile<<"P: " << P << "  I: " << I << "  D: " << D << endl;
}

/*

void PID::UpdateP(){
  Kp *= GetFactor();
}

void PID::UpdateI(){
  Ki *= GetFactor();
}

void PID::UpdateD(){
  Kd += GetFactor()
}

double PID::GetUpdate(unsigned short paramIndex){
  double param = GetParam(paramIndex);
  
  switch (twiddleIndex) {
    case 0:
      //p += dp
      param += 
      break;
    case 1:
      return 0.9
      break;
    case 2:
      return 0.9
      break;
    case 3:
      return 0.9
      break;
    default:
      cout<<endl<<"=============================================================="<<endl;
      cout<<"WARNING: Unhandled case statemet in GetFactor. twiddleIndex: " << twiddleIndex << endl;
      cout<<endl<<"=============================================================="<<endl;
      break;
  }
  if(numUpdates > numUpdatesBest){
    numUpdatesBest = numUpdates;
  }
}

double PID::GetParam(unsigned short paramIndex){
  double param = GetParam(paramIndex);
  
  switch (paramIndex) {
    case 0:
      return Kp;
      break;
    case 1:
      return Ki;
      break;
    case 2:
      return Kd;
      break;
    default:
      cout<<endl<<"=============================================================="<<endl;
      cout<<"WARNING: Unhandled case statemet in GetParam. paramIndex: " << paramIndex << endl;
      cout<<endl<<"=============================================================="<<endl;
      return Kp;
      break;
  }
}
 
 */

  


