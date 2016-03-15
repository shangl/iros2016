//=========================================================================
// Interface between Matlab and a function in friMain.h
//
// Input arguments:
//   listenningPort
//   newJointValues(7) (A1, A2, E1, A3, A4, A5, A6; in radians)
//   wait(boolean; default value: true)
// Output arguments:
//   <none>
//
// Author: Damien Teney
//=========================================================================

// MEX librairies
#include <mex.h>
#include <matrix.h>
// Main librairies
#include "friMain.h"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
  // Input
  int listenningPort;
  double *newJointValues;
  bool wait;

  // Check the number of arguments
  if (nrhs != 2 && nrhs != 3)
    mexErrMsgTxt ("Wrong number of input arguments !");

  // Get input arguments and check their size
  if ((int) mxGetN(prhs[1]) != 7)
    mexErrMsgTxt ("Wrong size of argument !");
  listenningPort = (int) *((double *) mxGetPr(prhs[0]));
  newJointValues = (double *) mxGetPr(prhs[1]);
  if (nrhs >= 3) // 3 arguments given
    wait = *((bool *) mxGetPr(prhs[2]));
  else // Only 2 arguments given
    wait = true; // Assign default value

  // Call the actual working code
  fri_moveJoints(listenningPort, newJointValues, wait);
}
