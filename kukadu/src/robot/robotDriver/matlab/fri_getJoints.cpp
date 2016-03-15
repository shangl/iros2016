//=========================================================================
// Interface between Matlab and a function in friMain.h
//
// Input arguments:
//   listenningPort
// Output arguments:
//   currentJointValues(7) (A1, A2, E1, A3, A4, A5, A6; in radians)
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
  // Output
  double *currentJointValues;

  // Check the number of arguments
  if (nrhs != 1)
    mexErrMsgTxt ("Wrong number of input arguments !");

  // Get input argument
  listenningPort = (int) *((double *) mxGetPr(prhs[0]));

  // Set output
  plhs[0] = mxCreateDoubleMatrix(1, 7, mxREAL);
  currentJointValues = mxGetPr(plhs[0]);

  // Call the actual working code
  fri_getJoints(listenningPort, currentJointValues);
}
