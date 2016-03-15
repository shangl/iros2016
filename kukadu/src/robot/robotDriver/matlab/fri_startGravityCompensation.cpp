//=========================================================================
// Interface between Matlab and a function in friMain.h
//
// Input arguments:
//   listenningPort
// Output arguments:
//   <none>
//
// Author: Damien Teney
//=========================================================================

// MEX librairies
#include <mex.h>
// Main librairies
#include "friMain.h"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
  // Input
  int listenningPort;

  // Check the number of arguments
  if (nrhs != 1)
    mexErrMsgTxt ("Wrong number of input arguments !");

  // Get input argument
  listenningPort = (int) *((double *) mxGetPr(prhs[0]));

  // Call the actual working code
  fri_startGravityCompensation(listenningPort);
}
