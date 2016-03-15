//=========================================================================
// Interface between Matlab and a function in friMain.h
//
// Input arguments:
//   listenningPort
// Output arguments:
//   controllerBusy (boolean)
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
  // Output
  bool controllerBusy;

  // Check the number of arguments
  if (nrhs != 1)
    mexErrMsgTxt ("Wrong number of input arguments !");

  // Get input argument
  listenningPort = (int) *((double *) mxGetPr(prhs[0]));

  // Call the actual working code
  controllerBusy = fri_isControllerBusy(listenningPort);

  // Set output
  plhs[0] = mxCreateLogicalScalar(controllerBusy);
}
