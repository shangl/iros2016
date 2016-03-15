//=========================================================================
// Interface between Matlab and a function in friMain.h
//
// Input arguments:
//   listenningPort
// Output arguments:
//   currentPositionValues(6)  (X, Y, Z, A, B, C; in meters/radians; A/B/C are euler angles of rotations around axes Z/Y/X in the order XYZ)
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
  double *currentPositionValues;

  // Check the number of arguments
  if (nrhs != 1)
    mexErrMsgTxt ("Wrong number of input arguments !");

  // Get input argument
  listenningPort = (int) *((double *) mxGetPr(prhs[0]));

  // Set output
  plhs[0] = mxCreateDoubleMatrix(1, 6, mxREAL);
  currentPositionValues = mxGetPr(plhs[0]);

  // Call the actual working code
  fri_getPosition(listenningPort, currentPositionValues);
}
