//=========================================================================
// Interface between Matlab and a function in friMain.h
//
// Input arguments:
//   <none>
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
  fri_printVersion();
}
