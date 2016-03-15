//=========================================================================
// Misc code
// Author: Damien Teney
//=========================================================================

// Standard librairies
#include <stdio.h>
#include <math.h>
#include <cstdlib>
// Matlab MEX librairies
#ifdef MATLAB
  #include <mex.h>
#endif

//=========================================================================
// Display a message and terminate the script
//=========================================================================
void fri_throwError(const char *message) {
  #ifdef MATLAB
    mexErrMsgTxt(message);
  #else
    printf("%s", message); printf("\n"); exit(-1);
  #endif
}

//=========================================================================
// Display a message
//=========================================================================
void fri_throwWarning(const char *message) {
  #ifdef MATLAB
    mexWarnMsgTxt(message);
  #else
    printf("WARNING: "); printf("%s", message); printf("\n");
  #endif
}

//=========================================================================
// Flush output stream of the mexPrintf() function
//=========================================================================
void flushPrintf(void) {
  #ifdef MATLAB
    mexEvalString("drawnow;");
  #else
    fflush(stdout);
  #endif
}

//=========================================================================
// Wait for a keypress from the user
//=========================================================================
void waitForKeyPress(void) {
  #ifdef MATLAB
    mexPrintf("Press any key to continue...\n");
    mexCallMATLAB(0, NULL, 0, NULL, "pause");
  #else
    int ch;
    printf("Press any key to continue...\n"); fflush(stdout);
    // Flush stdin
    do
      ch = fgetc(stdin); 
    while (ch != EOF && ch != '\n'); 
    // Wait for a (new) keypress
    getchar();
  #endif
}
