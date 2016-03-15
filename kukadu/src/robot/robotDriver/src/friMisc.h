#ifndef FRIMISC_H
#define FRIMISC_H

#ifndef PI
  #define PI 3.1415926535
#endif

#define RAD2DEG(x) ((double) 180.0 * ((double) x / PI))
#define DEG2RAD(x) ((double) x * (PI / 180.0))

void fri_throwError(const char *message);
void fri_throwWarning(const char *message);
void flushPrintf(void);
void waitForKeyPress(void);

#endif
