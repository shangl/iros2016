#ifndef FRIMAIN_H
#define FRIMAIN_H

#define GET_BUSY_FLAG()             (friInst.getFrmKRLBool(0))
#define SET_COMMAND_FLAG(commandId) (friInst.setToKRLInt(0, commandId))

#define COMMAND_STOP                       -1
#define COMMAND_NONE                       0
#define COMMAND_DEMO_COMMAND_MODE          1
#define COMMAND_MOVE_JOINTS                2
#define COMMAND_MOVE_POSITION              3
#define COMMAND_GET_JOINTS                 4
#define COMMAND_GET_POSITION               5
#define COMMAND_START_GRAVITY_COMPENSATION 6

#define COMMAND_GUIDED_MEASUREMENT         8

void fri_checkSetup(void);
void fri_printVersion(void);

bool fri_isControllerBusy(int listenningPort);
void fri_runDemoCommandMode(int listenningPort);

void fri_getJoints(int listenningPort, double *currentJointValues);
void fri_getPosition(int listenningPort, double *currentPositionValues);

void fri_moveJoints(int listenningPort, const double *newJointValues, bool wait);
void fri_movePosition(int listenningPort, const double *newPositionValues, bool wait);

void fri_startGravityCompensation(int listenningPort);
void fri_stopGravityCompensation(int listenningPort);

#endif
