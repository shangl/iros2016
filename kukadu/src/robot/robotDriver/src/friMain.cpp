//=========================================================================
// High-level functions to communicate with the robot using the FRI
// Author: Damien Teney
//=========================================================================

// Standard librairies
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <ctime>
#include <cstdio>
// Kuka librairies
#include "kuka/friUdp.h"
#include "kuka/friRemote.h"
// Custom librairies
#include "friMain.h"
#include "friMisc.h"

//=========================================================================
// fri_checkSetup()
// Check basic configuration things
//=========================================================================
void fri_checkSetup(void) {
  FRI_PREPARE_CHECK_BYTE_ORDER;
  if (!FRI_CHECK_BYTE_ORDER_OK) 
    fri_throwError("Byte order inappropriate !");
  if (!FRI_CHECK_SIZES_OK)
    fri_throwError("Sizes of datastructures inappropriate !");
}

//=========================================================================
// fri_printVersion()
// Print the version of the Kuka code used
//=========================================================================
void fri_printVersion(void) {
  printf("FRI version: %u.%u.%u.%u\n", FRI_MAJOR_VERSION, FRI_SUB_VERSION, FRI_DATAGRAM_ID_CMD, FRI_DATAGRAM_ID_MSR);
}

//=========================================================================
// fri_printVersion()
// Print the version of the Kuka code used
//=========================================================================
bool fri_isControllerBusy(int listenningPort) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);

  friInst.doDataExchange(); // Echange packets

  return GET_BUSY_FLAG();
}

//=========================================================================
// fri_runDemoCommandMode
// Run a demo using the "command" mode (sinewave motion of the robot)
//=========================================================================
void fri_runDemoCommandMode(int listenningPort) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);
  FRI_QUALITY lastQuality = FRI_QUALITY_UNACCEPTABLE;
  double timeCounter = 0;

  // Send the command
  SET_COMMAND_FLAG(COMMAND_DEMO_COMMAND_MODE);
  friInst.doDataExchange();

  // Wait for the controller to acknowledge the command (busy flag set)
  while (!GET_BUSY_FLAG())
    friInst.doDataExchange();

  // Reset the command
  SET_COMMAND_FLAG(COMMAND_NONE);
  friInst.doDataExchange();

  // Wait to be in 'command' mode
  while (1) {
    float jointValues[LBR_MNJ];
    for (int i = 0; i < LBR_MNJ; i++)
      jointValues[i] = friInst.getMsrCmdJntPosition()[i]; // Get current joint values

    friInst.doPositionControl(jointValues); // Exchange packets
    if (friInst.getQuality() != lastQuality) { // Connection quality just changed
      printf("Connection quality changed (%d -> %d)\n", lastQuality, friInst.getQuality());
      lastQuality = friInst.getQuality();
    }

    // Check conditions for breaking the loop
    if (friInst.getState() == FRI_STATE_CMD && friInst.isPowerOn()) // We are in 'command' mode
      break; // Go on
    if (!GET_BUSY_FLAG()) // Something's wrong on the controller side: it's not busy anymore
      return; // Stop
  }
  
  // We are in 'command' mode
  timeCounter = 0.0;
  while (1) {
	  printf("%f\n", timeCounter);
    float jointValues[LBR_MNJ];
    for (int i = 0; i < LBR_MNJ; i++)
      jointValues[i] = friInst.getMsrCmdJntPosition()[i]; // Get current joint values

    // Prepare new joint values
    for (int i = 0; i < LBR_MNJ; i++)
      jointValues[i] += (float) sin(timeCounter * PI  * 0.020 * 20 * 0.5) * (float) DEG2RAD(10.0); // Sine wave motion
    timeCounter += friInst.getSampleTime(); // Increment time counter

    if (timeCounter > 10.0) // Enough time elapsed
      SET_COMMAND_FLAG(COMMAND_STOP); // Clean stop

    friInst.doPositionControl(jointValues); // Exchange packets

    // Display debug information
    if (friInst.getSequenceCount() % 40 == 0) { // Every 40th packet
      printf("\tTime: %.2f\n", timeCounter); flushPrintf();
    }

    // Check conditions for breaking the loop
    if (!GET_BUSY_FLAG()) // Something's wrong on the controller side: it's not busy anymore
      break;
    if (friInst.getState() != FRI_STATE_CMD) // We are not in command mode anymore
      break;
  }
}

//=========================================================================
// fri_getJoints
// Return the current position of the robot joints
// (A1, A2, E1, A3, A4, A5, A6; in radians)
//=========================================================================
void fri_getJoints(int listenningPort, double *currentJointValues) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);

  friInst.doDataExchange(); // Exchange packets

  // Copy the values
  for (int i = 0; i < LBR_MNJ; i++)
    currentJointValues[i] = friInst.getMsrCmdJntPosition()[i];
}

//=========================================================================
// fri_getPosition
// Return the current (cartesian) position of the tool
// (X, Y, Z, A, B, C; in meters/radians)
//=========================================================================
void fri_getPosition(int listenningPort, double *currentPositionValues) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);

  // Send the command
  SET_COMMAND_FLAG(COMMAND_GET_POSITION);
  friInst.doDataExchange();

  // Wait for the controller to acknowledge the command (busy flag set)
  while (!GET_BUSY_FLAG())
    friInst.doDataExchange();

  // Reset the command
  SET_COMMAND_FLAG(COMMAND_NONE);
  friInst.doDataExchange();

  // Loop while the controller is busy
  while (GET_BUSY_FLAG())
    friInst.doDataExchange();

  for (int i = 0; i < 6; i++)
    currentPositionValues[i] = (double) friInst.getFrmKRLReal(i);

  /* Old version below (retrieve the position passively, as sent continuously by the KRC on the FRI, but it is relative to the robot base, not to the world frame)
  friInst.doDataExchange(); // Exchange packets

  // Copy the values
  for (int i = 0; i < FRI_CART_FRM_DIM; i++)
    currentPositionValues[i] = friInst.getMsrCartPosition()[i];
  */
}

//=========================================================================
// fri_moveJoints
// Move the robot joints to the given values
// (A1, A2, E1, A3, A4, A5, A6; in radians)
//=========================================================================
void fri_moveJoints(int listenningPort, const double *newJointValues, bool wait) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);

  // Send the command
  SET_COMMAND_FLAG(COMMAND_MOVE_JOINTS);
  for (int i = 0; i < 7; i++) // Copy the command data
    friInst.setToKRLReal(i, (float) newJointValues[i]);
  friInst.doDataExchange();

  // Wait for the controller to acknowledge the command (busy flag set)
  while (!GET_BUSY_FLAG())
    friInst.doDataExchange();

  // Reset the command
  SET_COMMAND_FLAG(COMMAND_NONE);
  friInst.doDataExchange();

  if (wait) {
    // Loop while the controller is busy
    while (GET_BUSY_FLAG()) {
      friInst.doDataExchange();
      //printf("."); flushPrintf(); // Debug display
    }
  }
}

//=========================================================================
// fri_movePosition
// Move the tool to the given (cartesian) position
// (X, Y, Z, A, B, C; in meters/radians)
//=========================================================================
void fri_movePosition(int listenningPort, const double *newPositionValues, bool wait) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);

  // Send the command
  SET_COMMAND_FLAG(COMMAND_MOVE_POSITION);
  for (int i = 0; i < 6; i++) // Copy the command data
    friInst.setToKRLReal(i, (float) newPositionValues[i]);
  friInst.doDataExchange();

  // Wait for the controller to acknowledge the command (busy flag set)
  while (!GET_BUSY_FLAG())
    friInst.doDataExchange();

  // Reset the command
  SET_COMMAND_FLAG(COMMAND_NONE);
  friInst.doDataExchange();

  if (wait) {
    // Loop while the controller is busy
    while (GET_BUSY_FLAG()) {
      friInst.doDataExchange();
      //printf("."); flushPrintf(); // Debug display
    }
  }
}

//=========================================================================
// fri_startGravityCompensation
// Enter gravity compensation mode
//=========================================================================
void fri_startGravityCompensation(int listenningPort) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);

  // Send the command
  SET_COMMAND_FLAG(COMMAND_START_GRAVITY_COMPENSATION);
  friInst.doDataExchange();

  // Wait for the controller to acknowledge the command (busy flag set)
  while (!GET_BUSY_FLAG())
    friInst.doDataExchange();

  // Reset the command
  SET_COMMAND_FLAG(COMMAND_NONE);
  friInst.doDataExchange();
}

//=========================================================================
// fri_stopGravityCompensation
// Exit gravity compensation mode
//=========================================================================
void fri_stopGravityCompensation(int listenningPort) {
  // Initializations
  fri_checkSetup();
  friRemote friInst(listenningPort);

  // Send the command
  SET_COMMAND_FLAG(COMMAND_STOP);
  friInst.doDataExchange();

  // Wait for the controller to actually get out of the gravity compensation mode
  while (GET_BUSY_FLAG())
    friInst.doDataExchange();
}
