friInnsbruckDriver
Librairy of high level function to use a Kuka Lightweight robot through the Fast Research Interface (FRI).
The librairy is written in C, and a Matlab interface is also provided.

--

Contents:
/bin/		Compiled files
/krc/		Program to be used on the robot controller
/matlab/	Matlab interface to the librairy
/src/		Source code of the librairy
/src/kuka/	Librairy of low level functions, provided by Kuka (friComm, friRemote, friUdp)

--

Installation (on the KRC, the robot controller)
- Copy the program /krc/friRun on the controller, e.g. with a USB key
  (Just copying the .dat and .src files may not be enough; you may need instead to create an empty program called 'friRun' on the controller, then copy/paste the provided code in it)

Installation (on the remote computer)
- Copy in /src/kuka the following files, which are provided by Kuka (and cannot be distributed):
  friComm.h
  friRemote.cpp, friRemote.h
  friUdp,cpp, friUdp.h

- Matlab interface:
  - Make sure your Matlab system is configured for compiling MEX files; if it is not, type 'mex -setup' in the Matlab prompt
  - Run /matlab/fri_make.m. This compiles the code and also adds the directory /bin/ (which contain the compiled files) to Matlab's path; you may want to save the path for future executions (Menu 'File' -> 'Set path...').
  - Test the setup: run 'friRun' on the controller, and, once it has performed the basic initializations, run '/matlab/fri_example.m' on the remote computer

--

Implementation notes

This library mainly uses the so-called 'monitor' mode of the FRI interface. The script on the controller, after performing basic initializations, runs an inifinite loop, in which it processes commands sent by the remote computer (through the FRI interface). This allows the remote computer to only control the robot in ways specifically provisioned for in the script running on the controller (as opposed to what would be possible using the 'control' mode of the FRI). This allows asynchronous commands, i.e. the remote computer only needs to send a command when something has to be done (again, as opposed to using the 'control' mode of the FRI).

--

Author:
Damien Teney
University of Liege, Belgium
damien.teney@ulg.ac.be
http://www.montefiore.ulg.ac.be/~dteney/
