function fri_make()
%FRI_MAKE Compile MEX files of the FRI library.
%   FRI_MAKE() compiles all MEX files of the FRI librairy.

%   Author: Damien Teney
% Compile the required librairies
mex -v -c -DMATLAB -I../src/ -outdir ../bin/ ../src/kuka/friRemote.cpp
mex -v -c -DMATLAB -I../src/ -outdir ../bin/ ../src/kuka/friUdp.cpp
mex -v -c -DMATLAB -I../src/ -outdir ../bin/ ../src/friMain.cpp
mex -v -c -DMATLAB -I../src/ -outdir ../bin/ ../src/friMisc.cpp

% Compile the main functions
% COMPFLAGS="$COMPFLAGS /Wall"
% Setup the extension of compiled (object) files
if isunix % Linux
  o = 'o';
else % Windows
  o = 'obj';
end
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_printVersion.cpp             ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_isControllerBusy.cpp         ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_runDemoCommandMode.cpp       ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_getJoints.cpp                ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_moveJoints.cpp               ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_getPosition.cpp              ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_movePosition.cpp             ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_startGravityCompensation.cpp ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];
command = ['mex -v -I../src/ -outdir ''../bin/'' fri_stopGravityCompensation.cpp  ../bin/friRemote.' o ' ../bin/friUdp.' o ' ../bin/friMain.' o ' ../bin/friMisc.' o];

% Add the directory containing the source and compiled files to Matlab's path
[path trash ext] = fileparts(mfilename('fullpath'));
binariesPath = [path filesep '..' filesep 'bin'];
addpath(path);
addpath(binariesPath);

% Delete temporary files
delete ../bin/*.' o '
delete ../bin/*.manifest
