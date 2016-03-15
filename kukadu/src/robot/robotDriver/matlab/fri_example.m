listenningPort = 49938;
%listenningPort = 49939;

%fri_make();

fri_printVersion();

joints = fri_getJoints(listenningPort);
fri_moveJoints(listenningPort, joints + deg2rad(5));
fri_movePosition(listenningPort, [0.220 0.640 0.220 deg2rad(-90) 0 0]);
fri_startGravityCompensation(listenningPort);
pause();
fri_stopGravityCompensation(listenningPort);
pause();

%fri_movePosition(49938, [0.220 +0.640 0.220 deg2rad(-90) 0 0]);
%fri_movePosition(49939, [0.220 -0.640 0.220 deg2rad(-90) 0 0]);

currentJoints = fri_getJoints(listenningPort)
currentPosition = fri_getPosition(listenningPort)

fri_runDemoCommandMode(listenningPort);

safeJoints = [0.0785    0.0349         0    1.2392         0   -0.7679    0.0175];
homeJoints = [0 0 0 0 0 0 0];

for i = 1:2
  fri_moveJoints(listenningPort, homeJoints, true)
  fri_moveJoints(listenningPort, safeJoints, true)
end
fri_moveJoints(listenningPort, homeJoints, false)
while fri_isControllerBusy(listenningPort)
  disp('busy');
  pause(0.5);
end

fri_moveJoints(listenningPort, safeJoints, true)
for i = 1:6
  position = fri_getPosition(listenningPort); % Get current position

  position(1) = position(1) - 50 / 1000;
  fri_movePosition(listenningPort, position, true);
  position(2) = position(2) - 50 / 1000;
  fri_movePosition(listenningPort, position, true);
  position(3) = position(3) - 50 / 1000;
  fri_movePosition(listenningPort, position, true);
end
