% Conectar a ROS
connectROS();

% Limpiar Matlab y Turtle
cleanTurtle();

% Inicializar turtle
[posePub, poseSub] = initTurtleROS();
pause(10);

% Mover turtle
writePoseROS(posePub,1,0,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,1,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,0,pi/2);
pause(1);
writePoseROS(posePub,1,0,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,1,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,1,1,pi);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,1,0,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,2,0);
pause(1);
readPoseROS(poseSub);
teleportPoseROS(0,0,0);
pause(1);
readPoseROS(poseSub);

% Desconectar ROS
disconnectROS();