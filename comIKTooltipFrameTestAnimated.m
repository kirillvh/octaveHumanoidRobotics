clear all;
close all;

R = Robot();

%Create a Manipulator
M0 = Manipulator();
M1 = Manipulator();

%0.01 seconds timestep
time_step = 0.01;
end_time = 10;

M0 = addJoint( M0,  0   , 0, 0.4 , 0, 'mass', 5, 'COM', [0;0;0]);
M0 = addJoint( M0,  0   , 0, 0.4 , 0, 'mass', 1, 'COM', [0.2;0;0]);
M0 = addJoint( M0,  0   , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);

M1 = addJoint( M1,  0   , 0, 0.4 , 0, 'mass', 5, 'COM', [0;0;0]);
M1 = addJoint( M1,  0   , 0, 0.4 , 0, 'mass', 1, 'COM', [0.2;0;0]);
M1 = addJoint( M1,  0   , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);

R = addManipulator(R, M0);
R = addManipulator(R, M1);

position_ref = zeros(3,1);
DOF = size(M0.joints,1);

%M.joints(1).angle = pi/2;
%M.joints(2).angle = -pi;
%M.joints(3).angle = pi/2;

positionRef = [];
positionResponse_log = [];
positionErr_log = [];
path = [];
pathM0 = [];
pathM1 = [];

R.manipulators(1).joints(1).angle = 5.3;
R.manipulators(1).joints(2).angle = 5.13;
R.manipulators(1).joints(3).angle = 2.13;

R.manipulators(2).joints(1).angle = 5.3;
R.manipulators(2).joints(2).angle = 5.13;
R.manipulators(2).joints(3).angle = 2.13;

M1jointAnglesRef = [5.3;5.13;2.13];

for y = -0.3:0.01:0.1
    
    position_ref = [y; 0.5; 0];
    %position_ref = [0; 0.2; 0];
    %there will also be a comIK() later on, so thats why I choose this name
    % Testing pseudo inverse Jacobian and transpose Jacobian
    
    jointAnglesRef = IK(R.manipulators(1).joints, position_ref,...
                            'maxiter', 100,...
                            'kX', 1,...
                            'maxposerr', 0.0001,...
                            'maxroterr', 0.0001,...
                            'mode', 'dls',...
                            'kTheta', 1,...
                            ...%'orientation', [sqrt(0.5) 0 0 -sqrt(0.5)],...
                            'useJointsAngles', true,...
                            'ikMode','com', ...
                            'frameMode', 'tooltip',...
                            'Robot', R);
                        
    pathM0 = [pathM0 ; jointAnglesRef'];
    
    pathM1 = [pathM1 ; M1jointAnglesRef'];
    
    for i=1:DOF
        R.manipulators(1).joints(i).angle = jointAnglesRef(i);
    end
end
path = [pathM0, pathM1];

AnimateRobot2D(R, path, 'plotCOM', true);