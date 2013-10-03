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
    position_ref_L2R = [y*0.3; -y*0.4; 0];
    %position_ref = [0; 0.2; 0];
    %there will also be a comIK() later on, so thats why I choose this name
    % Testing pseudo inverse Jacobian and transpose Jacobian
    
    [jointAnglesRef_Left, jointAnglesRef_Right] = bipedCOMIK(R, 1, 2,  position_ref, position_ref_L2R, [1 0 0 0],...
                            'maxiter', 100,...
                            'kX', 1,...
                            'maxposerr', 0.001,...%Kirill: I increased this to 1mm as tracking both legs under 0.1 mm takes too long
                            'maxroterr', 0.001,...
                            'mode', 'dls',...
                            'kTheta', 1 );
                        
    pathM0 = [pathM0 ; jointAnglesRef_Left'];   
    pathM1 = [pathM1 ; jointAnglesRef_Right'];
       
    for i=1:DOF
        R.manipulators(1).joints(i).angle = jointAnglesRef_Left(i);
        R.manipulators(2).joints(i).angle = jointAnglesRef_Right(i);
    end
    
    left_fk = forwardKine(R.manipulators(1).joints);
    COM = calcRobotCOM(R);
    right_fk = forwardKine(R.manipulators(2).joints);
    
    ERRL2COM = (COM(1:3)-TRANS(left_fk))-position_ref 
    ERRL2R = TRANS(right_fk)-TRANS(left_fk)-position_ref_L2R
end
path = [pathM0, pathM1];

AnimateRobot2D(R, path, 'plotCOM', true);