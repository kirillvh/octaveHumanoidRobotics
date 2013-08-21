%test script for calcQuaternion function
clear all

%Create a Manipulator
M = Manipulator();

%Add Joints to the Manipulator
disp('---------------------------------')
disp('----------Manipulator------------')
disp('---------------------------------')
disp('n |  alpha | theta |  a  | offset')
disp('1 |  pi/2  |   0   |  0  |   0   ')
disp('2 |    0   |   0   | 0.1 |   0   ')
disp('3 |    0   |   0   | 0.1 |   0   ')

disp(' ')
disp(' ')

M = addJoint( M, pi/2 , 0,  0  , 0);
M = addJoint( M,  0   , 0, 0.1 , 0);
M = addJoint( M,  0   , 0, 0.1 , 0);

disp('---------- ')
disp('--POSE 1-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |    0   ')
disp('2 |    0   ')
disp('3 |   pi/2 ')
disp(' ')

M.joints(1).angle  = 0;
M.joints(2).angle  = 0;
M.joints(3).angle  = pi/2;

Result = forwardKine(M.joints);
disp('----------------------------- ')
disp('--POSE 1 - ROTATION MATRIX -- ')
disp('----------------------------- ')
disp(' ')

R1 = ROT(Result)

disp('----------------------------- ')
disp('---- POSE 1 - QUATERNION ---- ')
disp('----------------------------- ')
disp(' ')

Q1 = calcQuaternion(R1)

disp('---------- ')
disp('--POSE 2-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |   pi/2 ')
disp('2 |   pi/2 ')
disp('3 |   pi/2 ')
disp(' ')

M.joints(1).angle  = pi/2;
M.joints(2).angle  = pi/2;
M.joints(3).angle  = pi/2;

Result = forwardKine(M.joints);
disp('----------------------------- ')
disp('--POSE 2 - ROTATION MATRIX -- ')
disp('----------------------------- ')
disp(' ')

R2 = ROT(Result)

disp('----------------------------- ')
disp('---- POSE 2 - QUATERNION ---- ')
disp('----------------------------- ')
disp(' ')

Q2 = calcQuaternion(R2)

disp('----------------------------- ')
disp('------ FUNCTION OUTPUT ------ ')
disp('----------------------------- ')
disp(' ')

MQ = multQuaternions(Q2,Q1)

disp('----------------------------- ')
disp('-- calcQuaternion(R2*R1)  --- ')
disp('----------------------------- ')
disp(' ')

MR = R2*R1;
MRQ = calcQuaternion(MR)

