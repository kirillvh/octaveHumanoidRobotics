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

%Testing FK

disp('---------- ')
disp('--Test 0-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |    0   ')
disp('2 |    0   ')
disp('3 |    0   ')

M.joints(1).angle  = 0;
M.joints(2).angle  = 0;
M.joints(3).angle  = 0;

Result = forwardKine(M.joints)
R = ROT(Result);
Q = calcQuaternion(R)

disp('---------- ')
disp('--Test 1-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |   pi/4 ')
disp('2 |    0   ')
disp('3 |    0   ')


M.joints(1).angle  = pi/4;
M.joints(2).angle  = 0;
M.joints(3).angle  = 0;

Result =forwardKine(M.joints)
R = ROT(Result);
Q = calcQuaternion(R)

disp('---------- ')
disp('--Test 2-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |    0   ')
disp('2 |   pi/2 ')
disp('3 |    0   ') 

M.joints(1).angle  = 0;
M.joints(2).angle  = pi/2;
M.joints(3).angle  = 0;

Result = forwardKine(M.joints)
R = ROT(Result);
Q = calcQuaternion(R)

disp('---------- ')
disp('--Test 3-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |    0   ')
disp('2 |    0   ')
disp('3 |   pi/2 ')

M.joints(1).angle  = 0;
M.joints(2).angle  = 0;
M.joints(3).angle  = pi/2;

Result = forwardKine(M.joints)
R = ROT(Result);
Q = calcQuaternion(R)

disp('---------- ')
disp('--Test 4-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |   pi/2 ')
disp('2 |   pi/2 ')
disp('3 |   pi/2 ')

M.joints(1).angle  = pi/2;
M.joints(2).angle  = pi/2;
M.joints(3).angle  = pi/2;

Result = forwardKine(M.joints)
R = ROT(Result);
Q = calcQuaternion(R)
