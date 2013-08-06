%test script for forwardKine function
clear all

%Create a Manipulator
M = Manipulator();

%Add Joints to the Manipulator
disp('---------------------------------')
disp('----------Manipulator------------')
disp('---------------------------------')
disp('n |  alpha | theta |  a  | offset')
disp('1 |    0   |   0   | 0.1 |   0   ')
disp('2 |    0   |   0   | 0.1 |   0   ')

disp(' ')
disp(' ')

M = addJoint( M,  0 , 0, 0.1 , 0);
M = addJoint( M,  0 , 0, 0.1 , 0);

%Testing FK

disp('---------- ')
disp('--Test 0-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |    0   ')
disp('2 |    0   ')

M.joints(1).DH.theta = 0;
M.joints(2).DH.theta = 0;

Result = forwardKine(M.joints)

disp('---------- ')
disp('--Test 1-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |   pi/4 ')
disp('2 |    0   ')


M.joints(1).DH.theta = pi/4;
M.joints(2).DH.theta = 0;

Result =forwardKine(M.joints)

disp('---------- ')
disp('--Test 2-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |    0   ')
disp('2 |   pi/4 ')

M.joints(1).DH.theta = 0;
M.joints(2).DH.theta = pi/4;

Result = forwardKine(M.joints)

disp('---------- ')
disp('--Test 3-- ')
disp('---------- ')
disp('n |  theta ')
disp('1 |   pi/4 ')
disp('2 |   pi/4 ')

M.joints(1).DH.theta = pi/4;
M.joints(2).DH.theta = pi/4;

Result = forwardKine(M.joints)