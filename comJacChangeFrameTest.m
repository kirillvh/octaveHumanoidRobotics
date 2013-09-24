
clear all;
close all;


R = Robot();

%Create a Manipulator
M0 = Manipulator();
M1 = Manipulator();

%0.01 seconds timestep
time_step = 0.01;
end_time = 10;

M0 = addJoint( M0,  0 , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);
M0 = addJoint( M0,  0 , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);

M1 = addJoint( M1,  0 , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);
M1 = addJoint( M1,  0 , 0, 0.2 , 0, 'mass', 1, 'COM', [0.1;0;0]);

R = addManipulator(R, M0);
%R = addManipulator(R, M1);

angle_old = zeros(2,1);
joint_velocity = zeros(2,1);
joint_rotrate = [0.1,0.1];
COMPos_old = zeros(3,1);

%for plotting data later
COM_vel_measured_log = [];
COM_vel_calculated_log = [];

for t=0:time_step:end_time
	%generate a joint angle, the 0.01 is a arbitary number, feel free to change
	R.manipulators(1).joints(1).angle = sin(joint_rotrate(1)*t);
	
	%calculate the joint velocity
	joint_velocity(1) = (R.manipulators(1).joints(1).angle - angle_old(1))/time_step;
	
	%update the joint angle
	angle_old(1) = R.manipulators(1).joints(1).angle;
		
	%now do the same for the second joint
	%generate a joint angle, the 0.01 is a arbitary number, feel free to change
	%R.manipulators(1).joints(2).angle = sin(joint_rotrate(2)*t);
	
	%calculate the joint velocity
	joint_velocity(2) = (R.manipulators(1).joints(2).angle - angle_old(2))/time_step;

	%update the joint angle
	angle_old(2) = R.manipulators(1).joints(2).angle;
	
	%next calculate the manipulators position and velocity in cartesian coordinates
	
	%selecting the position component of the 4x4 matrix with (1:3,4)
	wCOM = calcRobotCOM(R);
    wCOMPos = wCOM(1:3);
    fk = forwardKine(R.manipulators(1).joints);
	Rot = ROT(fk);
    T = TRANS(fk);
    COM = Rot'*(wCOMPos-T);
    COMPos = COM(1:3);
	COM_velocity_measured = (COMPos - COMPos_old)/time_step;
	COMPos_old = COMPos;
	
	%now to do the actual testing of the com jacobian
	%first get the jacobian
	wcom_jac = comJacobian(R.manipulators(1).joints, 'Robot', R);
    com_jac = comJacChangeFrame(wcom_jac, R.manipulators(1).joints, wCOM);
	%now since the kinematic Jacobian has the following relation
	% com_velocity = comJacobian * joint_velocity
	% it means that comJacobian * joint_velocity and tool_tip_velocity_measured should be almost equal except for a small error due to having a large timestep
	
	COM_velocity_calculated = com_jac * joint_velocity;
	%finally, log the data to check if the jacobians results correspond with the measured velocity later.
	COM_vel_measured_log = [COM_vel_measured_log; COM_velocity_measured'];
	COM_vel_calculated_log = [COM_vel_calculated_log; COM_velocity_calculated'];
end

plot(COM_vel_measured_log(2:end,2));% starting from 2 to skip the inital noise
hold all;
plot(COM_vel_calculated_log(2:end,2));
legend('y axis measured velocity','y axis calculated velocity(via Jacobian)')
title('The two lines should correspond if Jacobian is correct')