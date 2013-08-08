clear all;

%Create a Manipulator
M = Manipulator();

%0.01 seconds timestep
time_step = 0.01;
end_time = 10;

M = addJoint( M,  0 , 0, 0.1 , 0);
M = addJoint( M,  0 , 0, 0.1 , 0);

angle_old = zeros(2,1);
joint_velocity = zeros(2,1);
joint_rotrate = [0.1,0.1];
manip_position_old = zeros(3,1);

%for plotting data later
tool_tip_vel_measured_log = [];
tool_tip_vel_calculated_log = [];

for t=0:time_step:end_time
	%generate a joint angle, the 0.01 is a arbitary number, feel free to change
	M.joints(1).angle = sin(joint_rotrate(1)*t);
	
	%calculate the joint velocity
	joint_velocity(1) = (M.joints(1).angle - angle_old(1))/time_step;
	
	%update the joint angle
	angle_old(1) = M.joints(1).angle;
		
	%now do the same for the second joint
	%generate a joint angle, the 0.01 is a arbitary number, feel free to change
	M.joints(2).angle = sin(joint_rotrate(2)*t);
	
	%calculate the joint velocity
	joint_velocity(2) = (M.joints(2).angle - angle_old(2))/time_step;

	%update the joint angle
	angle_old(2) = M.joints(2).angle;
	
	%next calculate the manipulators position and velocity in cartesian coordinates
	
	%selecting the position component of the 4x4 matrix with (1:3,4)
	fk = forwardKine(M.joints);
	manip_position = fk(1:3,4);
	tool_tip_velocity_measured = (manip_position - manip_position_old)/time_step;
	manip_position_old = manip_position;
	
	%now to do the actual testing of the kinematic jacobian
	%first get the jacobian
	kine_jac = kineJacobian(M.joints);
	%now since the kinematic Jacobian has the following relation
	% cartesian_velocity = kineJacobian * joint_velocity
	% it means that kineJacobian * joint_velocity and tool_tip_velocity_measured should be almost equal except for a small error due to having a large timestep
	
	tool_tip_velocity_calculated = kine_jac * joint_velocity;
	%finally, log the data to check if the jacobians results correspond with the measured velocity later.
	tool_tip_vel_measured_log = [tool_tip_vel_measured_log; tool_tip_velocity_measured'];
	tool_tip_vel_calculated_log = [tool_tip_vel_calculated_log; tool_tip_velocity_calculated'];
end

plot(tool_tip_vel_measured_log(2:end,2));% starting from 2 to skip the inital noise
hold all;
plot(tool_tip_vel_calculated_log(2:end,2));
legend('y axis measured velocity','y axis calculated velocity(via Jacobian)')
title('The two lines should correspond if Jacobian is correct')