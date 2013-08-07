function Jac = kineJacobian(joints) 
%see http://www.elysium-labs.com/robotics-corner/learn-robotics/introduction-to-robotics/kinematic-jacobian/
DOF = size(joints,1); %the number of joints
Jac = zeros(6,DOF); %Jacobian is 6xN (6 = 3 linear + 3 rotational axes)

	disp('TODO: This function is a stub, please fill in the Jacobian')
	Jac
end