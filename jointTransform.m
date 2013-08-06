function T = jointTransform(joint) 
%see http://www.elysium-labs.com/robotics-corner/learn-robotics/introduction-to-robotics/joint-state-parameterization-and-forward-kinematics/
%using link transformation as defined in "Robot modeling and control" by M. Sprong, S. Hutchinson and M. Vidyasagar
T = zeros(4,4)
	T(1,1) = cos(joint.DH.theta+ joint.angle);	T(1,2) = -sin(joint.DH.theta+ joint.angle)*cos(joint.DH.alpha);	T(1,3) = sin(joint.DH.theta+ joint.angle)*sin(joint.DH.alpha);	T(1,4) = joint.DH.a*cos(joint.DH.theta + joint.angle);
	T(2,1) = sin(joint.DH.theta +  joint.angle);	T(2,2) = cos(joint.DH.theta +  joint.angle)*cos(joint.DH.alpha);	T(2,3) = -cos(joint.DH.theta +  joint.angle)*sin(joint.DH.alpha);	T(2,4) = joint.DH.a*sin(joint.DH.theta +  joint.angle);
	T(3,1) = 0.0;		T(3,2) = sin(joint.DH.alpha);			T(3,3) = cos(joint.DH.alpha);			T(3,4) = joint.DH.offset;
	T(4,1) = 0.0;		T(4,2) = 0.0;				T(4,3) = 0.0;				T(4,4) = 1.0;	
	
%	return T;
end