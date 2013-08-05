%define a robot according to DH params here and call some functions

manipulators = [];  %this will contain left arm, right leg, etc
joints = []; % this will contain some joints for a given manipulators

%see http://www.elysium-labs.com/robotics-corner/learn-robotics/introduction-to-robotics/joint-state-parameterization-and-forward-kinematics/
%joint 1
joint = struct('DH', struct('alpha', 0.1, 'theta', 0.1, 'a', 0.1, 'offset', 0.1),
	'angle', 0.0
	);
%can access joint structure with
joint.DH
joint.angle

%now push this joint into the joints array
joints = [joints; joint]

%and make a new joint by writing over 'joint'
%joint 2
joint = struct('DH', struct('alpha', 0.2, 'theta', 0.2, 'a', 0.2, 'offset', 0.2),
	'angle', 0.0
	);
joints = [joints; joint]

%can see the result with this
joints(1)
joints(2)

%now lets try getting the transformation matrix for joint 1
jointTransform(joints(1))

%next lets try getting the FK of this robot
forwardKine(joints)