function Jac = kineJacobian(joints) 
%see http://www.elysium-labs.com/robotics-corner/learn-robotics/introduction-to-robotics/kinematic-jacobian/
DOF = size(joints,1); %the number of joints
Jac = zeros(6,DOF); %Jacobian is 6xN (6 = 3 linear + 3 rotational axes)
rotn = eye(3);
    for i=1:DOF
        if(i>1)
            T = eye(4)*jointTransform(joints(i-1));
            rotn = rotn*T(1:3,1:3);
        end
        zn = rotn(:,3);
        fk = forwardKine(joints, i);
        xn = rotn*fk(1:3,4);
        Jac(:,i) = vertcat(cross(zn,xn),zn);
    end
end