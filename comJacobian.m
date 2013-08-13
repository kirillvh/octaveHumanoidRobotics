function Jac = comJacobian(joints) 
%see http://www.elysium-labs.com/robotics-corner/learn-robotics/biped-basics/com-jacobian/
%http://sourceforge.net/p/robotjoint/code/ci/master/tree/src/RobotManipulator.cxx#l272
    DOF = size(joints,1); %the number of joints
    mass = 0;
    for i=1:DOF
        mass = mass+joints(i).mass; 
    end
    Jac = zeros(6,DOF); %Jacobian is 6xN (6 = 3 linear + 3 rotational axes)
    pL = zeros(3,1);
    rotn = eye(3);
    for i=1:DOF
        if(i>1)
            T = eye(4)*jointTransform(joints(i-1));
            rotn = rotn*ROT(T);
            fk = forwardKine(joints, 1, i-1);
            pL = TRANS(fk);
        end
        zn = rotn(:,3);
        pCOM = calcCOM(joints, i);
        xn = -pL + pCOM(1:3);
        Jac(:,i) = vertcat((pCOM(4)/mass)*cross(zn,xn),zn);
    end
end