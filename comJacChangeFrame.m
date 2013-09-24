function outJac = comJacChangeFrame(comJac, joints, COM)
%COMJACCHANGEFRAME Summary of this function goes here
%   Detailed explanation goes here
    kineJac = kineJacobian(joints);
    fk = forwardKine(joints);
    R = ROT(fk)';
    pos = TRANS(fk);
    p = R*(COM(1:3)-pos);
    dof = DOF(joints);
    outJac = zeros(6,dof);
    for i=1:dof
        outJac(1:3,i) = R*(-kineJac(1:3,i) + comJac(1:3,i))-(cross(R*comJac(4:6,i),p));
        outJac(4:6,i) = R*comJac(4:6,i);
    end
end

