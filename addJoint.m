function obj = addJoint( M, alpha ,theta, a, offset)
    dh =  struct('alpha', alpha, 'theta', theta, 'a', a, 'offset', offset);
    joint = Joint(dh);
    M.joints = [M.joints; joint];
    obj = M;
end