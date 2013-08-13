%% ADDJOINT Add a joint to a manipulator
% ========================================================================
function obj = addJoint( M, alpha, theta, a, offset, varargin)
    dh =  struct('alpha', alpha, 'theta', theta, 'a', a, 'offset', offset);
    joint = Joint(dh);
    argl = length(varargin);
    if(argl > 1)
        if rem(argl,2) ~= 0
            error('Optional inputs must be in format ''variable name'',variable value')
        end
        for ii = 1:2:argl
            if strcmp(varargin{ii},'mass')
                joint.mass = varargin{ii+1};
            end
            if strcmp(varargin{ii},'COM')
                joint.COM = varargin{ii+1};
            end
        end
    end
    M.joints = [M.joints; joint];
    obj = M;
end
