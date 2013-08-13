function joints_ref = kineIK(joints, pos_ref, varargin) 
%http://sourceforge.net/p/robotjoint/code/ci/master/tree/src/RobotManipulator.cxx#l899
%http://groups.csail.mit.edu/drl/journal_club/papers/033005/buss-2004.pdf
% to think about: http://mi.ams.eng.osaka-u.ac.jp/pub/2011/tro2011sugihara.pdf
    %default values
    maxiter = 1000;
    maxposerr = 0.005;
    maxroterr = 0.0001;
    kTheta = 1;
    kX = 0.05;
    mode = 0;
    dump = 0.1;
    
    %local variables
    DOF = size(joints,1);
    argl = length(varargin);
    error = zeros(6,1);
    joints_ref = ones(DOF,1);
    dtheta = zeros(DOF,1);
    
    error_log = [];
    % Feed optional configurations
    if(argl > 1)
        if rem(argl,2) ~= 0
            error('Optional inputs must be in format ''variable name'',variable value')
        end
        for ii = 1:2:argl
            if strcmp(varargin{ii},'maxiter')
                maxiter = varargin{ii+1};
            end
            if strcmp(varargin{ii},'kX')
                kX = varargin{ii+1};
            end
            if strcmp(varargin{ii},'kTheta')
                kTheta = varargin{ii+1};
            end
            if strcmp(varargin{ii},'maxposerr')
                maxposerr = varargin{ii+1};
            end
            if strcmp(varargin{ii},'maxroterr')
                maxroterr = varargin{ii+1};
            end
            if strcmp(varargin{ii},'dump')
                dump = varargin{ii+1};
            end
            if strcmp(varargin{ii},'mode')
                if(strcmp(varargin{ii+1},'pinv'))
                    mode = 0;
                end
                if(strcmp(varargin{ii+1},'transp'))
                    mode = 1;
                end
                if(strcmp(varargin{ii+1},'dls'))
                    mode = 2;
                end
            end
        end
    end
    for cntr = 1:maxiter
        % Feed Joints Angles
        for i=1:DOF
            joints(i).angle = joints_ref(i,1);
        end
        % Calc Jacobian
        jac = kineJacobian(joints);
        % Calc PseudoInverse
        switch mode
            case 0
                %use Pseudo Inverse
                ijac = pinv(jac);
            case 1
                %use Tanspose (need to implement optimal alpha)
                ijac = jac';
            case 2
                %use Tanspose (need to implement optimal alpha)
                ijac = jac';
            otherwise
                ijac = pinv(jac);
        end
        % Get TransForm Matrix
        fk = forwardKine(joints);
        % Get Translation
        x = fk(1:3,4);
        % Calc the error
        error(1:3) = (pos_ref - x);
        %error_log = [error_log; error(1:3)'];
        if(max(abs(error(1:3))) < maxposerr && max(abs(error(4:6))) < maxroterr)
            break;
        end
        error(1:3) = kX*error(1:3);
        % Calc new joint angles
        if mode == 2
            dtheta = (ijac*inv(jac*ijac+(dump^2)*eye(6)))*error;
            joints_ref  = joints_ref + dtheta;
        else
            joints_ref  = joints_ref +ijac*error;
        end
        
        % Lets just remove the accumulated angle in excess
        joints_ref = mod(joints_ref, 2*pi);
    end
    %plot(error_log);
end