function  Animate2D(joints, path, varargin)
%ANIMATE2D Summary of this function goes here
%   Detailed explanation goes here
    DOF = size(joints, 1);
    frames = size(path, 1);
    plotCOM = false;
    argl = length(varargin);
    
    if(argl > 1)
        if rem(argl,2) ~= 0
            error('Optional inputs must be in format ''variable name'',variable value')
        end
        for ii = 1:2:argl
            if strcmp(varargin{ii},'plotCOM')
                plotCOM = varargin{ii+1};
            end        
        end
    end
    
    
    
    for f = 1:frames
        X = [0];
        Y = [0];
        clf;
        for i = 1:DOF
            joints(i).angle = path(f,i);
            fk = forwardKine(joints, 1, i);
            EPt = fk(1:2,4);
            X = [X EPt(1)];
            Y = [Y EPt(2)];
            IPt = EPt;
        end
        plot(X,Y,'-or',...
            'LineWidth',4,...
            'MarkerEdgeColor','k',...
            'MarkerFaceColor','g',...
            'MarkerSize', 10);
        axis([-1 1 -1 1]);
        if(plotCOM == true)
            hold on;
            comPOS = calcCOM(joints);
            plot(comPOS(1),comPOS(2),'-or',...
            'LineWidth',4,...
            'MarkerEdgeColor','b',...
            'MarkerFaceColor','b',...
            'MarkerSize', 10);
            hold off;
        end
        drawnow; 
        pause(0.1);
    end
end


