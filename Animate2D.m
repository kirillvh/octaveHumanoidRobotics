function  Animate2D(joints, path, rate)
%ANIMATE2D Summary of this function goes here
%   Detailed explanation goes here
    DOF = size(joints, 1);
    frames = size(path, 1);
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
        drawnow; 
        pause(0.1);
    end
end


