function obj = Joint(dh)
    obj.DH = dh;
    obj.angle = 0;
    obj.mass = 0;
    obj.COM = [0;0;0];
end
