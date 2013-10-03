function rot = calcRotationMatrix(quaternion)
%CALCROTATIONMATRIX Summary of this function goes here
%   Detailed explanation goes here

    qw = quaternion(1);
    qx = quaternion(2);
    qy = quaternion(3);
    qz = quaternion(4);

    rot = [ 1-2*(qy^2+qz^2)     2*(qx*qy-qw*qz)     2*(qx*qz+qw*qy)
            2*(qx*qy+qw*qz)     1-2*(qx^2+qz^2)     2*(qy*qz-qw*qx)
            2*(qx*qz-qw*qy)     2*(qy*qz+qw*qx)     1-2*(qx^2+qy^2)	];

end

