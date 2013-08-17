function T = comTransform(joint)
%COMTRANSFORM Summary of this function goes here
%   Detailed explanation goes here
x = joint.COM(1);
y = joint.COM(2);
z = joint.COM(3);

ct = cos(joint.DH.theta+ joint.angle);
st = sin(joint.DH.theta+ joint.angle);

ca = cos(joint.DH.alpha);
sa = sin(joint.DH.alpha);

T = zeros(4,4);
	T(1,1) = ct;	T(1,2) = -st*ca;	T(1,3) = st*sa;     T(1,4) = x*ct-ca*st*y;
	T(2,1) = st;	T(2,2) = ct*ca;     T(2,3) = -ct*sa;	T(2,4) = x*st+ca*ct*y;
	T(3,1) = 0.0;   T(3,2) = sa;		T(3,3) = ca;		T(3,4) = z+y*sa;
	T(4,1) = 0.0;	T(4,2) = 0.0;		T(4,3) = 0.0;		T(4,4) = 1.0;	

end

