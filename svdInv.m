function inv=svdInv(mat)
	[u,s,v]=svd(mat);
	sinv = eye(2,2);
	tolerence = 0.01;
	for i=1:size(mat,1)
		if(abs(s(i,i)) >tolerence )
			sinv(i,i) = 1.0 / s(i,i);
		else
			sinv(i,i) = 0.0;
		end
	end
	inv = v * sinv * u';
end