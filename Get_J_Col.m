% Computation of the jacobian column
function [J_col] = Get_J_Col (k, r, jointType)

if jointType == 1

J_a = [0; 0; 0];
J_l = k;

end

if jointType == 2

J_a = k; 
J_l = cross(k, r);   

end

J_col = [J_a;J_l];

end