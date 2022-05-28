function [cm] = Compute_c_m (m1,m2, cm1, cm2)
    
    cm = zeros(3, 1); 

    cm(1) = (m1 * cm1(1) + m2* cm2(1)) / (m1 + m2);
    cm(2) = (m1 * cm1(2) + m2* cm2(2)) / (m1 + m2);
    cm(3) = (m1 * cm1(3) + m2* cm2(3)) / (m1 + m2);
end