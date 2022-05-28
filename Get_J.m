function J = Get_J(T, cm, jointType, link)

    numLinks = length(jointType);
    J = zeros(6, numLinks);
    r = zeros(3, numLinks);
    z_ax = [0 0 1]';
    for i = 1 : link

        r(:, i) = cm(1:3) - T{1}((1:3),4,i);

        k = T{1}((1:3),(1:3),i) * z_ax;

        J(:, i) = Get_J_Col(k, r(:,i), jointType(i));

    end
end