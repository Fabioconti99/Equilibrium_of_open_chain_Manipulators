clc
clear

%%
Assembly_DataFile
smiData_es1=smiData;
Assembly_2_DataFile
smiData_es2=smiData;

%% Center of Mass of the Two Bodies
% Bodies' masses

% Body1
m_link1 = smiData_es1.Solid(2).mass * 0.453592 ; %Kg
cm_link1_es1 = smiData_es1.Solid(2).CoM; %Link1 smiData1
cm_link1_es2 = smiData_es2.Solid(3).CoM; %Link1 smiData2

% Motor
m_motor= smiData_es1.Solid(3).mass * 0.453592 ; %Kg
cm_motor_es1 = smiData_es1.Solid(3).CoM; %motor smiData1

cm_motor_es2_prismatic = smiData_es2.Solid(2).CoM; %motor1 prismatic smiData2
cm_motor_es2_rotiational = smiData_es2.Solid(4).CoM; %motor2 smiData2

% Body2
m_link2 =smiData_es1.Solid(4).mass * 0.453592 ; %Kg
cm_link2 = smiData_es1.Solid(4).CoM; %Link2 smiData1, smiData2

%% Computing COM exercise 1
CM_link_1_es1=Compute_c_m(m_link1,m_motor,cm_link1_es1,cm_motor_es1);
CM_link_2_es1=Compute_c_m(m_link2,m_motor,cm_link2,cm_motor_es1);

%% Computing COM exercise 2
CM_link_1_es2=Compute_c_m(m_link1,m_motor,cm_link1_es2,cm_motor_es2_prismatic);
CM_link_2_es2=Compute_c_m(m_link2,m_motor,cm_link2,cm_motor_es2_rotiational);

%% Gravity vector
g = [0; -9.81; 0]; %m/s^2

%% X, Y, Z axis
x_axis = [1, 0, 0];
y_axis = [0, 1, 0];
z_axis = [0, 0 ,1];

%% Joint-Types
jointTypes_es_1 = [2, 2]; % rotational joints
jointTypes_es_2 = [1, 2]; % prismatic and rotational joint

%% Relative r_ang between base link-link1 and link1-link2 CELL
r_ang = cell(2,5);

% configuration 1.1:
r_ang{1, 1} = [pi/2, -pi/2];

% configuration 1.2:
r_ang{1, 2} = [0, pi/2];

% configuration 1.3:
r_ang{1, 3} = [pi/6 , pi/3];

% configuration 1.4:
r_ang{1, 4} = [pi/6 , pi/3];

% configuration 1.5:
r_ang{1, 5} = [pi/6 , - pi/3]; 

% configuration 2.1:
r_ang{2, 1} = [0, pi/4];

% configuration 2.2:
r_ang{2, 2} = [0, pi/2];

% configuration 2.3:
r_ang{2, 3} = [0, pi/4];

% configuration 2.4:
r_ang{2, 4} = [0, pi/4];

% configuration 2.5:
r_ang{2, 5} = [0, pi/4];       

%% Points located the links
P = cell(2,3);

% Exercise 1
% Point P1 w.r.t. the frame of link2
P{1,1}(:,1) = [1000 ; 0 ; 0];

% Point P2 w.r.t. the frame of link2
P{1,2}(:,1) = [300 ; 0 ; 0];

% Point P1 w.r.t. the frame of link1
P{1,3}(:,1) = [900 ; 0 ; 0];

% Exercise 2
% Point P1 w.r.t. the frame of link2
P{2,1}(:,1) = [1000 ; 0 ; 0];

% Point P2 w.r.t. the frame of link2
P{2,2}(:,1) = [650 ; 0 ; 0];

% Point P1 w.r.t. the frame of link1
P{2,3}(:,1) = [0 ; 0 ; 800];


%% Wrenches Cell 
W = cell(2,5);

% exercise 1.1

% Wrench of the gravity force applied on link 1 and link 2
W{1,1}(:,1) = [zeros(3,1); (m_link1+m_motor)*g];
W{1,1}(:,2) = [zeros(3,1); (m_link2+m_motor)*g];

% exercise 1.2

% Wrench of the gravity force applied on link 1 and link 2
W{1,2}(:,1) = [zeros(3,1); (m_link1+m_motor)*g];
W{1,2}(:,2) = [zeros(3,1); (m_link2+m_motor)*g];

% exercise 1.3

% Wrench of the external force applied first on link 1 then link 2
W{1,3}(:,1) = [zeros(3,1); [-0.7 ; -0.5 ; 0]];

% exercise 1.4

% Wrench of the force acting on P3
W{1,4}(:,1) = [zeros(3,1); [1.5 ; -0.3 ; 0]];

% Wrench of the moment acting on P1
W{1,4}(:,2) = [[0; 0; 1200]; zeros(3,1)];

% exercise 1.5

% Wrench of the force acting on P2
W{1,5}(:,1) = [zeros(3,1); [1.2 ; -0.2; 0]];

% Wrench of the force acting on P3
W{1,5}(:,2) = [zeros(3,1); [-0.4 ; 1.2 ; 0]];

% Wrench of the gravity force applied on link 1 and link 2
W{1,5}(:,3) = [zeros(3,1); (m_link1+m_motor)*g];
W{1,5}(:,4) = [zeros(3,1); (m_link2+m_motor)*g];

% exercise 2.1
% Wrench of the gravity force applied on link 2
W{2,1}(:,1) = [zeros(3,1); (m_link2+m_motor)*g];

% exercise 2.2
% Wrench of the gravity force applied on link 2
W{2,2}(:,1) = [zeros(3,1); (m_link2+m_motor)*g];

% exercise 2.3
% Wrench of the force acting on P1
W{2,3}(:,1) = [0 ; 0 ; 0; [-0.8 ; -0.8; 0]];

% exercise 2.4
% Wrench of the force acting on P2
W{2,4}(:,1) = [0 ; 0 ; 0; [-0.8 ; -0.2; 0]];

% Wrench of the moment acting on P1
W{2,4}(:,2) = [[0; 0; 500]; zeros(3,1)];

% exercise 2.5
% Wrench of the force acting on P1
W{2,5}(:,1) = [0 ; 0 ; 0; [0.5 ; -0.6; 0]];

% Wrench of the force acting on P2
W{2,5}(:,2) = [0 ; 0 ; 0; [1.0 ; -0.4; 0]];

% Wrench of the gravity force applied on link 2
W{2,5}(:,3) = [zeros(3,1); (m_link2+m_motor)*g];

%% Frame distances cell
dist_b_f = cell(2,5);

%% COM wrt base frame 
CM_link_i_t_b = cell(2,5);

%% Rotational Matrices Cell
R = cell(2, 5);

%% Transformation Matrices Cel
T = cell(2, 5);

%% Jacobians Cell
J = cell(2, 5);

%% Assigning 

for i = 1 : 2       % Exercises 1 and 2

    for j = 1 : 5   % Point 1 to 5

        if (i == 1)  % Exercise 1

            % Rotational Matrices of the exercise 1:
            R{i,j}(:,:,1) = axang2rotm([z_axis, r_ang{i,j}(1,1)]);
            R{i,j}(:,:,2) = axang2rotm([z_axis, ...
                r_ang{i,j}(1,2) + r_ang{i,j}(1,1)]);

            % Distance between the base frame and the second link's frame:
            dist_b_f{i,j} = R{i,j}(:,:,1)*(smiData_es1.RigidTransform(5).translation)';

            % Transformation Matrices of the exercise 1:
            T{i,j}(:,:,1) = [R{i,j}(:,:,1),zeros(3, 1)];
            T{i,j}(:,:,2) = [R{i,j}(:,:,2),dist_b_f{i,j}];

            % Links' center of mass w.r.t. base of the exercise 1:
            CM_link_i_t_b{i,j}(:,1) = T{i,j}(:,:,1) * [CM_link_1_es1;1];
            CM_link_i_t_b{i,j}(:,2) = T{i,j}(:,:,2) * [CM_link_2_es1;1];

            % Jacobian matrix of the link's center of mass for each
            % configuration of the exercise 1:
            J{i,j}(:,:,1) = Get_J(T(i,j), ...
                CM_link_i_t_b{i,j}(:,1), ...
                jointTypes_es_1, 1);

            J{i,j}(:,:,2) = Get_J(T(i,j), ...
                CM_link_i_t_b{i,j}(:,2), ...
                jointTypes_es_1, 2);

        else % Exercise 2

            % Rotational Matrices of the exercise 2:
            R{i,j}(:,:,1) = axang2rotm([y_axis, pi/2]);
            R{i,j}(:,:,2) = axang2rotm([z_axis, ...
                r_ang{i,j}(1,2) + r_ang{i,j}(1,1)]);

            % Distance between the base frame and the second link's frame:
            dist_b_f{i,j} = (smiData_es2.RigidTransform(5).translation)';

            % Transformation Matrices of the exercise 2:
            T{i,j}(:,:,1) = [R{i,j}(:,:,1), zeros(3, 1)];
            T{i,j}(:,:,2) = [R{i,j}(:,:,2), dist_b_f{i,j}];

            % Links' center of mass w.r.t. base of the exercise 2:
            CM_link_i_t_b{i,j}(:,1) = T{i,j}(:,:,1) * [CM_link_1_es2;1];
            CM_link_i_t_b{i,j}(:,2) = T{i,j}(:,:,2) * [CM_link_2_es2;1];

            % Jacobian matrix of the link's center of mass for each
            % configuration of the exercise 2:
            J{i,j}(:,:,1) = Get_J(T(i,j), ...
                CM_link_i_t_b{i,j}(:,1), ...
                jointTypes_es_2, 1);

            J{i,j}(:,:,2) = Get_J(T(i,j), ...
                CM_link_i_t_b{i,j}(:,2), ...
                jointTypes_es_2, 2);
        end 

    end 

end

%% Tau equivalent
tau = cell(2,5);

%% COMPUTING TAU

%% Exercise 1.1

% Computing tau
tau{1,1}(:,1) = - J{1,1}(:,:,1)' * W{1,1}(:,1) - ...
    J{1,1}(:,:,2)' * W{1,1}(:,2);

%% Exercise 1.2

% Computing tau
tau{1,2}(:,1) = - J{1,2}(:,:,1)' * W{1,2}(:,1) - ...
    J{1,2}(:,:,2)' * W{1,2}(:,2);

%% Exercise 1.3

% question a

% Reprojecting P1 onto the base frame
P1_wrt_base= T{1,3}(:,:,2) * [P{1,1}(:,1);1];

% Computing the rigid body jacobian: point P1 - com link2
S_p1 = RigidBody_J(CM_link_i_t_b{1,3}(:,2)-P1_wrt_base);

% Computing tau
tau{1,3}(:,1) = - J{1,3}(:,:,2)' * S_p1' * W{1,3}(:,1);

% question b

% Reprojecting P2 onto the base frame
P2_wrt_base= T{1,3}(:,:,2) * [P{1,2}(:,1);1];

% Computing the rigid body jacobian: point P2 - com link2
S_p2 = RigidBody_J(CM_link_i_t_b{1,3}(:,2)-P2_wrt_base);

% Computing tau
tau{1,3}(:,2) = - J{1,3}(:,:,2)' * S_p2' * W{1,3}(:,1);


%% Exercise 1.4

% Reprojecting P3 onto the base frame
P3_wrt_base= T{1,4}(:,:,1) * [P{1,3}(:,1);1];

% Computing the rigid body jacobian: point P3 - com link1
S_p3 = RigidBody_J(CM_link_i_t_b{1,4}(:,1)-P3_wrt_base);

% Computing tau
tau{1,4}(:,1) = - (J{1,4}(:,:,1)' * S_p3' * W{1,4}(:,1)+ ...
    J{1,4}(:,:,2)'*W{1,4}(:,2));

%% Exercise 1.5

% Reprojecting P2 onto the base frame
P2_wrt_base= T{1,5}(:,:,2) * [P{1,2}(:,1);1];

% Reprojecting P3 onto the base frame
P3_wrt_base= T{1,5}(:,:,1) * [P{1,3}(:,1);1];

% Computing the rigid body jacobian: point P2 - com link2
S_p2 = RigidBody_J(CM_link_i_t_b{1,5}(:,2)-P2_wrt_base);

% Computing the rigid body jacobian: point P3 - com link1
S_p3 = RigidBody_J(CM_link_i_t_b{1,5}(:,1)-P3_wrt_base);

% Computing tau
tau{1,5}(:,1) = - (J{1,5}(:,:,1)' *S_p3' * W{1,5}(:,1) + ...
    J{1,5}(:,:,2)' * S_p2' *W{1,5}(:,2)) - ...
(J{1,5}(:,:,1)' * W{1,5}(:,3)+ J{1,5}(:,:,2)'*W{1,5}(:,4));
    
%% Exercise 2.1

% Computing tau
tau{2,1}(:,1) = - J{2,1}(:,:,2)' * W{2,1}(:,1);

%% Exercise 2.2

% Computing tau
tau{2,2}(:,1) = - J{2,2}(:,:,2)' *  W{2,2}(:,1);

%% Exercise 2.3

% Reprojecting P1 onto the base frame
P1_wrt_base= T{2,3}(:,:,2) * [P{2,1}(:,1);1];

% Computing the rigid body jacobian: point P1 - com link2
S_p1 = RigidBody_J(CM_link_i_t_b{2,3}(:,2)-P1_wrt_base);

% Computing tau
tau{2,3}(:,1) = - J{2,3}(:,:,2)' * S_p1' *  W{2,3}(:,1);

%% Exercise 2.4

% Reprojecting P1 onto the base frame
P1_wrt_base= T{2,4}(:,:,1) * [P{2,1}(:,1);1];

% Reprojecting P2 onto the base frame
P2_wrt_base= T{2,4}(:,:,2) * [P{2,2}(:,1);1];

% Computing the rigid body jacobian: point P2 - com link2
S_p2 = RigidBody_J(CM_link_i_t_b{2,4}(:,2)-P2_wrt_base);
S_p1 = RigidBody_J(CM_link_i_t_b{2,4}(:,1)-P1_wrt_base);

% Computing tau
tau{2,4}(:,1) = -(J{2,4}(:,:,2)' *S_p2' * W{2,4}(:,1) + ...
    J{2,4}(:,:,2)'*S_p2' *W{2,4}(:,2));

%% Exercise 2.5

% Reprojecting P1 onto the base frame
P1_wrt_base= T{2,5}(:,:,2) * [P{2,1}(:,1);1];

% Reprojecting P2 onto the base frame
P2_wrt_base= T{2,5}(:,:,2) * [P{2,2}(:,1);1];

% Computing the rigid body jacobian: point P1 - com link2
S_p1 = RigidBody_J(CM_link_i_t_b{2,5}(:,2)-P1_wrt_base);

% Computing the rigid body jacobian: point P2 - com link2
S_p2 = RigidBody_J(CM_link_i_t_b{2,5}(:,2)-P2_wrt_base);

% Computing tau
tau{2,5}(:,1) = - (J{2,5}(:,:,2)' * S_p2' * W{2,5}(:,1) + ...
    J{2,5}(:,:,2)' *S_p1' * W{2,5}(:,2)+ ...
    J{2,5}(:,:,2)' * W{2,5}(:,3)) ;

