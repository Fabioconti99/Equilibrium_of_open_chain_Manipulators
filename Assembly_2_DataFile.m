% Simscape(TM) Multibody(TM) version: 7.5

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(5).translation = [0.0 0.0 0.0];
smiData.RigidTransform(5).angle = 0.0;
smiData.RigidTransform(5).axis = [0.0 0.0 0.0];
smiData.RigidTransform(5).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 0 0];  % mm
smiData.RigidTransform(1).angle = 0;  % rad
smiData.RigidTransform(1).axis = [0 0 0];
smiData.RigidTransform(1).ID = "RootGround[Basis v2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [0 0 0];  % mm
smiData.RigidTransform(2).angle = 1.5707963267948959;  % rad
smiData.RigidTransform(2).axis = [0 1 0];
smiData.RigidTransform(2).ID = "SixDofRigidTransform[Motor_prismatic v3:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 0 0];  % mm
smiData.RigidTransform(3).angle = 1.5707963267948959;  % rad
smiData.RigidTransform(3).axis = [0 1 0];
smiData.RigidTransform(3).ID = "SixDofRigidTransform[Link1 v2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [1000 0 0];  % mm
smiData.RigidTransform(4).angle = 0;  % rad
smiData.RigidTransform(4).axis = [-0.42062268423316063 -0.3369388683955859 0.8423471709887751];
smiData.RigidTransform(4).ID = "SixDofRigidTransform[Motor v2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [1000 -5.329070518201e-14 2.1316282072799999e-13];  % mm
smiData.RigidTransform(5).angle = 7.7672232286087517e-16;  % rad
smiData.RigidTransform(5).axis = [0.099048249688299575 -0.87032636423986753 -0.48241213079969059];
smiData.RigidTransform(5).ID = "SixDofRigidTransform[Link2 v2:1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(5).mass = 0.0;
smiData.Solid(5).CoM = [0.0 0.0 0.0];
smiData.Solid(5).MoI = [0.0 0.0 0.0];
smiData.Solid(5).PoI = [0.0 0.0 0.0];
smiData.Solid(5).color = [0.0 0.0 0.0];
smiData.Solid(5).opacity = 0.0;
smiData.Solid(5).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 196.57356816160427;  % lbm
smiData.Solid(1).CoM = [0 -229.35195732797197 0];  % mm
smiData.Solid(1).MoI = [4300789.73572913 17727487.205783881 16377246.699499149];  % lbm*mm^2
smiData.Solid(1).PoI = [1.1772720922961413e-09 0 7.0038383496406566e-06];  % lbm*mm^2
smiData.Solid(1).color = [0.34901960784313724 0.58431372549019611 0.85490196078431369];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Basis v2.ipt_{2EB0C7B6-4E2A-1431-3D49-44B64925E9AE}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 24.933694437855664;  % lbm
smiData.Solid(2).CoM = [0 0 0];  % mm
smiData.Solid(2).MoI = [149017.77354111904 715200.93380733102 715200.95305258653];  % lbm*mm^2
smiData.Solid(2).PoI = [1.0266074081178642e-10 1.0266074079794829e-10 0];  % lbm*mm^2
smiData.Solid(2).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Motor_prismatic v3.ipt_{E28D9310-42FB-DFD5-2F1F-8E81CB1E01E7}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 221.79137450188148;  % lbm
smiData.Solid(3).CoM = [0 0 500];  % mm
smiData.Solid(3).MoI = [27303067.089011937 27248281.141442012 3381656.5650981348];  % lbm*mm^2
smiData.Solid(3).PoI = [0 -3.5931259284125265e-09 -8.2128592649429134e-10];  % lbm*mm^2
smiData.Solid(3).color = [0.62745098039215685 0.62745098039215685 0.62745098039215685];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Link1 v2.ipt_{1A54F49E-40D4-F930-4C0B-3FB0804A932E}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 24.933694437855664;  % lbm
smiData.Solid(4).CoM = [0 0 0];  % mm
smiData.Solid(4).MoI = [715200.95305258711 715200.9338073316 149017.77354111895];  % lbm*mm^2
smiData.Solid(4).PoI = [-1.0266074081178642e-10 -1.0266074081178642e-10 1.0266074081178642e-10];  % lbm*mm^2
smiData.Solid(4).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Motor v2.ipt_{2CE74B02-4A35-6005-D7E2-34837A6A4A9B}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 245.07526584227764;  % lbm
smiData.Solid(5).CoM = [531.64564160944803 0 0];  % mm
smiData.Solid(5).MoI = [5363228.6682486972 22294162.893418234 20712982.714835092];  % lbm*mm^2
smiData.Solid(5).PoI = [-8.2128592649429134e-10 6.5702874119543307e-09 0];  % lbm*mm^2
smiData.Solid(5).color = [0 0 0];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "Link2 v2.ipt_{C820D684-4AEC-B6D6-7240-4A87FB3ACCEB}";

