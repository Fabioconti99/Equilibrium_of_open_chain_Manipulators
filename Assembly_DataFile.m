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
smiData.RigidTransform(1).ID = "RootGround[Basis v4:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [0 0 0];  % mm
smiData.RigidTransform(2).angle = 0;  % rad
smiData.RigidTransform(2).axis = [0 0 0];
smiData.RigidTransform(2).ID = "SixDofRigidTransform[Link1 v2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 0 0];  % mm
smiData.RigidTransform(3).angle = 0;  % rad
smiData.RigidTransform(3).axis = [0 0 0];
smiData.RigidTransform(3).ID = "SixDofRigidTransform[Motor v2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [1000 0 0];  % mm
smiData.RigidTransform(4).angle = 0;  % rad
smiData.RigidTransform(4).axis = [0 0 0];
smiData.RigidTransform(4).ID = "SixDofRigidTransform[Motor v2:2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [1000 0 0];  % mm
smiData.RigidTransform(5).angle = 0;  % rad
smiData.RigidTransform(5).axis = [0 0 0];
smiData.RigidTransform(5).ID = "SixDofRigidTransform[Link2 v6:1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(4).mass = 0.0;
smiData.Solid(4).CoM = [0.0 0.0 0.0];
smiData.Solid(4).MoI = [0.0 0.0 0.0];
smiData.Solid(4).PoI = [0.0 0.0 0.0];
smiData.Solid(4).color = [0.0 0.0 0.0];
smiData.Solid(4).opacity = 0.0;
smiData.Solid(4).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 196.57356816160427;  % lbm
smiData.Solid(1).CoM = [0 -229.35195732797197 0];  % mm
smiData.Solid(1).MoI = [4300789.73572913 17727487.205783881 16377246.699499149];  % lbm*mm^2
smiData.Solid(1).PoI = [0 0 0];  % lbm*mm^2
smiData.Solid(1).color = [0.34901960784313724 0.58431372549019611 0.85490196078431369];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Basis v4.ipt_{4E082728-47ED-D2CD-266E-039A591DBE59}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 221.79137450188148;  % lbm
smiData.Solid(2).CoM = [500 0 0];  % mm
smiData.Solid(2).MoI = [3381656.5650981381 27248281.141441993 27303067.089011919];  % lbm*mm^2
smiData.Solid(2).PoI = [0 0 0];  % lbm*mm^2
smiData.Solid(2).color = [0.62745098039215685 0.62745098039215685 0.62745098039215685];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Link1 v2.ipt_{7EE978AD-45B0-8044-BF1A-F4BFF8F053DD}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 24.933694437855664;  % lbm
smiData.Solid(3).CoM = [0 0 0];  % mm
smiData.Solid(3).MoI = [715200.95305258711 715200.9338073316 149017.77354111895];  % lbm*mm^2
smiData.Solid(3).PoI = [0 0 0];  % lbm*mm^2
smiData.Solid(3).color = [0.70588235294117652 0.69803921568627447 0.65490196078431373];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Motor v2.ipt_{91169D95-482B-8D2B-768D-5B9DAEC733D5}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 245.07526584227764;  % lbm
smiData.Solid(4).CoM = [531.64564160944803 0 0];  % mm
smiData.Solid(4).MoI = [5363228.6682486972 22294162.893418234 20712982.714835092];  % lbm*mm^2
smiData.Solid(4).PoI = [0 0 0];  % lbm*mm^2
smiData.Solid(4).color = [0.74901960784313726 0.74901960784313726 0.74901960784313726];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Link2 v6.ipt_{526C9FB6-4221-856A-D7C2-868DFDD2AFA6}";

