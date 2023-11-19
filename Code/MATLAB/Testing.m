%%
clc;
clear all;
close;

%% Known Variables

% Paper level relative to frame 0 z axis
z_paper = -118;         % mm
z_lifted = z_paper + 5; % mm

% MOTOR ANGLE LIMITS
theta1_min = -55;
theta1_max = 55;

theta2_min = -115;
theta2_max = 0;

theta3_min = -108;
theta3_max = 0;

theta4_min = -103;
theta4_max = 103;

% Pen Angle
pen_angle = 129.75; % deg

% LINK LENGTHS
L2 = 140;           % mm
L3 = 140;           % mm
L4 = 40.389;        % mm
L5 = 25;            % mm  [PEN EXTENTION LENGTH]


% LINK ANGLES
% theta 2, 3, 4 are w.r.t. upward vertical axis
theta1 =  30;       % deg [EDIT]
theta2 = -30;       % deg [EDIT]
theta3 = -30 ;      % deg [EDIT]
theta4 = -30;       % deg [EDIT]
theta5 = -50.25;    % deg

%% Main

% TARGET POSITION
x_target = 250;
y_target = -50;
z_target = -65;



[theta1, theta2, theta3, theta4] = IK(x_target, y_target, z_target, L2, L4, pen_angle);


For = FK(L2,L3,L4,L5,theta1,theta2,theta3,theta4,theta5);

Move(theta1, theta2, theta3, theta4);

x1 = -1;
x2 = 2;
y1=-1;
y2 = 2;
dist = sqrt((x2-x1)^2+(y2-y1)^2);
steps = ceil(dist)
x_spacing = (x2-x1)/steps;
y_spacing = (y2-y1)/steps;
for i = 1:steps
    x = x1+(x_spacing*(i))
    y = y1+(y_spacing*(i))
    fprintf("-------------------------")
    z = z_paper;
end
%% FUNCTION DEFINITIONS

%//////////////////////////////////////////////////////////////////////////////////////////////////
% FORWARD KINEMATICS - units: degrees
%//////////////////////////////////////////////////////////////////////////////////////////////////
function T = FK(L2,L3,L4,L5,theta1,theta2,theta3,theta4,theta5)
    TF_1 = TF(     0,            0,         0,       theta1);
    TF_2 = TF(    90,            0,         0,     90 + theta2);
    TF_3 = TF(     0,            L2,        0,       theta3);
    TF_4 = TF(     0,            L3,        0,       theta4);
    TF_5 = TF(     0,            L4,        0,       theta5);
    TF_6 = TF(     0,            L5,        0,           0);
    
    % base to end effector T
    T = TF_1*TF_2*TF_3*TF_4*TF_5*TF_6;
end

%//////////////////////////////////////////////////////////////////////////////////////////////////
% INVERSE KINEMATICS - units: degrees and mm
%//////////////////////////////////////////////////////////////////////////////////////////////////
function [theta1, theta2, theta3, theta4] = IK(x_target, y_target, z_target, L2, L4, pen_angle)
    
    % Motor Rotation Limits (constrained by 3d part interferences)
    theta1_min = -55;
    theta1_max = 55;
    
    theta2_min = -115;
    theta2_max = 0;
    
    theta3_min = -108;
    theta3_max = 0;
    
    theta4_min = -103;
    theta4_max = 103;

    % This is an offset target that frame 4 should reach
    x = 0;      % recalculated later
    y = y_target;
    z = z_target + L4*sind(pen_angle-90) + 25;
    
    % Link 1 simply needs to point in the direction of the target
    theta1 = atan2d(y_target, x_target);
    
    % Recalculate the adjusted offset target x & y coordinates based on theta1 angle
    x = x_target - (L4 * cosd(pen_angle-90)*abs(cosd(theta1)));
    if theta1 > 0
        y = y_target - (L4 * cosd(pen_angle-90)*abs(sind(theta1)));
    elseif theta1 < 0
        y = y_target + (L4 * cosd(129.75-90)*abs(sind(theta1)));
    end
    
    % straight-line distance to adjusted target, used for calculations below
    L = sqrt(x^2+y^2+z^2);
    
    % The rest is purely geometrically derived
    theta2 = (90 - ( acosd(L/(2*L2)) + atan2d(z, sqrt(x^2+y^2)) )) * (-1);
    theta3 = -2 * acosd(L/(2*L2));
    theta4 = -1* (pen_angle - abs(theta2) - abs(theta3));
    
    % Bound Checking
    if theta1<theta1_min || theta2<theta2_min || theta3<theta3_min || theta4<theta4_min || theta1>theta1_max || theta2>theta2_max || theta3>theta3_max || theta4>theta4_max
        error("at least one calculated motor angle out of bounds");
    end
end

%//////////////////////////////////////////////////////////////////////////////////////////////////
% Make Everything Upright/Zero - units: degrees and mm
%//////////////////////////////////////////////////////////////////////////////////////////////////
function [theta1, theta2, theta3, theta4] = Zero(L2, L4, pen_angle)
    
end


%//////////////////////////////////////////////////////////////////////////////////////////////////
% Move - units: degrees and mm
%//////////////////////////////////////////////////////////////////////////////////////////////////
function Move(theta1, theta2, theta3, theta4)
    
    theta = [theta1 theta2 theta3 theta4];
    length(theta)
    % Motor angle 0-4095
    for i = 1:length(theta)
        if theta(i) < 0
            theta(i) = abs(theta(i)) / 360 * 4096;
        elseif theta(i) > 0
            theta(i) = 4096 - (abs(theta(i)) / 360 * 4096);
        end
        
    end
    
end

%//////////////////////////////////////////////////////////////////////////////////////////////////
% Calculate T Matrix - units: degrees and mm
%//////////////////////////////////////////////////////////////////////////////////////////////////
function T = TF(alpha,a,d,theta)
T = [cosd(theta)                 -sind(theta)            0           a
     sind(theta)*cosd(alpha) cosd(theta)*cosd(alpha) -sind(alpha) -sind(alpha)*d
     sind(theta)*sind(alpha) cosd(theta)*sind(alpha)  cosd(alpha)  cosd(alpha)*d
     0 0 0 1];
end