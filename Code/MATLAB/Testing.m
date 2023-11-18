%%
clc;
clear all;
close;

%% Known Variables

% MOTOR ANGLE LIMITS
theta1_min = -55;
theta1_max = 55;

theta2_min = -115;
theta2_max = 0;

theta3_min = -108;
theta3_max = 0;

theta4_min = -103;
theta4_max = 0;

% Pen Angle
pen_angle = 129.75;

% LINK LENGTHS
L2 = 140;           % mm
L3 = 140;           % mm
L4 = 40.389;        % mm
L5 = 25;            % mm  [PEN EXTENTION LENGTH]


% LINK ANGLES
% theta 2, 3, 4 are w.r.t. upward vertical axis
theta1 =  30 * pi / 180;       % rad [EDIT]
theta2 = -30 * pi / 180;       % rad [EDIT]
theta3 = -30 * pi / 180;       % rad [EDIT]
theta4 = -30 * pi / 180;       % rad [EDIT]
theta5 = -50.25 * pi / 180;       % rad


%% Symbolic Forward Kinematics
syms theta1_sym theta2_sym theta3_sym theta4_sym theta5_sym L2_sym L3_sym L4_sym L5_sym

TF_1 = TF(     0,            0,         0,   theta1_sym);
TF_2 = TF( sym(pi)/2,        0,         0,   sym(pi)/2 + theta2_sym);
TF_3 = TF(     0,          L2_sym,      0,   theta3_sym);
TF_4 = TF(     0,          L3_sym,      0,   theta4_sym);
TF_5 = TF(     0,          L4_sym,      0,   theta5_sym);
TF_6 = TF(     0,          L5_sym,      0,       0);

% base to end effector T
TF_0to6 = simplify(TF_1*TF_2*TF_3*TF_4*TF_5*TF_6);

% convert to function
forwardKin = matlabFunction(TF_0to6);

PenTip = forwardKin(L2,L3,L4,L5,theta1,theta2,theta3,theta4,theta5);


x_target = 250;
y_target = -50;
z_target = -65;

tic
for i = 1:300
    i = i/100;
    [theta1, theta2, theta3, theta4] = IK(x_target+i, y_target+i, z_target+i, L2, L4, pen_angle);
end
toc
%% Function Definitions

function T = TF(alpha,a,d,theta)
T = [cos(theta)            -sin(theta)            0           a
     sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
     sin(theta)*sin(alpha) cos(theta)*sin(alpha)  cos(alpha)  cos(alpha)*d
     0 0 0 1];
end

% Inverse Kinematics
function [theta1, theta2, theta3, theta4] = IK(x_target, y_target, z_target, L2, L4, pen_angle)

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
    
    theta1 = theta1*pi/180;
    theta2 = theta2*pi/180;
    theta3 = theta3*pi/180;
    theta4 = theta4*pi/180;
end

