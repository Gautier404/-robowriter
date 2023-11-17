% MAE C163A/C263A Project
% Team X
clc;
% Initialize
initialize();
input('Press any key to continue!');


% Main
theta1 = 0; % DH joint angle unit rad

Theta1 = theta1/2/pi*4096; % Motor angle 0-4095; You may also need to consider the offset, i.e., when theta1 = 0, Theta1 ~= 0.

% Move MX28_ID(1) to Theta1 angle

for r = 1:5
for pos = 0:15
    p = pos*1024/4
    for i = 2:6
        write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), MX28_GOAL_POSITION, typecast(int32(p), 'uint32'));
        pause(.05)
    end
end
end

% Terminate
input('Press any key to terminate!');
terminate();