function [Xi, g0, config] = HL0905Kinematics
%HL0905KINEMATICS 返回 HL-0905 机器人的空间旋量和零位参数。
%   [Xi, g0, config] = HL0905Kinematics
%
%   坐标和符号约定已经用随实验提供的 HL0905_1.slx 模型校验：
%     - q1、q4、q6 在零位绕基坐标系 +z 轴旋转；
%     - q2、q3、q5 在零位绕基坐标系 +y 轴旋转；
%     - 长度单位为 mm，关节角单位为 rad。

config.L1 = 491;
config.L2 = 450;
config.L3 = 450;
config.L4 = 84;

config.jointLimitsDeg = [ ...
    -170, 170; ...
    -120, 120; ...
    -140, 140; ...
    -170, 170; ...
    -120, 120; ...
    -360, 360];
config.jointLimitsRad = deg2rad(config.jointLimitsDeg);

h2 = config.L1;
h3 = config.L1 + config.L2;
hw = config.L1 + config.L2 + config.L3;

w = [0, 0, 0, 0, 0, 0; ...
     0, 1, 1, 0, 1, 0; ...
     1, 0, 0, 1, 0, 1];
q = [0, 0, 0, 0, 0, 0; ...
     0, 0, 0, 0, 0, 0; ...
     0, h2, h3, hw, hw, hw];

Xi = zeros(6, 6);
for i = 1:6
    Xi(:, i) = [-cross(w(:, i), q(:, i)); w(:, i)];
end

R0 = diag([-1, -1, 1]);
p0 = [0; 0; hw + config.L4];
g0 = [R0, p0; 0, 0, 0, 1];

config.Xi = Xi;
config.g0 = g0;
config.wristCenterHeight = hw;
end
