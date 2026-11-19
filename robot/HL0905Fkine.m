function T = HL0905Fkine(theta)
%HL0905FKINE 使用已校准参数计算 HL-0905 的正运动学。
%   theta 为 1x6 或 6x1 关节角向量，单位 rad。

validateattributes(theta, {'numeric'}, ...
    {'vector', 'numel', 6, 'real', 'finite'}, mfilename, 'theta');
[Xi, g0] = HL0905Kinematics;
T = POE(Xi, theta, g0);
end
