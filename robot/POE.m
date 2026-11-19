function g_st = POE(Xi, theta, g0)
%POE 使用空间旋量的指数积公式计算机器人正运动学。
%   g_st = POE(Xi, theta, g0)
%
%   输入：
%     Xi    - 6xN 旋量矩阵，每一列为 xi_i = [v_i; w_i]。
%             对非六轴机器人，也兼容每一行为一个旋量的 Nx6 矩阵。
%     theta - N 个关节变量组成的向量；转动关节单位为 rad，移动关节
%             的单位应与 v、g0 的长度单位一致。
%     g0    - 所有关节变量为零时，末端相对基坐标系的 4x4 齐次矩阵。
%
%   输出：
%     g_st  - 当前关节变量下的 4x4 末端齐次矩阵。
%
%   指数积顺序：
%     g_st = exp(xi_1^ theta_1) ... exp(xi_N^ theta_N) g0

validateattributes(Xi, {'numeric'}, {'2d', 'real', 'finite'}, mfilename, 'Xi');
validateattributes(theta, {'numeric'}, {'vector', 'real', 'finite'}, mfilename, 'theta');
validateattributes(g0, {'numeric'}, {'size', [4, 4], 'real', 'finite'}, mfilename, 'g0');

theta = theta(:);
n = numel(theta);

% 课程中常用 6xN（每列一个旋量）。当维数明确为 Nx6 时自动转置。
if size(Xi, 1) == 6 && size(Xi, 2) == n
    twists = Xi;
elseif size(Xi, 2) == 6 && size(Xi, 1) == n
    twists = Xi.';
else
    error('POE:DimensionMismatch', ...
        'Xi must contain %d twists as a 6xN or Nx6 matrix.', n);
end

g_st = eye(4);
for i = 1:n
    g_st = g_st * Transformation(twists(:, i), theta(i));
end
g_st = g_st * g0;

% 浮点运算可能使最后一行出现极小误差，将其恢复为齐次矩阵标准形式。
g_st(4, :) = [0, 0, 0, 1];
end
