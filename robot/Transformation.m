function g = Transformation(xi, theta)
%TRANSFORMATION 计算单个旋量的齐次矩阵指数 exp(xi^ * theta)。
%   xi 采用 [v; w] 排列，其中 v、w 均为 3x1 向量。该实现同时支持
%   转动关节、移动关节以及未严格归一化的转轴 w。

validateattributes(xi, {'numeric'}, {'vector', 'numel', 6, 'real', 'finite'}, ...
    mfilename, 'xi');
validateattributes(theta, {'numeric'}, {'scalar', 'real', 'finite'}, ...
    mfilename, 'theta');

xi = xi(:);
v = xi(1:3);
w = xi(4:6);
w_norm = norm(w);

if w_norm < 1e-12
    % 移动关节：exp([0 v; 0 0] theta)。
    R = eye(3);
    p = v * theta;
else
    % 对一般长度的 w 直接计算 Rodrigues 公式及其积分矩阵 V。
    W = skew3(w);
    phi = w_norm * theta;
    R = eye(3) ...
        + (sin(phi) / w_norm) * W ...
        + ((1 - cos(phi)) / w_norm^2) * (W * W);
    V = theta * eye(3) ...
        + ((1 - cos(phi)) / w_norm^2) * W ...
        + ((phi - sin(phi)) / w_norm^3) * (W * W);
    p = V * v;
end

g = [R, p; 0, 0, 0, 1];
end

function W = skew3(w)
%SKEW3 由三维向量构造反对称矩阵，使 W*x = w cross x。
W = [   0, -w(3),  w(2); ...
     w(3),     0, -w(1); ...
    -w(2),  w(1),     0];
end
