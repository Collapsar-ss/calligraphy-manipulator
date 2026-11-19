function [theta, info] = Ikine6s(T, config, verifyCandidates)
%IKINE6S 求解 HL-0905 六轴机器人的全部离散逆运动学分支。
%   theta = IKINE6S(T, config) 返回 Nx6 关节角矩阵，每一行是一组解，
%   单位为 rad。T 为 4x4 目标齐次矩阵。
%
%   config 可使用 HL0905Kinematics 返回的结构体，也可直接传入
%   [L1 L2 L3 L4]。传入 [] 时使用实验指导书给出的默认尺寸。
%
%   [theta, info] 还返回：
%     info.withinLimits     每组解是否满足关节限位
%     info.positionError    正解回代位置误差（mm）
%     info.orientationError 正解回代姿态误差（rad）
%     info.singular         是否属于腕部奇异代表解
%     info.reachable        目标是否可达
%     info.message          求解状态说明
%   第三个参数 verifyCandidates 默认为 true。false 时跳过候选解的正解
%   回代，误差字段返回 NaN，调用方必须对最终选中解另行验证。

% 允许直接以 Ikine6s(T) 调用；未传入尺寸配置时使用 HL-0905 默认参数。
if nargin < 2
    config = [];
end
if nargin < 3, verifyCandidates = true; end
validateattributes(verifyCandidates, {'logical','numeric'}, {'scalar','binary'}, ...
    mfilename, 'verifyCandidates');

validateattributes(T, {'numeric'}, {'size', [4, 4], 'real', 'finite'}, ...
    mfilename, 'T');
if norm(T(4, :) - [0, 0, 0, 1]) > 1e-9
    error('Ikine6s:InvalidTransform', ...
        'The last row of T must be [0 0 0 1].');
end

[lengths, limits] = parseConfig(config);
L1 = lengths(1);
L2 = lengths(2);
L3 = lengths(3);
L4 = lengths(4);

R = T(1:3, 1:3);
p = T(1:3, 4);
if norm(R.' * R - eye(3), 'fro') > 1e-7 || abs(det(R) - 1) > 1e-7
    error('Ikine6s:InvalidRotation', ...
        'T(1:3,1:3) must be a proper rotation matrix.');
end

% 末端零位相对第六关节坐标系绕 z 轴旋转 pi。
R0 = diag([-1, -1, 1]);
R06 = R * R0.';

% 工具坐标系的 z 轴与第六关节 z 轴重合，因此可直接退回腕心。
pW = p - R06 * [0; 0; L4];
rho = hypot(pW(1), pW(2));

tol = 1e-10;
if rho < tol
    q1Candidates = 0;
    shoulderSingular = true;
else
    % 子问题1：分别以 +x 和 -x 为起始向量绕 z 轴旋转到腕心投影，得到肩部两个分支。
    pProjection = [pW(1); pW(2); 0];
    q1Candidates = [subproblem1([0; 0; 1], [rho; 0; 0], pProjection), ...
                    subproblem1([0; 0; 1], [-rho; 0; 0], pProjection)];
    shoulderSingular = false;
end

solutions = zeros(0, 6);
singularFlags = false(0, 1);

for i1 = 1:numel(q1Candidates)
    q1 = q1Candidates(i1);
    wristInShoulderPlane = rotz3(-q1) * pW - [0; 0; L1];

    % 子问题3：求 q3，使 ||L2*ez + Ry(q3)*L3*ez|| 等于肩部到腕心的距离。
    q3Candidates = subproblem3([0; 1; 0], -L3 * [0; 0; 1], ...
        L2 * [0; 0; 1], norm(wristInShoulderPlane));
    if isempty(q3Candidates)
        continue
    end

    for i3 = 1:numel(q3Candidates)
        q3 = q3Candidates(i3);
        elbowVector = L2 * [0; 0; 1] + roty3(q3) * (L3 * [0; 0; 1]);

        % 子问题1：求 q2，使肌部连杆向量旋转到目标腕心向量。
        q2 = subproblem1([0; 1; 0], elbowVector, wristInShoulderPlane);

        R03 = rotz3(q1) * roty3(q2 + q3);
        R36 = R03.' * R06;
        [wristSolutions, wristSingular] = solveWristBySubproblems(R36);

        for iw = 1:size(wristSolutions, 1)
            candidate = [q1, q2, q3, wristSolutions(iw, :)];
            candidate = wrapPi(candidate);
            [solutions, singularFlags] = appendUnique( ...
                solutions, singularFlags, candidate, ...
                wristSingular(iw) || shoulderSingular);
        end
    end
end

theta = solutions;
n = size(theta, 1);
withinLimits = false(n, 1);
positionError = nan(n, 1);
orientationError = nan(n, 1);

if verifyCandidates, [Xi, g0] = modelFromLengths(lengths); end
for i = 1:n
    withinLimits(i) = all(theta(i, :) >= limits(:, 1).' - 1e-10 & ...
                          theta(i, :) <= limits(:, 2).' + 1e-10);
    if verifyCandidates
        Tcheck = POE(Xi, theta(i, :), g0);
        positionError(i) = norm(Tcheck(1:3, 4) - p);
        c = (trace(R.' * Tcheck(1:3, 1:3)) - 1) / 2;
        orientationError(i) = acos(max(-1, min(1, c)));
    end
end

info.withinLimits = withinLimits;
info.positionError = positionError;
info.orientationError = orientationError;
info.candidatesVerified = logical(verifyCandidates);
info.singular = singularFlags;
info.reachable = ~isempty(theta);
info.message = sprintf('Found %d unique inverse-kinematics branches.', n);
end

function [lengths, limits] = parseConfig(config)
if nargin < 1 || isempty(config)
    [~, ~, defaultConfig] = HL0905Kinematics;
    lengths = [defaultConfig.L1, defaultConfig.L2, ...
               defaultConfig.L3, defaultConfig.L4];
    limits = defaultConfig.jointLimitsRad;
elseif isstruct(config)
    required = {'L1', 'L2', 'L3', 'L4'};
    for i = 1:numel(required)
        if ~isfield(config, required{i})
            error('Ikine6s:InvalidConfig', ...
                'config is missing field %s.', required{i});
        end
    end
    lengths = [config.L1, config.L2, config.L3, config.L4];
    if isfield(config, 'jointLimitsRad')
        limits = config.jointLimitsRad;
    elseif isfield(config, 'jointLimitsDeg')
        limits = deg2rad(config.jointLimitsDeg);
    else
        [~, ~, defaultConfig] = HL0905Kinematics;
        limits = defaultConfig.jointLimitsRad;
    end
elseif isnumeric(config) && isvector(config) && numel(config) == 4
    lengths = config(:).';
    [~, ~, defaultConfig] = HL0905Kinematics;
    limits = defaultConfig.jointLimitsRad;
else
    error('Ikine6s:InvalidConfig', ...
        'config must be a length structure or [L1 L2 L3 L4].');
end

validateattributes(lengths, {'numeric'}, ...
    {'vector', 'numel', 4, 'positive', 'finite'}, mfilename, 'config lengths');
validateattributes(limits, {'numeric'}, ...
    {'size', [6, 2], 'real', 'finite'}, mfilename, 'joint limits');
end

function [wrist, singular] = solveWristBySubproblems(R)
% 腕部的 q4、q5 为两个相交轴(z、y)的子问题2；q6 再用子问题1求解。
[q45, singular] = subproblem2ZY(R(:, 3));
wrist = zeros(size(q45, 1), 3);
for i = 1:size(q45, 1)
    q4 = q45(i, 1);
    q5 = q45(i, 2);
    B = rotz3(q4) * roty3(q5);
    residual = B.' * R;
    q6 = subproblem1([0; 0; 1], [1; 0; 0], residual * [1; 0; 0]);
    wrist(i, :) = [q4, q5, q6];
end
end

function theta = subproblem1(w, p, q)
% 子问题1：求 theta 使 Rot(w,theta)*p = q（轴经过原点）。
w = w / norm(w);
pp = p - w * (w.' * p);
qq = q - w * (w.' * q);
if norm(pp) < 1e-10 || norm(qq) < 1e-10 || ...
        abs(norm(pp) - norm(qq)) > 1e-7 || abs(w.' * (p - q)) > 1e-7
    error('Ikine6s:Subproblem1Infeasible', ...
        'Subproblem 1 has no numerically valid solution.');
end
theta = atan2(w.' * cross(pp, qq), pp.' * qq);
end

function theta = subproblem3(w, p, q, d)
% 子问题3：求 theta 使 ||q - Rot(w,theta)*p|| = d。
w = w / norm(w);
pp = p - w * (w.' * p);
qq = q - w * (w.' * q);
axialDistance = w.' * (q - p);
d2 = d^2 - axialDistance^2;
if d2 < -1e-8 || norm(pp) < 1e-10 || norm(qq) < 1e-10
    theta = zeros(0, 1);
    return
end
d2 = max(0, d2);
c = (norm(pp)^2 + norm(qq)^2 - d2) / (2 * norm(pp) * norm(qq));
if c < -1 - 1e-8 || c > 1 + 1e-8
    theta = zeros(0, 1);
    return
end
c = max(-1, min(1, c));
base = atan2(w.' * cross(pp, qq), pp.' * qq);
delta = acos(c);
theta = [wrapPi(base + delta); wrapPi(base - delta)];
if abs(delta) < 1e-10
    theta = theta(1);
end
end

function [q45, singular] = subproblem2ZY(toolZ)
% 子问题2：求 q4,q5 使 Rz(q4)*Ry(q5)*ez = toolZ。
% 两根轴在腕心相交，因此这正是相交轴的两角子问题。
c5 = max(-1, min(1, toolZ(3)));
q5 = acos(c5);
s5 = sin(q5);
if abs(s5) > 1e-7
    q4 = atan2(toolZ(2), toolZ(1));
    q45 = [q4, q5; q4 + pi, -q5];
    singular = [false; false];
elseif c5 > 0
    q45 = [0, 0];
    singular = true;
else
    q45 = [0, pi];
    singular = true;
end
end

function [solutions, flags] = appendUnique(solutions, flags, candidate, flag)
for i = 1:size(solutions, 1)
    angularDifference = wrapPi(solutions(i, :) - candidate);
    if norm(angularDifference) < 1e-8
        flags(i) = flags(i) || flag;
        return
    end
end
solutions(end + 1, :) = candidate; %#ok<AGROW>
flags(end + 1, 1) = flag; %#ok<AGROW>
end

function [Xi, g0] = modelFromLengths(lengths)
L1 = lengths(1);
L2 = lengths(2);
L3 = lengths(3);
L4 = lengths(4);
h = [0, L1, L1 + L2, L1 + L2 + L3, ...
     L1 + L2 + L3, L1 + L2 + L3];
w = [0, 0, 0, 0, 0, 0; ...
     0, 1, 1, 0, 1, 0; ...
     1, 0, 0, 1, 0, 1];
Xi = zeros(6, 6);
for i = 1:6
    Xi(:, i) = [-cross(w(:, i), [0; 0; h(i)]); w(:, i)];
end
g0 = [diag([-1, -1, 1]), [0; 0; h(6) + L4]; 0, 0, 0, 1];
end

function R = roty3(q)
R = [cos(q), 0, sin(q); 0, 1, 0; -sin(q), 0, cos(q)];
end

function R = rotz3(q)
R = [cos(q), -sin(q), 0; sin(q), cos(q), 0; 0, 0, 1];
end

function a = wrapPi(a)
a = atan2(sin(a), cos(a));
end

function info = emptyInfo(reachable, message)
info.withinLimits = false(0, 1);
info.positionError = zeros(0, 1);
info.orientationError = zeros(0, 1);
info.singular = false(0, 1);
info.reachable = reachable;
info.message = message;
end
