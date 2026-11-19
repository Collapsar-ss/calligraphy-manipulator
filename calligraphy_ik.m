function [q,positionError,orientationError] = calligraphy_ik(tr,cfg,progressFcn,verifyCandidates)
% 复用实验四解析 IK；枚举合法 2*pi 等价角，保留实际关节行程。
% 第四参数仅用于诊断对照；默认跳过候选回代，保留逐点最终笔尖验证。
if nargin<4, verifyCandidates=false; end
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'robot'));
[Xi,g0,robot]=HL0905Kinematics;
frames=calligraphy_frames(cfg); modelToTip=frames.modelToTip;
robot.jointLimitsRad=deg2rad(cfg.jointLimitsDeg);
% IK 原函数把角度归一化到 [-pi,pi]；先返回所有分支，再按实际行程筛选。
solver=robot; solver.jointLimitsRad=repmat([-pi pi],6,1);
limits=robot.jointLimitsRad;
N=numel(tr.t); q=zeros(N,6); positionError=zeros(N,1); orientationError=zeros(N,1);
previous=deg2rad(cfg.qSeedDeg); R=tr.modelRotation;
for i=1:N
    T=[R,tr.modelPosition(i,:).';0 0 0 1];
    [solutions,info]=Ikine6s(T,solver,verifyCandidates);
    candidates=zeros(0,6);
    for b=1:size(solutions,1)
        if info.singular(b), continue; end
        candidate=solutions(b,:); valid=true;
        for j=1:6
            low=ceil((limits(j,1)-candidate(j)-1e-10)/(2*pi));
            high=floor((limits(j,2)-candidate(j)+1e-10)/(2*pi));
            if low>high, valid=false; break; end
            shift=min(high,max(low,round((previous(j)-candidate(j))/(2*pi))));
            candidate(j)=candidate(j)+2*pi*shift;
        end
        if valid, candidates(end+1,:)=candidate; end %#ok<AGROW>
    end
    if isempty(candidates)
        error('Calligraphy:IK','%s / %s / t=%.3f s 无非奇异且满足限位的逆解。', ...
            sampleLabel(tr,i),tr.phase(i),tr.t(i));
    end
    [~,best]=min(sum((candidates-previous).^2,2));
    q(i,:)=candidates(best,:);
    if i>1 && max(abs(q(i,:)-previous))>pi/6
        error('Calligraphy:Branch','%s t=%.3f s 发生大于 30 度的逆解跳变。',sampleLabel(tr,i),tr.t(i));
    end
    previous=q(i,:);
    % 只对选中的真实关节角回算笔尖，含工具旋转/偏移及 2*pi 等价角。
    % Xi/g0/工具标定在循环外计算，POE 与 HL0905Fkine 使用同一模型。
    tip=POE(Xi,previous,g0)*modelToTip;
    positionError(i)=norm(tip(1:3,4).'-tr.position(i,:));
    c=(trace(tr.tipRotation.'*tip(1:3,1:3))-1)/2;
    orientationError(i)=acos(max(-1,min(1,c)));
    if nargin>2 && ~isempty(progressFcn) && (mod(i,1000)==0 || i==N)
        progressFcn(sprintf('逆运动学 %d / %d',i,N));
    end
end
end

function label=sampleLabel(tr,i)
if isfield(tr,'characterIndex') && tr.characterIndex(i)>0
    k=tr.characterIndex(i);
    label=sprintf('第 %d 字“%s”第 %d 笔',k,tr.characters{k},tr.localStrokeId(i));
elseif tr.strokeId(i)==0
    label='安全点进出运动';
else
    label=sprintf('第 %d 笔',tr.strokeId(i));
end
end
