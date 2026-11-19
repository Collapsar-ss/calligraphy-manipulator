function report = test_ik_optimization
% 对照候选回代开启/关闭：选解、TCP误差、全部轨迹及实测耗时。
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'robot'));
saved=load(fullfile(root,'results','U04EBA','calligraphy_trajectory.mat'),'result');
result=saved.result; tr=result.trajectory; cfg=result.config;
small=subset(tr,1:2000);
calligraphy_ik(subset(tr,1:10),cfg,[],false); % 预热
baseline=zeros(1,3); optimized=baseline;
for repeat=1:3
    tic; [oldQ,oldP,oldR]=calligraphy_ik(small,cfg,[],true); baseline(repeat)=toc;
    tic; [newQ,newP,newR]=calligraphy_ik(small,cfg,[],false); optimized(repeat)=toc;
    assert(isequal(oldQ,newQ) && isequal(oldP,newP) && isequal(oldR,newR));
end
% 原始二参数 API 仍检查全部候选；快速 API 明确返回 NaN 而非假零误差。
T=[tr.modelRotation,tr.modelPosition(100,:).';0 0 0 1];
[a,info]=Ikine6s(T); [b,fast]=Ikine6s(T,[],false);
assert(isequal(a,b) && info.candidatesVerified && ~fast.candidatesVerified);
assert(all(isfinite(info.positionError)) && all(isnan(fast.positionError)));
assert(isequal(info.singular,fast.singular) && isequal(info.withinLimits,fast.withinLimits));
% 全部 32031 点与优化前保存的轨迹逐点比较。
[q,p,r]=calligraphy_ik(tr,cfg,[],false);
assert(max(abs(q-tr.q),[],'all')<1e-12);
assert(max(abs(p-tr.ikPositionError))<1e-12 && max(abs(r-tr.ikOrientationError))<1e-12);
% 非零工具旋转/偏移时，最终 TCP 验证仍与独立 Fkine 一致。
cfg.toolOffsetMm=[8 -5 90]; cfg.toolRpyDeg=[1 2 5];
frames=calligraphy_frames(cfg); altered=subset(tr,1:250);
altered.modelRotation=frames.modelR; altered.tipRotation=frames.tipR;
altered.modelPosition=altered.position-(frames.modelR*cfg.toolOffsetMm(:)).';
[q,p,r]=calligraphy_ik(altered,cfg,[],false);
for i=1:size(q,1)
    tip=HL0905Fkine(q(i,:))*frames.modelToTip;
    assert(abs(p(i)-norm(tip(1:3,4).'-altered.position(i,:)))<1e-12);
end
assert(max(p)<.01 && max(r)<1e-6);
% 快速模式仍拒绝不可达及无合法关节解。
bad=altered; bad.modelPosition(:,:)=5000;
mustFail(@()calligraphy_ik(bad,cfg,[],false),'Calligraphy:IK');
cfg.jointLimitsDeg=repmat([0 .001],6,1);
mustFail(@()calligraphy_ik(altered,cfg,[],false),'Calligraphy:IK');
report.samplesPerTiming=2000; report.baselineSeconds=median(baseline);
report.optimizedSeconds=median(optimized); report.speedup=report.baselineSeconds/report.optimizedSeconds;
report.fullTrajectorySamples=numel(tr.t); report.identicalResults=true;
report.simulinkRun=false;
file=fopen(fullfile(root,'tests','ik_optimization_report.json'),'wt','n','UTF-8');
cleanup=onCleanup(@()fclose(file)); fprintf(file,'%s',jsonencode(report,'PrettyPrint',true));
disp(report); fprintf('IK OPTIMIZATION TESTS PASSED.\n');
end

function out=subset(tr,ix)
out=tr;
fields={'t','position','modelPosition','strokeId','phase'};
for i=1:numel(fields), out.(fields{i})=tr.(fields{i})(ix,:); end
end
function mustFail(action,id)
try, action(); catch err, assert(strcmp(err.identifier,id)); return; end
error('Expected %s.',id);
end
