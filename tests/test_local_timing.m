function report = test_local_timing
% 局部时间律回归，不启动 Simulink。
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root);
characters={'一','人','永'}; report=struct([]);
for k=1:numel(characters)
    cfg=calligraphy_config(characters{k});
    legacy=legacyCartesianDuration(cfg);
    result=plan_calligraphy(cfg);
    verify(result);
    if k>1
        assert(result.trajectory.t(end)<legacy, '复杂笔画局部调速未改善整笔笛卡尔限速时长。');
    end
    export_calligraphy(result);
    report(k).character=characters{k}; report(k).legacyCartesianDuration=legacy;
    report(k).localDuration=result.trajectory.t(end);
    report(k).samples=numel(result.trajectory.t); report(k).validation=result.validation;
    report(k).writePeak=max(vecnorm(result.trajectory.cartesianVelocity(result.trajectory.phase=="write",:),2,2));
    if k==2
        [f,~]=plot_calligraphy_end_velocity(result);
        exportgraphics(f,fullfile(root,'tests','local_velocity_preview.png'),'Resolution',120); close(f);
    end
end
% 检查陡峭提按、倾斜纸面、工具偏置及低关节限制仍能局部重规划。
cfg=calligraphy_config('人'); cfg.Ts=.003;
cfg.paperRpyDeg=[2 -3 90]; cfg.toolOffsetMm=[4 -3 60]; cfg.toolRpyDeg=[1 2 0];
cfg.jointSpeedDeg=repmat(8,1,6); cfg.jointAccelDeg=repmat(12,1,6);
cfg.depthDefault=[0 1.2 3 .3 2 0];
stress=plan_calligraphy(cfg); verify(stress);
% 提高速度设定，在相同几何进度处深度仍由六点决定。
for i=1:numel(stress.config.depthProfiles)
    ix=stress.trajectory.strokeId==i & stress.trajectory.phase=="write";
    expected=calligraphy_depth(stress.config.depthProfiles{i},stress.trajectory.progress(ix),cfg.maxDepth);
    assert(max(abs(expected-stress.trajectory.depth(ix)))<1e-9);
end
% UI 生成入口走局部规划，参数变更使旧结果失效。
cfg=calligraphy_config('一'); cfg.Ts=.01; app=calligraphy_app(cfg);
app.generateButton.ButtonPushedFcn(app.generateButton,[]);
assert(strcmp(app.getResult().validation.timingMethod,'local-quintic'));
app.figure.CloseRequestFcn(app.figure,[]);
close all;
file=fopen(fullfile(root,'tests','local_timing_report.json'),'wt','n','UTF-8');
cleanup=onCleanup(@()fclose(file)); fprintf(file,'%s',jsonencode(report,'PrettyPrint',true));
fprintf('LOCAL TIMING TESTS PASSED. SIMULINK NOT RUN.\n');
end

function verify(result)
cfg=result.config; tr=result.trajectory; frames=calligraphy_frames(cfg);
assert(result.validation.passed && ~result.validation.simulinkRun);
assert(all(abs(diff(tr.t)-cfg.Ts)<1e-9));
assert(all(vecnorm(tr.cartesianVelocity,2,2)<=tr.speedLimit*(1+1e-6)));
assert(all(vecnorm(tr.cartesianAcceleration,2,2)<=tr.accelerationLimit*(1+1e-6)));
assert(all(abs(tr.cartesianVelocity*frames.paperR(:,3))<=cfg.verticalSpeed*(1+1e-6)));
assert(all(abs(tr.cartesianAcceleration*frames.paperR(:,3))<=cfg.verticalAcceleration*(1+1e-6)));
assert(all(abs(tr.qd)<=deg2rad(cfg.jointSpeedDeg)*(1+1e-6),'all'));
assert(all(abs(tr.qdd)<=deg2rad(cfg.jointAccelDeg)*(1+1e-6),'all'));
ends=round(tr.pieceEndTimes/cfg.Ts)+1;
assert(max(abs(tr.cartesianVelocity(ends,:)),[],'all')<1e-7);
assert(max(abs(tr.cartesianAcceleration(ends,:)),[],'all')<1e-6);
for k=1:numel(result.pieces)
    p=result.pieces(k); law=p.timeLaw;
    % 节点左右极限的速度、加速度，独立从多项式系数求导验证。
    dt=diff(law.time); c=law.coefficients;
    v0=c(:,2)./dt; v1=(c(:,2)+2*c(:,3)+3*c(:,4)+4*c(:,5)+5*c(:,6))./dt;
    a0=2*c(:,3)./dt.^2; a1=(2*c(:,3)+6*c(:,4)+12*c(:,5)+20*c(:,6))./dt.^2;
    assert(max(abs(v0(2:end)-v1(1:end-1)))<1e-6);
    assert(max(abs(a0(2:end)-a1(1:end-1)))<1e-5);
    assert(all(law.velocity(2:end-1)>0));
    ix=tr.pieceId==k & tr.t>sum([result.pieces(1:k-1).duration])+1e-8 & ...
        tr.t<sum([result.pieces(1:k).duration])-1e-8;
    assert(all(vecnorm(tr.cartesianVelocity(ix,:),2,2)>0));
end
% 从真正的位置采样差分验证速度，防止仅修改速度显示。
v=(tr.position(3:end,:)-tr.position(1:end-2,:))/(2*cfg.Ts);
assert(max(vecnorm(v-tr.cartesianVelocity(2:end-1,:),2,2))<.2);
assert(max(tr.ikPositionError)<.01 && max(tr.ikOrientationError)<1e-6);
end

function duration=legacyCartesianDuration(cfg)
[cfg,glyph,pieces]=calligraphy_prepare(cfg); frames=calligraphy_frames(cfg);
for iteration=1:8
    tr=calligraphy_sample(cfg,glyph,pieces);
    ratios=max([ones(numel(tr.t),1),vecnorm(tr.cartesianVelocity,2,2)./tr.speedLimit, ...
        sqrt(vecnorm(tr.cartesianAcceleration,2,2)./tr.accelerationLimit), ...
        abs(tr.cartesianVelocity*frames.paperR(:,3))/cfg.verticalSpeed, ...
        sqrt(abs(tr.cartesianAcceleration*frames.paperR(:,3))/cfg.verticalAcceleration)],[],2);
    factors=accumarray(tr.pieceId,ratios,[numel(pieces) 1],@max,1);
    if max(factors)<=1+1e-6, duration=tr.t(end); return; end
    for k=1:numel(pieces)
        if factors(k)>1+1e-6
            pieces(k).duration=ceil(pieces(k).duration*factors(k)*1.02/cfg.Ts)*cfg.Ts;
        end
    end
end
error('整笔对照规划未收敛。');
end
