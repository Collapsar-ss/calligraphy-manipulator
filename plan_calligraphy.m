function result = plan_calligraphy(cfg,progressFcn)
%PLAN_CALLIGRAPHY 纯离线轨迹计算；不运行 Simulink，不连接控制器。
if nargin<1, cfg=calligraphy_config; end
if nargin<2, progressFcn=@(message) fprintf('%s\n',message); end
[cfg,glyph,pieces]=calligraphy_prepare(cfg);
frames=calligraphy_frames(cfg);
originalDurations=[pieces.duration];
pieces=calligraphy_local_timing(cfg,glyph,pieces,progressFcn);
for iteration=1:cfg.maxTimeScalingIterations
    report(sprintf('轨迹检查 %d：%.2f s',iteration,sum([pieces.duration])));
    tr=calligraphy_sample(cfg,glyph,pieces);
    sampleScale=max([ones(numel(tr.t),1),vecnorm(tr.cartesianVelocity,2,2)./tr.speedLimit, ...
        sqrt(vecnorm(tr.cartesianAcceleration,2,2)./tr.accelerationLimit), ...
        abs(tr.cartesianVelocity*frames.paperR(:,3))/cfg.verticalSpeed, ...
        sqrt(abs(tr.cartesianAcceleration*frames.paperR(:,3))/cfg.verticalAcceleration)],[],2);
    if max(sampleScale)>1+1e-6
        pieces=calligraphy_refine_timing(pieces,tr,sampleScale,cfg); continue
    end
    [tr.q,tr.ikPositionError,tr.ikOrientationError]=calligraphy_ik(tr,cfg,progressFcn);
    tr.qd=derivative(tr.q,cfg.Ts); tr.qdd=derivative(tr.qd,cfg.Ts);
    sampleScale=max([ones(numel(tr.t),1),abs(tr.qd)./deg2rad(cfg.jointSpeedDeg), ...
        sqrt(abs(tr.qdd)./deg2rad(cfg.jointAccelDeg))],[],2);
    if max(sampleScale)>1+1e-6
        pieces=calligraphy_refine_timing(pieces,tr,sampleScale,cfg); continue
    end
    assert(max(tr.ikPositionError)<=.01 && max(tr.ikOrientationError)<=1e-6, ...
        'Calligraphy:Verification','笔尖正解回代误差超出允许范围。');
    result.config=cfg; result.glyph=glyph; result.pieces=pieces; result.trajectory=tr;
    result.validation.passed=true;
    result.validation.calibrated=logical(cfg.calibrated);
    result.validation.maxPositionErrorMm=max(tr.ikPositionError);
    result.validation.maxOrientationErrorRad=max(tr.ikOrientationError);
    result.validation.peakJointSpeedDeg=rad2deg(max(abs(tr.qd),[],1));
    result.validation.peakJointAccelDeg=rad2deg(max(abs(tr.qdd),[],1));
    result.validation.peakCartesianSpeed=max(vecnorm(tr.cartesianVelocity,2,2));
    result.validation.peakCartesianAcceleration=max(vecnorm(tr.cartesianAcceleration,2,2));
    result.validation.timeScale=sum([pieces.duration])/sum(originalDurations);
    result.validation.internalStops=0;
    result.validation.timingMethod='local-quintic';
    result.validation.sharedNodeAcceleration=true;
    result.validation.safePointBaseMm=calligraphy_safe_point(cfg);
    result.validation.timeNodeCount=sum(arrayfun(@(p)numel(p.timeLaw.distance),pieces));
    result.validation.strokeSmoothingErrorMm=cellfun(@(s)s.smoothingErrorMm,glyph.strokes);
    result.validation.smoothingFitScale=glyph.smoothingFitScale;
    result.validation.simulinkRun=false;
    result.validation.collisionChecked=false;
    result.source=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'data','source.json')));
    report(sprintf('通过：%d 点，%.3f s，笔尖最大回代误差 %.3g mm',numel(tr.t),tr.t(end),max(tr.ikPositionError)));
    return
end
error('Calligraphy:Limits','局部速度规划多次调整后仍超限，请检查速度限制或标定。');
    function report(message)
        if ~isempty(progressFcn), progressFcn(message); end
    end
end

function d=derivative(v,dt)
d=zeros(size(v));
d(2:end-1,:)=(v(3:end,:)-v(1:end-2,:))/(2*dt);
d(1,:)=(-3*v(1,:)+4*v(2,:)-v(3,:))/(2*dt);
d(end,:)=(3*v(end,:)-4*v(end-1,:)+v(end-2,:))/(2*dt);
end
