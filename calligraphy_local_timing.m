function pieces = calligraphy_local_timing(cfg,glyph,pieces,progressFcn)
% 在路径节点上建立笛卡尔/关节局部速度包络，再前后向传播。
frames=calligraphy_frames(cfg); nodes=cell(1,numel(pieces));
coarse.position=zeros(0,3); coarse.modelPosition=zeros(0,3);
coarse.strokeId=zeros(0,1); coarse.phase=strings(0,1);
coarse.tipRotation=frames.tipR; coarse.modelRotation=frames.modelR;
offset=0;
for k=1:numel(pieces)
    piece=pieces(k);
    if strcmp(piece.kind,'line')
        L=norm(piece.p1-piece.p0); r=linspace(0,L,max(4,ceil(L/.25)+1)).';
        derivative=repmat((piece.p1-piece.p0)/L,numel(r),1);
        second=zeros(size(derivative)); p=piece.p0+r.*derivative;
        vmax=cfg.travelSpeed; amax=cfg.cartAcceleration;
        if ~ismember(piece.phase,{'travel','safe_travel'}), vmax=cfg.verticalSpeed; amax=cfg.verticalAcceleration; end
    else
        st=glyph.strokes{piece.strokeId}; L=st.length;
        % 包含样条结点及其间四分点，捕捉狭窄急弯，不只取均匀点。
        b=st.cumulative(:); r=unique([linspace(0,L,max(4,ceil(L/.25)+1)).'; ...
            b;(3*b(1:end-1)+b(2:end))/4;(b(1:end-1)+b(2:end))/2; ...
            (b(1:end-1)+3*b(2:end))/4]);
        [xy,tangent,curvature]=calligraphy_stroke_at(st,r);
        [d,ds,dss]=calligraphy_depth(cfg.depthProfiles{piece.strokeId},r/L,cfg.maxDepth);
        p=[xy,-d]; derivative=[tangent,-ds/L]; second=[curvature,-dss/L^2];
        vmax=cfg.writeSpeed; amax=cfg.cartAcceleration;
    end
    cap=min(vmax./max(vecnorm(derivative,2,2),1e-12), ...
        sqrt(.35*amax./max(vecnorm(second,2,2),1e-12)));
    cap=min(cap,cfg.verticalSpeed./max(abs(derivative(:,3)),1e-12));
    cap=min(cap,sqrt(.35*cfg.verticalAcceleration./max(abs(second(:,3)),1e-12)));
    accel=min(.35*amax./max(vecnorm(derivative,2,2),1e-12), ...
        .35*cfg.verticalAcceleration./max(abs(derivative(:,3)),1e-12));
    count=numel(r); index=offset+(1:count); offset=offset+count;
    nodes{k}=struct('distance',r,'derivative',derivative,'speedCap',cap, ...
        'accelerationCap',accel,'index',index);
    coarse.position(index,:)=p*frames.paperR.'+frames.origin;
    coarse.modelPosition(index,:)=coarse.position(index,:)-(frames.modelR*cfg.toolOffsetMm(:)).';
    coarse.strokeId(index,1)=piece.strokeId; coarse.phase(index,1)=string(piece.phase);
end
coarse.t=(0:offset-1).';
coarse=calligraphy_identify_samples(coarse,glyph);
if ~isempty(progressFcn), progressFcn(sprintf('建立局部限速包络：%d 个路径节点',offset)); end
q=calligraphy_ik(coarse,cfg,progressFcn);
for k=1:numel(pieces)
    law=nodes{k}; r=law.distance; angles=q(law.index,:);
    qr=zeros(size(angles)); qrr=qr;
    for j=1:6
        qr(:,j)=gradient(angles(:,j),r);
        qrr(:,j)=gradient(qr(:,j),r);
    end
    law.speedCap=min(law.speedCap,min(deg2rad(cfg.jointSpeedDeg)./max(abs(qr),1e-12),[],2));
    law.speedCap=min(law.speedCap,min(sqrt(.35*deg2rad(cfg.jointAccelDeg)./max(abs(qrr),1e-12)),[],2));
    law.accelerationCap=min(law.accelerationCap,min(.35*deg2rad(cfg.jointAccelDeg)./max(abs(qr),1e-12),[],2));
    % referenceSpeed 以三维 TCP 线速度表示，仅作为节点限速参考。
    law.referenceSpeed=law.speedCap.*vecnorm(law.derivative,2,2);
    law=rmfield(law,{'index','derivative'});
    law=calligraphy_reduce_timing_nodes(law); law=calligraphy_time_law(law);
    pieces(k).timeLaw=law;
    pieces(k).duration=ceil(law.time(end)/cfg.Ts)*cfg.Ts;
end
end
