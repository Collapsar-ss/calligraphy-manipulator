function tr = calligraphy_sample(cfg,glyph,pieces)
% 每次时间缩放都重新计算原曲线、深度与笛卡尔导数。
frames=calligraphy_frames(cfg);
counts=round([pieces.duration]/cfg.Ts); N=1+sum(counts);
assert(N<=cfg.maxSamples,'Calligraphy:Samples','轨迹需要 %d 点，超过 maxSamples；请增大 Ts 或调整速度。',N);
tr.t=(0:N-1).'*cfg.Ts;
tr.paperPosition=zeros(N,3); tr.position=zeros(N,3);
tr.cartesianVelocity=zeros(N,3); tr.cartesianAcceleration=zeros(N,3);
tr.strokeId=zeros(N,1); tr.segmentId=zeros(N,1); tr.pieceId=zeros(N,1);
tr.phase=strings(N,1); tr.progress=nan(N,1); tr.depth=zeros(N,1);
tr.speedLimit=zeros(N,1); tr.accelerationLimit=zeros(N,1);
tr.localIntervalId=ones(N,1); tr.localSpeedLimit=zeros(N,1);
cursor=1;
for k=1:numel(pieces)
    piece=pieces(k); T=piece.duration; count=counts(k);
    u=(0:count).'/count;
    h=10*u.^3-15*u.^4+6*u.^5;
    hd=(30*u.^2-60*u.^3+30*u.^4)/T;
    hdd=(60*u-180*u.^2+120*u.^3)/T^2;
    interval=ones(size(u)); reference=nan(size(u));
    if isfield(piece,'timeLaw') && ~isempty(piece.timeLaw)
        [r,rd,rdd,interval,reference]=calligraphy_time_at(piece.timeLaw,u*T,T);
        L=piece.timeLaw.distance(end); h=r/L; hd=rd/L; hdd=rdd/L;
    end
    if strcmp(piece.kind,'line')
        delta=piece.p1-piece.p0;
        p=piece.p0+h.*delta; v=hd.*delta; a=hdd.*delta;
        s=nan(size(h));
        if ismember(piece.phase,{'travel','safe_travel'}), vMax=cfg.travelSpeed; aMax=cfg.cartAcceleration;
        else, vMax=cfg.verticalSpeed; aMax=cfg.verticalAcceleration; end
    else
        st=glyph.strokes{piece.strokeId}; L=st.length;
        [xy,tangent,curvature]=calligraphy_stroke_at(st,L*h);
        sd=hd; sdd=hdd; s=h;
        [d,ds,dss]=calligraphy_depth(cfg.depthProfiles{piece.strokeId},s,cfg.maxDepth);
        p=[xy,-d]; v=[tangent.*(L*hd),-ds.*sd];
        a=[curvature.*(L*hd).^2+tangent.*(L*hdd),-(dss.*sd.^2+ds.*sdd)];
        vMax=cfg.writeSpeed; aMax=cfg.cartAcceleration;
    end
    local=1:count+1; if k>1, local=2:count+1; end
    ix=cursor:cursor+numel(local)-1; cursor=ix(end)+1;
    tr.paperPosition(ix,:)=p(local,:);
    tr.position(ix,:)=p(local,:)*frames.paperR.'+frames.origin;
    tr.cartesianVelocity(ix,:)=v(local,:)*frames.paperR.';
    tr.cartesianAcceleration(ix,:)=a(local,:)*frames.paperR.';
    tr.strokeId(ix)=piece.strokeId; tr.segmentId(ix)=piece.segmentId;
    tr.pieceId(ix)=k; tr.phase(ix)=string(piece.phase);
    tr.progress(ix)=s(local); tr.depth(ix)=-p(local,3);
    tr.speedLimit(ix)=vMax; tr.accelerationLimit(ix)=aMax;
    tr.localIntervalId(ix)=interval(local);
    if all(isnan(reference)), reference(:)=vMax; end
    tr.localSpeedLimit(ix)=reference(local);
end
tr.tipRotation=frames.tipR; tr.modelRotation=frames.modelR;
tr.modelPosition=tr.position-(frames.modelR*cfg.toolOffsetMm(:)).';
tr.pieceEndTimes=cumsum([pieces.duration]);
tr=calligraphy_identify_samples(tr,glyph);
end
