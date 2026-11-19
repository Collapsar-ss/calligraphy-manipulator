function [cfg,glyph,pieces] = calligraphy_prepare(cfg)
cfg=calligraphy_validate(cfg); glyph=calligraphy_layout(cfg);
n=numel(glyph.strokes);
if isempty(cfg.depthProfiles)
    cfg.depthProfiles=repmat({[cfg.depthProgress(:),cfg.depthDefault(:)]},1,n);
end
assert(numel(cfg.depthProfiles)==n,'Calligraphy:Depth','深度曲线数量与笔画数不匹配。');
pieces=struct('kind',{},'strokeId',{},'segmentId',{},'phase',{},'p0',{},'p1',{},'duration',{},'timeLaw',{});
previous=[];
[safeBase,safePaper]=calligraphy_safe_point(cfg); frames=calligraphy_frames(cfg);
for i=1:n
    st=glyph.strokes{i}; first=st.segments(1).B(1,:); last=st.segments(end).B(4,:);
    profile=cfg.depthProfiles{i}; above=[first cfg.liftHeight];
    if i==1
        % 先在安全点高度水平移动，再垂直到第一笔起点上方。
        aboveBase=above*frames.paperR.'+frames.origin;
        transitBase=[aboveBase(1:2),safeBase(3)];
        transitPaper=(transitBase-frames.origin)*frames.paperR;
        addLine(safePaper,transitPaper,'safe_travel',cfg.travelSpeed,cfg.cartAcceleration);
        addLine(transitPaper,above,'safe_lower',cfg.verticalSpeed,cfg.verticalAcceleration);
    end
    if ~isempty(previous), addLine(previous,above,'travel',cfg.travelSpeed,cfg.cartAcceleration); end
    addLine(above,[first -profile(1,2)],'lower',cfg.verticalSpeed,cfg.verticalAcceleration);
    % 保留整笔阶段；plan_calligraphy 将附加局部非零速五次时间律。
    pieces(end+1)=struct('kind','stroke','strokeId',i,'segmentId',0, ...
        'phase','write','p0',[],'p1',[],'duration', ...
        duration(st.length,cfg.writeSpeed,cfg.cartAcceleration,cfg.Ts),'timeLaw',[]); %#ok<AGROW>
    addLine([last -profile(end,2)],[last cfg.liftHeight],'lift',cfg.verticalSpeed,cfg.verticalAcceleration);
    previous=[last cfg.liftHeight];
end
% 最后一笔抬笔后先竖直上升，再在安全高度返回同一起止点。
lastBase=previous*frames.paperR.'+frames.origin;
transitBase=[lastBase(1:2),safeBase(3)];
transitPaper=(transitBase-frames.origin)*frames.paperR;
addLine(previous,transitPaper,'safe_lift',cfg.verticalSpeed,cfg.verticalAcceleration);
addLine(transitPaper,safePaper,'safe_travel',cfg.travelSpeed,cfg.cartAcceleration);

    function addLine(a,b,phase,v,aMax)
        L=norm(b-a); if L<1e-10, return; end
        strokeId=i;
        if startsWith(phase,'safe_'), strokeId=0; end
        pieces(end+1)=struct('kind','line','strokeId',strokeId,'segmentId',0,'phase',phase, ...
            'p0',a,'p1',b,'duration',duration(L,v,aMax,cfg.Ts),'timeLaw',[]);
    end
end

function T=duration(L,v,a,Ts)
T=max([1.875*L/v,sqrt(5.773502692*L/a),10*Ts]);
T=ceil(T/Ts)*Ts;
end
