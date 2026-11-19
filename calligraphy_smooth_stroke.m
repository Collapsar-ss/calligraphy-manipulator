function st = calligraphy_smooth_stroke(original,cfg)
% 原中心线附近构造 C2 三次样条，按同参数对照限制采样偏差。
spacing=.5;
for iteration=1:10
    knots=[];
    for j=1:numel(original.segments)
        n=max(2,ceil(original.segments(j).length/spacing));
        local=linspace(original.cumulative(j),original.cumulative(j+1),n+1).';
        knots=[knots;local(1:end-1)]; %#ok<AGROW>
    end
    knots=[knots;original.length]; points=calligraphy_stroke_at(original,knots);
    pp=spline(knots,points.');
    probe=sort(unique([knots;(knots(1:end-1)*3+knots(2:end))/4; ...
        (knots(1:end-1)+knots(2:end))/2;(knots(1:end-1)+knots(2:end)*3)/4]));
    delta=vecnorm(ppval(pp,probe).'-calligraphy_stroke_at(original,probe),2,2);
    maxError=max(delta);
    if maxError<=cfg.strokeSmoothingToleranceMm, break; end
    spacing=spacing/2;
end
assert(maxError<=cfg.strokeSmoothingToleranceMm,'Calligraphy:Smoothing','平滑误差超限。');
segments=struct('B',{},'u',{},'ell',{},'length',{});
for j=1:numel(knots)-1
    dt=knots(j+1)-knots(j); c=pp.coefs(2*j-1:2*j,:);
    p0=c(:,4).'; v0=c(:,3).';
    p1=((c(:,1)*dt+c(:,2))*dt+c(:,3))*dt+c(:,4);
    v1=(3*c(:,1)*dt+2*c(:,2))*dt+c(:,3);
    B=[p0;p0+v0*dt/3;p1.'-v1.'*dt/3;p1.'];
    [u,ell]=calligraphy_arc_table(B,cfg.geometryToleranceMm);
    segments(end+1)=struct('B',B,'u',u,'ell',ell,'length',ell(end)); %#ok<AGROW>
end
st.segments=segments; st.cumulative=[0 cumsum([segments.length])];
st.length=st.cumulative(end); st.smoothingErrorMm=maxError;
st.originalSegmentCount=numel(original.segments);
end
