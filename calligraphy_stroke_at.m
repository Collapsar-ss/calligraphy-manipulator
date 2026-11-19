function [xy,tangent,curvature,segmentId] = calligraphy_stroke_at(st,ell)
% 整笔弧长查询；返回相对弧长的一、二阶导数。
ell=min(st.length,max(0,ell(:))); segmentId=discretize(ell,st.cumulative);
xy=zeros(numel(ell),2); tangent=xy; curvature=xy;
for j=unique(segmentId).'
    ix=segmentId==j; seg=st.segments(j);
    u=interp1(seg.ell,seg.u,ell(ix)-st.cumulative(j),'pchip');
    [p,du,duu]=calligraphy_bezier(seg.B,min(1,max(0,u)));
    g=vecnorm(du,2,2);
    assert(all(g>1e-12),'Calligraphy:Cusp','笔画曲线存在退化切向，无法连续通过。');
    xy(ix,:)=p; tangent(ix,:)=du./g;
    curvature(ix,:)=duu./g.^2-du.*sum(du.*duu,2)./g.^4;
end
end
