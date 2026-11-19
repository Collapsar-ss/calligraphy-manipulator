function [d,ds,dss] = calligraphy_depth(points,s,maxDepth)
% 六点分段五次插值；每个控制点处一、二阶导数为零，无过冲。
validateattributes(points,{'numeric'},{'size',[6 2],'real','finite'});
assert(abs(points(1,1))<1e-12 && abs(points(end,1)-1)<1e-12 && ...
    all(diff(points(:,1))>=.01-1e-12),'Calligraphy:Depth','六点进度应从 0 到 1，相邻至少间隔 0.01。');
assert(all(points(:,2)>=0 & points(:,2)<=maxDepth),'Calligraphy:Depth','下压深度超出 [0, maxDepth]。');
assert(all(isfinite(s(:))) && all(s(:)>=-1e-10 & s(:)<=1+1e-10),'Calligraphy:Depth','进度超出 [0,1]。');
s=min(1,max(0,s)); d=zeros(size(s)); ds=d; dss=d;
for k=1:5
    ix=s>=points(k,1) & s<=points(k+1,1);
    w=points(k+1,1)-points(k,1); u=(s(ix)-points(k,1))/w;
    delta=points(k+1,2)-points(k,2);
    d(ix)=points(k,2)+delta*(10*u.^3-15*u.^4+6*u.^5);
    ds(ix)=delta*(30*u.^2-60*u.^3+30*u.^4)/w;
    dss(ix)=delta*(60*u-180*u.^2+120*u.^3)/w^2;
end
end
