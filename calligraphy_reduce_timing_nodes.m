function law = calligraphy_reduce_timing_nodes(law)
% 密集节点用于建立约束；时间多项式保留速度/加速度包络特征点。
x=law.distance; n=numel(x); keep=false(n,1);
keep(unique(round(linspace(1,n,min(4,n)))))=true;
stack=[1 n]; values=log(max([law.speedCap,law.accelerationCap],1e-12));
while ~isempty(stack)
    left=stack(end,1); right=stack(end,2); stack(end,:)=[];
    if right-left<2, continue; end
    ix=(left:right).'; u=(x(ix)-x(left))/(x(right)-x(left));
    estimate=values(left,:)+u.*(values(right,:)-values(left,:));
    [err,local]=max(max(abs(values(ix,:)-estimate),[],2));
    if err>.025
        middle=ix(local);
    elseif x(right)-x(left)>1
        [~,local]=min(abs(x(ix)-(x(left)+x(right))/2)); middle=ix(local);
    else
        continue
    end
    if middle==left || middle==right, continue; end
    keep(middle)=true; stack=[stack;left middle;middle right]; %#ok<AGROW>
end
selected=find(keep); accel=law.accelerationCap(selected);
% 采用区间最小切向加速度上限，避免删点放宽原限制。
for k=1:numel(selected)-1
    limit=min(law.accelerationCap(selected(k):selected(k+1)));
    accel(k:k+1)=min(accel(k:k+1),limit);
end
law.originalNodeCount=n;
law.distance=x(selected); law.speedCap=law.speedCap(selected);
law.referenceSpeed=law.referenceSpeed(selected); law.accelerationCap=accel;
end
