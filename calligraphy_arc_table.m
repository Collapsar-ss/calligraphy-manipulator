function [u,ell] = calligraphy_arc_table(B,tol)
% Simpson 弧长积分，细化至反查询位置误差小于 tol/4。
n=32;
while true
    u=linspace(0,1,n+1).'; um=(u(1:end-1)+u(2:end))/2;
    [~,v]=calligraphy_bezier(B,u); [~,vm]=calligraphy_bezier(B,um);
    speeds=vecnorm(v,2,2); mids=vecnorm(vm,2,2);
    lengths=(speeds(1:end-1)+4*mids+speeds(2:end))/(6*n);
    ell=[0;cumsum(lengths)]; [keep,ix]=unique(ell,'stable');
    if numel(keep)<2, return; end
    inverse=pchip(keep,u(ix)); quarter=(u(1:end-1)+um)/2;
    [~,vq]=calligraphy_bezier(B,quarter);
    half=(speeds(1:end-1)+4*vecnorm(vq,2,2)+mids)/(12*n);
    estimate=ppval(inverse,ell(1:end-1)+half);
    errorMm=max(vecnorm(calligraphy_bezier(B,estimate)-calligraphy_bezier(B,um),2,2));
    if errorMm<=tol/4 && max(lengths)<=.1
        u=u(ix); ell=keep; return
    end
    n=n*2; assert(n<=131072,'Calligraphy:Geometry','弧长查表未达到精度要求。');
end
end
