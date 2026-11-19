function [r,rd,rdd,interval,reference] = calligraphy_time_at(law,t,duration)
scale=duration/law.time(end); query=min(law.time(end),max(0,t(:)/scale));
interval=discretize(query,law.time);
dt=law.time(interval+1)-law.time(interval);
u=(query-law.time(interval))./dt; c=law.coefficients(interval,:);
r=c(:,1)+u.*(c(:,2)+u.*(c(:,3)+u.*(c(:,4)+u.*(c(:,5)+u.*c(:,6)))));
rd=(c(:,2)+u.*(2*c(:,3)+u.*(3*c(:,4)+u.*(4*c(:,5)+u.*5.*c(:,6)))))./(dt*scale);
rdd=(2*c(:,3)+u.*(6*c(:,4)+u.*(12*c(:,5)+u.*20.*c(:,6))))./(dt*scale).^2;
% 理论单调；只消除浮点端点噪声，不遮掩真正的倒退。
assert(all(rd>=-1e-8),'Calligraphy:Timing','局部五次时间律出现进度倒退。');
r=min(law.distance(end),max(0,r)); rd=max(0,rd);
reference=interp1(law.distance,law.referenceSpeed,r,'linear');
end
