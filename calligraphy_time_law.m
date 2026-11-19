function law = calligraphy_time_law(law)
% 前后向速度包络 + 共享节点加速度的最小 jerk 五次 Hermite 连接。
x=law.distance; v=law.speedCap; cap=law.accelerationCap;
v([1 end])=0;
for k=2:numel(x)
    v(k)=min(v(k),sqrt(v(k-1)^2+min(cap(k-1:k))*(x(k)-x(k-1))));
end
for k=numel(x)-1:-1:1
    v(k)=min(v(k),sqrt(v(k+1)^2+min(cap(k:k+1))*(x(k+1)-x(k))));
end
dt=2*diff(x)./(v(1:end-1)+v(2:end));
assert(all(isfinite(dt) & dt>0),'Calligraphy:Timing','局部速度包络无法生成有效时间律。');
law.time=[0;cumsum(dt)]; law.velocity=v;
% 固定位置、速度、时长，最小化积分 jerk²，求共享节点加速度。
% 每段仅关联相邻两个加速度，形成稀疏带状系统，无需优化工具箱。
n=numel(x); H=spalloc(n,n,3*n); rhs=zeros(n,1);
G=[36 72 120;72 192 360;120 360 720]; M=[-1.5 .5;1.5 -1;-.5 .5];
base=coefficients(x,v,zeros(n,1),dt);
for k=1:n-1
    ix=k:k+1; A=M*dt(k)^2;
    H(ix,ix)=H(ix,ix)+(A.'*G*A)/dt(k)^5;
    rhs(ix)=rhs(ix)+(A.'*G*base(k,4:6).')/dt(k)^5;
end
a=zeros(n,1); interior=2:n-1;
if ~isempty(interior), a(interior)=-(H(interior,interior)\rhs(interior)); end
% 极值/匀速点取零；单调加减速区保留非零共享加速度。
secant=diff(v)./dt;
for k=2:n-1
    neighbors=secant(k-1:k);
    if prod(neighbors)<=0 || a(k)*neighbors(1)<=0
        a(k)=0;
    else
        limit=min(.95*cap(k),1.8*min(abs(neighbors)));
        a(k)=sign(a(k))*min(abs(a(k)),limit);
    end
end
% 检查多项式内部极值，只回退有问题节点，保持连接处共享加速度。
for iteration=1:24
    c=coefficients(x,v,a,dt); bad=false(n-1,1);
    for k=1:n-1
        cv=c(k,:);
        rootsA=realRoots([20*cv(6),12*cv(5),6*cv(4),2*cv(3)]);
        speed=polyval([5*cv(6),4*cv(5),3*cv(4),2*cv(3),cv(2)],[0;1;rootsA])/dt(k);
        rootsJ=realRoots([60*cv(6),24*cv(5),6*cv(4)]);
        accel=polyval([20*cv(6),12*cv(5),6*cv(4),2*cv(3)],[0;1;rootsJ])/dt(k)^2;
        bad(k)=min(speed)<-1e-8 || max(speed)>max(v(k:k+1))+1e-7 || ...
            max(abs(accel))>min(cap(k:k+1))*(1+1e-7);
    end
    if ~any(bad), break; end
    nodes=unique([find(bad);find(bad)+1]); a(nodes)=a(nodes)*.5;
    if iteration==23, a(nodes)=0; end
end
assert(~any(bad),'Calligraphy:Timing','共享加速度五次时间律未通过单调性/极值检查。');
law.acceleration=a; law.coefficients=c;
end

function c=coefficients(x,v,a,dt)
d=diff(x); v0=v(1:end-1).*dt; v1=v(2:end).*dt;
a0=a(1:end-1).*dt.^2; a1=a(2:end).*dt.^2;
c=[x(1:end-1),v0,a0/2,10*d-6*v0-4*v1-1.5*a0+.5*a1, ...
    -15*d+8*v0+7*v1+1.5*a0-a1,6*d-3*v0-3*v1-.5*a0+.5*a1];
end
function r=realRoots(c)
r=roots(c); r=real(r(abs(imag(r))<1e-9 & real(r)>0 & real(r)<1));
end
