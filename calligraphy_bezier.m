function [p,du,duu] = calligraphy_bezier(B,u)
% B: 4x2 控制点，u: Nx1。
u=u(:); v=1-u;
p=v.^3*B(1,:)+3*v.^2.*u*B(2,:)+3*v.*u.^2*B(3,:)+u.^3*B(4,:);
if nargout>1
    du=3*v.^2*(B(2,:)-B(1,:))+6*v.*u*(B(3,:)-B(2,:))+3*u.^2*(B(4,:)-B(3,:));
end
if nargout>2
    duu=6*v*(B(3,:)-2*B(2,:)+B(1,:))+6*u*(B(4,:)-2*B(3,:)+B(2,:));
end
end
