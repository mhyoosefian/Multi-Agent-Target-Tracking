function plot_confidence_ellipsoid(m,S,p,i,s, tr)
h = chi2inv(p,2);
% ellispoid is: {x | (x-m)'inv(S)(x-m) <= h}
% or: {z+m | norm(S^(-1/2)*z) <= sqrt(h)}
plot_ellipse(1/sqrt(h)*inv(sqrtm(S)),m,i,s, tr);
end



function plot_ellipse(A,x0,i,s, tr)
[~,S,V] = svd(A);
t = linspace(0,2*pi,200);
z = [cos(t); sin(t)]; % points on a circle
x = V/S * z + x0; % transformed points
c = colororder;
fill(x(1,:), x(2,:), c(i,:), ...
'facealpha',tr,...
'linewidth',1,...
'displayname', s)
axis equal
end
