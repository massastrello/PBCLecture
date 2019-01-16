close all 
clear all
%
k = 1;
b = 0.5;
di = 4;
x0 = [-.8,0];
xd = [0.5,0];
kd = 1;
Tsim = 40;

%% PLOT ENERGY FUNCTIONS
xx = linspace(-1,1,50);
[X,Y] = meshgrid(xx,xx);
% Autonomous system
H = (Y.^2)./2 + k.*(X.^2)./2;
figure()
surf(X,Y,H)
xlabel('\xi')
ylabel('\dot{\xi}')
zlabel('\mathcal{H}(x)')

% EB-PBC controlled system
xx1 = linspace(-1,1,50);
[X1,Y1] = meshgrid(xx1,xx1);
Hs = (Y1.^2-xd(2))./2 + kd.*((X1 - xd(1)).^2)./2;
figure()
surf(X1,Y1,Hs)
xlabel('\xi')
ylabel('\dot{\xi}')
zlabel('\mathcal{H}^*(x)')

%% Simulate Autonomous and Controlled Systems
opts = odeset('RelTol',1e-3,'AbsTol',1e-4);
% Autonomous
[tl,xl] = ode45(@(t,x) lsys(t,x,k,b),[0,Tsim],x0,opts);
% EB-PBC controlled system
[tnl,xnl] = ode45(@(t,x) nlsys(t,x,b,di,kd,xd),[0,Tsim],x0,opts);
% plot state-space trajectories
figure()
plot(tl,xl)
hold on
plot(tnl,xnl)
%
figure()
subplot(121)
[~,h] = contourf(X,Y,H);%,[0,.5/8,1/8,1/4,3/8,.5,5/8,6/8,7/8,1,1+1/8]);
set(h,'linestyle','none');
hold on
plot(xl(:,1),xl(:,2),'k','LineWidth',1.5)
plot(0,0,'ok')
subplot(122)
[~,h] = contourf(X1,Y1,Hs);%,[0,.5/8,1/8,1/4,3/8,.5,5/8,6/8,7/8,1,1+1/8]);
set(h,'linestyle','none');
%
hold on
plot(xnl(:,1),xnl(:,2),'k','LineWidth',1.5)
plot(.5,0,'ok')
%colorbar('location','Manual', 'position', [0.93 0.1 0.02 0.81]);
%
%% Save things in .dat
dataH = [ X(:) Y(:) H(:) ];
save H.dat dataH -ASCII
%
dataHe = [ X(:) Y(:) Hs(:) ];
save He.dat dataHe -ASCII

%% Functions Definition
% Linear System
function dxdt = lsys(t,x,k,b)
   dxdt = [x(2);...
          -k*x(1)-b*x(2)]; 
end
% Nonlinear System
function dxdt = nlsys(t,x,b,di,kd,xd)
    dxdt = [x(2);...
           -kd*(x(1)-xd(1))-(b+di)*x(2)];
end
