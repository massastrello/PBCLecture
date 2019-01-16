close all
clear all
%
k = 1;
b = 0;
x0 = [-0.9;0];
Tsim = 12;
%
opts = odeset('RelTol',1e-3,'AbsTol',1e-4);
% Autonomous

alpha = 0;
[t1,x1] = ode45(@(t,x) lsys(t,x,k,b,alpha),[0,Tsim],x0,opts);
%
V1 = 0.5.*(k*x1(:,1).^2 + x1(:,2).^2); 
dV1dt = k.*x1(:,1).*x1(:,2) + x1(:,2).*(-k.*x1(:,1)-(b+alpha).*x1(:,2));

alpha = 4;
[t2,x2] = ode45(@(t,x) lsys(t,x,k,b,alpha),[0,Tsim],x0,opts);
%
V2 = 0.5.*(k*x2(:,1).^2 + x2(:,2).^2); 
dV2dt = k.*x2(:,1).*x2(:,2) + x2(:,2).*(-k.*x2(:,1)-(b+alpha).*x2(:,2));
%
alpha = .5;
[t3,x3] = ode45(@(t,x) lsys(t,x,k,b,alpha),[0,Tsim],x0,opts);
%
V3 = 0.5.*(k*x3(:,1).^2 + x3(:,2).^2); 
dV3dt = k.*x3(:,1).*x3(:,2) + x3(:,2).*(-k.*x3(:,1)-(b+alpha).*x3(:,2));


%
xx = linspace(-1,1,50);
[X,Y] = meshgrid(xx,xx);
% Autonomous system
H = (Y.^2)./2 + k.*(X.^2)./2;


figure()
subplot(131)
[~,h] = contourf(X,Y,H);
set(h,'linestyle','none');
hold on
plot(x1(:,1),x1(:,2),'k','LineWidth',1.5)
plot(0,0,'ok')
xlabel('$\xi$','Interpreter','latex')
ylabel('$\dot{\xi}$','Interpreter','latex')
subplot(132)
[~,h] = contourf(X,Y,H);
set(h,'linestyle','none');
hold on
plot(x3(:,1),x3(:,2),'k','LineWidth',1.5)
plot(0,0,'ok')
xlabel('$\xi$','Interpreter','latex')
ylabel('$\dot{\xi}$','Interpreter','latex')
subplot(133)
[~,h] = contourf(X,Y,H);
set(h,'linestyle','none');
hold on
plot(x2(:,1),x2(:,2),'k','LineWidth',1.5)
plot(0,0,'ok')
xlabel('$\xi$','Interpreter','latex')
ylabel('$\dot{\xi}$','Interpreter','latex')
%
figure
subplot(311)
plot(t1,V1,'k','LineWidth',2)
hold on
plot(t1,dV1dt,':g','LineWidth',2)
xlabel('$t$','Interpreter','latex')
ylabel('$\mathcal{H}(x(t)),~\dot{\mathcal{H}}(x(t))$','Interpreter','latex')
leg = legend('$\mathcal{H}(x(t))$','$\dot{\mathcal{H}}(x(t))$');
set(leg,'Interpreter','latex');
title('$\alpha = 0$','Interpreter','latex')
%
subplot(312)
plot(t3,V3,'k','LineWidth',2)
hold on
plot(t3,dV3dt,':g','LineWidth',2)
xlabel('$t$','Interpreter','latex')
ylabel('$\mathcal{H}(x(t)),~\dot{\mathcal{H}}(x(t))$','Interpreter','latex')
%leg = legend('$\mathcal{V}(x(t))$','$\dot{\mathcal{V}}(x(t))$');
%set(leg,'Interpreter','latex');
title('$\alpha = 0.5$','Interpreter','latex')
%
%
subplot(313)
plot(t2,V2,'k','LineWidth',2)
hold on
plot(t2,dV2dt,':g','LineWidth',2)
xlabel('$t$','Interpreter','latex')
ylabel('$\mathcal{H}(x(t)),~\dot{\mathcal{H}}(x(t))$','Interpreter','latex')
%leg = legend('$\mathcal{H}(x(t))$','$\dot{\mathcal{H}}(x(t))$');
%set(leg,'Interpreter','latex');
title('$\alpha = 4$','Interpreter','latex')





function dxdt = lsys(t,x,k,b,alpha)
dxdt = [x(2);-k*x(1)-(b+alpha)*x(2)];
end