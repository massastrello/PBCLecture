close all
clear all
%
k = 1;
b = 0.5;
x0 = [-0.9;0];
Tsim = 40;
%
opts = odeset('RelTol',1e-3,'AbsTol',1e-4);
% Autonomous
[t,x] = ode45(@(t,x) lsys(t,x,k,b),[0,Tsim],x0,opts);
%
V = 0.5.*(k*x(:,1).^2 + x(:,2).^2); 
dVdt = k.*x(:,1).*x(:,2) + x(:,2).*(-k.*x(:,1)-b.*x(:,2));

figure
%
subplot(211)
plot(t,x,'LineWidth',2)
xlabel('$t$','Interpreter','latex')
ylabel('$x$','Interpreter','latex')
leg = legend('$\xi$','$\dot{\xi}$');
set(leg,'Interpreter','latex');
%
subplot(212)
plot(t,V,'k','LineWidth',2)
hold on
plot(t,dVdt,':g','LineWidth',2)
xlabel('$t$','Interpreter','latex')
ylabel('$\mathcal{H}(x(t)),~\dot{\mathcal{H}}(x(t))$','Interpreter','latex')
leg = legend('$\mathcal{H}(x(t))$','$\dot{\mathcal{H}}(x(t))$');
set(leg,'Interpreter','latex');





function dxdt = lsys(t,x,k,b)
dxdt = [x(2);-k*x(1)-b*x(2)];
end