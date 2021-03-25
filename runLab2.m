results = runtests('tests');
assert(all([results.Passed]));

% Robot parameters
param = robotParameters();

% Simulation parameters
dt  = 0.05;         % Evaluation time interval (simulation may internally use smaller steps) [s]
T   = 15;         	% Total simulation time [s]
t   = 0:dt:T;       % Specify times the output is returned
options = odeset('MaxStep',0.005);

% Set initial conditions
x0 = robotInitialConditions(param);

% Run the simulation
func = @(t,x) robotDynamics(t,x,inputTorque(t,x),param);
[t,x] = ode45(func,t,x0,options);
t = t.';
x = x.';

%% Plot history
N   = x(3,:);
E   = x(4,:);
psi = x(5,:);
u   = nan(1,length(t));
v   = nan(1,length(t));
r   = nan(1,length(t));
c   = nan(1,length(t));
H   = nan(1,length(t));
uL  = nan(1,length(t));
uR  = nan(1,length(t));
for i = 1:length(t)
    nu3 = robotVelocity(x(:,i),param);
    u(i) = nu3(1);
    v(i) = nu3(2);
    r(i) = nu3(3);
    Bc = robotConstraints(param);
    c(i) = Bc.'*nu3;
    H(i) = robotEnergy(x(:,i),param);
    tau = inputTorque(t(i),param);
    uL(i) = tau(1);
    uR(i) = tau(2);
end

figure(1);clf

subplot(3,3,1)
plot(t,N)
grid on
title('North position')
ylabel('N [m]')
subplot(3,3,4)
plot(t,E)
grid on
title('East position')
ylabel('E [m]')
subplot(3,3,7)
plot(t,psi*180/pi)
grid on
title('Yaw angle')
xlabel('Time [s]')

ylabel('r [\circ]')
subplot(3,3,2)
plot(t,u)
grid on
title('Surge velocity')
ylabel('u [m/s]')
subplot(3,3,5)
plot(t,v)
grid on
title('Sway velocity')
ylabel('v [m/s]')
subplot(3,3,8)
plot(t,r*180/pi)
grid on
title('Yaw angular velocity')
ylabel('r [\circ/s]')
xlabel('Time [s]')

subplot(4,3,3)
plot(t,uL)
grid on
title('Left wheel torque')
ylabel('\tau [N.m]')
subplot(4,3,6)
plot(t,uR)
grid on
title('Right wheel torque')
ylabel('\tau [N.m]')
subplot(4,3,9)
plot(t,H)
grid on
title('Total energy')
ylabel('H [J]')
subplot(4,3,12)
plot(t,c)
grid on
title('Nonholonomic constraint violation')
ylabel('B_c^T \nu [-]')
xlabel('Time [s]')

%% Animation
fig     = 2;
hf      = figure(fig); clf(fig);
hf.Color = 'w';
ax      = axes(hf,'FontSize',14);
hold(ax,'on');

hLt     = plot(ax,nan(size(E)),nan(size(N)),'r');
hRt     = plot(ax,nan(size(E)),nan(size(N)),'g');
hSt     = plot(ax,nan(size(E)),nan(size(N)),'b');
hP      = plot(ax,nan,nan,'k-');
hL      = plot(ax,0,0,'r.');
hR      = plot(ax,0,0,'g.');
hS      = plot(ax,0,0,'b.');
hB      = plot(ax,0,0,'k.');
hC      = plot(ax,0,0,'k.');
tL      = text(ax,0,0,' L','FontSize',10,'Color','r');
tR      = text(ax,0,0,' R','FontSize',10,'Color','g');
tS      = text(ax,0,0,' S','FontSize',10,'Color','b');
tB      = text(ax,0,0,' B','FontSize',10,'Color','k');
tC      = text(ax,0,0,' C','FontSize',10,'Color','k');

hold(ax,'off');
axis(ax,'equal');
axis(ax,[min(E)-0.5,max(E)+0.5,min(N)-0.5,max(N)+0.5]);
grid(ax,'on');
xlabel(ax,'East [m]');
ylabel(ax,'North [m]');

a = 0.15;
r = 0.2;
theta = linspace(3*pi/2,pi/2,50);
rPCb = [ [0;r;0], [a;r;0], [a;-r;0], [0;-r;0], r*[cos(theta);sin(theta);zeros(size(theta))] ];
Se3     = skew([0;0;1]);

rBNn = nan(3,length(t));
rCNn = nan(3,length(t));
rLNn = nan(3,length(t));
rRNn = nan(3,length(t));
rSNn = nan(3,length(t));
for i = 1:length(t)
    Rnb = expm(psi(i)*Se3);
    rBNn(:,i) = [N(i); E(i); 0];
    rCNn(:,i) = rBNn(:,i) + Rnb*param.rCBb;
    rLNn(:,i) = rBNn(:,i) + Rnb*param.rLBb;
    rRNn(:,i) = rBNn(:,i) + Rnb*param.rRBb;
    rSNn(:,i) = rBNn(:,i) + Rnb*param.rSBb;

    rPNn = rCNn(:,i) + Rnb*rPCb;
    hP.XData = rPNn(2,:);
    hP.YData = rPNn(1,:);
    
    hLt.XData = rLNn(2,:);
    hLt.YData = rLNn(1,:);
    hRt.XData = rRNn(2,:);
    hRt.YData = rRNn(1,:);
    hSt.XData = rSNn(2,:);
    hSt.YData = rSNn(1,:);
    
    hL.XData = rLNn(2,i);
    hL.YData = rLNn(1,i);
    tL.Position = [rLNn(2,i),rLNn(1,i),0];
    hR.XData = rRNn(2,i);
    hR.YData = rRNn(1,i);
    tR.Position = [rRNn(2,i),rRNn(1,i),0];
    hS.XData = rSNn(2,i);
    hS.YData = rSNn(1,i);
    tS.Position = [rSNn(2,i),rSNn(1,i),0];
    hB.XData = rBNn(2,i);
    hB.YData = rBNn(1,i);
    tB.Position = [rBNn(2,i),rBNn(1,i),0];
    hC.XData = rCNn(2,i);
    hC.YData = rCNn(1,i);
    tC.Position = [rCNn(2,i),rCNn(1,i),0];
    pause(dt);
    drawnow
end



