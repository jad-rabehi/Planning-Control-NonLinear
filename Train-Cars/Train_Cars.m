function Train_Cars

    function draw_tank(x,color)
        M = [-1 4 5 5 4 -1 -1 -1 0 0 -1 1 0 0  -1 1 0 0 3 3 3;      % Body
            -2 -2 -1 1 2 2 -2 -2 -2 -3 -3 -3 -3 3 3 3 3 2 2 3 -3;
            ones(1,21)];
        R = [cos(x(3)), -sin(x(3)), x(1); sin(x(3)), cos(x(3)), x(2); 0 0 1];
        M = R*M;
        plot(M(1,:),M(2,:),color,'LineWidth',2);
    end

    function xdot = f(x,u)
        xdot = [x(4)*cos(x(3));
            x(4)*sin(x(3));
            u(1);
            u(2)];
    end

    function u = controller(x,w,dw)
        A = [-x(4)*sin(x(3)) cos(x(3));
            x(4)*cos(x(3))   sin(x(3))];
        y = [x(1);x(2)];
        dy = [x(4)*cos(x(3));x(4)*sin(x(3))];
        ksi = 1.2;
        wn = 1;
        Kp = wn^2;
        Kd = 2*ksi*wn;
        v = Kp*(w-y)+Kd*(dw-dy);
        u = A\v;
    end

%% ------  Main  ------

clc;
clear all;
close all;

dt = 0.05;
xa = [1;0;1;1];
xb = [-10;-5;1;1];
xc = [-20;0;1;1];
Lx = 40; Ly =25;
s = 0:0.01:2*pi;
l = 10;


h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'TrainCart3.gif';
ts = 1;
n = 1;

for t = 0:dt:40
    %% Simulation
    clf; axis([-50 50 -40 40]); axis square; hold on;
    p = [Lx*cos(s);Ly*sin(s)]; plot(p(1,:),p(2,:),'magenta');
    
    % Controller for the 1st car
    wa = [Lx*cos(0.1*t);Ly*sin(0.1*t)];
    dwa = [-Lx*0.1*sin(0.1*t);Ly*0.1*cos(0.1*t)];
    %ddwa = [-Lx*0.1*0.1*cos(0.1*t);-Ly*0.1*0.1*sin(0.1*t)];
    ua = controller(xa,wa,dwa);
    
    % Controller for the 2nd car
    wb = [xa(1)-l*cos(xa(3)); xa(2)-l*sin(xa(3))];
    dwb = [xa(4)*cos(xa(3))+l*ua(1)*sin(xa(3));
          xa(4)*sin(xa(3))-l*ua(1)*cos(xa(3))];
    ub = controller(xb,wb,dwb);
    
    % Controller for the 3rd car
    wc = [xb(1)-l*cos(xb(3)); xb(2)-l*sin(xb(3))];
    dwc = [xb(4)*cos(xb(3))+l*ub(1)*sin(xb(3));
          xb(4)*sin(xb(3))-l*ub(1)*cos(xb(3))];
    uc = controller(xc,wc,dwc);
    
    plot(wa(1),wa(2),'o red');   
    draw_tank(xa,'red');
    
    plot(wb(1),wb(2),'o blue');   
    draw_tank(xb,'blue');
    
    plot(wc(1),wc(2),'o green');   
    draw_tank(xc,'green');
    
    xa = xa + dt*f(xa,ua);
    xb = xb + dt*f(xb,ub);
    xc = xc + dt*f(xc,uc);
    drawnow();
    
    if mod(t,ts)==0
        %% Save images in GIF
        frame = getframe(h);      % Capture the plot as an image
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        % Write to the GIF File
        if n == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append');
        end
        n = n+1;
    end
end
end