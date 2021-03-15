function SlidingCart
    function draw_tank(x,color)
        M = [-1 4 5 5 4 -1 -1 -1 0 0 -1 1 0 0  -1 1 0 0 3 3 3;      % Body
            -2 -2 -1 1 2 2 -2 -2 -2 -3 -3 -3 -3 3 3 3 3 2 2 3 -3;
            ones(1,21)];
        Rav = [-1 1; 0 0; 1 1];     %Front wheel
        R = [cos(x(3)), -sin(x(3)), x(1); sin(x(3)), cos(x(3)), x(2); 0 0 1];
        M = R*M;
        plot(M(1,:),M(2,:),color,'LineWidth',2);
    end
    function xdot = f(x,u)
        %% Kinematic model
        xdot = [x(4)*cos(x(3));
            x(4)*sin(x(3));
            u(1);
            u(2)];
    end
    function u = ClassicalController(x,w,dw,ddw)
        %% Classical Controller
        A = [-x(4)*sin(x(3)) cos(x(3));
            x(4)*cos(x(3))   sin(x(3))];
        y = [x(1);x(2)];
        dy = [x(4)*cos(x(3));x(4)*sin(x(3))];
        ksi = 0.7;
        wn = 5;
        Kp = wn^2;
        Kd = 2*ksi*wn ;
        ddy = Kp*(w-y)+Kd*(dw-dy)+ddw;
        u = A\ddy;
    end

    function u = SlidingModeControl(x,w,dw)
        %% Sliding Mode Controller
        K = 100;
        A = [-x(4)*sin(x(3)) cos(x(3));
            x(4)*cos(x(3))   sin(x(3))];
        y = [x(1);x(2)];
        dy = [x(4)*cos(x(3));x(4)*sin(x(3))];
        ddy = K*sign((w-y)+(dw-dy));
        u = A\ddy;
    end
%initialisation;
x = [10;0;1;1];
dt = 0.01;
L = 20;
l1 = 1;
l2 = 2;

h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
%filename = 'SlidingCarAnim_Classic.gif';
filename = 'SlidingCarAnim_SMC.gif';
ts = 0.1;
n = 1;

for t = 0:dt:10
    %% SIMULATION
    clf; axis([-30 30 -30 30]); axis square; hold on;
    s = 0:0.01:50;
    p = L*[cos(l1*s); sin(l2*s)];
    plot(p(1,:),p(2,:),'m--');
    
    w = L*[l1*cos(t); sin(l2*t)];
    dw = L*[-l1*sin(l1*t); l2*cos(l2*t)];
%     ddw = L*[-l1^2*cos(l1*t); -l2^2*sin(l2*t)];
% 
%     u = ClassicalController(x,w,dw,ddw);        % Classical Control

    u = SlidingModeControl(x,w,dw);              % Sliding Mode Control
    
    plot(w(1),w(2),'o blue','LineWidth',3)
    
    x = x + dt*f(x,u);
    draw_tank(x,'red')
    drawnow();
    if mod(t,ts) == 0
        %% Save images in GIF
        frame = getframe(h);
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