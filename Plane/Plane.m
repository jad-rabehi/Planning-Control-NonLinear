function Plane
%% Model-free Altittude Control an Air-craft 

    function draw_plane(x,u)
        uleft = -u(2)+u(3);
        uright = -u(2)-u(3);
        plane0 = [0 0 6 0 0 0 0 1 6 0;
            0 -1 0 1 -1 0 0 0 0 0;
            0 0 0 0 0 0 1 0.2 0 0;
            1 1 1 1 1 1 1 1 1 1 ];
        e = 0.5;
        aileron = [-e 0 0 -e -e;
            -e -e e e -e;
            0 0 0 0 0;
            1 1 1 1 1];
        R = [eulermat(-x(4),-x(5),x(6)),[x(1);x(2);-x(3)]; 0 0 0 1];
        ailLeft = R*[eulermat(0,uleft(1),0),[0;1-e;0];0 0 0 1]*aileron;
        ailRight = R*[eulermat(0,uright(1),0),[0;e-1;0];0 0 0 1]*aileron;
        plane = R*plane0;
        plot3(plane(1,:),plane(2,:),plane(3,:),'blue');
        plot3(ailLeft(1,:),ailLeft(2,:),ailLeft(3,:),'red');
        plot3(ailRight(1,:),ailRight(2,:),ailRight(3,:),'red');
        plot3(plane(1,:),plane(2,:),0*plane(3,:),'Color',[.5 .5 .5]); % shadow
        drawnow();
    end
    function dX = model(X,U)
        v = X(7:9);
        r = X(10:12);
        V = norm(v);
        alpha = atan(X(9)/X(7));       beta = asin(X(8)/V);
        phi = X(4);     theta = X(5);      psi = X(6);
        
        cf = cos(phi);  sf = sin(phi);  cb = cos(beta); sb = sin(beta);
        ca = cos(alpha); sa = sin(alpha);
        ct = cos(theta); st = sin(theta); tt = tan(theta);
        
        Fa = 0.002*V^2*...
            [-ca*cb, ca*sb, sa; sb, cb, 0; -sa*cb, sa*sb, -ca]*...
            [4*(-0.3+10*alpha+10*r(2)/V +2*U(3)+ 0.3*U(2))^2+abs(U(2))+3*abs(U(3));
            -50*beta+ (10*r(3)-3*r(1))/V;                                            % fautl in r(3): corrected
            10+ 500*alpha + 400*r(2)/V + 50*U(3) + 10*U(2)];
        
        g = 9.81;
        
        dX = [eulermat(phi, theta, psi)*v;
            [1 tt*sf tt*cf; 0 cf -sf; 0 sf/ct cf/ct ]*r;
            g*[-st; ct*sf; ct*cf] + Fa + [U(1);0;0] - cross(r,v);
            [-r(3)*r(2)-0.1*V^2*(beta+2*U(3)+(5*r(1)-r(3))/V);
            r(3)*r(1)-0.1*V^2*(0.1+2*alpha-0.2*U(3)+3*U(2)+30*r(2)/V);
            0.1*r(1)*r(2) + 0.1*V^2*(beta + U(3)/2 + 0.5*(r(1)-2*r(3))/V)] ];
        
    end
    function u = control(x)
        phi = x(4);     theta = x(5);      psi = x(6);
        v_norm = norm(x(7:9));
        
        theta_bar = -0.2*atan(0.1*(zbar-x(3)));
        psi_bar = atan2(x(2),x(1))+0.5*pi+atan(0.02*(norm(x(1:2))-rbar));
        
        phi_bar = 0.5*atan(5*atan(tan(0.5*(psi_bar-psi))));
        
        u = [5*(1+2/pi*(atan(vbar-v_norm)));
            -0.3*(2/pi*atan(5*(theta_bar-theta))+abs(sin(phi)) );
            -0.3*(2/pi)*atan(phi_bar-phi)];
    end

    function path_plot(rbar,zbar)
        C0 = [];
        for t1 = 0:0.1:2*pi
            C0 = [C0,[rbar*cos(t1); rbar*sin(t1); -zbar]];
        end
        plot3(C0(1,:),C0(2,:),C0(3,:),'green');
    end




%%   ----  MAIN  ----


h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'PlaneAnimation.gif';
n = 1;

d = 150;
clf;
axis([-d,d,-d,d,-20,100]);
axis square; hold on; grid on

x = zeros(12,1); x(3) = -5; x(7) = 30;
dt = 0.002;
%u = zeros(3,1);

zbar0 = -50;
zbar1 = -80;

zbar = zbar0;
vbar = 15;
rbar = 100;

path_plot(rbar,zbar0);
path_plot(rbar,zbar1);

k = 0;
for t = 0:dt:90
    %% Simulation
    if t>40
        % Change Altitude setpoint
        zbar = zbar1;
    end
    u = control(x);%[5; -0.2; 0];                        
    
    % Runge-Kutta 4 resolution method
    alpha1=dt*model(x,u);
    alpha2=dt*model(x+(alpha1)/2,u);
    alpha3=dt*model(x+(alpha2)/2,u);
    alpha4=dt*model(x+(alpha3),u);
    x=x+(alpha1+2*(alpha2+alpha3)+alpha4)/6;
    
    if (mod(k,500)<1)
        draw_plane(x,u);
        
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
    k = k+1;
end


end