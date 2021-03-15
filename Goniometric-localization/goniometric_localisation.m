function goniometric_localisation(N_Cars)
%--------------------
    function xdot = f(x,u)
        %% Car's kinematic model
        xdot = [x(4)*cos(x(5))*cos(x(3));
            x(4)*cos(x(5))*sin(x(3));
            x(4)*sin(x(5))/3;
            u(1); u(2)];
    end

    function [y, Gbeta, Ck] = g(x,landmarks)
        %% Output measurement model
        Ck = [0,0,1]; y = x(4);  beta = 1;
        for i = 1:length(landmarks),
            a = landmarks(:,i);
            plot(a(1),a(2),'oblack');
            da = a-x(1:2);
            delta = atan2(da(2),da(1))-x(3);
            if norm(da)<25
                yi =-a(1)*sin(x(3)+delta) +a(2)*cos(x(3)+delta);
                Cki = [-sin(x(3)+delta),cos(x(3)+delta), 0];
                plot(a(1),a(2),'o red','LineWidth',3);
                y = [y;yi];
                Ck = [Ck; Cki];
                beta = [beta,1];
            end
            Gbeta = diag(beta);
            y = y + mvnrnd(zeros(size(y)),Gbeta)';
        end
    end
%%%%% -------------------------------------------------%%%%%
    function OnlyOneCar
        %% One Car Mode
        u = [0;0];
        x=[15; -10; pi/3; 20; 0.1];
        zhat = [0;0;0]; Gz = 10^3*eye(3,3);
        Galpha = dt*0.001*eye(3,3);
        
        h = figure;
        axis tight manual % this ensures that getframe() returns a consistent size
        filename = 'GoniometricLocalization_OneCar.gif';
        ts = 0.1;
        n = 1;
        
        for t = 0:dt:20
            %% Simulation
            clf;     axis([-50, 50, -50, 50]);    hold on;
            [y, Gbeta, Ck] = g(x,landmarks);
            draw_car(x);
            %deltax = x(5); thetax = x(3);
            Ak = [1 0 dt*cos(x(5))*cos(x(3));
                0 1 dt*cos(x(5))*sin(x(3));
                0 0 1];
            uk = [0;0;dt*u(1)];
            [zhat,Gz] = kalman_filter(zhat,Gz,uk,y,Galpha,Gbeta,Ak,Ck);
            draw_ellipse(zhat(1:2),Gz(1:2,1:2),0.9,'green',2);
            alphax = 0*x;
            alphax([1;2;4]) = mvnrnd(zeros(3,1),Galpha);
            x = x +dt*f(x,u)+alphax;
            drawnow()
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

%%%%   -------------------------------------------------  %%%%%

    function [yab, Gab, Cab] = gab(xa,xb)  % Ra see Rb
        dab = xb(1:2)-xa(1:2);
        phi = atan2(dab(2),dab(1))-xa(3);
        yab = [];  Gab = []; Cab = [];
        if norm(dab)<15
            plot([xa(1), xb(1)],[xa(2), xb(2)],'red');
            Cab = [-sin(xa(3)+phi),cos(xa(3)+phi), 0, sin(xa(3)+phi),-cos(xa(3)+phi), 0 ];
            Gab = 1;
            yab = 0+mvnrnd(0,Gab)';
        end
    end

    function [y, Gbeta, Ck] = gtwocars(xa,xb,landmarks)  % Ra see Rb
        [ya,Ga,Cak] = g(xa,landmarks);
        [yb,Gb,Cbk] = g(xb,landmarks);
        [yab, Gab, Cab] = gab(xa,xb) ;
        y = [ya;yb;yab];
        Gbeta = blkdiag(Ga,Gb,Gab);
        Ck = [blkdiag(Cak,Cbk);Cab];
    end

    function  TwoCars
        %% Two Cars Mode
        ua = [0;0];  ub = [0;0];
        xa=[15; -10; pi/3; 20; 0.1];
        xb=[5; -20; pi/3; 15; 0.2];
        zhat = [0;0;0;0;0;0]; Gz = 10^3*eye(6,6);
        Galphaa = dt*diag([0.1,0.1,0.5]); %0.001*eye(3,3);
        Galphab = Galphaa;
        Galpha = blkdiag(Galphaa,Galphab);
        
        h = figure;
        axis tight manual % this ensures that getframe() returns a consistent size
        filename = 'GoniometricLocalization2.gif';
        ts = 0.1;
        n = 1;
        
        for t = 0:dt:20
            %% SIMULATION
            clf;     axis([-50, 50, -50, 50]);    hold on;
            [y, Gbeta, Ck] = gtwocars(xa,xb,landmarks);
            draw_car(xa); draw_car(xb);
            Aak = [1 0 dt*cos(xa(5))*cos(xa(3));
                0 1 dt*cos(xa(5))*sin(xa(3));
                0 0 1];
            uak = [0;0;dt*ua(1)];
            Abk = [1 0 dt*cos(xb(5))*cos(xb(3));
                0 1 dt*cos(xb(5))*sin(xb(3));
                0 0 1];
            ubk = [0;0;dt*ub(1)];
            Ak = blkdiag(Aak,Abk);
            uk = [uak;ubk];
            draw_ellipse(zhat(1:2),Gz(1:2,1:2),0.9,'green',2);
            draw_ellipse(zhat(4:5),Gz(4:5,4:5),0.9,'blue',2);
            [zhat,Gz] = kalman_filter(zhat,Gz,uk,y,Galpha,Gbeta,Ak,Ck);
            
            alphaxa = 0*xa;
            alphaxa([1;2;4]) = mvnrnd(zeros(3,1),Galphaa);
            alphaxb = 0*xb;
            alphaxb([1;2;4]) = mvnrnd(zeros(3,1),Galphab);
            xa = xa +dt*f(xa,ua)+alphaxa;
            xb = xb +dt*f(xb,ub)+alphaxb;
            drawnow()
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
%% -------- Main   ---------%
initialisation;
landmarks = [0 15 -30 15;   25 30 15 -20];
dt = 0.05;
if N_Cars ==1
    OnlyOneCar;
elseif N_Cars ==2
    TwoCars;
else
    disp('Number of Cars have to be in {1,2} !! ')
end

end