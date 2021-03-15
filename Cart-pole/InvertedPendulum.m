function InvertedPendulum()
    function v = f1(x,u)
        %% Nonlinear model for Cart-Pole
        v = [x(3); 
            x(4);
            (m*sin(x(2))*(g*cos(x(2))-l*x(4)^2)+u)/(M+m*sin(x(2))^2);
            (sin(x(2))*((M+m)*g-m*l*x(4)^2*cos(x(2)))+u*cos(x(2)))/(l*(M+m*sin(x(2))^2))];
    end


    function drawPendulum(x)
        %% Draw Cart-Pole
        clf();
        hold on;
        axis('off');
        axis([-3 3 -3 3]);
        plot([0, 0], [0,1.5*l*5],'k-.');
        plot([x(1), x(1)-4*l*sin(x(2))], [0,4*l*cos(x(2))],'blue','LineWidth',2);
        plot(x(1)+[-.5, .5, .5, -.5, -.5], [0,0,-.25,-.25,0],'black','lineWidth',4);
        plot(x(1)-4*l*sin(x(2)), 4*l*cos(x(2)),'ogreen','LineWidth',10);        
        drawnow();
    end

   

    function us = saturatedControl(u) 
        %% Saturation but not need in simulation
        if u > umax || u < -umax
            us = umax*sign(u);
        else
            us = u;
        end        
    end

    % Parameters    
    m = 0.01; M=0.5; l = 0.413; g = 9.81; J = m * l^2;
    
    % Local controller design  "Stabilize Pole locally"
    A = [0 0 1 0;
         0 0 0 1;
         0 m*g/M 0 0;
         0 (M+m)*g/(l*M) 0 0];
    B = [0; 0; 1/M; 1/(l*M)];        
    %P = [-10 -7 -5+1i  -5-1i];  
    P = [-10 -7 -5  -9];
    Kl = place(A,B,P);
    
    % Global Energie-based controller  "Swining up Pole"
    Emax = 2*m*l*g;
    Ke = 200;
    Theta_local = 0.3;
    
    % Cart PD controller  "Regulate cart position and velocity "    
    ksi = 0.4;
    wn = 8;    
    Kp = wn^2;
    Kd = 2*ksi*wn;
    
    
    umax = 100;
    
    dt = 0.005;
    x = [-0.1;0.1+pi; 0; 0];
    tf = 5;
    
    h = figure;
    axis tight manual % this ensures that getframe() returns a consistent size
    filename = 'PenduliumAnimated2.gif';
    ts = 0.1;
    n = 1;
    for t = 0:dt:tf 
        %% Simulation
        
        if abs(x(2))<Theta_local     % Theta_local desribes the attraction region arround Up-position for linear controller
            ulocal = -Kl*x;
            u = ulocal; 
        else
            E = m*g*l*(1+cos(x(2)))+ (J/2)*(x(4))^2 ;
            v = Ke*(E-Emax)*sign(x(4)*cos(x(2))) - Kd*x(3) - Kp*x(1);                            % PD controller +  Energy based controller
            uglobal = (M+m*(sin(x(2)))^2)*v + m*g*cos(x(2))*sin(x(2)) - m*l*sin(x(2))*x(4)^2;    % Global control based on Collocated Partial Feedback Linearization
            u = saturatedControl(uglobal);                                                       
        end        
                
   
        % Runge-Kutta 4 resolution Method
        alpha1=dt*f1(x,u);
        alpha2=dt*f1(x+(alpha1)/2,u);
        alpha3=dt*f1(x+(alpha2)/2,u);
        alpha4=dt*f1(x+(alpha3),u);
        x=x+(alpha1+2*(alpha2+alpha3)+alpha4)/6;
        
        drawPendulum(x);
        
        
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
