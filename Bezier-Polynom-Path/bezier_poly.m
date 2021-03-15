function bezier_poly

    function draw_tank(x,color,sizeShape)
        line1 = [-1 2 -1 -1];
        line2 = [1 0 -1 1];
        M =  [sizeShape*line1; sizeShape*line2;  ones(1,4)];
        R = [cos(x(3)), -sin(x(3)), x(1); sin(x(3)), cos(x(3)), x(2); 0 0 1];
        M = R*M;
        plot(M(1,:),M(2,:),color,'LineWidth',2);
    end
    
    function xdot = f(x,u)        
        xdot = [x(4)*cos(x(3)); x(4)*sin(x(3));u(1);u(2)];
    end
    
    function u = controller(x,w,dw)
        A = [-x(4)*sin(x(3)) cos(x(3));
            x(4)*cos(x(3))   sin(x(3))];        
        y = [x(1);x(2)];
        dy = [x(4)*cos(x(3));x(4)*sin(x(3))];
        v = (w-y)+2*(dw-dy);
        u = A\v;
    end

    function y = b(i,n,t)        
        if i==0, 
            y = (1-t)^n; 
        end
        if i == n, 
            y = t^n;  
        end
        if (i<n) && (i>0)
            y = prod(1:n)/(prod(1:i)*prod(1:n-i))*((1-t)^(n-i))*(t^i);
        end
    end

    function y = db(i,n,t)
        if i==0, 
            y = -n*(1-t)^(n-1); 
        end
        if i == n, 
            y = n*t^(n-1);  
        end
        if (i<n) && (i>0)
            y = prod(1:n)/(prod(1:i)*prod(1:n-i))*((1-t)^(n-i)*t^(i-1)*i-(n-i)*(1-t)^(n-i-1)*t^i);
        end
    end

    function w = setpoint(t)
        w = 0;
        for i=0:n
            w = w + b(i,n,t)*P(:,i+1); 
        end
    end

    function dw = dsetpoint(t)
        dw = 0;
        for i=0:n
            dw = dw + db(i,n,t)*P(:,i+1);
        end
    end


%initialisation
h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'BezierPolynomPath2.gif';
nf=1;


axis([-1 11 -1 11]); axis square; hold on 
A1 = [2 4 2; 0 2 7];
A2 = [7 8 3; 2 3 10];
fill(A1(1,:),A1(2,:), 'black')
fill(A2(1,:),A2(2,:), 'black')
xf = 8;
yf = 8;
P = [1 1 1 1 2 3 4 5 5 7 8 10 xf;
     1 4 7 9 10 8 6 4 1 0 0 1 yf];
 

n = length(P)-1; 
%plot(P(1,:),P(2,:), 'o red', 'LineWidth', 4);
plot(xf,yf, 'o red', 'LineWidth', 4);
x = [0;0;0;1];
dt = 0.05; tmax = 50;
k = 0;


for t = 0:dt:tmax
    %% SIMULATION
    w = setpoint(t/tmax);
    dw = (1/tmax)*dsetpoint(t/tmax);
    u = controller(x,w,dw);
    x = x + dt*f(x,u);
    if mod(k,50)==0
        draw_tank(x,'magenta',0.2); hold on
        plot(w(1),w(2),'. blue'); hold on
        plot(x(1),x(2),'o magenta');
        drawnow();
        
        frame = getframe(h);      % Capture the plot as an image
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        % Write to the GIF File
        if nf == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append');
        end
        nf = nf+1;
    end
    k = k+1;
end
end