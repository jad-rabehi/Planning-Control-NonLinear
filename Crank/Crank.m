function Crank
clc;
clear all;

    function drawCircle(c,r,color1,linewidth1)
        %% Draw Path
        if (exist('linewidth1')==0),  linewidth1=1; end
        if (exist('color1')==0),  color1='black'; end
        s = 0:0.01:2*pi;
        wref = c*ones(size(s))+r*[cos(s);sin(s)];
        plot(wref(1,:),wref(2,:),color1,'LineWidth',linewidth1);
    end

    function draw()
             clf();   hold on; 
             axis([-4 8 -4 8]);  axis square;
             drawCircle(c,r,'black',2)
             plot(w(1),w(2),'o red','LineWidth',3);
             plot([0 z(1) y(1)],[0 z(2) y(2)], 'magenta', 'Linewidth',4) 
             plot(y(1),y(2),'o blue','LineWidth',4);
             drawnow();
    end  

x = [-1;1]; dt = 0.05;
L1 = 4; L2 = 5;
c = [2;2]; r = 5;

h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'CrankAnimation.gif';
n = 1;
tanim = 0.2;

for t = 0:dt:10
    %% SIMULATION
    z = L1*[cos(x(1)); sin(x(1))];
    y = z + L2*[cos(x(1)+x(2)); sin(x(1)+x(2))];
    w = c + r*[cos(t); sin(t)];
    dw = r*[-sin(t); cos(t)];
    v = w-y+dw;
    A = [-y(2)  -y(2)+z(2);
          y(1)  y(1)-z(1)];
    u = A\v;
    x = x+u*dt;
    draw();
    if mod(t,tanim)==0
        %% Save images in GIF
        % Capture the plot as an image
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
              