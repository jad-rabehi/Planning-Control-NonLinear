function draw_ellipse(wbar,Gw,eta,color1,lineWidth1)
    if (exist('lineWidth1')==0)
        lineWidth1 = 1;
    end
    if (exist('color1')==0)
        color1 ='black';
    end
    s = 0:0.01:2*pi;
    w = wbar*ones(size(s)) + sqrtm(-2*log(1-eta)*Gw)*[cos(s);sin(s)];
    plot(w(1,:),w(2,:),color1,'lineWidth',lineWidth1);
end