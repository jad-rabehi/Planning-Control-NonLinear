from roblib import *

def draw(ap,aw):
    aw = -aw-ap
    c = 2*array([-sin(ap),cos(ap)])
    plot([0,c[0]],[0,c[1]],'magenta',linewidth=2)
    for i in arange(0,8):
        plot(c[0]+array([0,cos(aw+i*pi/4)]), c[1]+array([0,sin(aw+i*pi/4)]),'blue')
    #pause(0.01)

def syst(x,u):
    x = x.flatten()
    return array([ [x[1]], [a1*sin(x[0])-b1*u], [-a1*sin(x[0])+c1*u]])

def control(x):
    x = x.flatten()
    eta = a1*(c1-b1)
    ax = -b1*eta*cos(x[0])
    bx = eta*sin(x[0])*(a1*cos(x[0])-x[1]*x[1])
    y = c1*x[1]+b1*x[2]
    dy = eta*sin(x[0])
    ddy = eta*cos(x[0])*x[1]
    v = (0-y)+3*(0-dy)+3*(0-ddy)+0
    u = (v-bx)/ax
    return u


a1,b1,c1 = 10,1,2
dt = 0.1
x = array([[1],[0],[1]])
aw = 1
ax = init_figure(-3,3,-3,3)
for t in arange(0,10,dt):
    u = control(x)
    x = x+dt*syst(x,u)
    aw = aw+dt*x[2]
    clear(ax)
    draw(x[0],aw)    

