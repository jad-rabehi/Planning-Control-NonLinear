from roblib import *
import numpy as np


fig = figure()
ax = Axes3D(fig)

ech = 30

def draw_quadri(X):
    ax.clear()
    ax.set_xlim3d(-ech,ech)
    ax.set_ylim3d(-ech,ech)
    ax.set_zlim3d(0,1*ech)
    l = 1
    draw_quadrotor3D(ax,X,a,5*l)


def f(X,W):
    #B = array([[b,b,b,b],[-b*l,0,b*l,0],[0,-b*l,0,b*l],[-d,d,-d,d]])
    X = X.flatten()
    x,y,z,phi,theta,psi = list(X[0:6])
    vr = (X[6:9]).reshape(3,1)
    wr = (X[9:12]).reshape(3,1)
    W2 = W * np.absolute(W)
    tau = B @ W2.flatten()
    E = eulermat(phi,theta,psi)
    dvr = -adjoint(wr) @ vr + inv(E) @ array([[0],[0],[g]]) + array([[0],[0],[-tau[0]/m]])
    dp = E @ vr
    dangles = eulerderivative(phi,theta,psi) @ wr
    dwr = inv(I) @ (-adjoint(wr) @ I @ wr + tau[1:4].reshape(3,1))
    dX = vstack((dp,dangles,dvr,dwr))
    #dx = array([[0,0,0, 0,0,0, 0,0,0, 0,0,0]]).T
    return dX


def control_test(x):
    return array([[6],[5],[5],[5]])

def Van_der_Pol(x):
    x = x.flatten()
    vdp0 = x[1]
    vdp1 = -(0.001*(x[0]**2)-1)*x[1]-x[0]
    dx = array([[vdp0],[vdp1]])
    return dx


def control(X):
    X = X.flatten()
    x,y,z,phi,theta,psi = list(X[0:6])
    vr = (X[6:9]).reshape(3,1)
    wr = (X[9:12]).reshape(3,1)

    E = eulermat(phi,theta,psi)
    dp = E @ vr

    zd = -10  
    vd = 10
    fd = Van_der_Pol(array([[x],[y]]))

    taud0 = 300*tanh(z-zd) + 60*vr[2]

    phid =  0.5*tanh(10*sawtooth(angle(fd)-angle(dp)))
    vdif = vd-vr[0]
    vdiff = np.array(vdif, dtype=float)
    #print("vdif=", vdiff)
    #print("type vdif=", type(vdiff))
    #thetad =  -0.3*(exp(vdiff)-exp(-vdiff))/(exp(vdiff)+exp(-vdiff))
    #thetad =  -0.3*tanh(vd-vr[0])
    thetad =  -0.3*tanh(vdiff)
    psid = angle(dp)

    # inverse of block c
    wrd = 5*inv(eulerderivative(phi,theta,psi)) @ array([[sawtooth(phid-phi)], [sawtooth(thetad-theta)], [sawtooth(psid-psi)]])

    # inverse of block b
    taud13 = I @ ( 100*(wrd-wr) + adjoint(wr) @ I @ wr) 

    # inverse of block a
    
    W2 = invB @ vstack(([taud0],taud13))

    absW2 = np.absolute(W2)
    absW2f = np.array(absW2, dtype=float)
    sqabsW2f = np.sqrt(absW2f) 
    w = sqabsW2f * np.sign(W2)          
    #w = sqrt(abs(W2)) * sign(W2)           #Problem here with sqrt
    return w




x = array([[0,0,-5, 1,0,0, 2,-20,0, 0,0,0]]).T # x,y,z...
a = array([[0,0,0,0]]).T    #angles of the blades  
dt = 0.01

I = array([[10,0,0],[0,10,0],[0,0,20]])
invI = inv(I)
m,g,b,d,l = 10,9.81,2,1,1
B = array([[b,b,b,b],[-b*l,0,b*l,0],[0,-b*l,0,b*l],[-d,d,-d,d]])
invB = inv(B)

for t in arange(0,10,dt):
    w = control(x)
    x = x + dt*f(x,w)
    a = a + 50*dt*w
    draw_quadri(x)
    pause(0.0001)
pause(1)
     
