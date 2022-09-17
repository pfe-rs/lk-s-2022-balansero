from asyncio import constants
from tkinter import Variable
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
import scipy.integrate as integrate

data = np.random.randint(0, 10, size=(100000, 2))

pravila = [
    ["Z","N1","P1","N2","P2"],
    ["N1","N2","Z","N2","P1"],
    ["P1","Z","P2","N1","P2"],
    ["N2","N2","Z","N2","Z"],
    ["P2","Z","P2","Z","P2"]
]

out_ind = ["N2", "N1", "Z", "P1", "P2"]

def brdoD(p,k,m,x):
    if x < p:
        return 0
    if x > k:
        return m
    else:
        return (x-p)*m/(k-p)

def brdoL(p,k,m,x):
    if x < p:
        return m
    if x > k:
        return 0
    else:
        return (p-x)*m/(k-p)+m

def planina(p, s, k, m, x):
    if x < p or x > k:
        return 0
    elif x >= p and x < s:
        return (m / (s - p)) * (x - p)
    else:
        return (-m / (k - s)) * (x - k)


def trapez(p,s,k,m,h):
    if h == 0:
        return 0
    a=k-p
    b= a*(m-h)/m
    od = h*(k-s)/m #odsečak desnog trougla
    d = math.sqrt(h**2 + od**2)
    ol = h*(s-p)/m #odsečak levog trougla
    c = math.sqrt(ol**2 + h**2)
    f=p
    print(a,b,c,d,f)
    return centarMase(a, b, c, d, f) #f pomeraj


def centarMase(a,b,c,d,f):
    x = a/2 + (2*a+b)*(c**2-d**2)/(6*(b**2-a**2)) +f
    return x

def centarSvihMasa(MU,X):
    p=0
    q=0
    for i in range(5):
        p+=MU[i]*X[i]
        q+=X[i]
    if q==0:
        return 0
    else:
        return p/q


def N2planina(h):
    return trapez(-1,-0.6,-0.3,1, h)

def N1planina(h):
    return trapez(-0.6,-0.3,0,1,h)

def Zplanina(h):
    return trapez(-0.3,0,0.3,1,h)

def P1planina(h):
    return trapez(0,0.3,0.6,1,h)

def P2planina(h):
    return trapez(0.3,0.6,1,1,h)
 


def sporaBrzina(x): #spora
    return planina(-5, 0, 5, 1, x)

def umerenaBrzinaD(x): #umerena
    return planina(3, 6, 10, 1, x)
def brzaBrzinaD(x): #brza
    return brdoD(7, 15, 1, x)

def umerenaBrzinaL(x): #umerena
    return planina(-10, -6, -3, 1, x)
def brzaBrzinaL(x): #brza
    return brdoL(-15, -7, 1, x)


def maliUgao(x): #mali ugao
        return planina(-5, 0, 5, 1, x)

def srednjiUgaoD(x): #srednji ugao
        return planina(3, 12, 27, 1, x)
def velikiUgaoD(x): #veliki ugao
        return brdoD(20, 40, 1, x)

def srednjiUgaoL(x): #medium ugao
        return planina(-27, -12, -3, 1, x)
def velikiUgaoL(x): #veliki ugao
        return brdoL(-40, -20, 1, x)




def centroid(*points):
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    _len = len(points)
    centroid_x = sum(x_coords)/_len
    centroid_y = sum(y_coords)/_len
    return [centroid_x, centroid_y]



def plotujUgao():

    xpoints = np.arange(-50,50,0.01)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(maliUgao(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(srednjiUgaoD(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(velikiUgaoD(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(srednjiUgaoL(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(velikiUgaoL(xpoints[i]))
    plt.plot(xpoints,ypoints)
    plt.show()

def plotujBrzina():

    xpoints = np.arange(-20,20,0.01)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(sporaBrzina(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(umerenaBrzinaD(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(umerenaBrzinaL(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(brzaBrzinaD(xpoints[i]))
    plt.plot(xpoints,ypoints)
    ypoints = []
    for i in range(len(xpoints)):
        ypoints.append(brzaBrzinaL(xpoints[i]))
    plt.plot(xpoints,ypoints)
    plt.show()

theta =1
omega = -20


MU = [0] * 5
X = [0] * 5

for i in range(5):
    if i == 0:
        ugao_rez = maliUgao(theta)
    elif i==1:
        ugao_rez=srednjiUgaoL(theta)
    elif i==2:
        ugao_rez=srednjiUgaoD(theta)
    elif i==3:
        ugao_rez=velikiUgaoL(theta)
    elif i==4:
        ugao_rez=velikiUgaoD(theta)

    for j in range(5):
        if j == 0:
            omega_rez = sporaBrzina(omega)
        elif j == 1:
            omega_rez = umerenaBrzinaL(omega)
        elif j == 2:
            omega_rez = umerenaBrzinaD(omega)
        elif j == 3:
            omega_rez = brzaBrzinaL(omega)
        elif j == 4:
            omega_rez = brzaBrzinaD(omega)

        print(ugao_rez,omega_rez)
        mi = min(omega_rez, ugao_rez)
        mu_index = out_ind.index(pravila[i][j])

        MU[mu_index] = max(MU[mu_index], mi)

X = [N2planina(MU[0]), N1planina(MU[1]),Zplanina(MU[2]),P1planina(MU[3]),P2planina(MU[4])]

#plotujBrzina()
#plotujUgao()
print(MU)
print(X)
print(centarSvihMasa(MU,X))

