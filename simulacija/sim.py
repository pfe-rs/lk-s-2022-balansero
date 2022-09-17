from matplotlib import pyplot as plt
import pygame
import math
import time

pr_izvod_x = 0
pr_izvod_th = 0

class PID():

    def __init__(self, k_p: float, k_i: float, k_d: float, dt: float):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.dt = dt
        self.prethodni_ulaz = 0
        self.integral = 0

    def izracunaj(self, ulaz: float) -> float:
        self.integral += ulaz * dt
        izvod = (ulaz - self.prethodni_ulaz) / dt
        self.prethodni_ulaz = ulaz
        return self.k_p * ulaz + self.k_i * self.integral + self.k_d * izvod

x = 100
th = 0.2 # rad
M = 0

dt = 0.001

kontroler_x = PID(1, 0, 0, dt)
kontroler_th = PID(-2.0, 0.5, 0.0, dt)

r = 0.05 # m
l = 0.25 # m

def plant(motor_input: float, time_step: float) -> (float, float):
    
    # logika
    global pr_izvod_x
    global pr_izvod_th
    global x
    global th
    global r
    global l

    m1 = 0.2 # kg
    m2 = 3.0 # kg
    g = 9.81 # m/s^2
    M = motor_input # Nm
    
    dr_izvod_x = (M/(m2*l)*(1 + 13/12 * l/r) - g * th) / (1/12 + 39/24 * m1/m2)
    dr_izvod_th = 1/l * (g*th * (1 + 3/2 * m1/m2) - M/(m2*l) * (1 + 3/2 * m1/m2 + l/r)) / (1/12 + 39/24 * m1/m2)
    
    pr_izvod_x += dr_izvod_x * time_step
    pr_izvod_th += dr_izvod_th * time_step

    x += pr_izvod_x * time_step
    th += pr_izvod_th * time_step

    print("x/dx/ddx {:.3f} {:.3f} {:.3f} th/dth/ddth {:.3f} {:.3f} {:.3f} motorinput {:.3f}".format(x, pr_izvod_x, dr_izvod_x, th, pr_izvod_th, dr_izvod_th, moment_sile))
    
    return x, th

broj_iteracija = 50000

xx, thth, time_step = [0.0] * broj_iteracija, [0.0] * broj_iteracija, [0.0] * broj_iteracija
xx0, thth0 = [0.0] * broj_iteracija, [0.0] * broj_iteracija

kraj = 0

successes, failures = pygame.init()
print("{0} successes and {1} failures".format(successes, failures))


screen = pygame.display.set_mode((512, 512))
clock = pygame.time.Clock()
#FPS = 60  # Frames per second.

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

pixels_per_meter = 300

for i in range(broj_iteracija):
    # greska_ugla = kontroler_x.izracunaj(0 - x)
    moment_sile = kontroler_th.izracunaj(0 - th)
    time_step[i] = i * dt
    x, th = plant(moment_sile, dt)

    #pygame
    # clock.tick(FPS)

    ev = pygame.event.get()

    screen.fill(WHITE)
    r_pix = r*pixels_per_meter
    l_pix = l*pixels_per_meter
    circle_centar = (x, 512-r_pix)
    balansero_tip = (x + math.sin(th)*l_pix, 512-r_pix - math.cos(th)*l_pix)
    pygame.draw.circle(screen, RED, circle_centar, r_pix)
    pygame.draw.line(screen, BLUE, circle_centar, balansero_tip)
    pygame.display.update()

    # x0, th0 = plant(0, time_step[i])
    xx[i] = x
    thth[i] = th * 180 / 3.1415
    # xx0[i] = x0
    # thth0[i] = th0
    if abs(xx[i]) > 1000 or abs(thth[i]) > 90:
        break
    else:
        kraj += 1
    #time.sleep(0.00)

# plt.plot(time_step[:kraj], xx[:kraj], color='m', label="pozicija")
plt.plot(time_step[:kraj], thth[:kraj], color='b', label="ugao")
plt.plot(time_step[:kraj], [0.0] * kraj, '--', color='k')
# plt.plot(time_step, xx0, '--', color='m', label="pozicija bez kontrole")
# plt.plot(time_step, thth0, '--', color='b', label="ugao bez kontrole")
plt.legend()
plt.show()

# for petlja
    # da poziva kroz vreme simulaciju sa odredjenim signalom sa motora
    # dobije se niz x, th koji moze da se plotuje da se vidi sta se desilo

