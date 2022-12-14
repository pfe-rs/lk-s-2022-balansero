from matplotlib import pyplot as plt
import pygame
import math
import time

# Rezolucija i duzina simulacije
dt = 0.001
broj_iteracija = 100000

# Trenutno stanje robota
pr_izvod_x = 0
pr_izvod_th = 0

# Hronolosko stanje robota
xx, thth, time_step = [[0.0] * broj_iteracija] * 3

# Pocetno stanje robota
x = 0.2  # m
th = 0.2  # rad
M = 0  # Nm

# Fizicke dimenzije robota
m1 = 0.2  # kg
m2 = 3.0  # kg
g = 9.81  # m/s^2
motor_max = 0.5  # Nm
r = 0.05  # m
l = 0.25  # m


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


# Koeficijenti PID kontrolera
kontroler_x = PID(0.3, 0.0, 0.2, dt)  # C3
kontroler_dx = PID(0.045, 0, 0, dt)  # C2
kontroler_th = PID(-2.67, 0.0, -1.107, dt)  # C1, Ku = -2.67


def plant(motor_input: float, time_step: float):
    # logika
    global pr_izvod_x
    global pr_izvod_th
    global x
    global th
    global r
    global l
    global m1
    global m2
    global g
    global motor_max

    M = motor_input * motor_max

    c1 = M / m2 / l
    c2 = m1 / m2
    c3 = 1 + 3 / 2*c2
    c4 = 1 / 3 + 2*c2
    c5 = l / r
    c6 = 1 + 4 / 3*c5
    c7 = g*th

    dr_izvod_x = (c1*c6 - c7) / c4
    dr_izvod_th = (c7*c3 - c1*c3 - c1*c5) / c4 / l

    pr_izvod_x += dr_izvod_x * time_step
    pr_izvod_th += dr_izvod_th * time_step

    x += pr_izvod_x * time_step
    th += pr_izvod_th * time_step

    print("x/dx/ddx {:.3f} {:.3f} {:.3f} th/dth/ddth {:.3f} {:.3f} {:.3f} motorinput {:.3f}".format(
        x, pr_izvod_x,
        dr_izvod_x, th,
        pr_izvod_th,
        dr_izvod_th,
        moment_sile
    ))

    return x, th


kraj = 0

successes, failures = pygame.init()
print("{0} successes and {1} failures".format(successes, failures))

screen = pygame.display.set_mode((1000, 512))
clock = pygame.time.Clock()

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
PINK = (239, 66, 245)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

pixels_per_meter = 1000

zeljena_pozicija = 0.5
for i in range(broj_iteracija):
    zeljena_brzina = kontroler_x.izracunaj(zeljena_pozicija - x)  # C3
    zeljeni_ugao = kontroler_dx.izracunaj(zeljena_brzina - pr_izvod_x)  # C2
    moment_sile = kontroler_th.izracunaj(zeljeni_ugao - th)  # C1

    time_step[i] = i * dt
    x, th = plant(moment_sile, dt)

    ev = pygame.event.get()
    for event in ev:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                zeljena_pozicija -= 0.05
            if event.key == pygame.K_RIGHT:
                zeljena_pozicija += 0.05

    screen.fill(WHITE)
    r_pix = r * pixels_per_meter
    l_pix = l * pixels_per_meter
    l_peak_pix = 1.2*l * pixels_per_meter
    x_pix = x * pixels_per_meter
    circle_centar = (x_pix, 512 - r_pix)
    balansero_tip = (
        x_pix + math.sin(th) * l_peak_pix,
        512 - r_pix - math.cos(th) * l_peak_pix
    )
    balansero_mass = (
        x_pix + math.sin(th) * l_pix,
        512 - r_pix - math.cos(th) * l_pix
    )
    pygame.draw.circle(screen, PINK, circle_centar, r_pix)
    pygame.draw.line(screen, BLUE, circle_centar, balansero_tip)
    pygame.draw.circle(screen, GREEN, balansero_mass, r_pix/3)
    pygame.draw.circle(screen, (0, 0, 0), (1000 * zeljena_pozicija, 510), 5)
    pygame.display.update()

    xx[i] = x
    thth[i] = th * 180 / 3.1415

    # Prekid simulacije u slucaju pada robota
    if abs(xx[i]) > 1000 or abs(thth[i]) > 90:
        break
    else:
        kraj += 1

    # Usporavanje simulacije
    # time.sleep(0.02)

plt.plot(time_step[:kraj], xx[:kraj], color='m', label="pozicija")
plt.plot(time_step[:kraj], thth[:kraj], color='b', label="ugao")
plt.plot(time_step[:kraj], [0.0] * kraj, '--', color='k')
plt.legend()
plt.show()
