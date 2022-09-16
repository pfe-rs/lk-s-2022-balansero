from matplotlib import pyplot as plt

pr_izvod_x = 0
pr_izvod_theta = 0

x = 0
th = 0

dt = 0.001

def simulacija(motor_input: float, time_step: float):
    
    # logika
    global pr_izvod_x
    global pr_izvod_theta
    global x
    global th

    m1 = 0.2 # kg
    m2 = 1.0 # kg
    g = 9.81 # m/s^2
    r = 0.05 # m
    l = 0.25 # m
    M = motor_input # Nm
    
    dr_izvod_x = (M/m2*l*(1 + 13/12 * l/r) - g * th) / (1/12 + 39/24 * m1/m2)
    dr_izvod_theta = 1/l * (g*th * (1 + 3/2 * m1/m2) - M/m2*l * (1 + 3/2 * m1/m2 + l/r)) / (1/12 + 39/24 * m1/m2)
    
    pr_izvod_x += dr_izvod_x * time_step
    pr_izvod_theta += dr_izvod_theta * time_step

    x += pr_izvod_x * time_step
    th += pr_izvod_theta * time_step
    
    return x, th

broj_iteracija = 10

xx, thth, time_step = [0.0] * broj_iteracija, [0.0] * broj_iteracija, [0.0] * broj_iteracija

for i in range(broj_iteracija):
    motor_input = 10 if i <= 5 else 0
    time_step[i] = i * dt
    x, th = simulacija(motor_input, time_step[i])
    xx[i] = x
    thth[i] = th

plt.plot(time_step, xx, label="pozicija")
plt.plot(time_step, thth, label="ugao")
plt.legend()
plt.show()

# for petlja
    # da poziva kroz vreme simulaciju sa odredjenim signalom sa motora
    # dobije se niz x, th koji moze da se plotuje da se vidi sta se desilo

