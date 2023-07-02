import numpy as np

rng_seed = 4
nb_rng_scenario = 10
obst_names = ["perp", "front", "dont_overtake", "overtake"]
obst_x = [4.3, 8.0, 1.0, 2.0]
obst_y = [-4.4, 0.0, -2.0, -0.1]
obst_vx = [0.0, -0.7, 0.3, 0.35]
obst_vy = [0.55, 0.0, 0.45, 0.0]
rng = np.random.default_rng(seed=rng_seed)  # Fixed seed for comparability and reproducibility
nb_gen = 0
while nb_gen < nb_rng_scenario:
    # Generate interesting values for x, y, vx, vy:
    coll_point = rng.random() * 4.0 + 2.0  # Collision within [2.0:6.0] meters in front of the robot
    x = rng.random() * 8.0  # From 0.0 to 9.0
    y = rng.random() * 10.0 - 5.0  # From -5.0 to 5.0
    while x**2 + y**2 < 2.0:  # If it is not in legal bounds, re-roll
        x = rng.random() * 9.0
        y = rng.random() * 10.0 - 5.0

    # Then compute velocity to get to collision_point at the same time as the robot from the x, y position
    t = coll_point / 0.7 + 1.0  # Time for the robot to get to the collision point (+1.0 due to some delay)
    t = t + 0.5 * rng.random()  # Plus some noise to spice it up
    vx = -(x - coll_point) / t
    vy = -y / t
    nb_gen += 1
    obst_names.append("rnd_obst_" + str(nb_gen))
    obst_x.append(x)
    obst_y.append(y)
    obst_vx.append(vx)
    obst_vy.append(vy)

for k in range(len(obst_x)):
    print(obst_names[k], ", ", round(obst_x[k], 2), ", ", round(obst_y[k], 2), ", ", round(obst_vx[k], 2), ", ", round(obst_vy[k], 2))
