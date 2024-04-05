import numpy as np
import matplotlib.pyplot as plt 

V = [3, 3.5, 4, 4.5, 5, 5.5]
I = [1.37, 1.62, 1.80, 2.08, 2.21, 2.46]
pos = [238, 230, 221, 213, 204, 190]

# Calculate stall torque
alpha = [p - 180 for p in pos]
m = 3.55
l = 0.105
g = 9.81
friction_loss = 0.2

torques = [m*g*l*np.cos(np.deg2rad(a)) - friction_loss for a in alpha]
torques_15V = [t*15/v for t, v in zip(torques, V)]

print("Mean stall torque: ", np.mean(torques_15V))

plt.plot(alpha, torques_15V, 'o')
plt.xlabel('Angle (degrees)')
plt.ylabel('Stall Torque (N*m)')
plt.show()

# Linear regression on V vs I
V = np.array(V)
I = np.array(I)
a, b = np.polyfit(V, I, 1)

R = 1/a
print(f'Resistance: {R:.2f} ohms')

plt.plot(V, I, 'o', label='data')
plt.plot(V, a*V + b, label=f"linear fit: y = {a:.2f}x + {b:.2f}")
plt.xlabel('Voltage (V)')
plt.ylabel('Current (A)')
plt.legend()
plt.show()


