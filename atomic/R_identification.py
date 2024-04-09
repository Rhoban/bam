I = [0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4, 2.7, 3]

# MX 106 (Cassé):
V_1 = [1.3, 2, 2.6, 3.3, 4, 4.9, 5.8, 6.3, 7.3]

# MX 106 (Fonctionnel):
V_2 = [1.25, 1.88, 2.46, 3.13, 3.7, 4.4, 5.15, 5.35, 6.4]

# Linear regression
import numpy as np

V_1 = np.array(V_1)
V_2 = np.array(V_2)
I = np.array(I)

A = np.vstack([V_1, np.ones(len(V_1))]).T
m1, c1 = np.linalg.lstsq(A, I, rcond=None)[0]

A = np.vstack([V_2, np.ones(len(V_2))]).T
m2, c2 = np.linalg.lstsq(A, I, rcond=None)[0]

print('R1 =', 1/m1, 'Ohm')
print('R2 =', 1/m2, 'Ohm')

# Plot
import matplotlib.pyplot as plt

V = np.linspace(0, 8, 100)

plt.plot(V_1, I, 'ro', label='MX 106 (Cassé)')
plt.plot(V_2, I, 'go', label='MX 106 (Fonctionnel)')
plt.plot(V, m1*V + c1, 'b', label=f"y = {m1:.2f}x + {c1:.2f} (Cassé)")
plt.plot(V, m2*V + c2, 'y', label=f"y = {m2:.2f}x + {c2:.2f} (Fonctionnel)")
plt.xlabel('V (V)')
plt.ylabel('I (A)')
plt.title('I = f(V)')
plt.grid()
plt.legend()
plt.show()

