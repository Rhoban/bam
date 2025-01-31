import pickle

data = pickle.load(open("data.pkl", "rb"))
print(data)
# data looks like this
# data =  {
#     "torque_limit": [t0, t1, t2, ...],
#     "max_torque_limit": [],
#     "overload_torque": [],
#     "protective_torque": [],
#     "protection_current": [],
#     "present_current": []
#     "present_voltage": []
# }

# plot each in its subplot over time

import matplotlib.pyplot as plt

fig, axs = plt.subplots(4, 2, figsize=(10, 10))

for i, (key, values) in enumerate(data.items()):
    print(i//2, i%2)
    ax = axs[i // 2, i % 2]
    ax.plot(values)
    ax.set_title(key)

plt.show()