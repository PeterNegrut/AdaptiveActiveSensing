import numpy as np
import matplotlib.pyplot as plt

out_dir = "DEMO_OUTPUT/HMMWV/"

truth = np.loadtxt(out_dir + "truth.txt", skiprows=1)

fig, axes = plt.subplots(2, 1, sharex=True)
for ax, hz in zip(axes, [10, 60]):
    est = np.loadtxt(out_dir + f"estimates_{hz}hz.txt", skiprows=1)
    ax.plot(truth[:, 0], truth[:, 1], label="Ground truth")
    ax.scatter(est[:, 0], est[:, 1], s=8, color="red", label="Estimator")
    ax.set_title(f"{hz} Hz sensor, 100 Hz estimation")
    ax.set_ylabel("x [m]")
    ax.legend()

axes[-1].set_xlabel("time [s]")
plt.tight_layout()
plt.show()
