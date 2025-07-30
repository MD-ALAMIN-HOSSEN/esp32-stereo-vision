import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the saved CSV file
df = pd.read_csv("depth_data.csv")

# Convert to NumPy arrays
X = df["X"].to_numpy()
Y = df["Y"].to_numpy()
Z = df["Z"].to_numpy()

# Optional: filter out zero-depth values
valid = Z > 0
X, Y, Z = X[valid], Y[valid], Z[valid]

# Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

sc = ax.scatter(X, Y, Z, c=Z, cmap='viridis', s=5)

ax.set_xlim(0, 255)
ax.set_ylim(0, 255)
ax.set_zlim(0, 255)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z (Depth)")

plt.colorbar(sc, ax=ax, shrink=0.6, label="Depth (Z value)")
plt.title("Loaded Depth Map from CSV")

plt.show()
