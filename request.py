import requests
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Get data from ESP32
response = requests.get("http://192.168.4.1/get_depth")
data = response.json()

X = np.array(data["X"])
Y = np.array(data["Y"])
Z = np.array(data["Z"])

# Optional: remove invalid or zero-depth points
valid = Z > 0
X, Y, Z = X[valid], Y[valid], Z[valid]

# ✅ Save to CSV
df = pd.DataFrame({'X': X, 'Y': Y, 'Z': Z})
df.to_csv("depth_data.csv", index=False)
print("Depth data saved to depth_data.csv")

# Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

sc = ax.scatter(X, Y, Z, c=Z, cmap='viridis', s=5)

ax.set_xlim(0, 255)
ax.set_ylim(0, 255)
ax.set_zlim(0, 255)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z (Depth)')

plt.colorbar(sc, ax=ax, shrink=0.6, label="Depth (Z value)")
plt.title("ESP32-CAM Depth Map (0–255 Range)")

plt.show()
