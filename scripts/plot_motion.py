import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('drift_motion.csv')

plt.figure(figsize=(10, 5))
plt.plot(df['timestamp'], df['x'], label='position')
plt.xlabel('Time (seconds)')
plt.ylabel('Position')
plt.title('Mouse  Position Over a Horizontal Line')
plt.legend()
plt.grid(True)
plt.show()