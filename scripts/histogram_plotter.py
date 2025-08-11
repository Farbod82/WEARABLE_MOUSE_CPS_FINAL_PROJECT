import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("delay_intr2.csv")

plt.figure(figsize=(10, 6))
plt.hist(df['diff'] / 1_000_000, bins=30, weights=[100 / len(df)] * len(df), edgecolor='black')

plt.xlabel('Move Intervals (ms)')
plt.ylabel('Percentage (%)')
plt.title('Histogram of move time intervals')
plt.tight_layout()
plt.show()
