import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_and_resample(path, step=0.01):
    df = pd.read_csv(path)

    time_cols = [c for c in df.columns if c.lower() in ('time', 'timestamp', 't')]
    if not time_cols:
        raise ValueError(f"No time column found in {path}")
    tcol = time_cols[0]
    df['t'] = df[tcol].astype(float)

    if df['t'].max() > 1000:
        df['t'] /= 1000.0
    df['t'] -= df['t'].iloc[0]

    df = df.drop_duplicates(subset=['t'], keep='first')

    xcols = [c for c in df.columns if c.lower() in ('x', 'posx', 'px')]
    ycols = [c for c in df.columns if c.lower() in ('y', 'posy', 'py')]
    if not xcols or not ycols:
        raise ValueError(f"Could not find X,Y columns in {path}")
    df['x'] = df[xcols[0]].astype(float)
    df['y'] = df[ycols[0]].astype(float)

    df = df.set_index('t')

    new_t = np.arange(df.index.min(), df.index.max(), step)

    df_resampled = (
        df.reindex(df.index.union(new_t))
          .interpolate('index')
          .loc[new_t]
    ).reset_index().rename(columns={'index': 't'})

    return df_resampled[['t', 'x', 'y']]

def compute_jerk(df):
    t = df['t'].values
    x = df['x'].values
    y = df['y'].values

    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    jx = np.gradient(ax, t)
    jy = np.gradient(ay, t)

    jerk_mag = np.sqrt(jx**2 + jy**2)
    return t, jerk_mag

def main():
    files_input = input("Enter CSV filenames separated by commas: ").strip()
    names = ["time-triggered scheduling", "normal mouse movement", "preemptive interrupt mode"]
    files = [f.strip() for f in files_input.split(",") if f.strip()]
    plt.figure(figsize=(10, 6))

    for i, path in enumerate(files):
        if not os.path.isfile(path):
            print(f"File not found: {path}")
            continue
        try:
            df = load_and_resample(path, step=0.01)
            t, jerk = compute_jerk(df)
            jerk_rms = np.sqrt(np.mean(jerk**2))
            print(f"RMS Jerk for {names[i]} ({path}): {jerk_rms:.2f} pixels/s³")
        except Exception as e:
            print(f"Error processing {path}: {e}")
            continue

        plt.plot(t, jerk, label=names[i])

    plt.xlabel("Time (s)")
    plt.ylabel("Jerk magnitude (pixels/s³)")
    plt.title("Jerk over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()