import pandas as pd
import glob
import os

# Find the most recent metrics file
metrics_file = max(glob.glob('walking_metrics_*.csv'), key=os.path.getctime)
df = pd.read_csv(metrics_file)

# Analyze the data
print(f'\nAnalyzing metrics from {metrics_file}:\n')
print('Summary Statistics:')
print(df[['x_pos', 'height', 'forward_velocity', 'stability_score']].describe())

print('\nKey Events:')
# Find moments of significant height changes
critical_drops = df[df['height_change'] < -0.01]
if not critical_drops.empty:
    print(f'\nFound {len(critical_drops)} critical height drops:')
    print(critical_drops[['step', 'height', 'height_change', 'stability_score']].head())

# Find highest forward velocity achieved
max_velocity = df.loc[df['forward_velocity'].idxmax()]
print(f'\nHighest forward velocity:')
print(f'Step {max_velocity["step"]}: {max_velocity["forward_velocity"]:.3f} m/s')

# Check if we're maintaining target height
avg_height = df['height'].mean()
height_std = df['height'].std()
print(f'\nHeight maintenance:')
print(f'Average height: {avg_height:.3f}m (std: {height_std:.3f}m)')
