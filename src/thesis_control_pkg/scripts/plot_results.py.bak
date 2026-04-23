

"""
Thesis Results Visualization
Generates publication-quality plots from experiment data
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import glob
import os
import sys
from datetime import datetime

# Set thesis-ready plot style
plt.style.use('seaborn-v0_8-paper')
sns.set_context('paper', font_scale=1.2)
plt.rcParams['figure.figsize'] = (10, 6)
plt.rcParams['figure.dpi'] = 300

def load_experiment_data(filepath):
    """Load and preprocess experiment CSV data"""
    df = pd.read_csv(filepath)
    df['timestamp_sec'] = pd.to_numeric(df['timestamp_sec'], errors='coerce')
    df = df.dropna(subset=['timestamp_sec', 'tracking_error_dist'])
    return df

def compute_metrics(df):
    """Compute thesis metrics from data"""
    metrics = {}
    
    # Tracking error metrics
    metrics['rmse'] = np.sqrt(np.mean(df['tracking_error_dist']**2))
    metrics['mae'] = np.mean(np.abs(df['tracking_error_dist']))
    metrics['max_error'] = np.max(df['tracking_error_dist'])
    metrics['std_error'] = np.std(df['tracking_error_dist'])
    
    # Control effort metrics (ISE, IAE, ITAE)
    dt = np.diff(df['timestamp_sec']).mean() if len(df) > 1 else 0.1
    metrics['ise_v'] = np.sum(df['cmd_v']**2) * dt  # Integral Squared Error - velocity
    metrics['iae_v'] = np.sum(np.abs(df['cmd_v'])) * dt  # Integral Absolute Error
    metrics['itae_v'] = np.sum(np.abs(df['cmd_v']) * df['timestamp_sec']) * dt
    
    metrics['ise_omega'] = np.sum(df['cmd_omega']**2) * dt
    metrics['iae_omega'] = np.sum(np.abs(df['cmd_omega'])) * dt
    
    # EKF uncertainty
    if 'state_cov_xx' in df.columns:
        metrics['mean_uncertainty_x'] = np.mean(df['state_cov_xx'])
        metrics['max_uncertainty_x'] = np.max(df['state_cov_xx'])
    
    return metrics

def plot_trajectory(df, output_path):
    """Plot robot trajectory vs reference path"""
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    
    # Plot reference waypoints
    ref_x = df['reference_x'].drop_duplicates().values
    ref_y = df['reference_y'].drop_duplicates().values
    ax.plot(ref_x, ref_y, 'go--', label='Reference Waypoints', markersize=8, linewidth=2)
    
    # Plot actual robot path (subsample for clarity)
    step = max(1, len(df) // 500)
    ax.plot(df['state_x'].values[::step], df['state_y'].values[::step], 
            'b-', label='Actual Trajectory', linewidth=1.5, alpha=0.8)
    
    # Plot start and end
    ax.plot(df['state_x'].iloc[0], df['state_y'].iloc[0], 'ks', markersize=12, label='Start')
    ax.plot(df['state_x'].iloc[-1], df['state_y'].iloc[-1], 'r*', markersize=15, label='End')
    
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Robot Trajectory vs Reference Path', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f'✓ Saved trajectory plot: {output_path}')

def plot_tracking_error(df, output_path):
    """Plot tracking error over time"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    
    ax.plot(df['timestamp_sec'], df['tracking_error_dist'], 'b-', linewidth=1.5, label='Tracking Error')
    
    # Add statistics lines
    rmse = np.sqrt(np.mean(df['tracking_error_dist']**2))
    mean_err = np.mean(df['tracking_error_dist'])
    ax.axhline(y=rmse, color='r', linestyle='--', linewidth=1, label=f'RMSE = {rmse:.3f}m')
    ax.axhline(y=mean_err, color='g', linestyle=':', linewidth=1, label=f'Mean = {mean_err:.3f}m')
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Tracking Error (m)', fontsize=12)
    ax.set_title('Tracking Error Over Time', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f'✓ Saved error plot: {output_path}')

def plot_control_signals(df, output_path):
    """Plot control commands over time"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    # Linear velocity
    ax1.plot(df['timestamp_sec'], df['cmd_v'], 'b-', linewidth=1, label='Commanded v')
    if 'actual_v' in df.columns:
        ax1.plot(df['timestamp_sec'], df['actual_v'], 'r--', linewidth=1, label='Actual v', alpha=0.7)
    ax1.set_ylabel('Linear Velocity (m/s)', fontsize=11)
    ax1.legend(loc='best', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Control Commands', fontsize=13, fontweight='bold')
    
    # Angular velocity
    ax2.plot(df['timestamp_sec'], df['cmd_omega'], 'b-', linewidth=1, label='Commanded ω')
    if 'actual_omega' in df.columns:
        ax2.plot(df['timestamp_sec'], df['actual_omega'], 'r--', linewidth=1, label='Actual ω', alpha=0.7)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    ax2.legend(loc='best', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f'✓ Saved control plot: {output_path}')

def plot_uncertainty(df, output_path):
    """Plot EKF uncertainty over time"""
    if 'state_cov_xx' not in df.columns:
        return
    
    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    
    ax.plot(df['timestamp_sec'], np.sqrt(df['state_cov_xx']), 'b-', linewidth=1.5, 
            label='Position Uncertainty (σₓ)')
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Uncertainty (m)', fontsize=12)
    ax.set_title('Adaptive EKF Position Uncertainty', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f'✓ Saved uncertainty plot: {output_path}')

def generate_comparison_table(experiment_files, output_path):
    """Generate comparison table for thesis"""
    results = []
    
    for exp_file in experiment_files:
        exp_name = os.path.basename(exp_file).replace('.csv', '')
        df = load_experiment_data(exp_file)
        metrics = compute_metrics(df)
        
        results.append({
            'Experiment': exp_name,
            'Duration (s)': f"{metrics.get('duration_sec', 0):.1f}",
            'RMSE (m)': f"{metrics.get('rmse', 0):.3f}",
            'Max Error (m)': f"{metrics.get('max_error', 0):.3f}",
            'IAE_v': f"{metrics.get('iae_v', 0):.2f}",
            'ISE_ω': f"{metrics.get('ise_omega', 0):.2f}",
            'Uncertainty (m)': f"{np.sqrt(metrics.get('mean_uncertainty_x', 0)):.3f}"
        })
    
    # Create and save table
    df_results = pd.DataFrame(results)
    
    # Save as CSV and LaTeX table
    df_results.to_csv(output_path.replace('.tex', '.csv'), index=False)
    
    # LaTeX table for thesis
    with open(output_path, 'w') as f:
        f.write('\\begin{table}[h]\n')
        f.write('\\centering\n')
        f.write('\\caption{Experimental Results Comparison}\n')
        f.write('\\label{tab:results}\n')
        f.write('\\begin{tabular}{lcccccc}\n')
        f.write('\\toprule\n')
        f.write('Experiment & Duration (s) & RMSE (m) & Max Error (m) & IAE$_v$ & ISE$_\\omega$ & Uncertainty (m) \\\\\n')
        f.write('\\midrule\n')
        for _, row in df_results.iterrows():
            f.write(f"{row['Experiment']} & {row['Duration (s)']} & {row['RMSE (m)']} & "
                   f"{row['Max Error (m)']} & {row['IAE_v']} & {row['ISE_ω']} & {row['Uncertainty (m)']} \\\\\n")
        f.write('\\bottomrule\n')
        f.write('\\end{tabular}\n')
        f.write('\\end{table}\n')
    
    print(f'✓ Saved comparison table: {output_path}')
    print(df_results.to_string(index=False))

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 plot_results.py <experiment_csv_file> [output_dir]')
        print('   or: python3 plot_results.py --compare <dir_with_csvs> [output_dir]')
        return
    
    # Determine mode
    if sys.argv[1] == '--compare':
        # Comparison mode
        data_dir = sys.argv[2] if len(sys.argv) > 2 else '/root/sensor_fusion_ws/experiment_data'
        output_dir = sys.argv[3] if len(sys.argv) > 3 else data_dir
        
        csv_files = glob.glob(os.path.join(data_dir, '*.csv'))
        if not csv_files:
            print(f'No CSV files found in {data_dir}')
            return
        
        os.makedirs(output_dir, exist_ok=True)
        generate_comparison_table(csv_files, os.path.join(output_dir, 'comparison_table.tex'))
        
    else:
        # Single experiment mode
        csv_file = sys.argv[1]
        output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(csv_file)
        
        if not os.path.exists(csv_file):
            print(f'File not found: {csv_file}')
            return
        
        os.makedirs(output_dir, exist_ok=True)
        df = load_experiment_data(csv_file)
        
        print(f'\n📊 Processing: {csv_file}')
        print(f'   Samples: {len(df)}, Duration: {df["timestamp_sec"].max():.1f}s')
        
        # Compute and print metrics
        metrics = compute_metrics(df)
        print(f'\n📈 Key Metrics:')
        print(f'   RMSE: {metrics["rmse"]:.3f} m')
        print(f'   Max Error: {metrics["max_error"]:.3f} m')
        print(f'   IAE (velocity): {metrics["iae_v"]:.2f}')
        print(f'   ISE (angular): {metrics["ise_omega"]:.2f}')
        
        # Generate plots
        base_name = os.path.basename(csv_file).replace('.csv', '')
        plot_trajectory(df, os.path.join(output_dir, f'{base_name}_trajectory.png'))
        plot_tracking_error(df, os.path.join(output_dir, f'{base_name}_error.png'))
        plot_control_signals(df, os.path.join(output_dir, f'{base_name}_control.png'))
        plot_uncertainty(df, os.path.join(output_dir, f'{base_name}_uncertainty.png'))
        
        print(f'\n✅ All plots saved to: {output_dir}')

if __name__ == '__main__':
    main()

