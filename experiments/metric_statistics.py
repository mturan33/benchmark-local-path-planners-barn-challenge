import os
import pandas as pd
import numpy as np
from scipy import stats
from statsmodels.stats.multitest import multipletests
import seaborn as sns
import matplotlib.pyplot as plt
import glob
from itertools import combinations

# --- 1. Data Loading and Preparation ---

# Create directories for outputs if they don't exist
os.makedirs("statistical_results", exist_ok=True)

data_path = 'metrics/'
all_files = glob.glob(data_path + "metrics_summary_*.csv")
dataframes = []

print(f"Found files: {all_files}")

for file in all_files:
    alg_name = os.path.basename(file).replace('metrics_summary_', '').replace('.csv', '')
    try:
        df = pd.read_csv(file)
        df["Algorithm"] = alg_name.upper()
        dataframes.append(df)
    except pd.errors.EmptyDataError:
        print(f"Warning: {file} is empty or could not be read.")

if not dataframes:
    raise ValueError("No data files could be loaded. Please check the file paths.")

all_data = pd.concat(dataframes, ignore_index=True)

# --- 2. Data Cleaning and Filtering ---

print(f"\nInitial total number of data rows: {len(all_data)}")
successful_runs = all_data[all_data['Collision'] == 0].copy()
print(f"Number of successful runs (Collision == 0): {len(successful_runs)}")

max_path_efficiency = 100
max_total_time = 60
max_path_length = 100

initial_rows = len(successful_runs)
successful_runs = successful_runs[successful_runs['Path_Efficiency'] <= max_path_efficiency]
successful_runs = successful_runs[successful_runs['Total_Time'] <= max_total_time]
successful_runs = successful_runs[successful_runs['Path_Length'] <= max_path_length]
final_rows = len(successful_runs)
print(f"Outliers cleaned. {initial_rows - final_rows} rows removed. Remaining rows: {final_rows}")


# --- 3. Confidence Interval (CI) and Mean Analysis ---
print("\n--- Metric Summaries (Mean and 95% Confidence Interval) ---")

key_metrics = ['Total_Time', 'Path_Efficiency', 'Control_Effort', 'Avg_Jerk', 'My_Recovery_Count']
summary_stats = successful_runs.groupby('Algorithm')[key_metrics].agg(['mean', 'std', 'count'])

for metric in key_metrics:
    z = 1.96
    mean = summary_stats.loc[:, (metric, 'mean')]
    std = summary_stats.loc[:, (metric, 'std')]
    count = summary_stats.loc[:, (metric, 'count')]
    ci_margin = z * (std / np.sqrt(count))
    summary_stats.loc[:, (metric, 'CI_Lower')] = mean - ci_margin
    summary_stats.loc[:, (metric, 'CI_Upper')] = mean + ci_margin

for metric in key_metrics:
    print(f"\nMetric: {metric}")
    display_df = summary_stats[metric][['mean', 'CI_Lower', 'CI_Upper']].sort_values(by='mean')
    display_df['Mean ± 95% CI'] = display_df.apply(lambda row: f"{row['mean']:.2f} [{row['CI_Lower']:.2f} - {row['CI_Upper']:.2f}]", axis=1)
    print(display_df[['Mean ± 95% CI']])


# --- 4. Pairwise Comparison with Statistical Significance (MANUAL IMPLEMENTATION) ---
print("\n--- Pairwise Comparison (Wilcoxon Test + Holm Correction) ---")

for metric in key_metrics:
    print(f"\n*** Comparison Results for: {metric} ***")

    # Pivot the data to align algorithms by World for paired testing
    metric_pivot = successful_runs.pivot_table(index='World', columns='Algorithm', values=metric)

    algorithms = metric_pivot.columns
    p_values_raw = []
    pairs = []

    # Generate all unique pairs of algorithms
    for alg1, alg2 in combinations(algorithms, 2):
        # Create a temporary dataframe for the pair and drop worlds where either has a NaN value
        pair_data = metric_pivot[[alg1, alg2]].dropna()

        # Ensure there is enough data to perform a test
        if len(pair_data) < 10:
            continue

        # Perform the paired Wilcoxon signed-rank test
        try:
            stat, p_val = stats.wilcoxon(pair_data[alg1], pair_data[alg2])
            p_values_raw.append(p_val)
            pairs.append((alg1, alg2))
        except ValueError:
             # This can happen if all differences are zero
            p_values_raw.append(1.0)
            pairs.append((alg1, alg2))

    if not p_values_raw:
        print(f"Warning: Not enough common successful runs for '{metric}' to perform any statistical tests.")
        continue

    # Apply the Holm-Bonferroni correction to the raw p-values
    reject, p_values_corrected, _, _ = multipletests(p_values_raw, alpha=0.05, method='holm')

    # Create a DataFrame to store the corrected p-values in a matrix format
    results_df = pd.DataFrame(np.ones((len(algorithms), len(algorithms))), index=algorithms, columns=algorithms)
    for (alg1, alg2), p_corrected in zip(pairs, p_values_corrected):
        results_df.loc[alg1, alg2] = p_corrected
        results_df.loc[alg2, alg1] = p_corrected

    # Visualize the results as a heatmap
    plt.figure(figsize=(10, 8))
    sns.heatmap(results_df, annot=True, fmt=".3f", cmap="vlag_r", cbar=True, vmin=0, vmax=0.1, linewidths=.5, linecolor='lightgrey')
    plt.title(f"Holm-corrected p-values for {metric}\n(p < 0.05 indicates significant difference)")

    # Save the heatmap and the p-values to a CSV file
    output_filename = f"statistical_results/heatmap_{metric}"
    plt.savefig(f"{output_filename}.png", bbox_inches='tight')
    results_df.to_csv(f"{output_filename}.csv")

    plt.show()

    print(f"Heatmap saved to '{output_filename}.png' and p-values to '{output_filename}.csv'.")