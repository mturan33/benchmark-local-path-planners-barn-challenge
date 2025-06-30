import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.interpolate import UnivariateSpline

metrics_dir = './metrics'
csv_files = [
    'metrics_summary_dwa.csv',
    'metrics_summary_fastdwa.csv',
    'metrics_summary_eband.csv',
    'metrics_summary_teb.csv'
]

output_dir = './graphs_smooth'
os.makedirs(output_dir, exist_ok=True)

for csv_file in csv_files:
    file_path = os.path.join(metrics_dir, csv_file)
    df = pd.read_csv(file_path)

    algo_name = csv_file.replace('metrics_summary_', '').replace('.csv', '')

    print(f"\n==> İşleniyor: {csv_file}, Satır: {len(df)}")

    numeric_columns = df.select_dtypes(include='number').columns
    x = np.arange(len(df))

    for column in numeric_columns:
        y = df[column].values

        # Veride çok az nokta varsa spline uygunsuz olur
        if len(y) < 4:
            continue

        try:
            # Yumuşatma: spline derecesi 3, yumuşatma faktörü s
            spline = UnivariateSpline(x, y, s=len(y))  # s değeri artırılırsa daha düzleşir
            y_smooth = spline(x)

            plt.figure(figsize=(10, 5))
            plt.plot(x, y, 'o', alpha=0.3, label='Orijinal')
            plt.plot(x, y_smooth, '-', label='Yumuşatılmış')
            plt.xlabel('Deneme İndeksi')
            plt.ylabel(column)
            plt.title(f'{algo_name.upper()} - {column}')
            plt.grid(True)
            plt.legend()

            plot_filename = f'{algo_name}_{column}_smooth.png'
            plt.savefig(os.path.join(output_dir, plot_filename))
            plt.close()
        except Exception as e:
            print(f"[!] {column} için spline başarısız: {e}")

