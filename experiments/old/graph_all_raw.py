import pandas as pd
import matplotlib.pyplot as plt
import os

# CSV dosyalarının bulunduğu klasör
metrics_dir = './metrics'
csv_files = [
    'metrics_summary_dwa.csv',
    'metrics_summary_fastdwa.csv',
    'metrics_summary_eband.csv',
    'metrics_summary_teb.csv'
]

# Grafiklerin kaydedileceği klasör
output_dir = './graphs_raw'
os.makedirs(output_dir, exist_ok=True)

for csv_file in csv_files:
    file_path = os.path.join(metrics_dir, csv_file)
    df = pd.read_csv(file_path)

    algo_name = csv_file.replace('metrics_summary_', '').replace('.csv', '')

    print(f"\n==> İşleniyor: {csv_file}, Satır: {len(df)}")

    # Sayısal sütunları filtrele
    numeric_columns = df.select_dtypes(include='number').columns

    for column in numeric_columns:
        plt.figure(figsize=(10, 5))
        plt.plot(df[column], marker='o', linestyle='-', label=column)
        plt.xlabel('Deneme İndeksi')
        plt.ylabel(column)
        plt.title(f'{algo_name.upper()} - {column}')
        plt.grid(True)
        plt.legend()

        # Grafik dosyasını kaydet
        plot_filename = f'{algo_name}_{column}.png'
        plt.savefig(os.path.join(output_dir, plot_filename))
        plt.close()

print(f"\nTüm grafikler {output_dir} klasörüne kaydedildi.")

