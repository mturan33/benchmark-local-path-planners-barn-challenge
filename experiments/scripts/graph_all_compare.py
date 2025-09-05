import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import plotly.express as px
from sklearn.preprocessing import MinMaxScaler

# Grafiklerin kaydedileceği klasör
os.makedirs("graphs_compare", exist_ok=True)
os.makedirs("graphs_html", exist_ok=True)

# Kullanılacak CSV dosyaları ve etiketleri
files = {
    'dwa': 'metrics/metrics_summary_dwa.csv',
    'fastdwa': 'metrics/metrics_summary_fastdwa.csv',
    'eband': 'metrics/metrics_summary_eband.csv',
    'teb': 'metrics/metrics_summary_teb.csv'
}

# Grafikleştirilecek metrikler (filtrelenmiş hali)
metrics_to_plot = [
    "Total_Time", "Path_Length", "Deviation_From_Straight",
    "Avg_Velocity", "Path_Efficiency", "Navigation_Metric",
    "Collision", "My_Recovery_Count", "Path_Smoothness",
    "Max_Path_Deviation", "Control_Effort",
    "Path_Curvature", "Avg_Jerk", "Path_Overlap"
]

# Her algoritmanın datasını oku ve etiketle
dataframes = []
for alg, path in files.items():
    df = pd.read_csv(path)
    df["Algorithm"] = alg.upper()
    dataframes.append(df)

# Hepsini birleştir
all_data = pd.concat(dataframes, ignore_index=True)

# Her metrik için çizim
for metric in metrics_to_plot:
    plt.figure(figsize=(10, 5))

    # Aykırı değerleri IQR ile filtrele
    q1 = all_data[metric].quantile(0.25)
    q3 = all_data[metric].quantile(0.75)
    iqr = q3 - q1
    lower = q1 - 1.5 * iqr
    upper = q3 + 1.5 * iqr
    filtered = all_data[(all_data[metric] >= lower) & (all_data[metric] <= upper)]

    # Normalize et
    scaler = MinMaxScaler()
    filtered = all_data.loc[(all_data[metric] >= lower) & (all_data[metric] <= upper)].copy()
    normalized_values = scaler.fit_transform(filtered[[metric]])
    filtered = filtered.assign(Normalized=normalized_values)


    # Plot (Matplotlib)
    for alg in filtered["Algorithm"].unique():
        df_alg = filtered[filtered["Algorithm"] == alg]
        df_alg_sorted = df_alg.sort_values(by="Normalized").reset_index()
        plt.plot(df_alg_sorted["Normalized"].rolling(5, min_periods=1).mean(), label=alg)

    plt.title(f"{metric} - Karşılaştırmalı (Normalize + Smooth)")
    plt.xlabel("Veri Nokası (Normalize Sıralı)")
    plt.ylabel(f"{metric} (Normalize)")
    plt.legend()
    plt.grid(True)

    # Kayıt
    for ext in ["svg", "png", "pdf"]:
        plt.savefig(f"graphs_compare/{metric}.{ext}", bbox_inches='tight')

    plt.close()

    # Plotly ile interaktif grafik
    fig = px.line(
        filtered.sort_values(by="Normalized"),
        x=filtered.index,
        y="Normalized",
        color="Algorithm",
        title=f"{metric} - İnteraktif (Normalize + Filtered)",
        labels={"Normalized": f"{metric} (Normalize)", "index": "Veri"}
    )
    fig.write_html(f"graphs_html/{metric}.html")

