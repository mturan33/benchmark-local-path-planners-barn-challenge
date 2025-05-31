import pandas as pd
import matplotlib.pyplot as plt

# CSV dosyasını oku
df = pd.read_csv('metrics_log_dwa.csv')

# Verilerin başına bak
print(df.head())

# Path Length (Katedilen Yol) Zaman Grafiği
plt.figure(figsize=(10,5))
plt.plot(df['Time'], df['Path_Length'], label='Path Length (m)')
plt.xlabel('Zaman (s)')
plt.ylabel('Yol Uzunluğu (m)')
plt.title('Zamana Göre Katedilen Yol (Path Length)')
plt.legend()
plt.grid(True)
plt.show()

# Eğer Arrival Time (AT) sütunun varsa, sadece ilk dolu değerlerini çizmek için:
at_values = df['AT'].dropna()
if not at_values.empty:
    plt.figure(figsize=(6,4))
    plt.hist(at_values, bins=10)
    plt.xlabel('Arrival Time (s)')
    plt.title('Başarıya Ulaşma Süresi (Arrival Time) Dağılımı')
    plt.grid(True)
    plt.show()

# Çarpışma Oldu mu (Collision)? Grafiği
plt.figure(figsize=(10,2))
plt.plot(df['Time'], df['Collision'], label='Çarpışma (1=Var, 0=Yok)')
plt.xlabel('Zaman (s)')
plt.ylabel('Çarpışma')
plt.title('Zamana Göre Çarpışma Durumu')
plt.legend()
plt.grid(True)
plt.show()

