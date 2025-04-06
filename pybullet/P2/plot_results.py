import numpy as np
import matplotlib.pyplot as plt
import sys

def load_data(filename):
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    tiempo = data[:, 0]
    gasto_parcial = data[:, 2]
    return tiempo, gasto_parcial

def plot_gasto_total_vs_tiempo(filenames):
    plt.figure(figsize=(10, 6))

    for i, filename in enumerate(filenames):
        try:
            tiempo, gasto_parcial = load_data(filename)

            # Ordenar por tiempo por si acaso
            orden = np.argsort(tiempo)
            tiempo = tiempo[orden]
            gasto_parcial = gasto_parcial[orden]

            gasto_total = np.cumsum(gasto_parcial)

            plt.plot(tiempo, gasto_total, label=f'{filename}', linewidth=2)
        except Exception as e:
            print(f"Error al cargar {filename}: {e}")

    plt.xlabel("Tiempo")
    plt.ylabel("Gasto Mecánico Total (Nm·s)")
    plt.title("Evolución del Gasto Mecánico Total")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Uso por línea de comandos
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python plot_results.py archivo1.csv archivo2.csv ...")
    else:
        plot_gasto_total_vs_tiempo(sys.argv[1:])
