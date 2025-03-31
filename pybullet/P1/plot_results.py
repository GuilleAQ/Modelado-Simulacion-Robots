import numpy as np
import matplotlib.pyplot as plt
import sys

# Definir colores para los archivos (Rojo, Verde, Azul, y más si es necesario)
COLORS = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

# Cargar los datos del CSV
def load_data(filename):
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    return data[:, 1], data[:, 2]  # Extrae posición Y y velocidad Y

# Función para graficar múltiples archivos en un mismo gráfico con colores específicos
def plot_velocity_vs_position(filenames):
    plt.figure(figsize=(8, 6))

    for i, filename in enumerate(filenames):
        try:
            position, velocity = load_data(filename)
            color = COLORS[i % len(COLORS)]  # Asigna colores en orden
            plt.plot(position, velocity, linestyle='-', linewidth=1, color=color, label=f'{filename}')
        except Exception as e:
            print(f"Error al cargar {filename}: {e}")

    plt.xlabel('Posición Y (m)')
    plt.ylabel('Velocidad Y (m/s)')
    plt.title('Variación de la velocidad en función de la posición')
    plt.legend()
    plt.grid(True)
    
    plt.show()

# Obtener los nombres de los archivos desde los argumentos de línea de comandos
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python script.py archivo1.csv archivo2.csv ...")
    else:
        plot_velocity_vs_position(sys.argv[1:])
