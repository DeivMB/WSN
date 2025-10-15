import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
from datetime import datetime
import os
import re  # Para extraer números de strings

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')

        # Suscripción al topic del sensor
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10
        )

        # Listas temporales para las muestras del último intervalo
        self.temperatures = []
        self.timestamps = []

        # Timer para graficar cada 5 segundos
        self.timer = self.create_timer(5.0, self.plot_temperature)

        # Carpeta para guardar las imágenes
        self.save_dir = '/root/ros2_ws/volume'
        os.makedirs(self.save_dir, exist_ok=True)

    def listener_callback(self, msg):
        try:
            # Extraer número de la cadena (por ejemplo "Temperatura: 25C")
            match = re.search(r'\d+\.?\d*', msg.data)
            if match:
                temp = float(match.group())
                self.temperatures.append(temp)
                self.timestamps.append(datetime.now())
                self.get_logger().info(f'Temperatura recibida: {temp} °C')
            else:
                self.get_logger().warning(f'No se encontró un número en "{msg.data}"')
        except ValueError:
            self.get_logger().warning(f'No se pudo convertir "{msg.data}" a float')

    def plot_temperature(self):
        if not self.temperatures:
            self.get_logger().info('No hay datos para graficar en este intervalo')
            return

        plt.figure(figsize=(8,5))

        # Convertir tiempos a strings tipo "HH:MM:SS" para las etiquetas del eje X
        time_labels = [t.strftime('%H:%M:%S') for t in self.timestamps]
        x_positions = range(len(self.timestamps))

        # Graficar línea con puntos
        plt.plot(x_positions, self.temperatures, color='red', marker='o', linestyle='-')
        plt.xticks(ticks=x_positions, labels=time_labels, rotation=45, ha='right')

        plt.xlabel('Hora de muestra')
        plt.ylabel('Temperatura (°C)')
        plt.title('Temperatura vs Tiempo (últimos 5 segundos)')
        plt.grid(True)
        plt.tight_layout()

        # Guardar gráfico con nombre único
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.save_dir, f'temperature_plot_{timestamp}.png')
        plt.savefig(filename)
        plt.close()

        self.get_logger().info(f'Gráfico guardado: {filename}')

        # Limpiar datos para el siguiente intervalo
        self.temperatures.clear()
        self.timestamps.clear()


def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
