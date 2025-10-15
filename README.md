## Contenedor Docker para la aplicación ROS2

Este proyecto incluye un `Dockerfile` que copia automáticamente los scripts de aplicación y de configuración dentro de la imagen. Esto permite ejecutar el contenedor directamente, sin necesidad de compilar los paquetes dentro del contenedor.

# Ejecutar el contenedor

Para ejecutar el contenedor y vincular un volumen local donde se guardarán los gráficos generados, utiliza el siguiente comando:

$ docker run -it --name container_name -v /ruta/local/folder:/root/ros2_ws/volume image_name

**Nota:** No modifiques la ruta del volumen dentro del contenedor (/root/ros2_ws/volume), ya que el script plotter_node.py guardará las gráficas allí.
Si deseas guardar los gráficos en otra carpeta dentro del contenedor, primero debes modificar los scripts correspondientes antes de construir la imagen.
Cualquier cambio en los scripts debe realizarse antes de construir la imagen, de modo que al ejecutar el contenedor únicamente sea necesario iniciar los nodos, sin usar colcon build ni otros comandos de compilación adicionales.
