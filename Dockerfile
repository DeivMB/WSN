# Imagen base con ROS 2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop

# Entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Instalar herramientas necesarias
RUN apt update && apt upgrade -y && \
    apt install -y python3-colcon-common-extensions nano git && \
    apt install -y tree iproute2 net-tools iputils-ping

# Configurar entorno de ROS2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Crear workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Crear el paquete ROS2 dentro del workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /root/ros2_ws/src && \
    ros2 pkg create --build-type ament_python sensor_program --license MIT"


# Copiar archivos del paquete
COPY py_scripts/setup.py /root/ros2_ws/src/sensor_program/
COPY py_scripts/sensor_node.py /root/ros2_ws/src/sensor_program/sensor_program/
COPY py_scripts/reader_node.py /root/ros2_ws/src/sensor_program/sensor_program/
COPY py_scripts/plotter_node.py /root/ros2_ws/src/sensor_program/sensor_program/


# 5. Compilar el workspace con Colcon
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build"

# 6. Agregar el source del workspace al entorno del contenedor
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# 7. EntryPoint y CMD
CMD ["bash"]
