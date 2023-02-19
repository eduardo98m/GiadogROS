# Imagen base
FROM ros:noetic

# Dependencias adicionales
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y git vim libi2c-dev ros-noetic-rosserial-python wiringpi \
    && rm -rf /var/lib/apt/lists/*

# Crea el directorio de trabajo
WORKDIR /catkin_ws

# Dependencias en ROS
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                mkdir -p /catkin_ws/src && \
                cd /catkin_ws/src && \
                git clone https://github.com/mentor-dyun/ros-i2cpwmboard && \
                cd /catkin_ws && \
                catkin_make && \
                cd /catkin_ws/src && \
                catkin_create_pkg giadog std_msgs roscpp && \
                echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
                echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc"

# Copia el paquete de ROS a la imagen de Docker
COPY ./ /catkin_ws/src/giadog/

# Compilamos el proyecto
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                cd /catkin_ws && \
                catkin_make && \
                echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
                echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc"

# Establece el comando de inicio
CMD /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    roslaunch i2cpwm_board i2cpwm_node.launch"
