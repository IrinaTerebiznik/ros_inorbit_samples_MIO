1. A. generar nuestro propio rosbag, para eso:
    ros2 bag record /my_magnetic_field /my_temperature
    B. en otro tab corremos el archivo fake_sensor_sample_rep-pu:
        cd ros2_ws/src
        colcon build --packages-select inorbit_republisher --symlink-install
        source /opt/ros/humble/setup.bash
        cd ros2_ws/src/inorbit_republisher/inorbit_republisher
        ros2 run inorbit_republisher fake_sensor_sample_rep --ros-args -p config:="/root/ros2_ws/src/inorbit_republisher/config/example.yaml"
    NO ME ENCUENTRA EL EJECUTABLE --> edite republisher.py para que publique lo que YO quiera.... y me hago mi rosbag
    usando ros2 bag record /my_magnetic_field /my_temperature
    como hice el ros2 bag recordie desde ros2_ws/src, lo movi a donde queria con mv "file" path
2. Corremos el package: 
        primeor hay que buildearlo...
        cd ros2_ws/src
        colcon build --packages-select inorbit_republisher --symlink-instal
        source install/setup.bash
    luego, coror el file
        cd ros2_ws/src/inorbit_republisher/test/sample_data
        ros2 launch inorbit_republisher sample_data.launch.xml
 Segun ros2, debe estar en la crpeta de luinch.
 Igualmente no me lo reconoce. Pero si corro ros2 launch inorbit_republisher sample_data.launch SI LO RECONOCE:

3. Abro otra terminal:
        cd ros2_ws/src/inorbit_republisher/test/sample_data
        source /opt/ros/humble/setup.bash
        ros2 topic echo my_temperature


Si me tira errores wtf: vuelvo a copiar mi file en root/ros2_ws/src/install/inorbit_republisher/share/inorbit_republisher>
        cd
       cp ros2_ws/src/inorbit_republisher/launch/sample_data.launch.xml /root/ros2_ws/src/install/inorbit_republisher/share/inorbit_republisher

     cp ros2_ws/src/inorbit_republisher/test/sample_data/config.yaml /root/ros2_ws/src/inorbit_republisher/test/sample_data/config/
/// alternativa a mi launch.xml que no funciona
1. corro mi rosbag haciendo esto en una terminal:
        cd /ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag
        ros2 bag play rosbag2_2024_12_13-19_33_34_0.db3
    en otra terminal:
        source /opt/ros/humble/setup.bash
        ros2 topic echo my_temperature
//////
    Pyuede ser qyue no funcione xq mi rosbag es finito_
 - AttributeError: Attribute 'exec' of type '<class 'str'>' not found in Entity 'node' este es el error


///
Consola de colores:
        cd 
        apt update && apt install nano -y
        nano ~/.bashrc
    Dentro del bashrc, abajo de todo, copio>
        force_color_prompt=yes
    GUardo y cierro el file
        alias ls='ls --color=auto'
        source ~/.bashrc
Para dejarlo predeterminado>


Puede ser que el error se de x altgo de paquetres no instlados o mal instalado...

