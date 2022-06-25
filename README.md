# Aprendizaje de la utilidad de objetos mediante observación de relación movimento humano y objeto
## Descripción
Para navegación semántica, detección de interacción entre persona y objeto.
El paquete **affordances_ros** permite clasificar 5 clases de actividades con objetos en viviendas para robots mobiles.
## Arquitectura
![Arquitectura](https://drive.google.com/uc?export=view&id=1_pdSDJWVPDOzU_oOk0t_P_ohv795si4v)
## Requisitos

- Ubuntu 20.04 LTS + ROS Noetic
- darknet_ros [[github](https://github.com/leggedrobotics/darknet_ros "github")]
- video_stream_opencv [[github](https://github.com/ros-drivers/video_stream_opencv "github")]
- Python 3.7 y librerías adicionales dentro de requeriments.txt

## Instalar

Para instalar todos los archivos necesarios, seguir las estructuras de paquetes de ros.
- Clonar este repositorio actual en el directorio de trabajo de ROS
- Clonar los repositorios de los requisitos mencionados en la lista darknet_ros y video_stream_opencv en el directorio de trabajo de ROS
- Instalar los requerimientos de modulos utilizados para python con la siguiente instrucción: pip install -r requeriments.txt
