#!/bin/bash

# Archivo YAML de salida
OUTPUT_YAML="topics.yaml"

# Obtener namespace desde el primer argumento
NAMESPACE="$1"

# Diccionario de tipos válidos: ["gz_type_name"]="ros_type_name"
declare -A valid_types=(
  ["ignition.msgs.Actuators"]="actuator_msgs/msg/Actuators"
  ["ignition.msgs.Time"]="builtin_interfaces/msg/Time"
  ["ignition.msgs.Vector3d"]="geometry_msgs/msg/Point"
  ["ignition.msgs.Pose"]="geometry_msgs/msg/Pose"
  ["ignition.msgs.Pose_V"]="geometry_msgs/msg/PoseArray"
  ["ignition.msgs.PoseWithCovariance"]="geometry_msgs/msg/PoseWithCovariance"
  ["ignition.msgs.Quaternion"]="geometry_msgs/msg/Quaternion"
  ["ignition.msgs.Twist"]="geometry_msgs/msg/Twist"
  ["ignition.msgs.TwistWithCovariance"]="geometry_msgs/msg/TwistWithCovariance"
  ["ignition.msgs.Wrench"]="geometry_msgs/msg/Wrench"
  ["ignition.msgs.NavSat"]="gps_msgs/msg/GPSFix"
  ["ignition.msgs.Odometry"]="nav_msgs/msg/Odometry"
  ["ignition.msgs.OdometryWithCovariance"]="nav_msgs/msg/Odometry"
  ["ignition.msgs.Any"]="rcl_interfaces/msg/ParameterValue"
  ["ignition.msgs.Altimeter"]="ros_gz_interfaces/msg/Altimeter"
  ["ignition.msgs.Contact"]="ros_gz_interfaces/msg/Contact"
  ["ignition.msgs.Contacts"]="ros_gz_interfaces/msg/Contacts"
  ["ignition.msgs.Dataframe"]="ros_gz_interfaces/msg/Dataframe"
  ["ignition.msgs.Entity"]="ros_gz_interfaces/msg/Entity"
  ["ignition.msgs.EntityWrench"]="ros_gz_interfaces/msg/EntityWrench"
  ["ignition.msgs.Float_V"]="ros_gz_interfaces/msg/Float32Array"
  ["ignition.msgs.GUICamera"]="ros_gz_interfaces/msg/GuiCamera"
  ["ignition.msgs.JointWrench"]="ros_gz_interfaces/msg/JointWrench"
  ["ignition.msgs.Light"]="ros_gz_interfaces/msg/Light"
  ["ignition.msgs.LogicalCameraImage"]="ros_gz_interfaces/msg/LogicalCameraImage"
  ["ignition.msgs.Param"]="ros_gz_interfaces/msg/ParamVec"
  ["ignition.msgs.Param_V"]="ros_gz_interfaces/msg/ParamVec"
  ["ignition.msgs.SensorNoise"]="ros_gz_interfaces/msg/SensorNoise"
  ["ignition.msgs.StringMsg_V"]="ros_gz_interfaces/msg/StringVec"
  ["ignition.msgs.TrackVisual"]="ros_gz_interfaces/msg/TrackVisual"
  ["ignition.msgs.VideoRecord"]="ros_gz_interfaces/msg/VideoRecord"
  ["ignition.msgs.Clock"]="rosgraph_msgs/msg/Clock"
  ["ignition.msgs.BatteryState"]="sensor_msgs/msg/BatteryState"
  ["ignition.msgs.CameraInfo"]="sensor_msgs/msg/CameraInfo"
  ["ignition.msgs.FluidPressure"]="sensor_msgs/msg/FluidPressure"
  ["ignition.msgs.Image"]="sensor_msgs/msg/Image"
  ["ignition.msgs.IMU"]="sensor_msgs/msg/Imu"
  ["ignition.msgs.Model"]="sensor_msgs/msg/JointState"
  ["ignition.msgs.Joy"]="sensor_msgs/msg/Joy"
  ["ignition.msgs.LaserScan"]="sensor_msgs/msg/LaserScan"
  ["ignition.msgs.Magnetometer"]="sensor_msgs/msg/MagneticField"
  ["ignition.msgs.PointCloudPacked"]="sensor_msgs/msg/PointCloud2"
  ["ignition.msgs.Boolean"]="std_msgs/msg/Bool"
  ["ignition.msgs.Color"]="std_msgs/msg/ColorRGBA"
  ["ignition.msgs.Empty"]="std_msgs/msg/Empty"
  ["ignition.msgs.Float"]="std_msgs/msg/Float32"
  ["ignition.msgs.Double"]="std_msgs/msg/Float64"
  ["ignition.msgs.Header"]="std_msgs/msg/Header"
  ["ignition.msgs.Int32"]="std_msgs/msg/Int32"
  ["ignition.msgs.StringMsg"]="std_msgs/msg/String"
  ["ignition.msgs.UInt32"]="std_msgs/msg/UInt32"
  ["ignition.msgs.JointTrajectory"]="trajectory_msgs/msg/JointTrajectory"
  ["ignition.msgs.AnnotatedAxisAligned2DBox"]="vision_msgs/msg/Detection2D"
  ["ignition.msgs.AnnotatedAxisAligned2DBox_V"]="vision_msgs/msg/Detection2DArray"
  ["ignition.msgs.AnnotatedOriented3DBox"]="vision_msgs/msg/Detection3D"
  ["ignition.msgs.AnnotatedOriented3DBox_V"]="vision_msgs/msg/Detection3DArray"
)

echo "Esperando la adaptación del robot..."

sleep 10

echo "Generando listado de topics..."

# Obtener la lista de topics
topics=$(ign topic -l)

# Limpiar el archivo YAML si ya existe
echo "---" > "$OUTPUT_YAML"

# Iterar sobre cada topic
echo "Procesando topics..."
for topic in $topics; do
    echo "Analizando topic: $topic"

    # Filtro por namespace o topic /clock
    if [[ ! "$topic" =~ ^"/$NAMESPACE" ]]; then
        echo "Omitido por no coincidir con namespace o /clock"
        continue
    fi

    # Obtener información del topic
    type_info=$(ign topic -i -t "$topic" | grep -oP 'ignition.msgs.*' | head -n1)

    if [ -z "$type_info" ]; then
        type_info=$(ign topic -i -t "$topic" | grep -oP 'Subscribers \[.*\]:\s*\K.*' | awk '{print $NF}')
    fi

    if [ -z "$type_info" ]; then
        echo "No se pudo obtener el tipo del topic $topic, omitiendo..."
        continue
    fi

    ros_type="${valid_types[$type_info]}"
    if [ -z "$ros_type" ]; then
        echo "Tipo $type_info no está en la lista de tipos válidos, omitiendo..."
        continue
    fi

    # Guardar información en YAML
    echo "- ros_topic_name: \"$topic\"" >> "$OUTPUT_YAML"
    echo "  gz_topic_name: \"$topic\"" >> "$OUTPUT_YAML"
    echo "  ros_type_name: \"$ros_type\"" >> "$OUTPUT_YAML"
    echo "  gz_type_name: \"$type_info\"" >> "$OUTPUT_YAML"
    echo "  direction: GZ_TO_ROS" >> "$OUTPUT_YAML"

done

echo "Proceso finalizado. Archivo generado: $OUTPUT_YAML"

