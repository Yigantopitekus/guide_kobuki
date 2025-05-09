# guide_kobuki

© 2025 Hugo Gordo Gil,Mateo Ferreira Gonzalez y Roberto Donate Lario
Algunos derechos reservados. Este trabajo se entrega bajo la licencia [CC BY-SA 4.0].

En este README se analiza el proyecto desarrollado como práctica final para la asignatura de **Arquitectura de Software para Robots**. Incluye videos que muestran el robot en funcionamiento, un análisis detallado del código que compone el proyecto, los comandos necesarios para ejecutar todos los programas, y una sección dedicada a los recursos utilizados

---

### **OBJETIVO Y FUNCIONALIDAD DEL PROYECTO**

A través de la implementación de Nav2, el uso de behaviour trees y la integración de nodos externos, hemos desarrollado un conjunto de nodos que dan vvida a un **Robot Kobuki Bibliotecario**, este esta diseñado para poder guiar a los usuarios de manera eficiente dentro de una biblioteca. Medaiante la interacción con los botones incorporados en el robot o la lectura de códigos QR asociados a destinos especificos, el kobuki puede dirigir al usuario hacia la estantería correcta según la información porporcionada. Una vez cumplida su misión, el robot regresa de forma autónoma al punto de partida.

---

### **INSTRUCCIONES DE USO**  

#### **Elementos externos utilizados**  
Este proyecto hace uso de herramientas externas que deben instalarse previamente para garantizar su correcto funcionamiento:  

##### **Paquete de detección de objetos**  
El paquete `find_object_2d` se utiliza para detectar y reconocer objetos. Puedes instalarlo ejecutando el siguiente comando:  
```bash
sudo apt-get install ros-$ROS_DISTRO-find-object-2d
cd ~/asr_ws
git clone https://github.com/introlab/find-object.git src/find_object_2d
```
#### **Elementos de procesamiento de imagenes**
Instalación de `cv_brigde` y de OpenCV
```bash
cd ~/asr_ws/src
git clone https://github.com/ros-perception/vision_opencv.git -b humble
cd ~/asr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-slect cv_bridge
source install/setup.bash
ros2 pkg list | grep cv_bridge

```

#### **Comandos de lanzamiento del proyecto**
Para ejecutar el proyecto y poner el robot en funcionamiento, sigue los siguientes pasos:

1.- Clonar el repositorio:
```bash
cd home/asr_ws/src
git clone git@github.com:Yigantopitekus/guide_kobuki.git
```
2.- Compilar y configurar el espacio de trabajo:
```bash
cd /home/asr_ws
colcon build --symlink-install
source install/setup.bash
```
3.- Ejecutar los nodos del proyecto:
Primero lanzamos el robot junto a la camara y el laser, en este caso usaremos la camara xtion y el laser a2:
```bash
ros2 launch kobuki kobuki.launch.py xtion:= true lidar_a2 := true
```
Una veza lanzado el kobuki y el resto de elementos, lanzamos nav2, en nuestro caso le proporcionaremos la ruta completa al mapa que usaremos para la navegación:
```bash
ros2 launch kobuki navigation.launch.py map:= ./asr_ws/src/kobuki/maps/mapale.yaml
```
Ahora procedemos a lanzar el main de nuestro proyecto ademas del launch del paquete find_objesct_2d que instalamos previamente:

Main del proyecto:
```bash
ros2 launch guide_kobuki guide_kobuki.launch.py
```
Lanzamos find_object_2d añadiendo un remapping en el comando dandole el topic de la camara que vayamos a usar y el path hasta el lugar en el que hemos registrado los qr que usaremos y explicaremos mas adelante:
```bash
ros2 launch find_object_2d find_object_2d.launch.py image_topic:=/rgb/image_raw objects_path:=src/find-object/qrs/
```
---
### **DEMOSTRACION EN VIDEO**

#### Video de demostración del moviento de ida y vuelta del robot haciendo uso de los botones:

[![Demostración en Video](https://img.youtube.com/vi/2WWr73CPYi4/0.jpg)](https://youtube.com/shorts/2WWr73CPYi4)


#### Video de demostración del moviento de ida y vuelta del robot haciendo usando los qr:

[![Demostración en Video](https://img.youtube.com/vi/2WWr73CPYi4/0.jpg)](https://youtube.com/shorts/2WWr73CPYi4?feature=share)


---
### **ANALISIS DEL CODIGO**

Ahora vamos a analizar la estructura y el conjunto de nodos usados para el funcionamiento del programa además del behaviour tree:

#### **1.- Estructura completa del contenido**
```bash
.
├── bt_xml
│   └── library.xml
├── CMakeLists.txt
├── config
│   └── params.yaml
├── include
│   └── library_lib
│       ├── ctrl_support
│       │   └── BTActionNode.hpp
│       ├── NavObjective.hpp
│       ├── NavOdom.hpp
│       ├── ReadQR.hpp
│       ├── Search.hpp
│       └── StoreObject.hpp
|        |__ ReadFaces.hpp
├── launch
│   └── guide_kobuki.launch.py
├── package.xml
├── README.md
└── src
    ├── guide_nav_main.cpp
    └── library_lib
        ├── NavObjective.cpp
        ├── NavOdom.cpp
        ├── ReadQR.cpp
        ├── Search.cpp
        └── StoreObject.cpp
        |__ ReadFaces.cpp
```
#### **2.- Behaviour tree**
El Behaviour Tree del proyecto toma decisiones siguiendo una estructura jerárquica de nodos que ejecutan acciones o evaluaciones lógicas. A continuación, se explica brevemente el funcionamiento:  

##### **Estructura del árbol:**  
   - **Nodo principal:** Se utiliza un nodo `KeepRunningUntilFailure` que ejecuta continuamente las tareas hasta que alguna de ellas falla.  
   - **Secuencia (`Sequence`):** Todos los nodos hijos dentro de este deben completarse exitosamente en orden. Si uno falla, la secuencia completa falla.  
   - **Alternativa (`Fallback`):** Intenta ejecutar cada nodo hijo en orden hasta que uno de ellos tiene éxito. Si todos fallan, la alternativa falla.  

##### **Flujo de acciones:**  
   - El robot busca (`Search`) o lee un código QR (`ReadQR`) para determinar un destino.  
   - Navega hacia el objetivo (`NavObjective`).  
   - Al llegar al destino, toma una fotografía con `StoreObject` para guardar una referencia visual, que será utilizada posteriormente para verificar que el robot regresa correctamente al mismo punto.  
   - Una vez cumplida su tarea, utiliza datos de odometría (`NavOdom`) para regresar de manera autónoma al punto de partida.  

Este Behaviour Tree garantiza que el robot continúe intentando sus objetivos mientras no ocurra un fallo crítico, asegurando un ciclo completo de búsqueda, navegación, toma de datos y retorno al origen. 

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
        <KeepRunningUntilFailure>
            <Sequence>
                <Action ID="ReadFace"/>
                <Fallback>
                    <Action ID="Search"/>
                    <Action ID="ReadQR"/>
                </Fallback>
                <Action ID="NavObjective"/>
                <Action ID="NavOdom"/>
            </Sequence>
        </KeepRunningUntilFailure>
    </BehaviorTree>
    <!-- ////////// -->
    <TreeNodesModel>
        <Undefined ID=""/>
        <Action ID="NavObjective"/>
        <Action ID="NavOdom"/>
        <Action ID="ReadQR"/>
        <Action ID="Search"/>
        <Action ID="StoreObject"/>
        <Action ID="ReadFace"/>
    </TreeNodesModel>
    <!-- ////////// -->
</root>
```

#### **3.-Resto de Nodos importantes**
Los nodos mas importantes y destacables de los que hablaremos un poco ahora son: `NavObjective.cpp`, `NavOdom.cpp`, `Search.cpp`, `StoreObject.cpp`

##### **Nodo `NavObjective` - Navegación hacia el objetivo**

Este nodo utiliza la acción `NavigateToPose` para mover al robot hacia un destino obtenido de la blackboard, ejecutándose dentro del Behaviour Tree.

**Esto es lo más importante de este código:**  
```c++
void
NavObjective::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  config().blackboard->get("waypoint", goal);

  goal_.pose = goal;

  goal_.pose.header = goal.header;
  goal_.pose.header.stamp = node_->get_clock()->now();

}
```

##### **Nodo `NavOdom` - Retorno al punto de inicio**

Este nodo utiliza la acción `NavigateToPose` para guiar al robot de vuelta al punto de inicio. Establece un objetivo fijo con coordenadas predeterminadas y ejecuta la navegación dentro del Behaviour Tree.

**Esto es lo más importante de este código:**  
```c++
void
NavOdom::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "**ENTRO A ODOM**");

  geometry_msgs::msg::PoseStamped target_pose;


  target_pose.header.frame_id = "map";
  target_pose.header.stamp = node_->now();

  target_pose.pose.position.x = 0;
  target_pose.pose.position.y = 0;
  target_pose.pose.orientation.w = 0;

  goal_.pose = target_pose;

}

```

##### **Nodo `Search` - Selección de destino mediante botones**

Este nodo permite al usuario seleccionar un destino dentro de la biblioteca utilizando los botones del robot Kobuki. Cada botón tiene una funcionalidad específica: avanzar, retroceder o confirmar la selección. La ubicación seleccionada se almacena en la blackboard para su posterior uso en el Behaviour Tree.

**Esto es lo más importante de este código:**  
```c++
BT::NodeStatus
Search::tick()
{
  double x = 0.0;
  double y = 0.0;
  double w = 1.0;
  
  if(last_button_ == NULL)
  {
      return BT::NodeStatus::RUNNING;
  }
   switch(last_button_->button)
        {
          case 0:
            if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED)
            {
              idx_++;
              if(idx_ >= size_)
              {
                idx_ = 0;
              }
            }
            break;
          case 1:
            if(last_button_->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED)
              {
                idx_--;
                if(idx_ < 0)
                {
                  idx_ = size_-1;
                }

              }
            break;
          case 2:
            last_button_ = NULL;
            std::string name = arr_[idx_];
            if(idx_ == 4)
            {
              idx_ = 0;
              return BT::NodeStatus::FAILURE;
            }
            idx_ = 0;

            node_->get_parameter(name + ".x", x);
            node_->get_parameter(name + ".y", y);
            node_->get_parameter(name + ".w", w);

            wp_.header.frame_id = "map";
            wp_.pose.orientation.w = w;
            wp_.pose.position.x = x;
            wp_.pose.position.y = y;
            
            config().blackboard->set("waypoint", wp_);
            return BT::NodeStatus::SUCCESS;
      }
  last_button_ = NULL;
  return BT::NodeStatus::RUNNING;

}

```

##### **Nodo `StoreObject` - Captura y almacenamiento de imágenes** 

`StoreOBject no se usa finalmente en esta practica porque no hemos llegado a implementarlo, pero por separado el nodo es completamente funcional`

Este nodo suscribe al robot a un flujo de imágenes, captura una cuando está disponible y la guarda en el sistema de archivos con un nombre de archivo basado en un timestamp. El nodo asegura que solo se guarda una imagen y cambia su estado a **éxito** cuando la imagen ha sido almacenada correctamente.

**Esto es lo más importante de este código:**  
```c++
void StoreObject::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!image_saved_) {
        try {
            // Convertir la imagen de ROS a OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Nombre del archivo con timestamp
            std::string filename = "/home/hugo1234/Pictures/" + getTimestamp() + ".jpg";
            cv::imwrite(filename, cv_ptr->image);

            RCLCPP_INFO(node_->get_logger(), "Imagen guardada en: %s", filename.c_str());
            image_saved_ = true;  // Marca la imagen como guardada
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Error al procesar la imagen: %s", e.what());
        }
    }
}
```
#### **Nodo `ReadFace` - Reconocimiento de rostros y saludo personalizado**

Este nodo procesa un flujo de datos de identificación de rostros, interpreta el ID asociado al rostro detectado y genera un mensaje de bienvenida personalizado en la consola para cada persona reconocida.

**Esto es lo más importante de este código:**  
```c++
void
ReadFace::object_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if(msg != nullptr && !msg->data.empty())
  {
    last_object_ = *msg;
    id_ = msg->data[0];
  }
}
void library_lib::ReadFace::halt()
{

}
BT::NodeStatus
ReadFace::tick()
{

  if(last_object_.data.empty())
  {  
    return BT::NodeStatus::RUNNING;
  }
  switch(id_)
  {
    case id_Roberto_:
    std::cout << "Bienvenido Roberto" << std::endl;
      return BT::NodeStatus::SUCCESS;
    case id_Hugo_:
    std::cout << "Bienvenido Hugo" << std::endl;

        return BT::NodeStatus::SUCCESS;  
    case id_Mateo_:
    std::cout << "Bienvenido Mateo" << std::endl;

        return BT::NodeStatus::SUCCESS;
    
  }
  return BT::NodeStatus::RUNNING;

}
```


---

### **INTEGRACIÓN DE UN NODO USANDO UN PAQUETE NUEVO**

Para realizar la integración de los qr en el proyecto hemos hecho uso del paquete find_object_2d que hemos instalado previamente, ahora veremos su funcionamiento, su uso y la manera en la que lo integramos a nuestro programa:

#### **Funcionamiento**

##### **Registro de objetos**

En este video mostramos como registramos un qr:

[![Demostración en Video](https://img.youtube.com/vi/JCyi_G20dUE/0.jpg)](https://youtu.be/JCyi_G20dUE)

##### **Puesta en Funcionamiento y muestras de uso**

En el siguiente video podremos ver como después de registrar el qr anteriormente el objeto queda guardado y cuando se le vuelve a acercar la aplicación detecta el objeto ya registrado, a la derecha podemos ver como esta publica la información en un topic por lo que podemos comprobar que si lo esta detectando:

[![Demostración en Video](https://img.youtube.com/vi/tmRre-DyL5w/0.jpg)](https://youtu.be/tmRre-DyL5w)

 En este otro video podemos ver como también podemos usar esto para reconocer incluso caras:

[![Demostración en Video](https://img.youtube.com/vi/17mi5V6N8Ek/0.jpg)](https://youtu.be/17mi5V6N8Ek)



#### **Integracion**

##### **Nodo `ReadQR` - Lectura de códigos QR y asignación de destino**

Gracias al paquete `find_object-2d` que instalamos anteriormente, podemos ejecutar la aplicación de detección de objetos. Cuando registramos un objeto y lo colocamos frente a la cámara del robot Kobuki, podemos observar que el objeto se marca con un color diferente, indicando que ya está siendo detectado.

Para integrar esto en nuestro programa, hemos hecho un *echo* del topic en el que `find_object-2d` publica los mensajes de tipo `std_msgs/Float32MultiArray`. Este topic es `/objects`. Al hacerlo, podemos ver los datos mostrados en el video, los cuales contienen la siguiente información: `objectId1`, `objectWidth`, `objectHeight`, `h11`, `h12`, `h13`, `h21`, `h22`, `h23`, `h31`, `h32`, `h33`. Estos datos se muestran en la terminal una vez el objeto está cerca de la cámara.

De esta información, en nuestro código aprovechamos una suscripción al topic para obtener el primer valor del array `data[0]`, que corresponde al `id` del objeto detectado. Esto nos permite diferenciar entre los diferentes QR y asignarles movimientos específicos hacia lugares determinados. Para mejor comprensión este es el codigo que realiza esto: 

**Esto es lo más importante de este código:**
```c++
#include "library_lib/ReadQR.hpp"

enum{
  id_ciencia_ = 2,
  id_literatura_ = 3,
  id_historia_ = 4,
  id_infantil_ = 5,
};
namespace library_lib
{

ReadQR::ReadQR(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
{

  config().blackboard->get("node", node_);
  node_->get_parameter("ciencia.x", cienciax_);
  node_->get_parameter("ciencia.y", cienciay_);
  node_->get_parameter("ciencia.w", cienciaw_);

  node_->get_parameter("literatura.x", literaturax_);
  node_->get_parameter("literatura.y", literaturay_);
  node_->get_parameter("literatura.w", literaturaw_);

  node_->get_parameter("infantil.x", infantilx_);
  node_->get_parameter("infantil.y", infantily_);
  node_->get_parameter("infantil.w", infantilw_);

  node_->get_parameter("historia.x", historiax_);
  node_->get_parameter("historia.y", historiay_);
  node_->get_parameter("historia.w", historiaw_);

  object_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/objects", 10, std::bind(&ReadQR::object_callback,this, std::placeholders::_1));
  turn_ = false;



}


void
ReadQR::object_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

  if(msg != nullptr && !msg->data.empty())
  {
    last_object_ = *msg;
    id_ = msg->data[0];
  }
}
void library_lib::ReadQR::halt()
{

}
BT::NodeStatus
ReadQR::tick()
{

  if(last_object_.data.empty())
  {  
    return BT::NodeStatus::RUNNING;
  }
  switch(id_)
  {
    case id_literatura_:
      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = literaturaw_;
      wp_.pose.position.x = literaturax_;
      wp_.pose.position.y = literaturay_;

      config().blackboard->set("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_historia_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = historiaw_;
      wp_.pose.position.x = historiax_;
      wp_.pose.position.y = historiay_;

      config().blackboard->set("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_infantil_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = infantilw_;
      wp_.pose.position.x = infantilx_;
      wp_.pose.position.y = infantily_;

      config().blackboard->set("waypoint", wp_);

      return BT::NodeStatus::SUCCESS;
    case id_ciencia_:

      wp_.header.frame_id = "map";
      wp_.pose.orientation.w = cienciaw_;
      wp_.pose.position.x = cienciax_;
      wp_.pose.position.y =  cienciay_;

      config().blackboard->set("waypoint", wp_);
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<library_lib::ReadQR>("ReadQR");
}

```

----

### **FUENTES Y REFERENCIAS**

*[PAQUETE FIND_OBJECTS](https://github.com/introlab/find-object?tab=readme-ov-file)*

*[DOCUMENTACION FIND_OBJECTS](https://wiki.ros.org/find_object_2d)*

*[CV_BRIDGE OPENCV](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge#cv_bridge-in-ros-2)*

*[DOCUMENTACION1 CV_BRIDGE](https://docs.ros.org/en/noetic/api/cv_bridge/html/c++/namespacecv__bridge.html#acbf2da402f4d3e505613e95b5a2aed35)*

*[DOCUMENTACION2 CV_BRIDGE](https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)*























