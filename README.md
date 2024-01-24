# ROS2 WS
In deze map staat alle code van ROS2 voor de LD90 en TM5-900 met ZED 2i 3D camera.

# Build
Als eerst moet er gecontroleerd worden of alle dependencies zijn geinstalleerd die ROS2 nodig heeft voor de te builden package.
Dit kan gedaan worden door de volgende command line uit te voeren in de ros2_ws directory
```
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

Gebruik het volgende in de terminal om recourses te beperken bij het build process:

```
export MAKEFLAGS="-j2 -l5"
```

Ga vervolgens naar de ros2_ws map en doe:
```
colcon build
```

Dit kan eventueel ook op de volgende manier als je wil dat je bij het aanpassen van de code hij dit ook meteen meeneemt.
Let op dat als je bestandnamen veranderd of nieuwe bestanden toevoegd dit niet werkt.
```
colcon build --symlink-install
```

## 1. Mappen structuur
Bij het clonen van de ze reposetory is er de volgende mappen structuur:
``` bash
MobileManipulator_ros2
├── images
│   ├── picture_1.jpg
│   ├── picture_2.jpg
│   └── │ │ 
├── ros2_ws
│   ├── src
│       ├── LD-90
│       ├── moma
│       ├── omron_moma
│       ├── TM5
│       ├── zed-ros2-examples
│       └── zed-ros2-wrapper
└── README.md
```
Zodra de build is uitgevoerd ziet het er als volgt uit:
``` bash
MobileManipulator_ros2
├── images
│   ├── picture_1.jpg
│   ├── picture_2.jpg
│   └── │ │ 
├── ros2_ws
│   ├── src
│   │   ├── LD-90
│   │   ├── moma
│   │   ├── omron_moma
│   │   ├── TM5
│   │   ├── zed-ros2-examples
│   │   └── zed-ros2-wrapper
│   ├── build
│   ├── log
│   └── install
└── README.md
```
De build, log en install folders worden door colcon gemaakt. Om vervolgens de code te kunnen gebruiken moet je de install folder toevoegen aan je .bashrc bestand. Hierdoor wordt bij het opstarten van een nieuwe terminal of door de huidige te sourcen (source ~/.bashrc) de packages geladen en kan je deze dus ook uitvoeren met ros2 run en ros2 launch.


## 2. Ros2 packages uitgelegd

### 2.1 LD-90
In de LD-90 package zitten 4 sub packages. Dit zijn amr_visualisation, om_aiv_msg, om_aiv_navigation en om_aiv_util.

#### 2.1.1 amr_visualisation
De amr_visualisation package wordt gebruikt voor de RVIZ visualisatie van ros2. In deze package zit de ingescande map en de 3D modellen van de robot. Het 3D model wordt in de visualisatie geplaatst volgens het .urdf bestand. Ook zit er in deze package 4 C++ bestanden die data verwerken voor de visualisatie van de robot. Dit zijn data_points_marker.cpp, goals_marker.cpp, joints_publisher.cpp en laser_scans.cpp. 

De data_points_marker leest het data.map bestand dat van de LD90 is gehaald na het inscannen van de map. Het programma converteert deze data.map naar een formaat dat RVIZ het kan visualiseren. Het haalt ook de verboden gebieden op uit de map en visualiseert deze ook in RVIZ. 

De goals_marker ontvangt de goals van de LD90 en converteert dit naar een formaat zodat deze op de map van RVIZ is te zien.

De joints_publisher zorgt ervoor dat de joints van de robot in RVIZ worden gevisualiseerd. Dit zorgt er daardoor voor dat het 3D model van de robot op de juiste positie op de map staat en in real time mee beweegt met de fysieke robot.

De laser_scans zorgt ervoor dat de laserscan van de robot in RVIZ wordt gevisualiseerd. Dit zijn de lasers die voor de navigatie worden gebruikt door de LD90. Nu is alleen de hoofdlaser aan de voorkant te zien maar de andere 3 lasers (2 zei lasers en 1 lage laser voor) zijn ook mogelijk om hier aan toe te voegen.

De package kan worden uitgevoerd door de volgende command line uit te voeren:

```
ros2 launch amr_visualisation display.launch
```

#### 2.1.2 om_aiv_msg
De packages van de LD90 maken gebruik van custom messages. Deze messages worden gemaakt door de om_aiv_msg package. Deze package bevat de messages die nodig zijn voor de communicatie tussen de verschilende nodes die draaien. In deze custom message bestanden staat gedefinieerd welke variabelen er in de message zitten, van welk type deze zijn (int, uint, string, float, etc..) en of er ook een gedefinieerd response moet zijn. dit laatste is alleen bij services en niet bij topics.

#### 2.1.3 om_aiv_navigation



#### 2.1.4 om_aiv_util


### 2.2 moma

### 2.3 omron_moma

### 2.4 TM5


### 2.5 zed-ros2-examples

### 2.6 zed-ros2-wrapper

## 3. Demo programma

