# ROS2 WS
In deze map staat alle code van ROS2.

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

