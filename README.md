# CMPN462 Cognitive Robotics Term Project

## Run
- first Run Gazebo and Rviz
```bash
    roslaunch summit_xl_sim_bringup summit_xls_complete.launch
```
- Then In the project folder open a terminal and run
```bash
    catkin_make
    roslaunch launcher_pkg run_all.launch
 ```

## Directory Structure
```bash
.
├── build
├── Cognitive Robotics - Project document  .pdf
├── frames.pdf
├── imgs
│   ├── Map.png
│   └── ourMap.png
├── README.md
└── src
    ├── CMakeLists.txt
    ├── launcher_pkg
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   └── run_all.launch
    │   └── package.xml
    ├── mapping_pkg
    │   ├── CMakeLists.txt
    │   ├── msg
    │   │   └── Readings.msg
    │   ├── package.xml
    │   ├── script
    │   │   ├── mappingWithKnownPoses.py
    │   │   ├── __pycache__
    │   │   ├── sensorAlignment.py
    │   │   └── utils.py
    │   └── src
    └── motion_pkg
        ├── CMakeLists.txt
        ├── package.xml
        ├── script
        │   └── motion.py
        └── src

```


 ## ScreenShots
### Real Map
![Real Map](imgs/Map.png)

### partially Generated Map 
![Our Map](imgs/ourMap.png)

## Contributors

- [Ahmed Hussien](https://www.github.com/Ahmedh12)
- [Millania Sameh](https://www.github.com/millaniaSameh)
- [Nada Abdelrahman](https://www.github.com/nadaabdelgayed)
- [Nouran Shawky]()

