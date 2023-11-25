# Traffic simulation environment based on boids with the addition of destination tracking and velocity adaptation

This simulation environment is an expansion to the code base of a [previous bachelor student](https://github.com/Jojojoppe/FlockingAlgorithmForVehicles). It adds destination tracking and simple velocity adaptation, and provides a more realistic traffic scenario with the possibility to add road markers and more accurate collision detection.

## Description
This code base makes use of openGL compute shaders with python driver code. The python dependencies can be installed using:

```
pip install requirements.txt
```

At the time of writing this, the code has only been tested succesfully on linux systems (debian-based and arch). An attempt to run the code on windows 11 was made, but there appears to be an issue with openGL context management when closing the window.

## World file layout
The world files under /maps/ (.wls, .spn and .loc files) have a very basic set of options. For all three files, it is important to not have an empty line between rows, because this will likely result in an error.

#### .wls
In the wls files, a line can be "commented out" by including a pound sign (#). A road marker is defined by typing EBW in the first column of any line. Defining a line from <strong>bottom to top</strong> (x1, y1, x2, y2) will result in a normal vector pointing to the <strong>right</strong> and the other way around. This is important for creating a map. When creating a (sharp) curve, it is recommended to do so using smaller line segments. In the previous code base, the camera functions in the map builder and regular simulation seemed to have had some issues. The camera functions for the normal simulation have been restored, but unfortunately, the map builder still seems to have some issues, which means maps have to be created manually for now.

#### .spn
In the .spn files, a single row will have the following layout:
```
spawnx, spawny, directionx, directiony, rate, speed
```
Where spawnx and spawny represent the start position of the vehicle, directionx and directiony indicate the direction the vehicle is pointing to. According to the previous student's code, rate indicates the number of vehicles per second (? Changing it seemed to not have a significant impact). Speed is the starting speed of the vehicles.

#### .loc
The loc files contain the location markers. A line can be commented out by typing #NA at the start of the line. The layout of the file is the following:
```
loc1x, loc1y, loc2x, loc2y, loc3x, loc3y, ....etc....
```


#### Notes
If there are any questions, you can contact me using my student email: t.f.limbeek@student.utwente.nl, or my personal email thijmen.limbeek10@gmail.com.