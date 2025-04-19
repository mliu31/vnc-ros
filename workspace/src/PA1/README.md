# Assignment: PA1 - Shape Draw

#### Author: Megan Liu

#### Class: COSC81.01 Principles of Robot Design and Programming

#### Instructor: Alberto Quattrini Li

#### Term: Spring 2025 (4/18/2025)

#### Description: This code implements shape drawing behavior for the ROSbot2. It draws 3 shapes: a trapezoid, a D, and a polygon. For the trapezoid and D, the user must input a radius. For the polygon, the user inputs a sequence of unique coordinates (relative to odom reference frame) in any order.

## Requirements

- ROS2 -- tested on ROS2 humble, but other versions may work.

## How to run:

1. Run the robot node/simulator.
2. In terminal, change directory into PA1

```
cd PA1
```

3. Run the python file

```
python3 pa1_shapes.py
```

4. It will prompt you to choose (1) an isosceles trapezoid, (2) a D, or (3) a polygon. You can input 1, 2, or 3.

4a. If you chose 1 or 2, it will prompt you to enter a radius. Then it will draw a trapezoid or D of that radius.
4b. If you chose 3, it will prompt you to enter coordinates x,y until you press enter. Then it will draw a polygon with vertices as the coordinates entered starting and ending at its current position.
