# Game AI Project

This project implements a Flocking System (Boids) in Unreal Engine with an optional Spatial Partitioning (Cell Space Partitioning) optimization for efficient neighbor searching.
==================
-IMPORTANT
==================
To turn OFF/ON Spacial partitioning, press D 
Spacial partitioning is already working with debbuging, on my computer with 500 boids I have around 25 FPS (Without partitioning) and 28 with spacial partitioning - However withour render debbuging it will be even more.

Features I implemented:
- Separation - Avoid crowding neibhbors 
- Cohesion - move toward average neighbor position
- Aligment - match neighbour velocity
- Wander - random movement toward random point on circle in front of player
- Evade - evade a target agent

Flocking also use priority steering and avoid boid that player can move with mouse click