# Game AI Project

#Extra
Week 2: Combined Steering
Exercise: Wolf hunting pattern

Agents first seek the target directly when they are far away. Once they enter a defined range, they switch to a "formation", where each agent moves to a other position on a circle around the target, distributed based on the number of agents

To make it work just pick new behavior WolfPack (at the very bottom) and set the same target for the whole wolfpack

==================

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