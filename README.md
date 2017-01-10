# Pathfinding
There are many different kinds of maze solving algorithms. Our implementation of D*Lite enables the scope feature to analyze obstacles within sight.

Building on A* and LPA*, D* lite uses a dynamically enabled algorithm with heuristic weighting. We built a modified D*lite algorithm, to dynamically change based on surroundings. Given a starting point and a goal, you will move closer to it, and react to obstacles that come into view. Our algorithm limits the range of vision of the moving object, and only calculates a path based on obstacles that it can see or has seen previously.
