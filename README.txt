================================================================================
AI PROGRAMMING ASSIGNMENT README

================================================================================

1. INTRODUCTION
   ================================================================================

This project demonstrates the implementation of classical and modern search
algorithms used in Artificial Intelligence for pathfinding problems.

Three different scenarios are covered:

• Graph-based shortest path (Dijkstra)
• Grid-based navigation with known obstacles (A*)
• Real-time navigation with unknown dynamic obstacles (Adaptive A*)

These programs simulate how an Unmanned Ground Vehicle (UGV) finds optimal
paths under different environmental conditions.

================================================================================
2. FILES INCLUDED
=================

1.py         - Shortest path using Dijkstra algorithm
2.py       - A* pathfinding with static obstacles
3.py      - Adaptive replanning A* (dynamic environment)
README.txt          - Documentation file

================================================================================
3. SYSTEM REQUIREMENTS
======================

* Python 3.7 or higher
* No external libraries required

Standard libraries used:
heapq   → Priority queue operations
random  → Obstacle generation
time    → Execution time measurement

================================================================================
4. PROGRAM 1 - DIJKSTRA (CITY GRAPH)
====================================

DESCRIPTION:
Implements Dijkstra’s algorithm on a network of cities connected by roads.

FEATURES:

* Computes shortest distance from source to all cities
* Displays complete path for each destination
* Uses adjacency list for efficient representation

INPUT:

* Source city index

OUTPUT:

* Distance table
* Full path routes

COMPLEXITY:
Time  : O((V + E) log V)
Space : O(V + E)

================================================================================
5. PROGRAM 2 - A* PATHFINDING (STATIC OBSTACLES)
================================================

DESCRIPTION:
Simulates a UGV navigating a 70×70 grid where obstacles are fixed and
fully known before movement begins.

GRID DETAILS:

* Size        : 70 × 70
* Movement    : 8 directions
* Costs       : Straight = 10, Diagonal = 14

OBSTACLE LEVELS:

* Low    : 10%
* Medium : 25%
* High   : 40%

ALGORITHM:
A* Search using Octile Distance heuristic

KEY POINTS:

* Guarantees shortest path
* Avoids diagonal corner cutting
* Efficient compared to Dijkstra

OUTPUT:

* Grid visualization
* MOE statistics

================================================================================
6. PROGRAM 3 - ADAPTIVE A* (DYNAMIC OBSTACLES)
==============================================

DESCRIPTION:
Extends static pathfinding to real-world conditions where obstacles are
unknown and can appear dynamically during navigation.

FEATURES:

* Partial knowledge of environment
* Sensor-based updates (local visibility)
* Dynamic obstacle generation
* Continuous replanning

MAP TYPES:
world[][] → Actual environment
known[][] → Discovered environment

WORKING:

1. Initial scan and planning
2. Move step-by-step
3. Detect obstacles using sensor
4. Replan if path is blocked

ADVANTAGE:
Handles uncertainty and changing environments effectively

================================================================================
7. EXECUTION INSTRUCTIONS
=========================

Run each program using:

python dijkstra.py
python ugv_static.py
python ugv_dynamic.py

On Linux/Mac:
python3 filename.py

================================================================================
8. SAMPLE RUNS
==============

PROGRAM 2:
Input:
Density : Medium
Start   : (0,0)
Goal    : (69,69)

Output:
Path Found
Steps        : ~75
Cost         : ~1000
Nodes        : ~500

PROGRAM 3:
Output:
Replans      : Multiple events
Steps        : ~70–90
Dynamic Obs  : Random
Goal         : Reached / Blocked

================================================================================
9. ALGORITHMS OVERVIEW
======================

Dijkstra       → Uninformed search (no heuristic)
A*             → Informed search (heuristic-based)
Adaptive A*    → Real-time replanning algorithm

WHY A*?

* Faster than Dijkstra in grids
* Uses heuristic to guide search

WHY ADAPTIVE A*?

* Works with unknown environments
* Suitable for real-world robotics

================================================================================
10. PERFORMANCE METRICS (MOE)
=============================

The following are measured:

• Path length (steps)
• Path cost
• Nodes explored
• Computation time
• Obstacle density
• Replanning count (dynamic case)
• Dynamic obstacles generated
• Goal reached status

These metrics evaluate efficiency and reliability.

================================================================================
11. PYTHON VS C COMPARISON
==========================

Python:
- Easier to write and debug
- Built-in data structures
- Slower execution

C:
- Faster performance
- Manual memory handling
- More complex code

Both implementations produce identical logical results.

================================================================================
12. ASSUMPTIONS
===============

* Obstacles are randomly generated
* Results vary between executions
* Grid is square and uniform
* Sensor range is limited in dynamic case
* Movement cost approximates real distance

================================================================================
END OF DOCUMENT
===============
