-# CS 440: Introduction to Artificial Intelligence
-# Professor Wang Xintong
-# Shreya Shukla and Medhasri Veldurthi
-# Programming Assignment #1
-# 09/30/2024

This assignment explores various algorithms and heuristics to help PacMan explore the map and obtain it's objectives to win the game. Some of the algorithms we programmed were BFS, DFS, UCS, and AStar graph searching algorithms. We also utilized different heuristics to optimize Pacman's search, such as Manhattan and Maze Distances, depending on the problem PacMan was trying to solve. To understand how to run our Pacman problem, you should run "python pacman.py -h" to learn about the different layouts, search agents, and problems you can run.
# Search: Implementing Pathfinding Algorithms for Pacman

## Introduction
In this project, I implemented search algorithms for Pacman to navigate through a maze, find food, and solve more complex problems. The project involved developing general search algorithms and applying them to different Pacman scenarios.

## Table of Contents
- [Introduction](#introduction)
- [Q1: Depth-First Search](#q1-depth-first-search)
- [Q2: Breadth-First Search](#q2-breadth-first-search)
- [Q3: Uniform Cost Search](#q3-uniform-cost-search)
- [Q4: A* Search](#q4-a-search)
- [Q5: Finding All Corners](#q5-finding-all-corners)
- [Q6: Corners Problem Heuristic](#q6-corners-problem-heuristic)
- [Q7: Eating All The Dots (Bonus)](#q7-eating-all-the-dots-bonus)
- [Q8: Suboptimal Search (Bonus)](#q8-suboptimal-search-bonus)
- [Submission](#submission)

## Q1: Depth-First Search
Implemented the depth-first search (DFS) algorithm in `search.py`. The agent successfully finds paths through mazes by exploring deeper nodes first.

### Execution Commands:
```bash
python pacman.py -l tinyMaze -p SearchAgent -a fn=dfs
python pacman.py -l mediumMaze -p SearchAgent -a fn=dfs
python autograder.py -q q1
```

## Q2: Breadth-First Search
Implemented the breadth-first search (BFS) algorithm in `search.py`. BFS finds the shortest path by expanding nodes layer by layer.

### Execution Commands:
```bash
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5
python autograder.py -q q2
```

## Q3: Uniform Cost Search
Implemented uniform-cost search (UCS) in `search.py` to account for varying path costs.

### Execution Commands:
```bash
python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
python autograder.py -q q3
```

## Q4: A* Search
Implemented A* search in `search.py`, using heuristics to improve search efficiency.

### Execution Commands:
```bash
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
python autograder.py -q q4
```

## Q5: Finding All Corners
Implemented a search problem that finds the shortest path visiting all four corners of the maze.

### Execution Commands:
```bash
python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
python autograder.py -q q5
```

## Q6: Corners Problem Heuristic
Developed a heuristic function to improve the A* search efficiency for the corners problem.

### Execution Commands:
```bash
python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5
python autograder.py -q q6
```

## Q7: Eating All The Dots (Bonus)
Implemented a heuristic function to efficiently find an optimal path that collects all food dots.

### Execution Commands:
```bash
python pacman.py -l testSearch -p AStarFoodSearchAgent
python pacman.py -l trickySearch -p AStarFoodSearchAgent
python autograder.py -q q7
```

## Q8: Suboptimal Search (Bonus)
Implemented a suboptimal search strategy where Pacman greedily eats the closest dot.

### Execution Commands:
```bash
python pacman.py -l bigSearch -p ClosestDotSearchAgent -z .5x
python autograder.py -q q8
```

## Submission
1. Submitted the project as a ZIP file `search_sol.zip` on Canvas, containing all `.py` files.
2. Included details of any discussions or collaborations in the assignment comments.

This project provided hands-on experience with classical search algorithms and their applications in AI-driven pathfinding problems.
