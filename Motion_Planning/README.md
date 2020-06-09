# FCND - 3D Motion Planning
[Detailed Project Explantion Page](https://github.com/udacity/FCND-Motion-Planning)

### Goals for this Project
1. Load the 2.5D map in the 'colliders.csv' file
2. Descritize the environment into a grid or graph representation
3. Define the start and goal locations
4. Perform a search using A* algorithm (or any other)
5. Use a collinearity test to remove any unnecessary points.
6. Return waypoints in local ECEF coordinates

### Files
1. `motion_planning.py` (available from the github repo link above)
2. `planning_utils.py`
3.`motion_planning_sid.py`
4. `planning_utils_sid.py`
5. `colliders.csv`
6. Videos folder contains .mov files of two tests
7. Need to use the software release for Udacity FCND platform developed on Unity.

### Code Write Up

#### Explanation of the starter code
To get an idea of the project background I ran the base files `motion_planning.py` and
`backyard_flyer_solution.py`. The comparison between their implementation is discussed below -

* Both the implementations are essentially [finite-state machines](https://en.wikipedia.org/wiki/Finite-state_machine). The `motion_planning.py` file has
an extra planning state than the `backyard_flyer_solution.py` implementation.

* The planning state is used to generate waypoints for the drone to reach its destination.

* More on the planning state and how [motion_planning.py](./motion_planning.py) works:

  - The mao is loaded from "Collider.csv" file and obtaining obstacle in the map**
    ```
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
    ```
  - The grid is calculated at [line 136](./motion_planning.py#L136) using the method [`create_grid`](./planning_utils.py#L6-L41) from the module [`planning_utils.py`](./planning_utils.py).
  - In the grid a Start and Goal Point are decalred
  ```
   grid_start = (-north_offset, -east_offset)

   grid_goal = (-north_offset + 10, -east_offset + 10)
  ```
  - To find the path to the goal, [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) is executed on [line 151](./motion_planning.py#L151) using the [`a_star`](./planning_utils.py#L91-L140) method from the module [`planning_utils.py`](./planning_utils.py).
  - The waypoints are generated at [line 157](./motion_planning.py#L157), and they are sent to the simulator using the method [`send_waypoints`](././motion_planning.py#L109-L112) at [line 161](./motion_planning.py#L161).

* Now we have some idea of how the function ``plan_path()`` in motion_planning.py works with the functions given under [planning_utils.py](./planning_utils.py). Now let us see how to further use these in finishing the project.

#### Implementation of new code
1. We saw before how the starter code works, where the starting and end location for drone
were where the drone initialized. Now we will see how to generalize that with any starting location.

  - The home position is read starting at [motion_planning_sid.py line 149](./motion_planning.py#L149). It uses the function [`latlon`](./planning_utils.py#L201) and ['position'](./planning_utils_sid#L193) added to `planning_utils_sid.py`.

2. In the starter code the drone takes off from the center of the map, but now we generalize to take off from anywhere.
   - A coordinate transformation is done for this to retrieve drone's current position in geodetic coordinates from `self._latitude()`, `self._longitude()` and `self._altitude()`. Then we use the utility function `global_to_local()` to convert to local position. This can be seen starting at [line 155](./motion_planning_sid.py#L155).

3. The starter code has our initial position hard coded in the code we change that to our local position.
   - This can be seen starting at [line 175](./motion_planning_sid.py#L175).

4. The goal position in the starter code was 10 m offset. Now we change to that to two arbitrary locations on the map.
   - The first goal position are:
   ```
   goal_longitude = -122.398414
   goal_latitude = 37.7939265
   ```
   - The second goal positions are:
   ```
   goal_longitude = -122.40199327
   goal_latitude = 37.79245808
   ```

5. Now the search algorithm implemented has following attributes:
   - Add Diagonal Motion Cost into Action class
   ```
       NORTH_WEST = (-1, -1, np.sqrt(2))
       NORTH_EAST = (-1, 1, np.sqrt(2))
       SOUTH_WEST = (1, -1, np.sqrt(2))
       SOUTH_EAST = (1, 1, np.sqrt(2))
       ```

       Also Check Obstacle for Diagonal Motion
   ```
       if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
           valid_actions.remove(Action.NORTH_WEST)
       if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
           valid_actions.remove(Action.NORTH_EAST)
       if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
           valid_actions.remove(Action.SOUTH_WEST)
       if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
           valid_actions.remove(Action.SOUTH_EAST)
   ```
6. In the starter code we are not reducing any unwanted waypoints. Let's implement that now.
   - Implemented the `prune_path()` function in [Line 163](./planning_utils_sid.py#L163) in `planning_utils_sid.py` file. I use a 3 point `collinearity_test` which is implemented at [Line 186](./planning_utils_sid.py#L186).

### Flight Execution
There were two tests executed after implementing the plannig algorithm above.
1. First goal positions shown in [video1](https://drive.google.com/drive/folders/1smfVqCxIpbu2L3pdMV6J-PktKnhIaPNm?usp=sharing)
2. Second goal positions shown in [video2](https://drive.google.com/drive/folders/1smfVqCxIpbu2L3pdMV6J-PktKnhIaPNm?usp=sharing)

Project done through Udacity's course Flying Autonomous Cars.
Author: Siddhant Tandon
Date: 8 June, 2020
