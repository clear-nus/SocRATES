
### All below answers were without the scene graph image

Now answer the following questions:
1. If I am at node with id = 6, which nodes can I reach? 
2. If I am at node with id =  3, which nodes can I reach?
3. If I am at node with id = 1, which nodes can i reach?

To determine which nodes you can reach from a given node in a scene graph, you need to refer to the connections (edges) specified in the 'links' section of the JSON. Since the graph is bidirectional, you can move from one node to another and back through these connections.

1. If you are at node with id = 6, which nodes can you reach?
   - Direct connections from node 6 as per 'links':
     - Node 6A: {"source": "6", "target": "6A"}
     - Node 6B: {"source": "6", "target": "6B"}
     - Node 24: {"source": "6", "target": "24"}
     - Node 7: {"source": "6", "target": "7"}
     - Node 23: {"source": "6", "target": "23"}
     
     So from node 6, you can reach nodes: 6A, 6B, 24, 7, 23.

2. If you are at node with id = 3, which nodes can you reach?
   - Direct connections from node 3 as per 'links':
     - Node 3A: {"source": "3", "target": "3A"}
     - Node 3B: {"source": "3", "target": "3B"}
     - Node 3C: {"source": "3", "target": "3C"}
     - Node 5: {"source": "3", "target": "5"}
     - Node 4: {"source": "3", "target": "4"}
     - Node 6: {"source": "3", "target": "6"}
     - Node 24: {"source": "3", "target": "24"}
     
     So from node 3, you can reach nodes: 3A, 3B, 3C, 5, 4, 6, 24.

3. If you are at node with id = 1, which nodes can you reach?
   - Direct connections from node 1 as per 'links':
     - Node 1A: {"source": "1", "target": "1A"}
     - Node 1B: {"source": "1", "target": "1B"}
     - Node 2: {"source": "1", "target": "2"}
     
     So from node 1, you can reach nodes: 1A, 1B, 2.

By analyzing the 'links' section of the JSON, you can determine the direct connections (edges) for any given node (id). This approach helps to find which nodes are reachable directly without traversing through other nodes.


Now answer the following questions:
1. If I am at node with id = 23, which nodes can I reach? 
2. If I am at node with id =  9, which nodes can I reach?
3. If I am at node with id = 31, which nodes can I reach?
4. If I am at node with id = 2, which nodes can I reach?


To determine the nodes you can reach from a specific node in a bidirectional graph, you can directly look at the connections or edges listed in the 'links' section of the JSON. Here are the reachable nodes for each specified node:

1. **If you are at node with id = 23, which nodes can you reach?**
   - From the JSON 'links' section:
     - `23` has connections to `23A`, `23B`, `23C`, `7`, `6`, `21`, `24`
   - Reachable nodes: **23A, 23B, 23C, 7, 6, 21, 24**

2. **If you are at node with id = 9, which nodes can you reach?**
   - From the JSON 'links' section:
     - `9` has connections to `9A`, `9B`, `15`
   - Reachable nodes: **9A, 9B, 15**

3. **If you are at node with id = 31, which nodes can you reach?**
   - From the JSON 'links' section:
     - `31` has connections to `31A`, `31B`, `31C`, `30`, `32`, `22`
   - Reachable nodes: **31A, 31B, 31C, 30, 32, 22**

4. **If you are at node with id = 2, which nodes can you reach?**
   - From the JSON 'links' section:
     - `2` has connections to `2A`, `2B`, `5`, `1`
   - Reachable nodes: **2A, 2B, 5, 1**

For ensuring the reachability, the type of node (INTERSECTION, PASSAGEWAY, OPEN AREA, CORNER, child) does not restrict the movement as all nodes are interconnected based on the edges specified.

For the following questions, ignore 'child' type nodes.
Q. If I walk from node 31 to node 32 and take a left turn which node will I reach first? 
Q. If I walk from node 9 towards node 15 and then take a left turn, which node will I reach first?
Q. If I walk from node 25 towards node 20 and take a left turn, which node will I reach first?
Q. If I walk from node 3 to node 24 and take a left turn which node will I reach first?
Q. If I walk from node 21 towards 22 and continue in the same direction, which node will I reach first?
Q. If I walk from node 2 towards node 5, then keep walking, which node will I reach first and then after that node which node will I reach second?

To answer these questions, we need to use the scene graph provided in the JSON format. 

Firstly, we should define the concept of "taking a left turn." The JSON doesn't provide explicit compass directions, so we will assume that "taking a left turn" means turning counterclockwise relative to the current direction of travel. Since the graph is bidirectional, we need to consider the layout implied by the positions of the nodes.

Let's address each question:

### Q1. If I walk from node 31 to node 32 and take a left turn which node will I reach first?

1. Nodes on the path: 31 -> 32.
2. At 32, "taking a left turn" usually implies moving counterclockwise.
3. From the graph data:
   - Node 32 is connected to nodes 31, 32A, 32B, and 33, but we ignore child nodes.
   - Adjacent non-child node apart from 31 and 32: Node 33.
4. Therefore, you will reach Node 33 first.

### Q2. If I walk from node 9 towards node 15 and then take a left turn, which node will I reach first?

1. Nodes on the path: 9 -> 15.
2. At 15, "taking a left turn" usually implies moving counterclockwise.
3. From the graph data:
   - Node 15 is connected to nodes 7, 9, 15A, 15B, 15C, 16, and 21.
   - Adjacent non-child nodes: Nodes 7, 9, 16, and 21.
4. Considering the position:
   - Counterclockwise turn: From node 9 (at top), the left turn should ideally lead to node 21.
5. Therefore, you will reach Node 21 first.

### Q3. If I walk from node 25 towards node 20 and take a left turn, which node will I reach first?

1. Nodes on the path: 25 -> 20.
2. At 20, "taking a left turn" usually implies moving counterclockwise.
3. From the graph data:
   - Node 20 is connected to nodes 19 and 25.
   - Adjacent non-child nodes: Node 25, node 19.
4. Considering the position and counterclockwise direction: likely Node 19.
5. Therefore, you will reach Node 19 first.

### Q4. If I walk from node 3 to node 24 and take a left turn which node will I reach first?

1. Nodes on the path: 3 -> 24.
2. At 24, "taking a left turn" usually implies moving counterclockwise.
3. From the graph data:
   - Node 24 is connected to nodes 3, 6, 23.
   - Adjacent non-child nodes: Nodes 3, 6, 23.
4. Considering the position and counterclockwise direction: likely Node 23.
5. Therefore, you will reach Node 23 first.

### Q5. If I walk from node 21 towards 22 and continue in the same direction, which node will I reach first?

1. Nodes on the path: 21 -> 22.
2. Continuing in the same direction:
3. From the graph data:
   - Node 22 is connected to nodes 21 and 25.
   - Adjacent non-child nodes: Node 25.
4. Therefore, you will reach Node 25 first.

### Q6. If I walk from node 2 towards node 5, then keep walking, which node will I reach first and then after that node which node will I reach second?

1. Nodes on the path: 2 -> 5.
2. From the graph data:
   - First we reach Node 5.
   - Keeping walking:
     - Node 5 is connected to nodes 2, 3, 5A, and 5B.
     - Ignoring child nodes, only option is: Node 3.
3. Therefore, first you will reach Node 5, then Node 3.

So, the answers for the questions are:
1. Node 33
2. Node 21
3. Node 19
4. Node 23
5. Node 25
6. First Node 5, then Node 3

<!-- TERRIBLE PERFORMANCE HERE, This is in 2nd try-->
Q. If I walk from node 23 to node 21, which direction am I walking? 
Q. If I walk from node 29 to node 30, which direction am I walking?
Q. If I walk from node 20 to node 14, which direction am I walking?
Q. If I walk from node 6 to node 7, which direction am I walking?


Using the provided scene graph JSON data, we can determine the directions by comparing the pixel positions of the nodes involved in each question. Here’s the process for each question:

### Q1. If I walk from node 23 to node 21, which direction am I walking?
- Node 23 position: [491, 419]
- Node 21 position: [688, 506]

Comparing the x and y coordinates:
- x-coordinates: 688 (node 21) is greater than 491 (node 23), therefore direction is East.
- y-coordinates: 506 (node 21) is greater than 419 (node 23), therefore direction is South.

Overall direction: Southeast

### Q2. If I walk from node 29 to node 30, which direction am I walking?
- Node 29 position: [594, 659]
- Node 30 position: [760, 659]

Comparing the x and y coordinates:
- x-coordinates: 760 (node 30) is greater than 594 (node 29), therefore direction is East.
- y-coordinates: 659 (node 30) is equal to 659 (node 29), there is no change in y direction.

Overall direction: East

### Q3. If I walk from node 20 to node 14, which direction am I walking?
- Node 20 position: [1216, 323]
- Node 14 position: [1248, 151]

Comparing the x and y coordinates:
- x-coordinates: 1248 (node 14) is greater than 1216 (node 20), therefore direction is slightly East.
- y-coordinates: 151 (node 14) is less than 323 (node 20), therefore direction is North.

Overall direction: Northeast

### Q4. If I walk from node 6 to node 7, which direction am I walking?
- Node 6 position: [334, 326]
- Node 7 position: [500, 312]

Comparing the x and y coordinates:
- x-coordinates: 500 (node 7) is greater than 334 (node 6), therefore direction is East.
- y-coordinates: 312 (node 7) is less than 326 (node 6), therefore direction is slightly North.

Overall direction: Northeast

Thus, the directions are:
1. **Southeast** for walking from node 23 to node 21.
2. **East** for walking from node 29 to node 30.
3. **Northeast** for walking from node 20 to node 14.
4. **Northeast** for walking from node 6 to node 7.

Q. If I walk from node 30 to node 22, which direction should I turn to reach node 16?
Q. If I walk from node 8 to node 7, which direction should I turn to reach node 15?
Q. If I walk from node 19 to node 18, which direction should I turn to reach node 17?

<!--  -->
To determine the directions for the tasks, we need to look at the relative positions of the nodes involved. Here's a step-by-step approach for each question:

### Direction from node 30 to node 22 to reach node 16

1. **Node Coordinates:**
   - Node 22: (872, 490)
   - Node 16: (779, 319)
   - Node 30: (760, 659)

2. **Path from node 30 to node 22:**
   - Node 30 (760, 659)
   - Move North to reach node 22 (872, 490)

3. **Determine Direction to node 16:**
   - From node 22 (872, 490), node 16 (779, 319) is in the Northwest direction (both North and East).

### Direction from node 8 to node 7 to reach node 15

1. **Node Coordinates:**
   - Node 8: (506, 159)
   - Node 7: (500, 312)
   - Node 15: (662, 315)

2. **Path from node 8 to node 7:**
   - Node 8 (506, 159)
   - Move North to reach node 7 (500, 312)

3. **Determine Direction to node 15:**
   - From node 7 (500, 312), node 15 (662, 315) is in the East direction.

### Direction from node 19 to node 18 to reach node 17

1. **Node Coordinates:**
   - Node 19: (1122, 314)
   - Node 18: (1002, 308)
   - Node 17: (892, 315)

2. **Path from node 19 to node 18:**
   - Node 19 (1122, 314)
   - Move West to reach node 18 (1002, 308)

3. **Determine Direction to node 17:**
   - From node 18 (1002, 308), node 17 (892, 315) is in the West direction.

### Summary:
1. If you walk from node 30 to node 22, turn Northwest to reach node 16.
2. If you walk from node 8 to node 7, turn East to reach node 15.
3. If you walk from node 19 to node 18, turn West to reach node 17.

<!-- SCENARIO GEN -->

Let's initialize a simulator (like Gazebo) with this warehouse world and try to orchestrate a scenario: I want to generate a scenario where a human (John) walks along a straight passageway and encounters another human (Jessie) at an intersection. Assuming that the 2 humans move at the same speed, What should John and Jessie's trajectories be to create this scenario? Describe their trajectories as a sequence of nodes. Remember that they can only walk from one node to another if there is a link between the nodes.

