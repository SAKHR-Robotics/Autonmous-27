import os
import numpy as np
from PIL import Image
import heapq

def astar(map_data, start, goal):
    height, width = map_data.shape
    neighbors = [
        (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
    ]
    
    open_set = []
    heapq.heappush(open_set, (0.0, start))
    
    came_from = {}
    g_score = {start: 0.0}
    
    def heuristic(p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        
    iterations = 0
    max_iterations = 100000
    
    while open_set and iterations < max_iterations:
        iterations += 1
        current_f, current = heapq.heappop(open_set)
        
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, iterations
            
        for dr, dc, cost_multiplier in neighbors:
            neighbor = (current[0] + dr, current[1] + dc)
            
            if not (0 <= neighbor[0] < height and 0 <= neighbor[1] < width):
                continue
                
            cell_cost = map_data[neighbor[0], neighbor[1]]
            if cell_cost >= 100:
                continue
                
            move_cost = cost_multiplier * (1.0 + float(cell_cost) / 10.0)
            tentative_g_score = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
                
    return None, iterations

def test_map(map_path, start, goal):
    print(f"Testing map: {map_path}")
    img = Image.open(map_path).convert("L")
    img_arr = np.array(img)
    
    # Replicate costmap conversion
    cost_arr = np.zeros_like(img_arr)
    cost_arr[img_arr == 0] = 100
    cost_arr[img_arr == 255] = 0
    gray_mask = (img_arr > 0) & (img_arr < 255)
    cost_arr[gray_mask] = (100 - (img_arr[gray_mask] / 2.55)).astype(np.int8)
    
    # Flipud as in publisher
    cost_arr = np.flipud(cost_arr)
    
    # start / goal mapping
    # start: [-4.0, -4.0], origin: [-5.0, -5.0], resolution: 0.05
    start_col = int((-4.0 - -5.0) / 0.05)
    start_row = int((-4.0 - -5.0) / 0.05)
    goal_col = int((4.0 - -5.0) / 0.05)
    goal_row = int((4.0 - -5.0) / 0.05)
    
    print(f"Mapped Start: ({start_row}, {start_col}), value: {cost_arr[start_row, start_col]}")
    print(f"Mapped Goal: ({goal_row}, {goal_col}), value: {cost_arr[goal_row, goal_col]}")
    
    path, iters = astar(cost_arr, (start_row, start_col), (goal_row, goal_col))
    if path:
        print(f"Path found! Iterations: {iters}, Length: {len(path)}")
    else:
        print(f"Path NOT found! Iterations: {iters}")

if __name__ == "__main__":
    test_map("maps/marsyard_slopes.png", (-4.0, -4.0), (4.0, 4.0))
    test_map("maps/marsyard_labyrinth.png", (-4.0, -4.0), (4.0, 4.0))
