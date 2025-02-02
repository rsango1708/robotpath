import heapq
#import networkx as nx
import rospy
import matplotlib.pyplot as plt
import math
import string
import itertools
import time
from time import sleep
import ld06
#from unittest import mock
import RPi.GPIO as gpio
#from unittest import mock
from std_msgs.msg import String
end=False
# Create a fake GPIO module
'''class MockGPIO:
    BCM = "BCM"
    OUT = "OUT"
    LOW = "LOW"
    
    def setmode(self, mode):
         print(f"Mock GPIO mode set to {mode}")
        
    
    def setup(self, pin, mode):
        print(f"Mock GPIO setup: Pin {pin}, Mode {mode}")
        
    
    def output(self, pin, value):
        print(f"Mock GPIO output: Pin {pin}, Value {value}")
    
    def PWM(self, pin, freq):
        print(f"Mock GPIO PWM setup: Pin {pin}, Frequency {freq}")
        return mock.Mock(start=lambda: print("PWM started"), ChangeDutyCycle=lambda speed: print(f"PWM speed set to {speed}"))

# Replace RPi.GPIO with the mock class
gpio = MockGPIO()

gpio.setmode(gpio.BCM)'''
import json
'''class MockLD06:
    def __init__(self, port):
        print(f"Mock LiDAR initialized on {port}")

    def start_motor(self):
        print("Mock LiDAR motor started")

    def stop_motor(self):
        print("Mock LiDAR motor stopped")

    def disconnect(self):
        print("Mock LiDAR disconnected")

    def iter_scans(self):
        # Fake some LiDAR scans
        return iter([[(90, 13), (0, 23), (180, 250), (270, 400)]])'''

# Replace ld06 import with the mock class
#ld06 = MockLD06
lidar = ld06('/dev/ttyS0')
OBSTACLE_THRESHOLD = 25.5  

def process_lidar_data():
    
    lidar.start_motor()
    print("Starting scan...")

    try:
        for scan in lidar.iter_scans():
            
            obstacle_detected = False
            dynamic_obstacle_nodes = set()

            for (angle, distance) in scan:
                if distance < OBSTACLE_THRESHOLD:
                    print(f"Obstacle detected at angle {angle}° and distance {distance}mm")
                    dynamic_obstacle_nodes.add((angle, distance))  
                    obstacle_detected = True

           
            if obstacle_detected:
                print("Obstacle detected in scan!")

            
            visualize_obstacles(dynamic_obstacle_nodes)
            time.sleep(0.1)  
            
    finally:
        return 
def visualize_obstacles(dynamic_obstacle_nodes):
   
    if dynamic_obstacle_nodes:
        angles = [angle for angle, distance in dynamic_obstacle_nodes]
        distances = [distance for angle, distance in dynamic_obstacle_nodes]

        
        plt.figure()
        plt.subplot(111, projection='polar')
        plt.scatter([math.radians(a) for a in angles], distances, c='r', label='Obstacles')
        plt.title("LiDAR Obstacle Detection")
        plt.legend()
        plt.show()


process_lidar_data()


print("a")
def dynamic_obstacle_recognition(obstacles, path, node_positions):
    dynamic_updates = set()
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
    # Check each node in the path for obstacles
        for scan in lidar.iter_scans():
         for (angle,distance) in scan:
            if distance<OBSTACLE_THRESHOLD:
                obstacle_node = get_node_from_angle_distance(angle, distance, node_positions, path[0])

                if obstacle_node and obstacle_node in node_positions:
                    print(f"Obstacle detected at node {obstacle_node}")
                    dynamic_updates.add(obstacle_node)
                

    obstacles.update(dynamic_updates)
    
    updated_path = [node for node in path if node not in obstacles]
    
    return updated_path, obstacles




class Node:
    def __init__(self, name, heuristic):
        self.name = name
        self.heuristic = heuristic
    
    def __lt__(self, other):
        return self.heuristic < other.heuristic

# Greedy Best-First Search for Hierarchical Routing
def greedy_best_first_search_hierarchical(graph, start, goal, heuristic, region_map):
    
    priority_queue = []
    heapq.heappush(priority_queue, Node(start, heuristic[start]))

    visited = set()  
    path = {start: None}

    while priority_queue:
        current_node = heapq.heappop(priority_queue).name
        
        
        if current_node == goal:
            return reconstruct_path(path, start, goal)
        if current_node in visited:
            continue

        visited.add(current_node)

        
        current_region = region_map[current_node]
        for neighbor in graph[current_node]:
            if neighbor not in visited and region_map[neighbor] == current_region:
                heapq.heappush(priority_queue, Node(neighbor, heuristic[neighbor]))
                if neighbor not in path:
                    path[neighbor] = current_node

        
        for neighbor in graph[current_node]:
            if neighbor not in visited and region_map[neighbor] != current_region:
                heapq.heappush(priority_queue, Node(neighbor, heuristic[neighbor]))
                if neighbor not in path:
                    path[neighbor] = current_node

    return None  
def reconstruct_path(path, start, goal):
    current = goal
    result_path = []
    while current is not None:
        result_path.append(current)
        current = path[current]
    result_path.reverse()
    return result_path


'''def visualize_graph(graph, path, pos, region_map):
    G = nx.Graph()

    
    for node, neighbors in graph.items():
        for neighbor in neighbors:
            G.add_edge(node, neighbor)

    
    plt.figure(figsize=(10, 8))
    node_colors = []
    for node in G.nodes:
        if obstacles and node in obstacles:
            node_colors.append("black")  
        elif path and node in path:
            node_colors.append("lightgreen")  
        else:
            node_colors.append("skyblue")  
    
    nx.draw(G, pos, with_labels=True, node_size=4000, node_color=node_colors,
             font_size=15,node_shape='s', font_weight='bold', edge_color='gray')

    
    if path:
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='black', width=2.5)
        nx.draw_networkx_nodes(G, pos, nodelist=path, node_color='lightgreen',node_shape='s')
    
         

   
    for node, region in region_map.items():
        plt.text(pos[node][0], pos[node][1] - 0.2, f" {region}", fontsize=8, color='grey')

    plt.title("Greedy Best-First Search for Hierarchical Routing", size=20)
    plt.show()
    plt.axis("equal")'''






def generate_region_map(pos, rows, cols, region_rows, region_cols):
    region_map = {}
    region_count = 1

    for key, (x, y) in pos.items():
        
        region_x = x // region_cols
        region_y = y // region_rows
        region_number = region_y * (cols // region_cols) + region_x + 1
        region_map[key] = region_number

    return region_map


region_rows, region_cols = 5, 5


def generate_coordinates(rows, cols):
    pos = {}
    letters = list(string.ascii_uppercase)  # A-Z

   
    keys = []
    for length in range(1, 3):  
        keys.extend([''.join(k) for k in itertools.product(letters, repeat=length)])

    
    for y in range(rows):
        for x in range(cols):
            pos[keys[y*cols+x]] = (x, y)

    return pos
def generate_graph(pos, rows, cols):
    graph = {}

    for key, (x, y) in pos.items():
        neighbors2 = []

        # Check for valid neighbors (left, right, top, bottom)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < cols and 0 <= ny < rows:
                neighbor_key = next(k for k, v in pos.items() if v == (nx, ny))
                neighbors2.append(neighbor_key)

        graph[key] = neighbors2

    return graph
    
pos = generate_coordinates(24, 28)

graph=generate_graph(pos,24,28)


rows, cols = 25, 29

region_map = generate_region_map(pos, 25, 29, region_rows, region_cols)


obstacle_nodes = set()
x1,y1=6,26
x2,y2=18,29
for x,y in pos.items():
    
    if y1-1<=y[0]<=y2-1 and x1-1<=y[1]<=x2-1:
        
        obstacle_nodes.add(x)
x1,y1=13,6
x2,y2=17,20
for x,y in pos.items():
    
    if y1-1<=y[0]<=y2-1 and x1-1<=y[1]<=x2-1:
        
        obstacle_nodes.add(x)
x1,y1=20,6
x2,y2=23,20
for x,y in pos.items():
    
    if y1-1<=y[0]<=y2-1 and x1-1<=y[1]<=x2-1:
        
        obstacle_nodes.add(x)
x1,y1=7,6
x2,y2=12,10
for x,y in pos.items():
    
    if y1-1<=y[0]<=y2-1 and x1-1<=y[1]<=x2-1:
        
        obstacle_nodes.add(x)

obstacles=set(obstacle_nodes)
def get_node_from_angle_distance(angle, distance, pos,current_node):
    
    
   
    angle_rad = math.radians(angle)
    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)
    x = x / 25.5
    y = y / 20
    x1, y1 = pos[current_node]
    closest_node = None
    min_distance = float('inf')

    
    for node, (nx, ny) in pos.items():
        d = math.sqrt((nx - (x1 + x))**2 + (ny - (y1 + y))**2)
        if d < min_distance:
            min_distance = d
            closest_node = node

    return closest_node


motor_select_pins = {
    
    'M1': {'In1': 22, 'In2': 23},
    'M2': {'In1': 5, 'In2': 6},
    'M3': {'In1': 17,  'In2': 27},
    'M4': {'In1': 26, 'In2': 16}

}
for motor, pins in motor_select_pins.items():
    gpio.setup(pins['In1'], gpio.OUT)
    gpio.setup(pins['In2'], gpio.OUT)
pwm_pin=13
gpio.setup(pwm_pin, gpio.OUT)

pwm = gpio.PWM(pwm_pin, 100)  # 100 Hz frequency
pwm.start()
def reset_pins():
        for pin in motor_select_pins.values():
            gpio.output(pin, gpio.LOW)


def motor1(direction, speed):
    """Control Motor 1"""
    gpio.output(motor_select_pins['M1']['In1'], direction == 'forward')
    gpio.output(motor_select_pins['M1']['In2'], direction == 'backward')
    pwm.ChangeDutyCycle(speed)

def motor2(direction, speed):
    """Control Motor 2"""
    gpio.output(motor_select_pins['M2']['In1'], direction == 'forward')
    gpio.output(motor_select_pins['M2']['In2'], direction == 'backward')
    pwm.ChangeDutyCycle(speed)

def motor3(direction, speed):
    """Control Motor 3"""
    gpio.output(motor_select_pins['M3']['In1'], direction == 'forward')
    gpio.output(motor_select_pins['M3']['In2'], direction == 'backward')
    pwm.ChangeDutyCycle(speed)

def motor4(direction, speed):
    """Control Motor 4"""
    gpio.output(motor_select_pins['M4']['In1'], direction == 'forward')
    gpio.output(motor_select_pins['M4']['In2'], direction == 'backward')
    pwm.ChangeDutyCycle(speed)

a=0
def movement( direction2, duration=2):
    reset_pins()
    for pin in motor_select_pins.values():
        gpio.output(pin, gpio.LOW)

    if direction2 == "forward":
        motor1(direction='forward',speed=50)
        motor2(direction='forward',speed=50)
        motor3(direction='forward',speed=50)
        motor4(direction='forward',speed=50)
        

    elif direction2 == 'backward':
        motor1(direction='backward',speed=50)
        motor2(direction='backward',speed=50)
        motor3(direction='backward',speed=50)
        motor4(direction='backward',speed=50)
        
    elif direction2 == "right":
        motor1(direction='backward',speed=50)
        motor2(direction='forward',speed=50)
        motor3(direction='forward',speed=50)
        motor4(direction='backward',speed=50)
        
    elif direction2 == "left":
        motor1(direction='forward',speed=50)
        motor2(direction='backward',speed=50)
        motor3(direction='backward',speed=50)
        motor4(direction='forward',speed=50)

    print(f"Moving {direction2}")
    global a
    a=a+1
    print("A is",a)
    
    result_path = greedy_best_first_search_hierarchical(graph, start_node, goal_node, heuristic, region_map)
    print("Len is",len(result_path))
    

    sleep(duration)

    

    # Reset all pins to low to stop motors completely
    for pin in motor_select_pins.values():
        gpio.output(pin, gpio.LOW)
    reset_pins()
    print("Motors stopped.")   
    return a 

def move_to_node(current_node, next_node, node_positions):
    print(f"Moving from {current_node} to {next_node}")
    x1, y1 = node_positions[current_node]
    x2, y2 = node_positions[next_node]

    if x2 > x1:
        movement( direction2="right",  duration=2)
    elif x2 < x1:
        movement( direction2="left",  duration=2)
    elif y2 > y1:
        movement( direction2="forward",  duration=2)
    elif y2 < y1:
        movement( direction2="backward",  duration=2)
    next_node = current_node

def execute_path(path, node_positions):
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
        move_to_node(current_node, next_node, node_positions)
        if current_node==goal_node:
            end=True

        

start_node = 'BZ'
goal_node = 'XY'

def get_coordinates(goal_node, pos):
    return pos.get(goal_node, None)
target_point=get_coordinates(goal_node,pos)

def euclidean_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
heuristic = {
     point: (float('inf') if point in obstacles else round(euclidean_distance(coord, target_point), 2))
    for point, coord in pos.items()
}

'''result_path = greedy_best_first_search_hierarchical(graph, start_node, goal_node, heuristic, region_map)
print("Len is",len(result_path))
def run_navigation_loop(graph, start_node, goal_node, heuristic, region_map, pos):
    global a
    result_path = greedy_best_first_search_hierarchical(graph, start_node, goal_node, heuristic, region_map)
    print("Len is",len(result_path))
    while a!=len(result_path):
        result_path = greedy_best_first_search_hierarchical(graph, start_node, goal_node, heuristic, region_map)
        print(a)
        # Perform LiDAR scan and update obstacles
        dynamic_obstacles = dynamic_obstacle_recognition(obstacles, result_path, pos)

        # Recalculate the path based on the updated obstacles
        result_path = greedy_best_first_search_hierarchical(graph, start_node, goal_node, heuristic, region_map)

        # Execute the new path
        execute_path(result_path, pos)

        print(f"Updated Path from {start_node} to {goal_node}: {result_path}")
        print(f"Updated obstacles: {obstacles}")

        # Visualize the updated graph and path
        

        sleep(0.5)  # Delay to simulate time for new LIDAR data to be gathered
        
    #visualize_graph(graph, result_path, pos, region_map)
    print("out of loop")
#visualize_graph(graph, result_path, pos, region_map)
# Start the loop
#run_navigation_loop(graph, start_node, goal_node, heuristic, region_map, pos)'''






result_path = greedy_best_first_search_hierarchical(graph, start_node, goal_node, heuristic, region_map)
dynamic_obstacles=dynamic_obstacle_recognition(obstacles,result_path,pos)
result_path2 = greedy_best_first_search_hierarchical(graph, start_node, goal_node, heuristic, region_map)
execute_path(result_path2,pos)





print("Path from {} to {}: {}".format(start_node, goal_node, result_path))
print(f"obstacles: {obstacles}")
print(f"obstacle nodes: {obstacle_nodes}")



def simulate_ros_publish(pos, heuristic):
    # Create the map_data as a dictionary
    map_data = {node: {"coordinates": coord, "heuristic": heuristic[node]} for node, coord in pos.items()}
    
    # Convert map_data to JSON string
    map_data_json = json.dumps(map_data, indent=4)
    
    # Initialize ROS node (replace 'map_publisher_node' with a suitable node name)
    rospy.init_node('map_publisher_node', anonymous=True)

    # Create a publisher object to publish to the /map topic
    map_pub = rospy.Publisher('/map', String, queue_size=10)

    # Wait until the publisher is connected to a subscriber
    rospy.sleep(1)  # Wait for 1 second to ensure connection
    
    # Publish the map data
    map_pub.publish(map_data_json)

    # Optionally, print the published data for debugging
    print(f"Published map data to /map topic:\n{map_data_json}")

# Example usage
simulate_ros_publish(pos, heuristic)