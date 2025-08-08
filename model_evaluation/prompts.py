import math
import json


robots = {
    'robot_3': {'robot_id': 'robot_3', 'x': -1.3, 'y':  0.0, 'available': True},
    'robot_2': {'robot_id': 'robot_2', 'x':  0.0, 'y': -1.3, 'available': True},
    'robot_1': {'robot_id': 'robot_1', 'x':  1.3, 'y':  0.0, 'available': True}
}

stations = {
    'S1': {'station_id': 'S1', 'x':  1.0, 'y':  1.0, 'item': 'A'},
    'S2': {'station_id': 'S2', 'x':  0.8, 'y':  0.3, 'item': 'B'},
    'S3': {'station_id': 'S3', 'x': -0.5, 'y': -0.6, 'item': 'C'},
    'S4': {'station_id': 'S4', 'x':  0.4, 'y':  1.3, 'item': 'D'},
    'S5': {'station_id': 'S5', 'x': -0.8, 'y':  0.6, 'item': '-'},
    'S6': {'station_id': 'S6', 'x':  0.5, 'y': -1.1, 'item': '-'}
}

items = {
    'A': {'name': 'A', 'pos': [1.0, 1.0]},
    'B': {'name': 'B', 'pos': [0.8, 0.3]},
    'C': {'name': 'C', 'pos': [-0.5, -0.6]},
    'D': {'name': 'D', 'pos': [0.4, 1.3]}
}

item_names = list(items.keys())

location_names = list(stations.keys())

robot_names = list(robots.keys())

robot_status = {k: 'available' if v['available']
                else 'busy' for (k, v) in list(robots.items())}

robot_position = {k: [v['x'], v['y']] for (k, v) in list(robots.items())}

item_position = {k: v['pos'] for (k, v) in list(items.items())}

delivery_location_position = {k: [v['x'], v['y']]
                              for (k, v) in list(stations.items())}

distance_from_item_to_robot = {ki: {kr: int(10 * math.hypot(kv['x'] - vi['pos'][0], kv['y'] - vi['pos'][1])) / 10 for (
    kr, kv) in list(robots.items())} for (ki, vi) in list(items.items())}

closest_robots_to_item = {ki: [v[0] for v in sorted([[kr, int(10 * math.hypot(kv['x'] - vi['pos'][0], kv['y'] - vi['pos'][1])) / 10] for (
    kr, kv) in list(robots.items())], key=lambda v: v[1])] for (ki, vi) in list(items.items())}

# print('item_names', item_names)
# print('location_names', location_names)
# print('robot_names', robot_names)
# print('item_position', item_position)
# print('robot_status', robot_status)
# print('robot_position', robot_position)
# print('delivery_location_position', delivery_location_position)
# print('distance_from_item_to_robot', distance_from_item_to_robot)
# print('closest_robots_to_item', closest_robots_to_item)


def generate_simple_prompt(msg, robots, stations, items):
    return f"""
[INST] You are a robot dispatcher. The task is to get an item from a specific location and deliver it to an another station. You need to fetch from the user input the requested item and the delivery station. If you cannot find item or the delivery station, then return action "not_found". Return the found item and delivery station as a JSON.

Available items: {json.dumps(list(items.keys()))}

Available delivery station: {json.dumps(list(stations.keys()))}

Available actions: navigate, not_found

User request: "{msg}"

Respond with JSON: {{"action": <action_name>, "parameters": {{"item": <item found>, "delivery_station": <the found delivery station>}}, "reasoning": <explanation>}}

Examples:
- "bring Z to S10" → {{"action": "navigate", "parameters": {{"item": "Z", "delivery_station": "S10",}}, "reasoning": "Item is with name Z and the name of the delivery station is S10}}

Return only JSON. No other text. [/INST]
"""


def generate_advanced_prompt(msg, robots, stations, items):
    return f"""
[INST] You are a robot dispatcher. The task is to dispatch a robot to get an item from the item position and to move it to specified delivery station position. You need to fetch from the user input the requested item and the delivery station. If you cannot find the item or the delivery station, then return action "not_found". After you have found the positions of the item and the delivery station, then you need to find the first robot from with status set to "available" which is in the ordered list of the robots closest to the item. If there is no robot with available status return the action "robots_not_available" action. Return the found item position and delivery station position in the JSON.

Available items: {json.dumps(list(items.keys()))}

Positions of the items: {json.dumps({k: v['pos'] for (k, v) in list(items.items())})}

Available delivery station: {json.dumps(list(stations.keys()))}

Positions of the delivery stations: {json.dumps({k: [v['x'], v['y']] for (k, v) in list(stations.items())})}

Status of the robots: {json.dumps({k: 'available' if v['available'] else 'busy' for (k, v) in list(robots.items())})}

Ordered list of the robots closest to the item: {json.dumps({ki: [v[0] for v in sorted([[kr, int(10 * math.hypot(kv['x'] - vi['pos'][0], kv['y'] - vi['pos'][1])) / 10] for (kr, kv) in list(robots.items())], key=lambda v: v[1])] for (ki, vi) in list(items.items())})}

Available actions: navigate, robots_not_available, not_found

User request: "{msg}"

Respond with JSON: {{"action": <action_name>, "parameters": {{"robot": <closest to the item available robot>, "item": <the found item>, "item_position": <position of the found item as a list of two coordinates [x, y]>, "delivery_station": <the found delivery station>, "delivery_station_position": <position of the found delivery station as a list of two coordinates [x, y]>}}, "reasoning": <explanation>}}

Examples:
- "bring Z to S10" → {{"action": "navigate", "parameters": {{"robot": "robot_1", "item": "Z", "item_position": [8, 8], "delivery_station": "S10", delivery_sttation_position: [10, 10]}}, "reasoning": "Navigate robot_1 to [8, 8] to pickup Z and then to drop it at S10"}}

Return only JSON. No other text. [/INST]
"""


def generate_expert_prompt(msg, robots, stations, items):
    return f"""
[INST] You are a robot dispatcher. The task is to dispatch a robot to get an item from the item position and to move it to specified delivery station position. You need to fetch from the user input the requested item and the delivery station. If you cannot find the item or the delivery station, then return action "not_found". After you have found the positions of the item and the delivery station, then you need to find the closest to the item position robot with status set to "available". The distance between the positions of the robot [x1, y1] and the position of the item [x2, y2] is measured with Euclidean distance as distance([x1,y1], [x2,y2]) = sqrt((x1 - x2)*(x1 - x2)+(y1 - y2)*(y1 - y2)). If there is no robot with available status return the action "robots_not_available" action. Return the found item position and delivery station position in the JSON.

Available items: {json.dumps(list(items.keys()))}

Positions of the items: {json.dumps({k: v['pos'] for (k, v) in list(items.items())})}

Available delivery stations: {json.dumps(list(stations.keys()))}

Positions of the delivery stations: {json.dumps({k: [v['x'], v['y']] for (k, v) in list(stations.items())})}

Status of the robots: {json.dumps({k: 'available' if v['available'] else 'busy' for (k, v) in list(robots.items())})}

Position of the robots: {json.dumps({k: [v['x'], v['y']] for (k, v) in list(robots.items())})}

Available actions: navigate, robots_not_available, not_found

User request: "{msg}"

Respond with JSON: {{"action": <action_name>, "parameters": {{"robot": <closest to the item available robot>, "item": <the found item>, "item_position": <position of the found item as a list of two coordinates [x, y]>, "delivery_station": <the found delivery station>, "delivery_station_position": <position of the found delivery station as a list of two coordinates [x, y]>}}, "reasoning": <explanation>}}

Examples:
- "bring Z to S10" → {{"action": "navigate", "parameters": {{"robot": "robot_1", "item": "Z", "item_position": [8, 8], "delivery_station": "S10", delivery_station_position: [10, 10]}}, "reasoning": "Navigate robot_1 to [8, 8] to pickup Z and then to drop it at S10"}}

Return only JSON. No other text. [/INST]
"""

# print(generate_simple_prompt("Bring A to S3", robots, stations, items))
# print(generate_advanced_prompt("Bring A to S3", robots, stations, items))
# print(generate_expert_prompt("Bring A to S3", robots, stations, items))
