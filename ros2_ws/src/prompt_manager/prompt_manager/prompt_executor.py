import json
import requests
import rclpy
from rclpy.node import Node
import math
import traceback

from std_msgs.msg import String


class PromptExecutor(Node):

    def __init__(self):
        super().__init__('prompt_executor')
        self.subscription = self.create_subscription(
            String,
            'prompt',
            self.listener_callback,
            11)

        self.station_monitor = self.create_subscription(
            String,
            'station_status',
            self.station_status_callback,
            100)

        self.stations = {}

        self.robot_monitor = self.create_subscription(
            String,
            'robot_status',
            self.robot_status_callback,
            100)
        self.robot_monitor

        self.robots = {}

        self.items = {}

        self.publisher_ = self.create_publisher(String, 'response', 10)

        self.cmd_pos_ = self.create_publisher(String, 'cmd_pos', 10)

        self.ollama_api = "http://192.168.1.131:11434"

        self.tools = {
            'dispatch_robot': {
                'function': self.dispatch_robot,
                'description': 'Dispatch the closest robot to pick up an item and deliver it to a station',
                'parameters': {
                    'item': {'type': 'string', 'description': 'The item to be picked up (A, B, C, D, etc.)'},
                    'station': {'type': 'string', 'description': 'The delivery station (S1, S2, S3, S4, S5, S6, etc.)'}
                }
            }
        }

        self.tool_definitions = self.get_tool_definitions(self.tools)

    def station_status_callback(self, msg):
        station = json.loads(msg.data)
        self.stations[station['station_id']] = station
        if station['item'] != '-':
            self.items[station['item']] = {
                'name': station['item'], 'pos': [station['x'], station['y']]}

    def calc_distances(self):
        return {i: [p[0] for p in sorted([[r, math.hypot(rd['x']-id['pos'][0], rd['y']-id['pos'][1])] for (r, rd) in self.robots.items()], key=lambda v: v[1])] for (i, id) in self.items.items()}

    def robot_status_callback(self, msg):
        robot = json.loads(msg.data)
        self.robots[robot['robot_id']] = robot

    def listener_callback(self, msg):
        request = json.loads(msg.data)

        requestData = {
            'model': request['model'],
            'think': False,
            'format': 'json',
            'stream': False,
        }

        if request['mode'] == 'simple':
            requestData['prompt'] = self.generate_simple_prompt(
                request['input'])
        elif request['mode'] == 'advanced':
            requestData['prompt'] = self.generate_advanced_prompt(
                request['input'])
        elif request['mode'] == 'expert':
            requestData['prompt'] = self.generate_expert_prompt(
                request['input'])
        elif request['mode'] == 'tool':
            requestData['messages'] = [
                {
                    'role': 'user',
                    'content': self.generate_tool_prompt(request['input'])
                }
            ]
            requestData['tools'] = self.tool_definitions

        match request['model']:
            case 'deepseek-r1:1.5b':
                requestData['think'] = False

        self.get_logger().info(
            f'PROMPT:\n{requestData['messages' if request['mode'] == 'tool' else 'prompt']}\n----------\n')
        r = requests.post(
            f'{self.ollama_api}/api/{'chat' if request['mode'] == 'tool' else 'generate'}', json=requestData)

        if r.status_code != requests.codes.ok:
            self.get_logger().error(
                f'CONNECTION ERROR {r.status_code}: {r.text}')
            msg = String()
            msg.data = f'Error {r.status_code}: {r.text}'
            self.publisher_.publish(msg)
            return

        try:
            response = r.json()
            self.get_logger().info(f'RESPONSE:\n{response}\n----------\n')
            if request['mode'] == 'tool':
                result = self.process_tool_response(response)
            else:
                result = self.process_prompt_response(response, request['mode'])
        except Exception as x:
            traceback.print_exception(x)
            msg = String()
            msg.data = 'Error: Cannot understand you!'
            self.publisher_.publish(msg)
            return

        try:
            result_dict = json.loads(result)
            if 'action' in result_dict and result_dict['action'] == 'navigate':
                cmd = String()
                cmd.data = json.dumps(result_dict['parameters'])
                self.cmd_pos_.publish(cmd)
            msg.data = result_dict['reasoning']
        except Exception as x:
            traceback.print_exception(x)

        self.get_logger().info(f'PUBLISHED: {msg.data}')
        self.publisher_.publish(msg)

    def process_prompt_response(self, response, mode):
        data = None
        try:
            data = json.loads(response['response'][7:-3])
        except:
            pass
        if not data:
            try:
                data = json.loads(response['response'][response['response'].find(
                    '```json')+7:response['response'].find('}\n```')])
            except:
                pass
        if not data:
            try:
                data = json.loads(response['response'][response['response'].find(
                    '```json')+7:response['response'].find('}```')])
            except:
                pass
        if not data:
            try:
                data = json.loads(response['response'][response['response'].find(
                    '```json')+7:response['response'].find('} ```')])
            except:
                pass
        if not data:
            try:
                data = json.loads(response['response'])
            except:
                pass

        result = json.dumps({'reasoning': 'Error: Cannot understand you!'})
        if data:
            if mode == 'simple':
                if data['action'] == 'navigate':
                    item = data['parameters']['item']
                    station = data['parameters']['delivery_station']
                    result = self.dispatch_robot(item=item, station=station)
            else:
                if data['action'] == 'navigate':
                    result = json.dumps(data)

        return result

    def process_tool_response(self, response):
        result = json.dumps({'reasoning': 'Error: Cannot understand you!'})

        try:
            assistant_message = response.get("message", {})
            tool_calls = assistant_message.get("tool_calls", [])
            if tool_calls:
                tool_call = tool_calls[0]
                function_info = tool_call.get("function", {})
                function_name = function_info.get("name")
                function_args = function_info.get("arguments", {})
                if isinstance(function_args, str):
                    function_args = json.loads(function_args)
                result = self.execute_tool(
                    self.tools, function_name, function_args)
        except:
            pass

        return result

    def get_tool_definitions(self, tools):
        tool_definitions = []
        for name, tool in tools.items():
            tool_def = {
                'type': 'function',
                'function': {
                    'name': name,
                    'description': tool['description'],
                    'parameters': {
                        'type': 'object',
                        'properties': tool['parameters'],
                        'required': list(tool['parameters'].keys())
                    }
                }
            }
            tool_definitions.append(tool_def)
        return tool_definitions

    def execute_tool(self, tools, tool_name, arguments):
        try:
            result = tools[tool_name]['function'](**arguments)
            return result
        except Exception as e:
            return f'Error executing tool: {str(e)}'

    def dispatch_robot(self, **kwargs):
        item = kwargs.get('item')
        station = kwargs.get('station')

        if not item or not station or item not in self.items or station not in self.stations:
            return json.dumps({'action': 'not_found', 'parameters': {}, 'reasoning': 'Error: Wrong item or station.'})

        item_position = self.items[item]['pos']
        station_position = [self.stations[station]
                            ['x'], self.stations[station]['y']]

        available_robots = [robot for robot in list(
            self.robots.values()) if robot['available']]
        if not available_robots:
            return json.dumps({'action': 'robots_not_available', 'parameters': {}, 'reasoning': 'Error: No robots are available.'})
        closest_robot = min([[r['robot_id'], int(10 * math.hypot(r['x'] - self.items[item]['pos'][0],
                            r['y'] - self.items[item]['pos'][1])) / 10] for r in available_robots], key=lambda v: v[1])
        closest_robot = closest_robot[0]

        return json.dumps({
            'action': 'navigate',
            'parameters': {
                'robot': closest_robot,
                'item': item,
                'item_position': item_position,
                'delivery_station': station,
                'delivery_station_position': station_position
            },
            'reasoning': f'Navigate {closest_robot} to {item_position} to pickup {item} and then deliver it to {station} at {station_position}',
        })

    def generate_simple_prompt(self, msg):
        return f"""
[INST] You are a robot dispatcher. The task is to get an item from a specific location and deliver it to an another station. You need to fetch from the user input the requested item and the delivery station. If you cannot find item or the delivery station, then return action "not_found". Return the found item and delivery station as a JSON.

Available items: {json.dumps(list(self.items.keys()))}

Available delivery station: {json.dumps(list(self.stations.keys()))}

Available actions: navigate, not_found

User request: "{msg}"

Respond with JSON: {{"action": <action_name>, "parameters": {{"item": <item found>, "delivery_station": <the found delivery station>}}, "reasoning": <explanation>}}

Examples:
- "bring Z to S10" → {{"action": "navigate", "parameters": {{"item": "Z", "delivery_station": "S10",}}, "reasoning": "Item is with name Z and the name of the delivery station is S10}}

Return only JSON. No other text. [/INST]
"""

    def generate_advanced_prompt(self, msg):
        return f"""
[INST] You are a robot dispatcher. The task is to dispatch a robot to get an item from the item position and to move it to specified delivery station position. You need to fetch from the user input the requested item and the delivery station. If you cannot find the item or the delivery station, then return action "not_found". After you have found the positions of the item and the delivery station, then you need to find the first robot from with status set to "available" which is in the ordered list of the robots closest to the item. If there is no robot with available status return the action "robots_not_available" action. Return the found item position and delivery station position in the JSON.

Available items: {json.dumps(list(self.items.keys()))}

Positions of the items: {json.dumps({k: v['pos'] for (k, v) in list(self.items.items())})}

Available delivery station: {json.dumps(list(self.stations.keys()))}

Positions of the delivery stations: {json.dumps({k: [v['x'], v['y']] for (k, v) in list(self.stations.items())})}

Status of the robots: {json.dumps({k: 'available' if v['available'] else 'busy' for (k, v) in list(self.robots.items())})}

Ordered list of the robots closest to the item: {json.dumps({ki: [v[0] for v in sorted([[kr, int(10 * math.hypot(kv['x'] - vi['pos'][0], kv['y'] - vi['pos'][1])) / 10] for (kr, kv) in list(self.robots.items())], key=lambda v: v[1])] for (ki, vi) in list(self.items.items())})}

Available actions: navigate, robots_not_available, not_found

User request: "{msg}"

Respond with JSON: {{"action": <action_name>, "parameters": {{"robot": <closest to the item available robot>, "item": <the found item>, "item_position": <position of the found item as a list of two coordinates [x, y]>, "delivery_station": <the found delivery station>, "delivery_station_position": <position of the found delivery station as a list of two coordinates [x, y]>}}, "reasoning": <explanation>}}

Examples:
- "bring Z to S10" → {{"action": "navigate", "parameters": {{"robot": "robot_1", "item": "Z", "item_position": [8, 8], "delivery_station": "S10", delivery_sttation_position: [10, 10]}}, "reasoning": "Navigate robot_1 to [8, 8] to pickup Z and then to drop it at S10"}}

Return only JSON. No other text. [/INST]
"""

    def generate_expert_prompt(self, msg):
        return f"""
[INST] You are a robot dispatcher. The task is to dispatch a robot to get an item from the item position and to move it to specified delivery station position. You need to fetch from the user input the requested item and the delivery station. If you cannot find the item or the delivery station, then return action "not_found". After you have found the positions of the item and the delivery station, then you need to find the closest to the item position robot with status set to "available". The distance between the positions of the robot [x1, y1] and the position of the item [x2, y2] is measured with Euclidean distance as distance([x1,y1], [x2,y2]) = sqrt((x1 - x2)*(x1 - x2)+(y1 - y2)*(y1 - y2)). If there is no robot with available status return the action "robots_not_available" action. Return the found item position and delivery station position in the JSON.

Available items: {json.dumps(list(self.items.keys()))}

Positions of the items: {json.dumps({k: v['pos'] for (k, v) in list(self.items.items())})}

Available delivery stations: {json.dumps(list(self.stations.keys()))}

Positions of the delivery stations: {json.dumps({k: [v['x'], v['y']] for (k, v) in list(self.stations.items())})}

Status of the robots: {json.dumps({k: 'available' if v['available'] else 'busy' for (k, v) in list(self.robots.items())})}

Position of the robots: {json.dumps({k: [v['x'], v['y']] for (k, v) in list(self.robots.items())})}

Available actions: navigate, robots_not_available, not_found

User request: "{msg}"

Respond with JSON: {{"action": <action_name>, "parameters": {{"robot": <closest to the item available robot>, "item": <the found item>, "item_position": <position of the found item as a list of two coordinates [x, y]>, "delivery_station": <the found delivery station>, "delivery_station_position": <position of the found delivery station as a list of two coordinates [x, y]>}}, "reasoning": <explanation>}}

Examples:
- "bring Z to S10" → {{"action": "navigate", "parameters": {{"robot": "robot_1", "item": "Z", "item_position": [8, 8], "delivery_station": "S10", delivery_station_position: [10, 10]}}, "reasoning": "Navigate robot_1 to [8, 8] to pickup Z and then to drop it at S10"}}

Return only JSON. No other text. [/INST]
"""

    def generate_tool_prompt(self, msg):
        return f"""
You are a robot dispatcher assistant. Extract the item to be moved and the destination station from the user's instruction if they exists. Then use the dispatch_robot tool to generate navigation instructions.

User instruction: {msg}
"""


def main(args=None):
    rclpy.init(args=args)

    prompt_executor = PromptExecutor()

    rclpy.spin(prompt_executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    prompt_executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
