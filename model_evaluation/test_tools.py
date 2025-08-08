import json
import requests
import math
import copy


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


def get_tool_definitions(tools):
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


def execute_tool(tools, tool_name, arguments):
    try:
        result = tools[tool_name]['function'](**arguments)
        return result
    except Exception as e:
        return f'Error executing tool: {str(e)}'


def dispatch_robot(**kwargs):
    item = kwargs.get('item')
    station = kwargs.get('station')

    if not item or not station or item not in items or station not in stations:
        return json.dumps({'action': 'not_found', 'parameters': {}, 'reasoning': 'Error: Wrong item or station.'})

    item_position = items[item]['pos']
    station_position = [stations[station]['x'], stations[station]['y']]

    available_robots = [robot for robot in list(
        robots.values()) if robot['available']]
    if not available_robots:
        return json.dumps({'action': 'robots_not_available', 'parameters': {}, 'reasoning': 'Error: No robots are available.'})
    closest_robot = min([[r['robot_id'], int(10 * math.hypot(r['x'] - items[item]['pos'][0],
                        r['y'] - items[item]['pos'][1])) / 10] for r in available_robots], key=lambda v: v[1])

    return json.dumps({
        'action': 'navigate',
        'parameters': {
            'robot': closest_robot[0],
            'item': item,
            'item_position': item_position,
            'delivery_station': station,
            'delivery_station_position': station_position
        },
        'reasoning': f'Navigate {closest_robot[0]} to {item_position} to pickup {item} and then deliver it to {station} at {station_position}',
    })


def run_test(msg, model):
    tools = {
        'dispatch_robot': {
            'function': dispatch_robot,
            'description': 'Dispatch the closest robot to pick up an item and deliver it to a station',
            'parameters': {
                'item': {'type': 'string', 'description': 'The item to be picked up (A, B, C, D, etc.)'},
                'station': {'type': 'string', 'description': 'The delivery station (S1, S2, S3, S4, S5, S6, etc.)'}
            }
        }
    }

    prompt = f"""
You are a robot dispatcher assistant. Extract the item to be moved and the destination station from the user's instruction if they exists. Then use the dispatch_robot tool to generate navigation instructions.

User instruction: {msg}
"""

    messages = [
        {
            'role': 'user',
            'content': prompt
        }
    ]

    requestData = {
        'model': model,
        'messages': messages,
        'tools': get_tool_definitions(tools),
        'stream': False,
        'format': 'json',
        'think': False
    }

    r = requests.post('http://192.168.1.131:11434/api/chat', json=requestData)
    print(robots)
    print('-------')
    print(prompt)
    print('-------')
    print(r.status_code, r.text)
    print('-------')
    try:
        response = r.json()
        print(response)
    except:
        print("ERROR: r.json()")
    print('-------')
    try:
        print(json.loads(response['response']))
    except:
        print("ERROR: json.loads(response['response'])")
    print('-------')
    assistant_message = response.get("message", {})
    print('assistant_message', assistant_message)
    tool_calls = assistant_message.get("tool_calls", [])
    print('tool_calls', tool_calls)
    if not tool_calls:
        print("INFO: Model did not call any tools.")
        print('-------')
    else:
        tool_call = tool_calls[0]
        function_info = tool_call.get("function", {})
        function_name = function_info.get("name")
        function_args = function_info.get("arguments", {})
        print('tool function', function_info)
        if isinstance(function_args, str):
            try:
                function_args = json.loads(function_args)
            except json.JSONDecodeError:
                print(f'Error parsing arguments: {function_args}')
        print('extracted parameters:', function_args)
        result = execute_tool(tools, function_name, function_args)
        print('-------')
        try:
            result_json = json.loads(result)
            print(result_json)
        except:
            print(f'ERROR: {result}')
        print('-------')


models = ['mistral:7b', 'mistral-small:24b', 'mistral-small3.2:24b',
          'gemma3:4b', 'deepseek-r1:8b', 'deepseek-r1:1.5b', 'llama3.2:3b']
msgs = ['Hi, how are you today?', 'Bring B to S5', 'Bring C to S5']


def run_all_tests():
    global robots
    robots_v = [copy.deepcopy(robots), copy.deepcopy(
        robots), copy.deepcopy(robots)]
    robots_v[1]['robot_1']['available'] = False
    robots_v[2]['robot_1']['available'] = False
    robots_v[2]['robot_2']['available'] = False
    robots_v[2]['robot_3']['available'] = False
    for msg in msgs:
        for rs in robots_v:
            robots = rs
            for model in models:
                print(f'\n-------')
                try:
                    run_test(msg, model)
                except:
                    print(f'ERROR: {msg} {model} {robots}')
                    print('-------')


run_all_tests()
