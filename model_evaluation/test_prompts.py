import requests
import json
import copy
from prompts import items, robots, stations, generate_simple_prompt, generate_advanced_prompt, generate_expert_prompt


models = ["mistral:7b", "mistral-small:24b", "mistral-small3.2:24b",
          "gemma3:4b", "deepseek-r1:8b", "deepseek-r1:1.5b", "llama3.2:3b"]
msgs = ["Bring B to S5", "Bring C to S5"]


def run_test(pf, msg, model, robots):
    prompt = pf(msg, robots, stations, items)

    requestData = {
        'model': model,
        'think': False,
        'format': 'json',
        'stream': False,
        'prompt': prompt
    }

    r = requests.post(
        'http://192.168.1.131:11434/api/generate', json=requestData)
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


def run_all_tests():
    robots_v = [copy.deepcopy(robots), copy.deepcopy(
        robots), copy.deepcopy(robots)]
    prompts_f = [generate_simple_prompt,
                 generate_advanced_prompt, generate_expert_prompt]
    robots_v[1]['robot_1']['available'] = False
    robots_v[2]['robot_1']['available'] = False
    robots_v[2]['robot_2']['available'] = False
    robots_v[2]['robot_3']['available'] = False
    for pf in [prompts_f[1], prompts_f[2]]:
        for msg in msgs:
            for rs in robots_v:
                for model in models:
                    run_test(pf, msg, model, rs)


run_all_tests()
