from openai import OpenAI
import json
import time
from Navigator import Navigator
from use2 import move_robot

# 连接ROS
viz = Navigator('192.168.3.32')

# 大模型客户端
client = OpenAI(api_key="api_key", base_url="https://api.deepseek.com")


def move_robot_to_location(location):
    print(f"函数调用中，目标位置：{location}")
    move_robot(viz, location)
    return {"status": "机器人到达目标位置", "location": location}


def control_robot_operate(task):
    print(f"函数调用中，执行任务：{task}")
    time.sleep(2.0)
    return {"status": "机器人操作结束", "task": task}


def sleep(sec):
    print(f"休眠中，休眠秒数：{sec}")
    time.sleep(sec)
    return {"status": "机器人休眠结束", "sleep_time": sec}


def run_conversation():
    messages = [{"role": "user", "content": "移动机器人到桌子后面，暂停5秒后移动机器人到桌子前面"}]

    tools = [
        {
            "type": "function",
            "function": {
                "name": "move_robot_to_location",
                "description": "移动机器人到指定位置",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {"type": "string", "description": "目标位置"}
                    },
                    "required": ["location"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "sleep",
                "description": "休眠指定秒数",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "sec": {"type": "integer", "description": "休眠秒数"}
                    },
                    "required": ["sec"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "control_robot_operate",
                "description": "控制机器人操作",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "task": {"type": "string", "description": "具体任务"}
                    },
                    "required": ["task"]
                }
            }
        }
    ]

    max_steps = 10  # 防止无限循环
    for _ in range(max_steps):
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            tools=tools,
            tool_choice="auto"
        )

        message = response.choices[0].message
        messages.append(message)

        if not message.tool_calls:
            return message.content  # 没有更多函数调用时返回最终回复

        # 处理所有函数调用
        for tool_call in message.tool_calls:
            func_name = tool_call.function.name
            args = json.loads(tool_call.function.arguments)

            if func_name == "move_robot_to_location":
                result = move_robot_to_location(args["location"])
            elif func_name == "sleep":
                result = sleep(args["sec"])
            elif func_name == "control_robot_operate":
                result = control_robot_operate(args["task"])

            messages.append({
                "tool_call_id": tool_call.id,
                "role": "tool",
                "name": func_name,
                "content": json.dumps(result)
            })

    return "达到最大执行步骤"


print(run_conversation())

viz.shutdown()
