import rclpy
from rclpy.node import Node
from .livewhisper import StreamHandler
import json
import os

from openai import OpenAI
from ar_interfaces.msg import ARWhisperCommand, ARWhisperCommands



class ARWhisperCommander(Node):

    def __init__(self):
        super().__init__('ARWhisperCommander')
        self.openai_client = OpenAI(
          # This is the default and can be omitted
          api_key=os.environ.get("OPENAI_API_KEY")
      )
        
        self.ar_whisper_command_publisher = self.create_publisher(ARWhisperCommands, "/ar_whisper_commands", 10)

        self.running = True
        self.talking = False
        self.prompted = False
        self.handler = StreamHandler(self)
        self.handler.listen()

    def analyze(self, input):  # Callback fired by livewhisper
        chatgpt_message = """
You are to convert human directions as instructions for movement and rotation for a robotic arm. The robotic arm also has a gripper.

If movement is given on an axis with no specified distance use the default value of 0.05.
If rotation is given on an axis with no specified distance use the default value of 0.05.

If instructions for a gipper is not mention, use the value "keep". Otherwise use the value "open" or "closed".

For movement and rotation:
    left is negative in the x axis
    right is positive in the x axis
    up is positive in the z axis
    down is negative in the z axis
    forwards is negative in the y axis
    backwards is positive in the y axis

If told to "move home", set the position_name field to "home",
If told to "move upgright", set the position_name field to "upright".

Do not include any explanations, only provide a RFC8259 compliant JSON response following this format without deviation:
[
  {
    "translation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    },
    "rotation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    },
    "gripper_state": "open|closed|keep",
    "position_name": ""
  }
]

If the directions have multiple steps, you may return those steps as multiple objects inside the top level JSON array.
If the directions are not valid, return an empty array.

---

""" + input

        self._logger.info("Parsing via ChatGPT")
        completion = self.openai_client.chat.completions.create(
            messages=[
                {
                    "role": "user",
                    "content": chatgpt_message,
                }
            ],
            model="gpt-3.5-turbo",
        )

        if len(completion.choices) >= 1:
            result = json.loads(completion.choices[0].message.content)
            self.get_logger().info(f"Response from GTP: {result}")
            commands = []
            for data in result:
                self.get_logger().info(f"Command from GTP: {data}")
                msg = ARWhisperCommand()
                msg.translate_x = data["translation"]["x"]
                msg.translate_y = data["translation"]["y"]
                msg.translate_z = data["translation"]["z"]
                msg.rotate_x = data["rotation"]["x"]
                msg.rotate_y = data["rotation"]["y"]
                msg.rotate_z = data["rotation"]["z"]
                msg.gripper_state = data["gripper_state"]
                msg.position_name = data["position_name"]

                commands.append(msg)

            if len(commands) > 0:
                ar_whisper_commands = ARWhisperCommands()
                ar_whisper_commands.commands = commands
                self.ar_whisper_command_publisher.publish(ar_whisper_commands)

    

def main(args=None):
    rclpy.init(args=args)
    commander_node = ARWhisperCommander()

    rclpy.spin(commander_node)
    commander_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
