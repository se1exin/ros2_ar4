from livewhisper import StreamHandler

import os
import re

class WhisperServer:
    def __init__(self):
      self.running = True
      self.talking = False
      self.prompted = False

    def analyze(self, input):  # This is the decision tree for the assistant
      
      command = input.lower().strip()

      print("WE GOT")
      print(command)

      if not command.startswith("move"):
         print("Move command not found. Ignoring")
         return

      parts = command.split(" ")
      if len(parts) == 3:
         # First part is "move"
         # Second part is distance
         # Third part is direction

         distance = re.sub(r'\W+', '', int(parts[1]))  # Remove all non-alphanumeric
         direction = re.sub(r'\W+', '', parts[2])

         print(distance)
         print(direction)


def main():
  try:
      server = WhisperServer()
      handler = StreamHandler(server)
      handler.listen()
  except (KeyboardInterrupt, SystemExit): pass
  finally:
      print("\n\033[93mQuitting..\033[0m")
      if os.path.exists('dictate.wav'): os.remove('dictate.wav')

if __name__ == '__main__':
    main()