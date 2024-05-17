# OpenAI Whisper controller for robot arm

See: https://github.com/openai/whisper


Requirements:

```
sudo apt install ffmpeg libportaudio2 python3-flask
```


## Setup

From inside the `whisper_server` folder

```
python3 -m venv venv

source venv/bin/activate

pip install -r requirements.txt

```

## Start the server

```
python3 server.py

```

## Start the ROS2 Node

```
ros2 run ar_whisper ar_whisper
```