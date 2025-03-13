import argparse
import os
import time


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--ip", type=str, default="192.168.42.1")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--speak", action="store_true")
args = arg_parser.parse_args()

kps = [32]
trajectories = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]

command_base = f"python3 feetech/record.py --mass {args.mass} --length {args.length}"
command_base += f" --ip {args.ip} --logdir {args.logdir} --motor {args.motor}"


for kp in kps:
    for trajectory in trajectories:
        sentence = f"Kp {kp}, trajectory {trajectory.replace('_', ' ')}"
        print(sentence)

        if args.speak:
            from gtts import gTTS 
            myobj = gTTS(text=sentence, lang='en', slow=False) 
            myobj.save("/tmp/message.mp3")
            os.system("mpg321 /tmp/message.mp3")

        command = f"{command_base} --kp {kp} --trajectory {trajectory}"
        os.system(command)

        if trajectory == "sin_time_square":
            time.sleep(3)
