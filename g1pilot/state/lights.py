import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactoryInitialize(0, sys.argv[1])

    audio_client = AudioClient()  
    audio_client.SetTimeout(10.0)
    audio_client.Init()

    for i in range (100):
        audio_client.LedControl(255,0,0)
        time.sleep(0.5)
        audio_client.LedControl(0,0,255)
        time.sleep(0.5)
    