#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2023 iRobot Corporation. All rights reserved.
#

# A simple example to learn about color sensor getters.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Robot, Root
ColorID = Root.ColorID

robot = Root(Bluetooth())

@event(robot.when_play)
async def play(robot):
    while True:
        # get raw values under all lighting conditions
        for c in Root.ColorLighting:
            RGB = []
            print(c, await robot.get_color_values(c, Root.ColorFormat.ADC_COUNTS))
            #print(type(await robot.get_color_values(c, Root.ColorFormat.ADC_COUNTS)))
            mean = sum(await robot.get_color_values(c, Root.ColorFormat.ADC_COUNTS))
            print('avg is ' + str(mean/32))

        # get robot's parsed colors
        print("IDs", await robot.get_color_ids())
        await robot.wait(1)

robot.play()
robot.play()
