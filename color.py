#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2023 iRobot Corporation. All rights reserved.
#

# A simple example to learn about color sensor getters.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Robot, Root
ColorID = Root.ColorID

robot = Root(Bluetooth())

def get_color_name(red, green, blue):
    if r > 200 and g > 200 and b > 200:
        return 'White'
    elif r > 200 and g < 50 and b < 50:
        return 'Red'
    elif r < 50 and g > 200 and b < 50:
        return 'Green'
    elif r < 50 and g < 50 and b > 200:
        return 'Blue'
    elif r > 200 and g > 200 and b < 50:
        return 'Yellow'
    elif r > 200 and g < 50 and b > 200:
        return 'Magenta'
    elif r < 50 and g > 200 and b > 200:
        return 'Cyan'
    elif r < 50 and g < 50 and b < 50:
        return 'Black'
    else:
        return 'Unknown'

@event(robot.when_play)
async def play(robot):
    RGB = []
    while True:
        while len(RGB) < 5:
            #RGB = []
            # get raw values under all lighting conditions
            for c in Root.ColorLighting:
                print(c, await robot.get_color_values(c, Root.ColorFormat.ADC_COUNTS))
                #print(type(await robot.get_color_values(c, Root.ColorFormat.ADC_COUNTS)))
                summation = sum(await robot.get_color_values(c, Root.ColorFormat.ADC_COUNTS))
                #print('avg is ' + str(summation/32))
                mean = summation/32
                RGB.append(mean)
                #RGB = RGB + [mean]
                print(RGB)
                #print(RBG)

            # get robot's parsed colors
            print("IDs", await robot.get_color_ids())
            await robot.wait(1)
        red = RGB[1]
        green = RGB[2]
        blue = RGB[3]
        color_name = get_color_name(red, green, blue)
        print(color_name)
        RGB = []

robot.play()
