import pygame
import time
from request import Request

pygame.init()

def main():
    # PyGame automatically adds devices as they are connected
    joysticks = {}
    request = Request()

    done = False
    while not done:
        # Process events on the controller as they happen
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag that we are done so we exit this loop.

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

        # Get the hat values and the x for the joysticks connected
        for joystick in joysticks.values():

            if joystick.get_button(0) == 1:
                print("X hit")
                request.sendRequest("auto")
            if joystick.get_button(11) == 1:
                print("Forward hit")
                request.sendRequest("forward")
            if joystick.get_button(12) == 1:
                print("Backwards hit")
                request.sendRequest("backwards")
            if joystick.get_button(13) == 1:
                print("Left hit")
                request.sendRequest("left")
            if joystick.get_button(14) == 1:
                print("Right hit")
                request.sendRequest("right")
            else:
                request.sendRequest("stop")

            #Delay to allow for adequate request time
            time.sleep(0.25)

if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()