import pygame
import serial

pygame.init()

def main():
    joysticks = {}
    done = False
    serialPort = serial.Serial("COM4", 115200)
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag that we are done so we exit this loop.

            # if event.type == pygame.JOYBUTTONDOWN:
            #     print("Joystick button pressed.")
            #     print(event.button)
            #
            # if event.type == pygame.JOYBUTTONUP:
            #     print("Joystick button released.")
            #     print(event.button)
            #
            # if event.type == pygame.JOYAXISMOTION:
            #     print("Joystick button released.")
            #     print(event.axis)
            #     print(event.value)

            if event.type == pygame.JOYHATMOTION:
                x, y = event.value
                print("Joystick hat pressed.")
                print(x)
                print(y)
                cmd = 'S'
                if x == 1:
                    cmd = 'D'
                if x == -1:
                    cmd = 'G'
                if y == 1:
                    cmd = 'A'
                if y == -1:
                    cmd = 'R'
                print(cmd)
                serialPort.write(cmd.encode())

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connencted")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")
                done = True  # Flag that we are done so we exit this loop.


if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()