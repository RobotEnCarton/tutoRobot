import pygame
import serial

pygame.init()
clock = pygame.time.Clock()
print(pygame.joystick.get_count())

def main():
    joysticks = {}
    done = False
    if pygame.joystick.get_count() == 0 : exit()
    # serialPort = serial.Serial("COM4", 115200)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    while not done:
        pygame.event.pump()
        cmdD = (joystick.get_axis(5) + 1) * 128 - (joystick.get_axis(4) + 1) * 128 - (joystick.get_axis(0)) * 128
        cmdG = (joystick.get_axis(5) + 1) * 128 - (joystick.get_axis(4) + 1) * 128 + (joystick.get_axis(0)) * 128

        cmdD = min(255, cmdD)
        cmdG = min(255, cmdG)
        cmdD = max(-255, cmdD)
        cmdG = max(-255, cmdG)
        cmd = str(int(cmdD)) + ":" + str(int(cmdG))
        print(cmd)
        # serialPort.write(cmd.encode())
        # serialPort.write("\r\n".encode())

        # Limit to 10 per second
        clock.tick(10)

if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()