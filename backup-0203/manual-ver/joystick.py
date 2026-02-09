import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() > 0:
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("Joystick detected:", joy.get_name())

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                print("Button pressed:", event.button)
            elif event.type == pygame.JOYAXISMOTION:
                print("Axis:", event.axis, "Value:", event.value)
            # Add more event types as needed (e.g., JOYHATMOTION for D-pad)

    joy.quit()
pygame.quit()