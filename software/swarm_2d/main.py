"""
swarm_2d -- run with:  python3 main.py

Controls:
  1-6        toggle that surveillance region active/inactive
  left click on a robot   remove it
  A          add a robot
  R          reload the map (new random obstacles, resets robots/regions)
  Esc / close window      quit
"""
import sys

import pygame

import config
from simulation import Simulation


def main():
    pygame.init()
    pygame.display.set_caption("swarm_2d")
    screen = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 16, bold=True)
    small_font = pygame.font.SysFont("arial", 14)

    sim = Simulation()

    running = True
    while running:
        dt = clock.tick(config.FPS) / 1000.0
        dt = min(dt, 0.05)  # avoid huge steps if the window was dragged/paused

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_a:
                    sim.add_robot()
                elif event.key == pygame.K_r:
                    sim.reset_map(keep_active=False)
                elif pygame.K_1 <= event.key <= pygame.K_6:
                    region_id = event.key - pygame.K_1
                    sim.toggle_region(region_id)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                sim.kill_nearest_robot(event.pos)

        sim.update(dt)
        sim.draw(screen, font, small_font)
        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
