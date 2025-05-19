import pygame
import sys

def convertion(s):
    x, y = s.split(",")
    return float(x), float(y)

def main():
    filename = "posBalle2.txt"
    liste_pos = []
    try:
        with open(filename, "r") as file:
            for line in file:
                liste_pos.append(convertion(line))
    except FileNotFoundError:
        print(f"Le fichier {filename} n'a pas été trouvé.")
        return
    except Exception as e:
        print(f"Une erreur est survenue: {e}")
        return

    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Ball Animation")

    clock = pygame.time.Clock()
    ball_radius = 10
    index = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        screen.fill((255, 255, 255))  # Clear screen with white background

        if index < len(liste_pos):
            x, y = liste_pos[index]
            pygame.draw.circle(screen, (0, 0, 255), (int(x * 400), int(600 - y * 600)), ball_radius)
            index += 1

        pygame.display.flip()
        clock.tick(60)  # 60 frames per second

if __name__ == "__main__":
    main()