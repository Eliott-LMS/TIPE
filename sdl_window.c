#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 800
#define FIELD_WIDTH 2.0
#define FIELD_HEIGHT 2.0
#define BALL_RADIUS 0.025
#define BALL_RADIUS_SDL (BALL_RADIUS * SCREEN_WIDTH / FIELD_WIDTH)
#define GRAVITY 9.18   // Accélération gravitationnelle
#define RESTITUTION 0.1 // Coefficient de restitution
#define FPS 60
#define DT 0.001
#define DECALAGE 1 // 1 pour activer le décalage, 0 pour le désactiver
#define TAILLE_CELL BALL_RADIUS*4

//---------------------------------------------------------------------------------------
typedef struct {
    double m ;
    double x, y;
    double vy; // Vitesse verticale
    double vx; // Vitesse horizontale
    double ax; // acceleration en x
    double ay; // acceleration en y
} Ball;
typedef struct {
    double xmin;
    double ymin;
    double xmax;
    double ymax;
    double cx ;
    double cy ;
    } Rectangle;

typedef struct {
    int nb_ligne;
    int nb_colonne;
    Ball** tab;
    } Grille;

typedef struct { double x,y ;} Couple; //Type pour les coordonnées
//---------------------------------------------------------------------------------------
double random_double_range(double min, double max) {
    return min + (max - min) * ((double)rand() / (double)RAND_MAX);
}

double distance(double x, double y, double x2, double y2) {
    return sqrt (pow(x-x2, 2) + pow(y-y2, 2));
}

double norme (double x, double y) {
    return sqrt(pow(x,2) + pow(y,2));
}

void collission_balle(Ball *b1, Ball *b2) {

    // Calcul de la différence de position entre les deux balles
    double dx = b2->x - b1->x; // Différence en x
    double dy = b2->y - b1->y; // Différence en y

    // Calcul de la distance entre les centres des deux balles
    double d = sqrt(dx * dx + dy * dy);

    // Vérifie si les deux balles sont en collision (distance < 2 * rayon)
    if (d < 2 * BALL_RADIUS) {
        // Calcul du vecteur normalisé de la collision (direction de la collision)
        double nx = dx / d; // Composante x de la normale
        double ny = dy / d; // Composante y de la normale

        // Calcul de la différence de vitesse entre les deux balles
        double dvx = b2->vx - b1->vx; // Différence de vitesse en x
        double dvy = b2->vy - b1->vy; // Différence de vitesse en y

        // Produit scalaire entre la différence de vitesse et la normale
        // Cela permet de déterminer la composante de la vitesse dans la direction de la collision
        double dot_product = dvx * nx + dvy * ny;

        // Si le produit scalaire est négatif, cela signifie que les balles se rapprochent
        if (dot_product < 0) { // Empêche les chevauchements inutiles
            // Calcul de l'impulsion de collision
            double impulse = 2 * dot_product / (b1->m + b2->m) * RESTITUTION;

            // Mise à jour des vitesses des deux balles en fonction de l'impulsion
            b1->vx += impulse * b2->m * nx; // Mise à jour de la vitesse en x pour la balle 1
            b1->vy += impulse * b2->m * ny; // Mise à jour de la vitesse en y pour la balle 1
            b2->vx -= impulse * b1->m * nx; // Mise à jour de la vitesse en x pour la balle 2
            b2->vy -= impulse * b1->m * ny; // Mise à jour de la vitesse en y pour la balle 2
            if (DECALAGE) {
                // Calcul du chevauchement entre les deux balles
                double overlap = 2 * BALL_RADIUS - d;

                // Correction des positions pour éviter que les balles restent imbriquées
                // On déplace les balles le long de la normale de collision
                b1->x -= overlap * 0.5 * nx; // Déplacement de la balle 1 en x
                b1->y -= overlap * 0.5 * ny; // Déplacement de la balle 1 en y
                b2->x += overlap * 0.5 * nx; // Déplacement de la balle 2 en x
                b2->y += overlap * 0.5 * ny; // Déplacement de la balle 2 en y
            }
        }
    }
}

void move (Ball liste_balles[], int nb_balles) {
    for (int i = 0; i < nb_balles; i++) {
        liste_balles[i].vy += liste_balles[i].ay * DT;
        liste_balles[i].y += liste_balles[i].vy * DT;
        liste_balles[i].vx += liste_balles[i].ax * DT;
        liste_balles[i].x += liste_balles[i].vx * DT;
        if (liste_balles[i].y - BALL_RADIUS < 0) { 
            liste_balles[i].y = BALL_RADIUS; 
            liste_balles[i].vy = -liste_balles[i].vy;
        }
        if (liste_balles[i].y + BALL_RADIUS > FIELD_HEIGHT) { 
            liste_balles[i].y = FIELD_HEIGHT - BALL_RADIUS; 
            liste_balles[i].vy = -liste_balles[i].vy;
        }
        if (liste_balles[i].x - BALL_RADIUS < 0) { 
            liste_balles[i].x = BALL_RADIUS; 
            liste_balles[i].vx = -liste_balles[i].vx; 
        }
        if (liste_balles[i].x + BALL_RADIUS > FIELD_WIDTH) { 
            liste_balles[i].x = FIELD_WIDTH - BALL_RADIUS; 
            liste_balles[i].vx = -liste_balles[i].vx; 
        }
    }
}


void gestion_collision(Ball liste_balles[], int nb_balles) {
    for (int i = 0; i < nb_balles; i++) {
        for (int j = i + 1; j < nb_balles; j++) {
            collission_balle(&liste_balles[i], &liste_balles[j]);
        }
    }
}

Ball* initialisation_balles(int nb_balles) {
    Ball* liste_balles = malloc(sizeof(Ball) * nb_balles);
    for (int i = 0; i < nb_balles; i++) {
        liste_balles[i] = (Ball){
            1, 
            random_double_range(0, FIELD_WIDTH), 
            random_double_range(0, FIELD_HEIGHT), 
            random_double_range(-2, 2), 
            random_double_range(-2, 2), 
            0, 
            -GRAVITY
        };
    }
    return liste_balles;
}

int main(int argc, char *argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("Erreur d'initialisation de SDL : %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Balles rebondissantes", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        printf("Erreur de création de la fenêtre : %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("Erreur de création du renderer : %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    int nb_balles = 500;
    Ball* liste_balles = initialisation_balles(nb_balles);
    double temps = 0;

    int running = 1;
    SDL_Event event;

    double rapportx = SCREEN_WIDTH / FIELD_WIDTH;
    double rapporty = SCREEN_HEIGHT / FIELD_HEIGHT;
    int screen_radius = BALL_RADIUS * rapportx;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            }
        }
        temps += DT;

        // Mise à jour des balles
        move(liste_balles, nb_balles);
        gestion_collision(liste_balles, nb_balles);
        // gestion_collision(liste_balles, nb_balles);
        // gestion_collision(liste_balles, nb_balles);
        // gestion_collision(liste_balles, nb_balles);
        // gestion_collision(liste_balles, nb_balles);


        // Effacer l'écran
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Dessiner les balles
        for (int i = 0; i < nb_balles; i++) {
            int screen_x = (int)(liste_balles[i].x * rapportx);
            int screen_y = (int)(SCREEN_HEIGHT - liste_balles[i].y * rapporty);

            // Dessiner un cercle rempli avec SDL2_gfx
            filledCircleRGBA(renderer, screen_x, screen_y, screen_radius, 0, 0, 255, 255);
        }

        // Mettre à jour l'écran
        SDL_RenderPresent(renderer);

        SDL_Delay(1); // Convertir DT en millisecondes
    }

    free(liste_balles);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}