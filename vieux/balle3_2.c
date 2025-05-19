#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SCREEN_WIDTH 2.0
#define SCREEN_HEIGHT 2.0
#define BALL_RADIUS 0.050
#define GRAVITY 9.18   // Accélération gravitationnelle
#define RESTITUTION 1 // Coefficient de restitution
#define FPS 60
#define DT 0.001
#define FILENAME "posBalle3.txt"

typedef struct {
    double m;
    double x, y;
    double vy; // Vitesse verticale
    double vx; // Vitesse horizontale
    double ax; // Acceleration en x
    double ay; // Acceleration en y
} Ball;

typedef struct { double x, y; } Couple;

double random_double_range(double min, double max) {
    return min + (max - min) * ((double)rand() / (double)RAND_MAX);
}

double distance(double x, double y, double x2, double y2) {
    return sqrt(pow(x - x2, 2) + pow(y - y2, 2));
}

void gestion_collision(Ball *b1, Ball *b2) {
    double dx = b2->x - b1->x;
    double dy = b2->y - b1->y;
    double d = sqrt(dx * dx + dy * dy);
    if (d < 2 * BALL_RADIUS) {
        double nx = dx / d;
        double ny = dy / d;

        double dvx = b2->vx - b1->vx;
        double dvy = b2->vy - b1->vy;
        double dot_product = dvx * nx + dvy * ny;
        
        if (dot_product < 0) { // Empêcher chevauchement inutile
            double impulse = 2 * dot_product / (b1->m + b2->m);
            b1->vx += impulse * b2->m * nx;
            b1->vy += impulse * b2->m * ny;
            b2->vx -= impulse * b1->m * nx;
            b2->vy -= impulse * b1->m * ny;

            double overlap = 2 * BALL_RADIUS - d;
            b1->x -= overlap * 0.5 * nx;
            b1->y -= overlap * 0.5 * ny;
            b2->x += overlap * 0.5 * nx;
            b2->y += overlap * 0.5 * ny;
        }
    }
}
// void gestion_collision(Ball *b1, Ball *b2) {
//     double dx = b2->x - b1->x;
//     double dy = b2->y - b1->y;
//     double d = sqrt(dx * dx + dy * dy);

//     if (d < 2 * BALL_RADIUS) {
//         double nx = dx / d;
//         double ny = dy / d;

//         // Backtracking : Ramener les balles à leur position au moment exact de la collision
//         double dvx = b2->vx - b1->vx;
//         double dvy = b2->vy - b1->vy;
//         double dot_product = dvx * nx + dvy * ny;

//         if (dot_product < 0) { // Vérifier que les balles se rapprochent
//             double correction_time = (2 * BALL_RADIUS - d) / sqrt(dvx * dvx + dvy * dvy + 1e-8);
//             b1->x -= b1->vx * correction_time;
//             b1->y -= b1->vy * correction_time;
//             b2->x -= b2->vx * correction_time;
//             b2->y -= b2->vy * correction_time;

//             // Conservation de la quantité de mouvement
//             double impulse = 2 * dot_product / (b1->m + b2->m);
//             b1->vx += impulse * b2->m * nx;
//             b1->vy += impulse * b2->m * ny;
//             b2->vx -= impulse * b1->m * nx;
//             b2->vy -= impulse * b1->m * ny;

//             // Correction finale de position
//             double overlap = 2 * BALL_RADIUS - distance(b1->x, b1->y, b2->x, b2->y);
//             b1->x -= overlap * 0.5 * nx;
//             b1->y -= overlap * 0.5 * ny;
//             b2->x += overlap * 0.5 * nx;
//             b2->y += overlap * 0.5 * ny;
//         }
//     }
// }

int main() {
    FILE* fichier = fopen(FILENAME, "w");
    if (fichier == NULL) {
        printf("Impossible d'ouvrir le fichier %s\n", FILENAME);
        return 1;
    }
    int fps = 1/DT;
    printf("FPS : %i\n", fps);
    fprintf(fichier, "%f,%f,%f,%i\n", (double) SCREEN_WIDTH, (double) SCREEN_HEIGHT, BALL_RADIUS, fps);
    double time = 0;
    int nb_balles = 10;
    Ball *liste_balle = malloc(sizeof(Ball) * nb_balles);
    int taille_list = 10000;

    for (int i = 0; i < nb_balles; i++) {
        liste_balle[i] = (Ball){1, random_double_range(0, 2), random_double_range(0, 2), random_double_range(0, 2), random_double_range(0, 2), 0, -GRAVITY};
    }

    for (int i = 1; i < taille_list; i++) {
        time += DT;
        for (int j = 0; j < nb_balles; j++) {
            liste_balle[j].vy += liste_balle[j].ay * DT;
            liste_balle[j].y += liste_balle[j].vy * DT;
            liste_balle[j].vx += liste_balle[j].ax * DT;
            liste_balle[j].x += liste_balle[j].vx * DT;
            liste_balle[j].vx *= RESTITUTION;
            liste_balle[j].vy *= RESTITUTION;
            if (liste_balle[j].y - BALL_RADIUS < 0) { liste_balle[j].y = BALL_RADIUS; liste_balle[j].vy = -liste_balle[j].vy; }
            if (liste_balle[j].y + BALL_RADIUS > SCREEN_HEIGHT) { liste_balle[j].y = SCREEN_HEIGHT - BALL_RADIUS; liste_balle[j].vy = -liste_balle[j].vy; }
            if (liste_balle[j].x - BALL_RADIUS < 0) { liste_balle[j].x = BALL_RADIUS; liste_balle[j].vx = -liste_balle[j].vx; }
            if (liste_balle[j].x + BALL_RADIUS > SCREEN_WIDTH) { liste_balle[j].x = SCREEN_WIDTH - BALL_RADIUS; liste_balle[j].vx = -liste_balle[j].vx; }
        }

        for (int j = 0; j < nb_balles; j++) {
            for (int k = j + 1; k < nb_balles; k++) {
                gestion_collision(&liste_balle[j], &liste_balle[k]);
            }
        }

        for (int j = 0; j < nb_balles; j++) {
            fprintf(fichier, "%f,%f\t", liste_balle[j].x, liste_balle[j].y);
        }
        fprintf(fichier, "\n");
    }

    free(liste_balle);
    fclose(fichier);
    return 0;
}
