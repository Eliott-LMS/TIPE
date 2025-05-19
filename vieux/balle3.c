#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SCREEN_WIDTH 2
#define SCREEN_HEIGHT 2
#define BALL_RADIUS 0.050
#define GRAVITY 9.18   // Accélération gravitationnelle
#define RESTITUTION 1 // Coefficient de restitution
#define FPS 60
#define DT 0.001
#define FILENAME "posBalle3.txt"

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
            double impulse = 2 * dot_product / (b1->m + b2->m);

            // Mise à jour des vitesses des deux balles en fonction de l'impulsion
            b1->vx += impulse * b2->m * nx; // Mise à jour de la vitesse en x pour la balle 1
            b1->vy += impulse * b2->m * ny; // Mise à jour de la vitesse en y pour la balle 1
            b2->vx -= impulse * b1->m * nx; // Mise à jour de la vitesse en x pour la balle 2
            b2->vy -= impulse * b1->m * ny; // Mise à jour de la vitesse en y pour la balle 2

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

void move (Ball liste_balles[], int nb_balles) {
    for (int i = 0; i < nb_balles; i++) {
        liste_balles[i].vy += liste_balles[i].ay * DT;
        liste_balles[i].y += liste_balles[i].vy * DT;
        liste_balles[i].vx += liste_balles[i].ax * DT;
        liste_balles[i].x += liste_balles[i].vx * DT;
        liste_balles[i].vx *= RESTITUTION;
        liste_balles[i].vy *= RESTITUTION;
        if (liste_balles[i].y - BALL_RADIUS < 0) { liste_balles[i].y = BALL_RADIUS; liste_balles[i].vy = -liste_balles[i].vy; }
        if (liste_balles[i].y + BALL_RADIUS > SCREEN_HEIGHT) { liste_balles[i].y = SCREEN_HEIGHT - BALL_RADIUS; liste_balles[i].vy = -liste_balles[i].vy; }
        if (liste_balles[i].x - BALL_RADIUS < 0) { liste_balles[i].x = BALL_RADIUS; liste_balles[i].vx = -liste_balles[i].vx; }
        if (liste_balles[i].x + BALL_RADIUS > SCREEN_WIDTH) { liste_balles[i].x = SCREEN_WIDTH - BALL_RADIUS; liste_balles[i].vx = -liste_balles[i].vx; }
    }
}

void gestion_collision(Ball liste_balles[], int nb_balles) {
    for (int i = 0; i < nb_balles; i++) {
        for (int j = i + 1; j < nb_balles; j++) {
            collission_balle(&liste_balles[i], &liste_balles[j]);
        }
    }
}

void enregistrement (FILE* fichier, Ball liste_balles[], int nb_balles) {
    for (int i = 0; i < nb_balles; i++) {
        fprintf(fichier, "%f,%f\t", liste_balles[i].x, liste_balles[i].y);
    }
    fprintf(fichier, "\n");
}

Ball* initialisation_balles(int nb_balles) {
    Ball* liste_balles = malloc(sizeof(Ball) * nb_balles);
    for (int i = 0; i < nb_balles; i++) {
        liste_balles[i] = (Ball){1, random_double_range(0, 2), random_double_range(0, 2), random_double_range(0, 2), random_double_range(0, 2), 0, -GRAVITY};
    }
    return liste_balles;
}

Rectangle* initialisation_rectangle(int nb_rectangles) {
    Rectangle* liste_rectangles = malloc(sizeof(Rectangle) * nb_rectangles);
    double xmin = random_double_range(0, SCREEN_WIDTH*1.2);
    double ymin = random_double_range(0, SCREEN_HEIGHT*1.2);
    double xmax = random_double_range(xmin, SCREEN_WIDTH*1.2);
    double ymax = random_double_range(ymin, SCREEN_HEIGHT*1.2);
    double cx = xmin + (xmax - xmin)/2;
    double cy = ymin + (ymax - ymin)/2;
    for (int i = 0; i < nb_rectangles; i++) {
        liste_rectangles[i] = (Rectangle){xmin, ymin, xmax, ymax, cx, cy};
    }

    return liste_rectangles;
}
//---------------------------------------------------------------------------------------
int main() {

    //Ouverture du fichier contenant les positions de la balle
    FILE* fichier = fopen(FILENAME, "w"); 
    if (fichier == NULL) {
        printf("Impossible d'ouvrir le fichier %s\n", FILENAME);
        return 1;
    }

    int fps = 1/DT;
    printf("FPS : %i\n", fps);
    fprintf(fichier, "%d,%d,%f,%i\n", SCREEN_WIDTH, SCREEN_HEIGHT, BALL_RADIUS, fps);


    double time = 0; //temps
    int nb_balles = 10; //nombre de balles
    int taille_list = 100000;    //nombre de positions des balles
    
    Ball * liste_balle = initialisation_balles(nb_balles);

    for (int i=1; i<taille_list; i++ ) {
        time = time + DT;
        move(liste_balle, nb_balles);
        gestion_collision(liste_balle, nb_balles);
        enregistrement(fichier, liste_balle, nb_balles);
    }

    free(liste_balle);
    fclose(fichier);
    return 0;
}

// gcc -Wall balle3.c -0 balle3 -lm
// ./balle3