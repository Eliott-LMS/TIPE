
#include <stdio.h>
#include <stdlib.h>

#define SCREEN_WIDTH 2
#define SCREEN_HEIGHT 1
#define BALL_RADIUS 0.010
#define GRAVITY 9.18   // Accélération gravitationnelle
#define RESTITUTION 1 // Coefficient de restitution
#define FPS 60
#define DT 0.1
#define FILENAME "posBalle2.txt"


typedef struct {
    float m ; 
    float x, y;
    float vy; // Vitesse verticale
    float vx; // Vitesse horizontale
    float ax; // acceleration en x
    float ay; // acceleration en y
} Ball;

typedef struct
{
    float x,y ;
} Couple;

int main() {
    FILE* fichier = fopen(FILENAME, "w"); //OUverture du fichier contenant les positions de la balle 
    if (fichier == NULL) {
        printf("Impossible d'ouvrir le fichier %s\n", FILENAME);
        return 1;
    }
    float time = 0; //temps
    float dt = 0.01/24; //pas de temps
    Ball balle = {m:1, x:0.5, y:0.5, vy:1, vx: 1, ax: 0, ay: -GRAVITY}; //balle
    int taille_list = 10000;    //nombre de positions de la balle
    Couple* t = malloc(taille_list * sizeof(Couple)); //tableau de positions de la balle
    Couple c = {x: balle.x, y: balle.y} ; //position de la balle a t= 0
    t[0] = c;
    for (int i=1; i<taille_list; i++ ) {
        time = time + dt;
        //Calcul des nouvelles positions de la balle
        balle.vy = balle.vy + balle.ay*dt;
        balle.y = balle.y + balle.vy * dt ;
        balle.vx = balle.vx + balle.ax*dt;
        balle.x = balle.x + balle.vx * dt ;
        //Gestion des collisions au rebords de l'écran
        if (balle.y - BALL_RADIUS < 0 ) {balle.y = BALL_RADIUS; balle.vy = - balle.vy;} 
        if (balle.y + BALL_RADIUS > SCREEN_HEIGHT ) {balle.y = SCREEN_HEIGHT - BALL_RADIUS; balle.vy = - balle.vy;} 
        if (balle.x - BALL_RADIUS < 0 ) {balle.x = BALL_RADIUS; balle.vx = - balle.vx;} 
        if (balle.x - BALL_RADIUS > SCREEN_WIDTH ) {balle.x = SCREEN_WIDTH - BALL_RADIUS; balle.vx = - balle.vx;} 
        //Enregistrement de la position de la balle
        c.x = balle.x;
        c.y = balle.y;
        t[i] = c;
        fprintf(fichier, "%f,%f\n", c.x, c.y);
    }
    free(t);
    fclose(fichier);
    return 0;
}
