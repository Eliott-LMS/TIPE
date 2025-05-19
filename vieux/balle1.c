
#include <stdio.h>
#include <stdlib.h>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define BALL_RADIUS 10
#define GRAVITY 9.18   // Accélération gravitationnelle
#define RESTITUTION 1 // Coefficient de restitution
#define FPS 60
#define DT 0.1

typedef struct {
    float m ; 
    float x, y;
    float vy; // Vitesse verticale
    float a; // acceleration
} Ball;

typedef struct
{
    float x,y ;
} Couple;

int main() {
    float time = 0;
    float dt = (0.100)/24;
    Ball balle = {m:1, x:100, y:100, vy:0, a: -GRAVITY};
    int taille_list = 1250;
    Couple* t = malloc(taille_list * sizeof(Couple));
    Couple c = {x: balle.x, y: balle.y} ;
    t[0] = c;
    for (int i=1; i<taille_list; i++ ) {
        time = time + dt;
        balle.vy = balle.vy + balle.a*dt;
        balle.y = balle.y + balle.vy * dt ;
        if (balle.y - BALL_RADIUS < 0 ) {balle.y = BALL_RADIUS; balle.vy = - balle.vy;} 
        c.x = balle.x;
        c.y = balle.y;
        t[i] = c;
        printf("%f,%f\n", t[i].x, t[i].y);
    }
    free(t);
}
