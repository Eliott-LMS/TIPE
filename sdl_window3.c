#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 800
#define FIELD_WIDTH 10.0
#define FIELD_HEIGHT 10.0
#define BALL_RADIUS 0.050
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

typedef struct Cell {
    Ball* ball;           // Pointeur vers une balle
    struct Cell* next;    // Pointeur vers la balle suivante dans la cellule
} Cell;

typedef struct Grid {
    int rows;             // Nombre de lignes
    int cols;             // Nombre de colonnes
    double cell_width;    // Largeur d'une cellule
    double cell_height;   // Hauteur d'une cellule
    Cell** cells;         // Tableau de pointeurs vers les cellules
} Grid;
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


// void gestion_collision(Ball liste_balles[], int nb_balles) {
//     for (int i = 0; i < nb_balles; i++) {
//         for (int j = i + 1; j < nb_balles; j++) {
//             collission_balle(&liste_balles[i], &liste_balles[j]);
//         }
//     }
// }

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

Grid* creer_grille(int rows, int cols, double field_width, double field_height) {
    Grid* grid = malloc(sizeof(Grid));
    grid->rows = rows;
    grid->cols = cols;
    grid->cell_width = field_width / cols;
    grid->cell_height = field_height / rows;
    grid->cells = malloc(rows * cols * sizeof(Cell*));

    for (int i = 0; i < rows * cols; i++) {
        grid->cells[i] = NULL; // Initialiser chaque cellule à NULL
    }

    return grid;
}

int get_indexe_case(Grid* grid, double x, double y) {
    int col = (int)(x / grid->cell_width);
    int row = (int)(y / grid->cell_height);
    if (col >= grid->cols) col = grid->cols - 1;
    if (row >= grid->rows) row = grid->rows - 1;
    return row * grid->cols + col; // Index dans le tableau 1D
}

void add_balle_grille(Grid* grid, Ball* ball) {
    int index = get_cell_index(grid, ball->x, ball->y);

    // Créer une nouvelle cellule pour la balle
    Cell* new_cell = malloc(sizeof(Cell));
    new_cell->ball = ball;
    new_cell->next = grid->cells[index]; // Ajouter au début de la liste chaînée
    grid->cells[index] = new_cell;
}

void clear_grille(Grid* grid) {
    for (int i = 0; i < grid->rows * grid->cols; i++) {
        Cell* current = grid->cells[i];
        while (current != NULL) {
            Cell* temp = current;
            current = current->next;
            free(temp); // Libérer chaque cellule
        }
        grid->cells[i] = NULL; // Réinitialiser la cellule
    }
}

void gestion_collisison(Grid* grid) {
    for (int i = 0; i < grid->rows * grid->cols; i++) {
        Cell* cell = grid->cells[i];
        while (cell != NULL) {
            Ball* b1 = cell->ball;

            // Vérifier les collisions avec les autres balles dans la même cellule
            Cell* other = cell->next;
            while (other != NULL) {
                Ball* b2 = other->ball;
                collission_balle(b1, b2);
                other = other->next;
            }

            // Vérifier les collisions avec les balles des cellules voisines
            int row = i / grid->cols;
            int col = i % grid->cols;
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    int neighbor_row = row + dr;
                    int neighbor_col = col + dc;
                    if (neighbor_row >= 0 && neighbor_row < grid->rows &&
                        neighbor_col >= 0 && neighbor_col < grid->cols) {
                        int neighbor_index = neighbor_row * grid->cols + neighbor_col;
                        Cell* neighbor = grid->cells[neighbor_index];
                        while (neighbor != NULL) {
                            Ball* b2 = neighbor->ball;
                            if (b1 != b2) {
                                collission_balle(b1, b2);
                            }
                            neighbor = neighbor->next;
                        }
                    }
                }
            }

            cell = cell->next;
        }
    }
}

int main(int argc, char *argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("Erreur d'initialisation de SDL : %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Moteur 2D", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
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

    int nb_balles = 100;
    Ball* liste_balles = initialisation_balles(nb_balles);
    Grid* grid = create_grid(10, 10, FIELD_WIDTH, FIELD_HEIGHT); // 10x10 grille
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
        for (int i = 0; i < 1; i++) {
            temps += DT;

            // Mise à jour des balles
            move(liste_balles, nb_balles);
            
            clear_grid(grid);
            for (int i = 0; i < nb_balles; i++) {
                add_ball_to_grid(grid, &liste_balles[i]);
            }
            handle_collisions(grid);}

        // Effacer l'écran
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
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

        SDL_Delay(DT*1000/100); // Convertir DT en millisecondes
    }

    free(liste_balles);
    clear_grid(grid);
    free(grid->cells);
    free(grid);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}