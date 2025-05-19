#include <SDL2/SDL.h>

#include <SDL2/SDL2_gfxPrimitives.h>

#include <stdio.h>

#include <stdlib.h>

#include <math.h>



#define SCREEN_WIDTH 800

#define SCREEN_HEIGHT 700

#define FIELD_WIDTH 2.0

#define FIELD_HEIGHT 2.0



#define NB_BALL 1000

#define BALL_RADIUS 0.010

#define BALL_RADIUS_SDL (BALL_RADIUS * SCREEN_WIDTH / FIELD_WIDTH)



#define RESTITUTION 0.1 // Coefficient de restitution

#define GRAVITY 9.18   // Accélération gravitationnelle



#define FPS 60

#define DT 0.001

#define DECALAGE 1 // 1 pour activer le décalage, 0 pour le désactiver



#define TAILLE_CELL BALL_RADIUS*4



#define STR(x) #x 

#define XSTR(x) STR(x)



#define DOC "-----Documentation : -----\n " "Generation de " XSTR(NB_BALL) " balles \n Forces : gravite  \n Restitution :" XSTR(RESTITUTION) " \n Methode : deplacement discret puis gestion collision par methode matrice \n" "--------------------------\n"



#define CLUSTER_DISTANCE (BALL_RADIUS * 6)





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





void gestion_collision(Ball liste_balles[], int nb_balles) {

    for (int i = 0; i < nb_balles; i++) {

        for (int j = i + 1; j < nb_balles; j++) {

            collission_balle(&liste_balles[i], &liste_balles[j]);

        }

    }

}



Ball* initialisation_balles(int nb_balles) {

    Ball* liste_balles = malloc(sizeof(Ball) * nb_balles);

    double center_x = 1.0;

    double center_y = 1.5;

    double radius_init = 0.05; // rayon initial pour répartir autour



    for (int i = 0; i < nb_balles; i++) {

        double angle = random_double_range(0, 2 * M_PI);

        double r = random_double_range(0, radius_init);



        double x = center_x + r * cos(angle);

        double y = center_y + r * sin(angle);



        liste_balles[i] = (Ball){

            .m = 1,

            .x = x,

            .y = y,

            .vx = random_double_range(-0.5, 0.5),

            .vy = random_double_range(-0.5, 0.5),

            .ax = 0,

            .ay = -GRAVITY

        };

    }



    return liste_balles;

}





Grid* create_grid(int rows, int cols, double field_width, double field_height) {

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



int get_cell_index(Grid* grid, double x, double y) {

    int col = (int)(x / grid->cell_width);

    int row = (int)(y / grid->cell_height);

    if (col >= grid->cols) col = grid->cols - 1;

    if (row >= grid->rows) row = grid->rows - 1;

    return row * grid->cols + col; // Index dans le tableau 1D

}



void add_ball_to_grid(Grid* grid, Ball* ball) {

    int index = get_cell_index(grid, ball->x, ball->y);



    // Créer une nouvelle cellule pour la balle

    Cell* new_cell = malloc(sizeof(Cell));

    new_cell->ball = ball;

    new_cell->next = grid->cells[index]; // Ajouter au début de la liste chaînée

    grid->cells[index] = new_cell;

}



void clear_grid(Grid* grid) {

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



void handle_collisions(Grid* grid) {

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



int* cluster_balles(Ball *b, int nb_balles, int *nb_clusters) {

    int *cluster_ids = malloc(nb_balles * sizeof(int));

    for (int i = 0; i < nb_balles; i++) cluster_ids[i] = -1;



    int current_cluster = 0;



    for (int i = 0; i < nb_balles; i++) {

        if (cluster_ids[i] != -1) continue;



        // Démarrer une nouvelle cluster

        cluster_ids[i] = current_cluster;



        for (int j = 0; j < nb_balles; j++) {

            if (i != j && cluster_ids[j] == -1) {

                double d = distance(b[i].x, b[i].y, b[j].x, b[j].y);

                if (d < CLUSTER_DISTANCE) {

                    cluster_ids[j] = current_cluster;

                }

            }

        }



        current_cluster++;

    }



    *nb_clusters = current_cluster;

    return cluster_ids;

}



void draw_balls_with_clusters(SDL_Renderer* renderer, Ball *b, int nb, int *clusters, int nb_clusters, double rapportx, double rapporty, int screen_radius) {

    int cluster_counts[nb_clusters];

    for (int i = 0; i < nb_clusters; i++) cluster_counts[i] = 0;



    // Compter le nombre de balles par cluster

    for (int i = 0; i < nb; i++) {

        cluster_counts[clusters[i]]++;

    }



    for (int i = 0; i < nb; i++) {

        int screen_x = (int)(b[i].x * rapportx);

        int screen_y = (int)(SCREEN_HEIGHT - b[i].y * rapporty);



        int count = cluster_counts[clusters[i]];

        // Plus il y a de balles, plus le bleu est foncé (max 255)

        int intensity = count * 255 / NB_BALL;

        if (intensity > 255) intensity = 255;



        filledCircleRGBA(renderer, screen_x, screen_y, screen_radius, 0, 0, intensity, 255);

    }

}





int orientation(Couple p, Couple q, Couple r) {

    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (fabs(val) < 1e-9) return 0;

    return (val > 0) ? 1 : 2;

}



int compare_couples(const void* a, const void* b) {

    Couple* p1 = (Couple*)a;

    Couple* p2 = (Couple*)b;

    if (p1->x != p2->x)

        return (p1->x < p2->x) ? -1 : 1;

    return (p1->y < p2->y) ? -1 : 1;

}



int convex_hull(Couple points[], int n, Couple result[]) {

    if (n < 3) return 0;



    qsort(points, n, sizeof(Couple), compare_couples);

    Couple lower[n], upper[n];

    int l = 0, u = 0;



    for (int i = 0; i < n; i++) {

        while (l >= 2 && orientation(lower[l - 2], lower[l - 1], points[i]) != 2) l--;

        lower[l++] = points[i];

    }

    for (int i = n - 1; i >= 0; i--) {

        while (u >= 2 && orientation(upper[u - 2], upper[u - 1], points[i]) != 2) u--;

        upper[u++] = points[i];

    }



    int idx = 0;

    for (int i = 0; i < l - 1; i++) result[idx++] = lower[i];

    for (int i = 0; i < u - 1; i++) result[idx++] = upper[i];

    return idx;

}



void draw_cluster_outline(SDL_Renderer* renderer, Ball *balls, int *clusters, int nb_balls, int cluster_id, double rapportx, double rapporty) {

    Couple pts[NB_BALL];

    int count = 0;



    for (int i = 0; i < nb_balls; i++) {

        if (clusters[i] == cluster_id) {

            pts[count++] = (Couple){balls[i].x, balls[i].y};

        }

    }



    if (count < 3) return;



    Couple hull[NB_BALL];

    int hull_size = convex_hull(pts, count, hull);



    Sint16 vx[hull_size], vy[hull_size];

    for (int i = 0; i < hull_size; i++) {

        vx[i] = (Sint16)(hull[i].x * rapportx);

        vy[i] = (Sint16)(SCREEN_HEIGHT - hull[i].y * rapporty);

    }



    

        // Adoucir un peu la forme (moyenne mobile)

    for (int i = 0; i < hull_size; i++) {

        int prev = (i - 1 + hull_size) % hull_size;

        int next = (i + 1) % hull_size;

        vx[i] = (vx[prev] + vx[i] + vx[next]) / 3;

        vy[i] = (vy[prev] + vy[i] + vy[next]) / 3;

            }

            



       // Couleur en fonction de la taille du cluster

    int intensity = count * 255 / NB_BALL;

    if (intensity > 255) intensity = 255;

   

    filledPolygonRGBA(renderer, vx, vy, hull_size, 0, 0, intensity, 60);  // bleu plus foncé si dense

    aapolygonRGBA(renderer, vx, vy, hull_size, 0, 0, intensity, 180);     // contour plus net

   

}





void draw_cluster_outline(SDL_Renderer* renderer, Ball *balls, int *clusters, int nb_balls, int cluster_id, double rapportx, double rapporty);





int main(int argc, char *argv[]) {

    printf("%s", DOC);

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



    int nb_balles = NB_BALL;

    Ball* liste_balles = initialisation_balles(nb_balles);

    Grid* grid = create_grid(10, 10, FIELD_WIDTH, FIELD_HEIGHT); // 10x10 grille

    double temps = 0;



    int running = 1;

    int pause = 0;

    SDL_Event event;



    double rapportx = SCREEN_WIDTH / FIELD_WIDTH;

    double rapporty = SCREEN_HEIGHT / FIELD_HEIGHT;

    int screen_radius = BALL_RADIUS * rapportx;



    while (running) {

        while (SDL_PollEvent(&event)) {

            if (event.type == SDL_QUIT) {

                running = 0;

            }

            if (event.type == SDL_KEYDOWN) {

                if (event.key.keysym.sym == SDLK_SPACE) {

                    pause = 1 ; 

                }

            }

        }

        if  (pause == 0) {

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

            int nb_clusters = 0;

            int *clusters = cluster_balles(liste_balles, nb_balles, &nb_clusters);



            // (Optionnel) Dessin des contours d'enveloppe ici, si tu as ajouté la fonction draw_cluster_outline

            for (int id = 0; id < nb_clusters; id++) {

                draw_cluster_outline(renderer, liste_balles, clusters, nb_balles, id, rapportx, rapporty);

            }



            // Dessin des balles en couleur par cluster

            //draw_balls_with_clusters(renderer, liste_balles, nb_balles, clusters, nb_clusters, rapportx, rapporty, screen_radius);



            free(clusters);





            // Mettre à jour l'écran

            SDL_RenderPresent(renderer);



            SDL_Delay(DT*1000/100); // Convertir DT en millisecondes

        }

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

