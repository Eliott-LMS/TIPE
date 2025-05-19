#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SCREEN_WIDTH_SDL 1600
#define SCREEN_HEIGHT_SDL 800
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 800
#define FIELD_WIDTH 2.0
#define FIELD_HEIGHT 2.0

#define NB_BALL 1000
#define BALL_RADIUS 0.010
#define BALL_RADIUS_SDL (BALL_RADIUS * SCREEN_WIDTH / FIELD_WIDTH)

#define RESTITUTION 0.1 // Coefficient de restitution
#define GRAVITY 9.18   // Accélération gravitationnelle

#define FPS 60
#define DT 0.01
#define DECALAGE 1 // 1 pour activer le décalage, 0 pour le désactiver

#define TAILLE_CELL BALL_RADIUS*4

#define STR(x) #x 
#define XSTR(x) STR(x)

#define DOC "-----Documentation : -----\n " "Generation de " XSTR(NB_BALL) " balles \n Forces : gravite  \n Restitution :" XSTR(RESTITUTION) " \n Methode : deplacement discret puis gestion collision par methode matrice \n" "--------------------------\n"

#define PERF_HISTORY 10

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

Grid* create_grid(int rows, int cols, double field_width, double field_height) {
    /*
    O(rows * cols)
    */
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
    for (int i = 0; i < grid->rows * grid->cols; i++) { //O(cols * rows)
        Cell* cell = grid->cells[i];
        while (cell != NULL) { //O(nb_balles)
            Ball* b1 = cell->ball;

            // Vérifier les collisions avec les autres balles dans la même cellule
            Cell* other = cell->next;
            while (other != NULL) { //O(nb_balles)
                Ball* b2 = other->ball;
                collission_balle(b1, b2); //O(1)
                other = other->next;
            } //O(nb_balles * 1)

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
                        while (neighbor != NULL) { //O(nb_balles)
                            Ball* b2 = neighbor->ball;
                            if (b1 != b2) {
                                collission_balle(b1, b2);//O(1)
                            }
                            neighbor = neighbor->next;
                        } //O(nb_balles * 1)
                    }
                }
            } //O(9 * nb_balles)

            cell = cell->next;
        }//O(nb_balles ** 2)
    } //O(nb_balles ** 2)
}

void chrono(double temps, double decalage, SDL_Renderer* renderer, TTF_Font* font) {
    char chrono_text[60];
    sprintf(chrono_text, "%.2f s", temps);

    // Définir la couleur du texte pour le chrono
    SDL_Color textColor = {0, 0, 0, 255}; // Noir

    // Créer une surface pour le texte du chrono
    SDL_Surface* textSurface = TTF_RenderText_Blended(font, chrono_text, textColor);
    if (!textSurface) {
        printf("Erreur de création de la surface texte : %s\n", TTF_GetError());
    }

    // Créer une texture à partir de la surface
    SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
    if (!textTexture) {
        printf("Erreur de création de la texture texte : %s\n", SDL_GetError());
    }

    // Définir la position et la taille du texte du chrono
    SDL_Rect textRect = {SCREEN_WIDTH - textSurface->w - 110, 10, textSurface->w, textSurface->h}; // En haut à droite

    // Copier la texture du chrono sur le renderer
    SDL_RenderCopy(renderer, textTexture, NULL, &textRect);

    // Libérer la surface et la texture du chrono
    SDL_FreeSurface(textSurface);
    SDL_DestroyTexture(textTexture);

    // Afficher le décalage
    char decalage_text[50];
    sprintf(decalage_text, "%.2f", decalage);

    // Définir la couleur du texte pour le décalage
    SDL_Color decalageColor = (decalage < 0) ? (SDL_Color){255, 0, 0, 255} : (SDL_Color){0, 255, 0, 255}; // Rouge si négatif, vert sinon

    // Créer une surface pour le texte du décalage
    SDL_Surface* decalageSurface = TTF_RenderText_Blended(font, decalage_text, decalageColor);
    if (!decalageSurface) {
        printf("Erreur de création de la surface texte pour le décalage : %s\n", TTF_GetError());
    }

    // Créer une texture à partir de la surface du décalage
    SDL_Texture* decalageTexture = SDL_CreateTextureFromSurface(renderer, decalageSurface);
    if (!decalageTexture) {
        printf("Erreur de création de la texture texte pour le décalage : %s\n", SDL_GetError());
    }

    // Définir la position et la taille du texte du décalage
    SDL_Rect decalageRect = {SCREEN_WIDTH - 100, 10, decalageSurface->w, decalageSurface->h}; // À côté du chrono

    // Copier la texture du décalage sur le renderer
    SDL_RenderCopy(renderer, decalageTexture, NULL, &decalageRect);

    // Libérer la surface et la texture du décalage
    SDL_FreeSurface(decalageSurface);
    SDL_DestroyTexture(decalageTexture);
}

void draw_performance_graph(SDL_Renderer* renderer, double* total, double* affichage, double* calcul, int index, TTF_Font* font) {
    int graph_x = SCREEN_WIDTH; // 800
    int graph_y = 0;
    int graph_w = SCREEN_WIDTH; // 800
    int graph_h = SCREEN_HEIGHT; // 800

    // Trouver le max pour l'échelle
    // double max_val = 1.0;
    // for (int i = 0; i < PERF_HISTORY; i++) {
    //     if (total[i] > max_val) max_val = total[i];
    //     if (affichage[i] > max_val) max_val = affichage[i];
    //     if (calcul[i] > max_val) max_val = calcul[i];
    // }
    // if (max_val < 1.0) max_val = 1.0;

    double max_val = 100.0; // Fixe l'échelle à 100 ms

    // Tracer les axes
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
    SDL_RenderDrawLine(renderer, graph_x, graph_y + graph_h - 1, graph_x + graph_w, graph_y + graph_h - 1);
    SDL_RenderDrawLine(renderer, graph_x, graph_y, graph_x, graph_y + graph_h);

    // Tracer les graduations et valeurs sur l'axe Y
    int nb_graduations = 10;
    for (int i = 0; i <= nb_graduations; i++) {
        int y = graph_y + graph_h - (i * graph_h / nb_graduations);
        SDL_SetRenderDrawColor(renderer, 220, 220, 220, 255);
        SDL_RenderDrawLine(renderer, graph_x, y, graph_x + graph_w, y);

        // Afficher la valeur (en ms) à gauche de la graduation
        char label[16];
        double val = (max_val * i) / nb_graduations;
        sprintf(label, "%.1f ms", val);

        // Affichage du texte (optionnel, nécessite SDL_ttf et une police)
        // Ici, exemple si tu as accès à renderer et font globaux :
        SDL_Color color = {0, 0, 0, 255};
        SDL_Surface* surf = TTF_RenderText_Blended(font, label, color);
        if (surf) {
            SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, surf);
            SDL_Rect rect = {graph_x + 5, y - surf->h / 2, surf->w, surf->h};
            SDL_RenderCopy(renderer, tex, NULL, &rect);
            SDL_FreeSurface(surf);
            SDL_DestroyTexture(tex);
        }
    }

    // Tracer les courbes
    for (int i = 1; i < PERF_HISTORY; i++) {
        int idx0 = (index + i - 1) % PERF_HISTORY;
        int idx1 = (index + i) % PERF_HISTORY;
        int x0 = graph_x + (i - 1) * (graph_w / (PERF_HISTORY - 1));
        int x1 = graph_x + i * (graph_w / (PERF_HISTORY - 1));
        // Total (noir)
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderDrawLine(renderer, x0, graph_y + graph_h - (int)(total[idx0] / max_val * graph_h),
                                   x1, graph_y + graph_h - (int)(total[idx1] / max_val * graph_h));
        // Affichage (bleu)
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        SDL_RenderDrawLine(renderer, x0, graph_y + graph_h - (int)(affichage[idx0] / max_val * graph_h),
                                   x1, graph_y + graph_h - (int)(affichage[idx1] / max_val * graph_h));
        // Calcul (rouge)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderDrawLine(renderer, x0, graph_y + graph_h - (int)(calcul[idx0] / max_val * graph_h),
                                   x1, graph_y + graph_h - (int)(calcul[idx1] / max_val * graph_h));
    }
}

int main(int argc, char *argv[]) {
    //affiche une explication du programme dans la console
    printf("%s", DOC); 

    //Initialise SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("Erreur d'initialisation de SDL : %s\n", SDL_GetError());
        return 1;
    }

    //Initialise module texte SDL
    if (TTF_Init() == -1) {
        printf("Erreur d'initialisation de SDL_ttf : %s\n", TTF_GetError());
        SDL_Quit();
        return 1;
    }

    //Création de la fenêtre SDL
    SDL_Window* window = SDL_CreateWindow("Moteur 2D", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          SCREEN_WIDTH_SDL, SCREEN_HEIGHT_SDL, SDL_WINDOW_SHOWN);
    if (!window) {
        printf("Erreur de création de la fenêtre : %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    //initalise une zone
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("Erreur de création du renderer : %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    //Charge la police d'écriture 
    TTF_Font* font = TTF_OpenFont("D:/_documents/TIPE/police/pixel_arial_14/pixel-arial-14.ttf", 24); // Remplacez par le chemin de votre fichier de police
    if (!font) {
        printf("Erreur de chargement de la police : %s\n", TTF_GetError());
        TTF_Quit();
        SDL_Quit();
        return 1;
    }

    int nb_balles = NB_BALL;
    Ball* liste_balles = initialisation_balles(nb_balles);
    Grid* grid = create_grid(10, 10, FIELD_WIDTH, FIELD_HEIGHT); // 10x10 grille
    double temps = 0; //temps simulé 

    int running = 1; // ON/OFF SDL
    int pause = 0; // pause simulation
    SDL_Event event;

    double rapportx = SCREEN_WIDTH / FIELD_WIDTH;   // 800 / 2.0
    double rapporty = SCREEN_HEIGHT / FIELD_HEIGHT; // 800 / 2.0
    int screen_radius = BALL_RADIUS * rapportx;

    // Initialiser le temps de la dernière frame
    //Uint32 start_time = SDL_GetTicks();
    Uint32 current_time;
    double delta_time;

    Uint64 perf_freq = SDL_GetPerformanceFrequency();
    double perf_total[PERF_HISTORY] = {0};
    double perf_affichage[PERF_HISTORY] = {0};
    double perf_calcul[PERF_HISTORY] = {0};
    int perf_index = 0;

    while (running) {
        Uint64 t0 = SDL_GetPerformanceCounter();

        // Calculer le temps écoulé depuis la dernière frame
        current_time = SDL_GetTicks();
        delta_time = (current_time)/1000.0; // Convertir en secondes;


        // Gérer les événements
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_SPACE) {
                    pause = (pause + 1) % 2; // Toggle pause
                }
            }
        }

        if (pause == 0) {
            Uint64 t1 = SDL_GetPerformanceCounter();
            // Incrémenter le chronomètre interne
            temps += DT;

            // --- Partie calculs ---
            move(liste_balles, nb_balles); //O(nb_balles)
            clear_grid(grid); //O(rows * cols)
            for (int i = 0; i < nb_balles; i++) {
                add_ball_to_grid(grid, &liste_balles[i]);} //O(nb_balles * 1)
            handle_collisions(grid);

            Uint64 t2 = SDL_GetPerformanceCounter();

            // --- Partie affichage ---
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderClear(renderer);
            for (int i = 0; i < nb_balles; i++) {
                int screen_x = (int)(liste_balles[i].x * rapportx);
                int screen_y = (int)(SCREEN_HEIGHT - liste_balles[i].y * rapporty);
                filledCircleRGBA(renderer, screen_x, screen_y, screen_radius, 0, 0, 255, 255);
            }
            SDL_Rect sim_rect = {0, 0, SCREEN_WIDTH, SCREEN_HEIGHT};
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderDrawRect(renderer, &sim_rect);
            chrono(temps, delta_time, renderer, font);

            Uint64 t3 = SDL_GetPerformanceCounter();

            // --- Stockage des performances ---
            perf_calcul[perf_index] = (double)(t2 - t1) / perf_freq * 1000.0;      // ms
            perf_affichage[perf_index] = (double)(t3 - t2) / perf_freq * 1000.0;   // ms
            perf_total[perf_index] = (double)(t3 - t0) / perf_freq * 1000.0;       // ms
            perf_index = (perf_index + 1) % PERF_HISTORY;

            // --- Affichage du graphique ---
            draw_performance_graph(renderer, perf_total, perf_affichage, perf_calcul, perf_index, font);

            SDL_RenderPresent(renderer);

            // Limiter la vitesse de la boucle à 60 FPS
            //SDL_Delay(1000 / FPS);
        }
    }

    free(liste_balles);
    clear_grid(grid);
    free(grid->cells);
    free(grid);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    TTF_CloseFont(font);
    TTF_Quit();
    printf("Fin du programme.\n");

    return 0;
}