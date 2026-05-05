/*
 * =============================================================================
 * Système de Détection de Collision pour Essaim Autonome (UAV)
 * École des Sciences de l'Information — Programmation Avancée en C
 * Pr. Tarik HOUICHIME
 * =============================================================================
 *
 * DESCRIPTION :
 *   Détection en temps réel des deux drones les plus proches dans un essaim
 *   de N=10 000 micro-drones autonomes, en O(N log N) via tri par axe + sweep.
 *
 * CONTRAINTES RESPECTÉES :
 *   - Allocation dynamique unique (malloc) sur le tas
 *   - Aucune indexation par crochets [] : navigation par arithmétique de pointeurs
 *   - Structure hétérogène Drone {int id; float x; float y; float z;}
 * =============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* ============================================================
 * 1. DÉFINITION DE LA STRUCTURE DRONE
 * ============================================================ */

typedef struct {
    int   id;   /* Identifiant unique du drone              */
    float x;    /* Coordonnée spatiale X (est/ouest)  en m  */
    float y;    /* Coordonnée spatiale Y (nord/sud)   en m  */
    float z;    /* Coordonnée spatiale Z (altitude)   en m  */
} Drone;

/* ============================================================
 * 2. CALCUL DE LA DISTANCE EUCLIDIENNE (distance au carré)
 *    On évite sqrt() pour économiser les cycles CPU.
 *    La comparaison de d² suffit pour trouver le minimum.
 * ============================================================ */

/*
 * distance_carree : retourne le carré de la distance entre deux drones.
 * Entrées : deux pointeurs constants vers des Drone
 * Sortie  : float (d²)
 */
static inline float distance_carree(const Drone *a, const Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return dx*dx + dy*dy + dz*dz;
}

/* ============================================================
 * 3. COMPARATEUR POUR qsort — TRI PAR AXE X
 *    qsort() attend int comparator(const void*, const void*)
 * ============================================================ */

/*
 * comparer_par_x : comparateur de deux Drone par leur coordonnée x.
 * Utilisé par qsort pour trier l'essaim selon l'axe X.
 */
static int comparer_par_x(const void *pa, const void *pb) {
    const Drone *a = (const Drone *)pa;
    const Drone *b = (const Drone *)pb;
    if (a->x < b->x) return -1;
    if (a->x > b->x) return  1;
    return 0;
}

/* ============================================================
 * 4. ALGORITHME PRINCIPAL : SWEEP LINE SUR L'AXE X
 *
 *    Stratégie (O(N log N)) :
 *      a) Trier les N drones par x          → O(N log N)
 *      b) Balayer avec une fenêtre glissante → O(N · k) ≈ O(N)
 *         où k est le nombre moyen de drones dans la fenêtre δ
 *
 *    Pour chaque drone i, seuls les drones j tels que
 *    (x_j - x_i) < δ_courant (meilleure distance connue)
 *    peuvent être plus proches. Les autres sont éliminés.
 * ============================================================ */

/*
 * trouver_paire_minimale :
 *   Cherche les deux drones les plus proches dans l'essaim trié par x.
 *
 *   Paramètres :
 *     essaim   : pointeur vers le début du tableau de drones (déjà trié par x)
 *     n        : nombre de drones
 *     id_a     : sortie — id du premier drone de la paire minimale
 *     id_b     : sortie — id du second  drone de la paire minimale
 *
 *   Retour : distance euclidienne minimale (float)
 */
float trouver_paire_minimale(const Drone *essaim, int n,
                             int *id_a, int *id_b) {
    float dist_min_carree = FLT_MAX; /* Plus petit carré de distance trouvé  */
    int   best_i = 0, best_j = 1;   /* Indices de la paire optimale courante */

    /* Pointeur de balayage sur le drone gauche de la fenêtre */
    int gauche = 0;

    /* --- Boucle principale : i parcourt chaque drone de référence --- */
    for (int i = 0; i < n - 1; i++) {

        /* Accès au drone i via arithmétique de pointeurs (sans crochets) */
        const Drone *drone_i = essaim + i;

        /*
         * Avancer la borne gauche de la fenêtre :
         * Tout drone j tel que (x_i - x_j)² > dist_min_carree
         * ne peut pas former la paire minimale avec drone_i,
         * ni avec aucun drone plus à droite.
         */
        while (gauche < i) {
            const Drone *drone_gauche = essaim + gauche;
            float dx = drone_i->x - drone_gauche->x;
            if (dx * dx < dist_min_carree) {
                break; /* drone_gauche est encore dans la fenêtre utile */
            }
            gauche++; /* Expulser drone_gauche de la fenêtre */
        }

        /*
         * Comparer drone_i avec tous les drones dans la fenêtre [gauche, i-1]
         * et continuer vers j > i tant que (x_j - x_i)² < dist_min_carree
         */
        for (int j = gauche; j < n; j++) {
            if (j == i) continue; /* Ne pas comparer un drone avec lui-même */

            const Drone *drone_j = essaim + j;

            /* Élagage précoce sur l'axe X (critère de sortie de fenêtre) */
            float dx = drone_j->x - drone_i->x;
            if (j > i && dx * dx >= dist_min_carree) {
                break; /* Tous les drones suivants sont hors fenêtre */
            }

            /* Calcul de la distance au carré */
            float d2 = distance_carree(drone_i, drone_j);

            /* Mise à jour du minimum */
            if (d2 < dist_min_carree) {
                dist_min_carree = d2;
                best_i = i;
                best_j = j;
            }
        }
    }

    /* Restitution des identifiants de la paire optimale */
    *id_a = (essaim + best_i)->id;
    *id_b = (essaim + best_j)->id;

    return sqrtf(dist_min_carree); /* Distance euclidienne réelle */
}

/* ============================================================
 * 5. GÉNÉRATION DES DONNÉES DE SIMULATION
 *    Remplace le flux radar pour les tests sur banc.
 * ============================================================ */

/*
 * generer_essaim :
 *   Alloue et initialise N drones avec des coordonnées aléatoires.
 *
 *   Paramètres :
 *     n       : nombre de drones à générer
 *     espace  : demi-étendue du cube de simulation (en mètres)
 *
 *   Retour : pointeur vers le premier Drone du tableau alloué,
 *            ou NULL en cas d'échec d'allocation.
 *
 *   Responsabilité mémoire : l'appelant doit libérer avec free().
 */
Drone *generer_essaim(int n, float espace) {
    /* Allocation d'un bloc contigu pour N drones */
    Drone *essaim = (Drone *)malloc((size_t)n * sizeof(Drone));
    if (!essaim) {
        fprintf(stderr, "[ERREUR] malloc : échec d'allocation pour %d drones.\n", n);
        return NULL;
    }

    /* Initialisation via arithmétique de pointeurs — crochets interdits */
    Drone *ptr = essaim;
    for (int i = 0; i < n; i++, ptr++) {
        ptr->id = i;
        /* Position aléatoire dans [-espace, +espace] */
        ptr->x = espace * (2.0f * ((float)rand() / RAND_MAX) - 1.0f);
        ptr->y = espace * (2.0f * ((float)rand() / RAND_MAX) - 1.0f);
        ptr->z = espace * (2.0f * ((float)rand() / RAND_MAX) - 1.0f);
    }

    return essaim;
}

/* ============================================================
 * 6. POINT D'ENTRÉE PRINCIPAL
 * ============================================================ */

int main(void) {
    const int   N      = 10000;   /* Taille de l'essaim             */
    const float ESPACE = 5000.0f; /* Cube de vol : ±5 000 m         */

    srand((unsigned)time(NULL));

    printf("=== Système de Détection de Collision — Essaim UAV ===\n");
    printf("Nombre de drones : %d\n\n", N);

    /* --- 6.1 Génération de l'essaim --- */
    Drone *essaim = generer_essaim(N, ESPACE);
    if (!essaim) return EXIT_FAILURE;

    /* --- 6.2 Tri par axe X en O(N log N) --- */
    printf("[1/3] Tri de l'essaim par coordonnée X...\n");
    clock_t t0 = clock();
    qsort(essaim, (size_t)N, sizeof(Drone), comparer_par_x);
    clock_t t1 = clock();
    printf("      Tri terminé en %.3f ms\n\n",
           1000.0 * (double)(t1 - t0) / CLOCKS_PER_SEC);

    /* --- 6.3 Recherche de la paire minimale par sweep line --- */
    printf("[2/3] Balayage de la fenêtre glissante (sweep line)...\n");
    int id_a = -1, id_b = -1;
    clock_t t2 = clock();
    float dist = trouver_paire_minimale(essaim, N, &id_a, &id_b);
    clock_t t3 = clock();
    printf("      Balayage terminé en %.3f ms\n\n",
           1000.0 * (double)(t3 - t2) / CLOCKS_PER_SEC);

    /* --- 6.4 Résultat & déclenchement de la manœuvre --- */
    printf("[3/3] RÉSULTAT DE LA DÉTECTION\n");
    printf("      Paire critique  : Drone #%d <-> Drone #%d\n", id_a, id_b);
    printf("      Distance minimale: %.4f m\n", dist);

    /* Récupération des positions via arithmétique de pointeurs */
    Drone *ptr = essaim;
    Drone *drone_a = NULL, *drone_b = NULL;
    for (int i = 0; i < N; i++, ptr++) {
        if (ptr->id == id_a) drone_a = ptr;
        if (ptr->id == id_b) drone_b = ptr;
    }
    if (drone_a && drone_b) {
        printf("\n      Drone #%d : (%.2f, %.2f, %.2f)\n",
               drone_a->id, drone_a->x, drone_a->y, drone_a->z);
        printf("      Drone #%d : (%.2f, %.2f, %.2f)\n",
               drone_b->id, drone_b->x, drone_b->y, drone_b->z);
    }

    printf("\n>>> ALERTE : Manœuvre d'évitement déclenchée !\n");

    /* --- 6.5 Libération de la mémoire --- */
    free(essaim);
    essaim = NULL;

    return EXIT_SUCCESS;
}

/*
 * Compilation recommandée :
 *   gcc -O2 -Wall -Wextra -o collision_essaim collision_essaim.c -lm
 *
 * Exécution :
 *   ./collision_essaim
 */
