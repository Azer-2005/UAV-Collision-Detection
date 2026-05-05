# UAV-Collision-Detection
🚁 UAV Collision Detection — Essaim Autonome

Projet Industriel — Programmation Avancée en C
École des Sciences de l'Information | Pr. Tarik HOUICHIME


📌 Description
Système de détection de collision en temps réel pour un essaim de 10 000 micro-drones autonomes.
L'algorithme identifie instantanément les deux drones les plus proches afin de déclencher une manœuvre d'évitement avant tout crash en chaîne.

⚡ Complexité
ApprocheComplexitéTemps estimé (N=10 000)Naïve (double boucle)O(N²)~100 ms ❌ TimeoutNotre solution (Sort + Sweep Line)O(N log N)~0,27 ms ✅
Facteur d'accélération : 376×

🏗️ Architecture

Tri par axe X — qsort() sur la coordonnée x → O(N log N)
Sweep Line — Fenêtre glissante δ éliminant les candidats éloignés → O(N) moyenne


📋 Contraintes Respectées

✅ Structure Drone { int id; float x; float y; float z; }
✅ Allocation unique malloc() sur le tas (160 Ko pour 10 000 drones)
✅ Zéro crochet [] — navigation exclusivement par arithmétique de pointeurs
✅ Complexité O(N log N) prouvée formellement
✅ Libération mémoire sécurisée (free + NULL)


🔧 Compilation & Exécution
bashgcc -O2 -Wall -Wextra -o uav collision_essaim.c -lm
./uav
Exemple de sortie :
=== Système de Détection de Collision — Essaim UAV ===
Nombre de drones : 10000

[1/3] Tri de l'essaim par coordonnée X...
      Tri terminé en 2.341 ms

[2/3] Balayage de la fenêtre glissante (sweep line)...
      Balayage terminé en 0.187 ms

[3/3] RÉSULTAT DE LA DÉTECTION
      Paire critique  : Drone #4821 <-> Drone #7302
      Distance minimale: 0.4731 m

