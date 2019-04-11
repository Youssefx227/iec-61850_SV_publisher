#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(void)
{
   long clk_tck = CLOCKS_PER_SEC;
   clock_t t1, t2;
   int i;
   int a = 0;

   /* Recuperation du temps initial en "clock ticks" */
   t1 = clock();

   /* realisation du calcul */
   for ( i = 0; i < 10000000; i++) {a++;}
   (void)printf("a = %d\n", a);

   /* Recuperation du temps final en "clock ticks" */
   t2 = clock();

   /* Affichage des différents temps et temps consommes */
   (void)printf("Nb ticks/seconde = %ld,  Nb ticks depart : %ld, "
                "Nb ticks final : %ld\n",
                clk_tck, (long)t1, (long)t2);
   (void)printf("Temps consomme (s) : %lf \n",
                (double)(t2-t1)/(double)clk_tck);

   return EXIT_SUCCESS;
}
