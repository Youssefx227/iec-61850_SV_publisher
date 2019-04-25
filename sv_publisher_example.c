
#include <iec61850_server.h>
#include <iec61850_model.h>
#include <sv_publisher.h>
#include <hal_thread.h>
#include <static_model.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <pthread.h>
#include <sched.h>
#define _USE_MATH_DEFINES
#define GNU_SOURCE
#define gettid() syscall(__NR_gettid)

#define TWO_PI_OVER_THREE  2.0943951

#define SCHED_DEADLINE	6

 /* XXX use the proper syscall numbers */
 #ifdef __x86_64__
 #define __NR_sched_setattr	314
 #define __NR_sched_getattr	315
 #endif

static volatile int done;


/* ces champs sont utilisées pour le SCHED_DEADLINE */
 struct sched_attr {
    /* Taille de la structure */
	uint32_t size;
    /* Police (SCHED_*) */
	uint32_t sched_policy;
    /*flags*/
	uint64_t sched_flags;

	/* SCHED_NORMAL, SCHED_BATCH */
	uint32_t sched_nice; /* Valeur du nice (SCHED_OTHER,
SCHED_BATCH) */

/*Priorité statique (SCHED_FIFO,
SCHED_RR) */
	uint32_t sched_priority;

	/* SCHED_DEADLINE (nsec) */
	uint64_t sched_runtime;
	uint64_t sched_deadline;
	uint64_t sched_period;
 };
/*
 * fonction pour affecter les attributs 
 */
 int sched_setattr(pid_t pid,
		  const struct sched_attr *attr,
		  unsigned int flags)
 {
	return syscall(__NR_sched_setattr, pid, attr, flags);
 }
/*
 * fonction pour récupérer les attributs 
 */
 int sched_getattr(pid_t pid,
		  struct sched_attr *attr,
		  unsigned int size,
		  unsigned int flags)
 {
	return syscall(__NR_sched_getattr, pid, attr, size, flags);
 }


/*
 * données partagées entre threads
*/
typedef struct data_{

    char* interface;
    float* f_nominal,* samplesPerCycle,* fech,* f,* w,* phase,* n, * Vamp , * Iamp;
    int*   Va,* Vb,* Vc,* Vn,* ia,* ib,* ic,* in;
    double* theta ;
    pthread_mutex_t mutex;

}data_;

/*
    Fonction timer
*/

void time_add_ns( struct timespec *t,int ns)
{

    t->tv_nsec += ns;

    if(t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }

}

void *publish (void *donnees){

    /* appeler malloc pour demander de la mémoire
    si l'allocation a marché, notre pointeur contient une adresse
    L'allocation dynamique permet notamment de créer un entier/float/tableau dont
    la taille est déterminée par une variable au moment de l'exécution */
    data_* param = malloc(sizeof(data_));
    param = donnees;
    int* Va = malloc(sizeof(int));
     Va = (*param).Va;
    int* ia = malloc(sizeof(int));
     ia = (*param).ia;
     int* Vb = malloc(sizeof(int));
     Vb = (*param).Vb;
    int* ib = malloc(sizeof(int));
     ib = (*param).ib;
    int* Vc = malloc(sizeof(int));
     Vc = (*param).Vc;
    int* ic = malloc(sizeof(int));
     ic = (*param).ic;
    int* Vn = malloc(sizeof(int));
     Vn = (*param).Vn;
    int* in = malloc(sizeof(int));
     in = (*param).in;
    float* n = malloc(sizeof(float));
     n = (*param).n;
    char* interface = malloc(sizeof(char));
     interface =(*param).interface ;

    Quality q = QUALITY_VALIDITY_GOOD;
    unsigned int flags =0;
    struct sched_attr attr;
    struct timespec t;
    struct timeval maintenant;
    struct timeval debut_thread;
    struct timeval debut_programme;
    long int duration;
     int i=0,taille=60;
    long int duree[taille];

    /* Création d'un publisheur avec une interface spécifiée*/
    SVPublisher svPublisher  = SVPublisher_create(NULL,interface); // Crée un nouveau publisher de sampled values selon l'IEC61850-9-2
    /* Ajout d'un bloc de données ASDU dans le publisher*/
    SVPublisher_ASDU asdu1 = SVPublisher_addASDU(svPublisher, "svpub1isher", NULL, 1); 

    /* allocation mémoire dans le bloc de données */
    int Ima = SVPublisher_ASDU_addINT32(asdu1);
    int ampaq = SVPublisher_ASDU_addQuality(asdu1);
    int Imb = SVPublisher_ASDU_addINT32(asdu1);
    int ampbq = SVPublisher_ASDU_addQuality(asdu1);
    int Imc = SVPublisher_ASDU_addINT32(asdu1);
    int ampcq = SVPublisher_ASDU_addQuality(asdu1);
    int Imn = SVPublisher_ASDU_addINT32(asdu1);
    int ampnq = SVPublisher_ASDU_addQuality(asdu1);
    int Vma = SVPublisher_ASDU_addINT32(asdu1);
    int Vmaq = SVPublisher_ASDU_addQuality(asdu1);
    int Vmb = SVPublisher_ASDU_addINT32(asdu1);
    int Vmbq = SVPublisher_ASDU_addQuality(asdu1);
    int Vmc = SVPublisher_ASDU_addINT32(asdu1);
    int Vmcq = SVPublisher_ASDU_addQuality(asdu1);
    int Vmn = SVPublisher_ASDU_addINT32(asdu1);
    int Vmnq = SVPublisher_ASDU_addQuality(asdu1);

    SVPublisher_ASDU_setSmpCntWrap(asdu1, 4000);
    SVPublisher_ASDU_setRefrTm(asdu1, 0);

    SVPublisher_setupComplete(svPublisher);
    /*
    float time_theoric = 999750.0; //us
    float marge = 250.0; //us
    */
        /* ====== Deadline scheduler ====== */
      /* période de la tâche publication de 1s */
    /* paramètres du scheduler deadline */

    attr.size = sizeof(struct sched_attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_flags = 0;
    attr.sched_nice = -20;
    attr.sched_priority = 0;
    attr.sched_policy = SCHED_DEADLINE;
      /* nanosecondes*/
    attr.sched_runtime = 0.8e9;
    attr.sched_deadline = 0.9e9;
    attr.sched_period  = 1.0e9;

    sched_setattr(0, &attr, flags);

    clock_gettime(CLOCK_MONOTONIC, &t);
    // stockage dans un tableau des temps de cycles
    gettimeofday(&debut_programme,NULL);

    while (!done) {  /* Boucle infinie */

        gettimeofday(&debut_thread,NULL);
       
        while(*n<4000){

        /*verouillage du mutex*/
        pthread_mutex_lock(&(param->mutex));

        SVPublisher_ASDU_setSmpSynch(asdu1,2);
        SVPublisher_ASDU_setINT32(asdu1, Ima,*ia);  //écriture dans le bloc de donnée asud1 de la variable courant phase 1
        SVPublisher_ASDU_setQuality(asdu1, ampaq,q);
        SVPublisher_ASDU_setINT32(asdu1, Imb,*ib);
        SVPublisher_ASDU_setQuality(asdu1, ampbq,q);
        SVPublisher_ASDU_setINT32(asdu1, Imc,*ic);
        SVPublisher_ASDU_setQuality(asdu1, ampcq,q);
        SVPublisher_ASDU_setINT32(asdu1, Imn,*in);
        SVPublisher_ASDU_setQuality(asdu1, ampnq,q);
        SVPublisher_ASDU_setINT32(asdu1, Vma,*Va);  //écriture dans le bloc de donnée asud1 de la variable courant phase 1
        SVPublisher_ASDU_setQuality(asdu1, Vmaq,q);
        SVPublisher_ASDU_setINT32(asdu1, Vmb,*Vb);
        SVPublisher_ASDU_setQuality(asdu1, Vmbq,q);
        SVPublisher_ASDU_setINT32(asdu1, Vmc,*Vc);
        SVPublisher_ASDU_setQuality(asdu1, Vmcq,q);
        SVPublisher_ASDU_setINT32(asdu1, Vmn,*Vn);
        SVPublisher_ASDU_setQuality(asdu1, Vmnq,q);
        SVPublisher_ASDU_setRefrTm(asdu1, Hal_getTimeInMs());
	
/*	vérification sur la console des valeurs écrites
        printf("\t sample :  %f ", *n );   
        printf(" Tension Va: %i ", *Va);
        printf(" Tension Vb: %i ", *Vb);
        printf(" Tension Vc: %i ", *Vc);
        printf(" Tension Vn: %i ", *Vn);
        printf(" courant ia: %i ", *ia);
        printf(" courant ib: %i ", *ib);
        printf(" courant ic: %i ", *ic);
        printf(" courant in: %i\n",*in);
*/

        SVPublisher_ASDU_increaseSmpCnt(asdu1);
	/* publication des données */
        SVPublisher_publish(svPublisher);
	/* incrémentation du numéro de la donnée à publier */
        *n+=1;
        /*déverouillage du mutex*/
        pthread_mutex_unlock(&(param->mutex));
        /*reveil de l'horloge */
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME, &t, NULL);
        /* attente le temps d'avoir 250us entre chaque échantillon*/
        time_add_ns(&t,250000);
        }
        /*mesure du temps final*/
        gettimeofday(&maintenant,NULL);
         /*calcul du temps de cycle*/
        duration  = (maintenant.tv_sec*1000000   + maintenant.tv_usec);
        duration -= (debut_thread.tv_sec*1000000 + debut_thread.tv_usec);
        

        /* remplissage dans un tableau des temps d'éxecution de la tâche de publication*/
        duree [i] =  duration;

        if (maintenant.tv_sec - debut_programme.tv_sec > taille){ // supérieur à une durée fixé dans la variable taille
            int j;
            /*création d'un fichier de stockage des valeurs */
            FILE *fichier;
            fichier = fopen("./temps_cycle.csv", "w+");
            /*chemin du fichier dans le docker*/
           //fichier = fopen("/log/temps_cycle.csv", "w+");

             if(fichier != NULL) {
                /*-- affichage après un certains temps des temps de cycle --*/
                for (j=0;j<taille;j++){
                    /*affichage dans le shell*/
                    printf(" temps consommée [%i] = %ld  us\n",j,duree[j]);
                     /* enregistrement en format tableur csv*/
                    fprintf(fichier,"%i",j);
                    fprintf(fichier,"\t%ld\n",duree[j]);
                /* Affichage des différents temps et temps consommes */
                /*if (duree[j]< (time_theoric - marge) || duree[j] > (time_theoric + marge))
                    fprintf(fichier," erreur : non respect de la durée limite \n");
                */
                }
                fclose(fichier);
            }
            break;
        }
         /* réinitialisation --- nouveau cycle */
        *n=0;
        i+=1;
    }
    /* Nettoyage - libéaration des ressources */
    SVPublisher_destroy(svPublisher);
    /* fin du thread */
    pthread_exit(NULL);
}

void *create_signal(void *donnees)
{
    /* allocation mémoire */
    data_* param = malloc(sizeof(data_));
    param  = donnees;
    float* w = malloc(sizeof(float));
    w     = (*param).w;
    float* phase = malloc(sizeof(float));
    phase = (*param).phase;
    float* fech = malloc(sizeof(float));
    fech  = (*param).fech;
    float* n = malloc(sizeof(float));
    n     = (*param).n;
    double* theta = malloc(sizeof(double));
    theta = (*param).theta;
    float* Vamp = malloc(sizeof(float));
    Vamp  = (*param).Vamp;
    float* Iamp = malloc(sizeof(float));
    Iamp  = (*param).Iamp;

    int* Va = malloc(sizeof(int));
    Va = (*param).Va;
    int* ia = malloc(sizeof(int));
    ia = (*param).ia;
    int* Vb = malloc(sizeof(int));
    Vb = (*param).Vb;
    int* ib = malloc(sizeof(int));
    ib = (*param).ib;
    int* Vc = malloc(sizeof(int));
    Vc = (*param).Vc;
    int* ic = malloc(sizeof(int));
    ic = (*param).ic;
    int* Vn = malloc(sizeof(int));
    Vn = (*param).Vn;
    int* in = malloc(sizeof(int));
    in = (*param).in;

    while(1){ /* Boucle infinie */

       /*verrouillage du mutex*/
        pthread_mutex_lock(&(*param).mutex);

        *theta = *w *(double)(*n * 1/(*fech));
        *Va =  *Vamp * sin(*theta)*100;
        *Vb =  *Vamp * sin(*theta - TWO_PI_OVER_THREE)*100;
        *Vc =  *Vamp * sin(*theta + TWO_PI_OVER_THREE)*100;
        *Vn =  *Va + *Vb + *Vc;
        *ia =  *Iamp * sin(*theta - *phase)*1000;
        *ib =  *Iamp * sin(*theta - TWO_PI_OVER_THREE -  *phase)*1000;
        *ic =  *Iamp * sin(*theta + TWO_PI_OVER_THREE -  *phase)*1000;
        *in =  *ia + *ib + *ic;
        /* Deverouillage du mutex */
        pthread_mutex_unlock(&(*param).mutex);
    }
	/* fin du thread */
        pthread_exit(NULL);
}

int
main(int argc, char** argv)
{
    char* Interface;
    float n=0.0,fech=4000.0;
    int Va,Vb,Vc,Vn,ia,ib,ic,in;
    float Vamp =  11000.f*sqrt(2);
    float Iamp =  20.f*sqrt(2);
    double theta = 0.0;
    float f_nominal =50.0,samplesPerCycle=80.0,f=50.0,w= 2*M_PI*f,phase=M_PI/6;

    if (argc > 1)
        Interface = argv[1];
    else
        Interface = "eth0";
    printf("Using Interface : %s \n",Interface);

    /* création d'un bloc de données de thread de types data_ */
    data_ thread_data;

    /* affectation de l'adresse ou doivent pointer les variables de la structure */
    thread_data.fech = &fech;
    thread_data.n  = &n;
    thread_data.Va = &Va;
    thread_data.Vb = &Vb;
    thread_data.Vc = &Vc;
    thread_data.Vn = &Vn;
    thread_data.ia = &ia;
    thread_data.ib = &ib;
    thread_data.ic = &ic;
    thread_data.in = &in;
    thread_data.Vamp = &Vamp;
    thread_data.Iamp = &Iamp;
    thread_data.f_nominal = &f_nominal;
    thread_data.f = &f;
    thread_data.w =&w;
    thread_data.n =&n;
    thread_data.phase=&phase;
    thread_data.samplesPerCycle = &samplesPerCycle;
    thread_data.fech = &fech;
    thread_data.theta = &theta;
    thread_data.interface = Interface;

    /* Initialisation du mutex */
    pthread_mutex_init(&thread_data.mutex,NULL);
    
    /* décalaration des threads */
    pthread_t thread_publish;
    pthread_t thread_create_signal;

    /* création des threads de publication et création de signaux */
    pthread_create(&thread_publish,NULL,publish,&thread_data);
    pthread_create(&thread_create_signal,NULL,create_signal,&thread_data);

    /* le programme principal attend la fin des deux tâches */
    pthread_join(thread_publish,NULL);
    pthread_join(thread_create_signal,NULL);

    return EXIT_SUCCESS;
} /* main() */



