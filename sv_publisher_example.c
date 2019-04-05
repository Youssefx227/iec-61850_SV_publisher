
#include <iec61850_server.h>
#include <sv_publisher.h>
#include <hal_thread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/syscall.h>
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
 #define __NR_sched_setattr		314
 #define __NR_sched_getattr	    315
 #endif


 static volatile int done;

/* Remaining fields are for SCHED_DEADLINE */
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

 int sched_setattr(pid_t pid,
		  const struct sched_attr *attr,
		  unsigned int flags)
 {
	return syscall(__NR_sched_setattr, pid, attr, flags);
 }

 int sched_getattr(pid_t pid,
		  struct sched_attr *attr,
		  unsigned int size,
		  unsigned int flags)
 {
	return syscall(__NR_sched_getattr, pid, attr, size, flags);
 }


typedef struct data_{

    char* interface;
    float* f_nominal,* samplesPerCycle,* fech,* f,* w,* Ueff,* Vamp,* Zmag,* Iamp,* phase,* n;
    double* theta,* Va,* Vb,* Vc,* Vn,* ia,* ib,* ic,* in;
    pthread_mutex_t mutex;


}data_;


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
    double* Va = malloc(sizeof(double));
     Va = (*param).Va;
    double* ia = malloc(sizeof(double));
    ia = (*param).ia;
    double* Vb = malloc(sizeof(double));
    Vb = (*param).Vb;
    double* ib = malloc(sizeof(double));
    ib = (*param).ib;
    double* Vc = malloc(sizeof(double));
    Vc = (*param).Vc;
    double* ic = malloc(sizeof(double));
    ic = (*param).ic;
    double* Vn = malloc(sizeof(double));
    Vn = (*param).Vn;
    double* in = malloc(sizeof(double));
    in = (*param).in;
    float* n = malloc(sizeof(float));
    n = (*param).n;
    char* interface = malloc(sizeof(char));
    interface =(*param).interface ;
    Quality q= QUALITY_VALIDITY_GOOD;


    SVPublisher svPublisher  = SVPublisher_create(NULL,interface); // Crée un nouveau publisher de sampled values selon l'IEC61850-9-2
    SVPublisher_ASDU asdu1 = SVPublisher_addASDU(svPublisher, "svpub1isher", NULL,1); // création d'un bloc de donnée qui sera générer

    // allocation mémoire dans le bloc de données
    int Ima = SVPublisher_ASDU_addINT32(asdu1);
    int Imb = SVPublisher_ASDU_addINT32(asdu1);
    int Imc = SVPublisher_ASDU_addINT32(asdu1);
    int Imn = SVPublisher_ASDU_addINT32(asdu1);
    int ampaq = SVPublisher_ASDU_addQuality(asdu1);
    int ampbq = SVPublisher_ASDU_addQuality(asdu1);
    int ampcq = SVPublisher_ASDU_addQuality(asdu1);
    int ampnq = SVPublisher_ASDU_addQuality(asdu1);
    int Vma = SVPublisher_ASDU_addINT32(asdu1);
    int Vmb = SVPublisher_ASDU_addINT32(asdu1);
    int Vmc = SVPublisher_ASDU_addINT32(asdu1);
    int Vmn = SVPublisher_ASDU_addINT32(asdu1);
    int Vmaq = SVPublisher_ASDU_addQuality(asdu1);
    int Vmbq = SVPublisher_ASDU_addQuality(asdu1);
    int Vmcq = SVPublisher_ASDU_addQuality(asdu1);
    int Vmnq = SVPublisher_ASDU_addQuality(asdu1);
    int ts1 = SVPublisher_ASDU_addTimestamp(asdu1);

    SVPublisher_ASDU_setSmpCntWrap(asdu1, 4000);
    SVPublisher_ASDU_setRefrTm(asdu1, 0);

    SVPublisher_setupComplete(svPublisher);

    clock_t tinit,tfinal;

    double time_used=0.0;
    double time_theoric = 0.999750;
    double marge = 250.0e-6;
    /* ====== Deadline scheduler ====== */
  //  int runtime = 30000000;
  //  int deadline = 1e9;
    /*---- période de la tâche publication de 1s --- */
//    int period = 1e9;
    int ret;
    unsigned int flags =0;
    struct sched_attr attr;

//  affectation des attributs
    printf("deadline thread started [%ld]\n", gettid());

	attr.size = sizeof(struct sched_attr);
	attr.sched_policy = SCHED_DEADLINE;
	attr.sched_flags = 0;
	attr.sched_nice = 0;
	attr.sched_priority = 0;
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = 0.9e9;
	attr.sched_period = attr.sched_deadline = 1.0e9;
	ret = sched_setattr(0, &attr, flags);

		if (ret < 0) {
			done = 0;
			perror("sched_setattr");
			exit(-1);
		}
    /* period in milliseconds */
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);

    while (!done) {  /* Boucle infinie */

        tinit = clock();
        while(*n<4000){

        /*verouillage du mutex*/
        pthread_mutex_lock(&(param->mutex));
        Timestamp ts;
        Timestamp_clearFlags(&ts);
        Timestamp_setTimeInMilliseconds(&ts, Hal_getTimeInMs());

        SVPublisher_ASDU_setFLOAT64(asdu1,Ima, *ia);  //écriture dans le bloc de donnée asud1 de la variable courant phase 1
        SVPublisher_ASDU_setQuality(asdu1,ampaq,q);
        SVPublisher_ASDU_setFLOAT64(asdu1, Imb,*ib);
        SVPublisher_ASDU_setQuality(asdu1, ampbq,q);
        SVPublisher_ASDU_setFLOAT64(asdu1, Imc,*ic);
        SVPublisher_ASDU_setQuality(asdu1, ampcq,q);
        SVPublisher_ASDU_setFLOAT64(asdu1, Imn,*in);
        SVPublisher_ASDU_setQuality(asdu1, ampnq,q);
        SVPublisher_ASDU_setFLOAT64(asdu1, Vma,*Va);  //écriture dans le bloc de donnée asud1 de la variable courant phase 1
        SVPublisher_ASDU_setQuality(asdu1, Vmaq,q);
        SVPublisher_ASDU_setFLOAT64(asdu1, Vmb,*Vb);
        SVPublisher_ASDU_setQuality(asdu1, Vmbq,q);
        SVPublisher_ASDU_setFLOAT64(asdu1, Vmc,*Vc);
        SVPublisher_ASDU_setQuality(asdu1, Vmcq,q);
        SVPublisher_ASDU_setFLOAT64(asdu1, Vmn,*Vn);
        SVPublisher_ASDU_setQuality(asdu1, Vmnq,q);
        SVPublisher_ASDU_setTimestamp(asdu1,ts1,ts);

        SVPublisher_ASDU_setRefrTm(asdu1, Hal_getTimeInMs());
        SVPublisher_ASDU_increaseSmpCnt(asdu1);

        SVPublisher_publish(svPublisher);
/*
        printf("\t sample :  %.2f ", *n );    // vérification sur le terminal des valeurs écrites
        printf(" Tension Va: %.2f ", *Va);
        printf(" Tension Vb: %.2f ", *Vb);
        printf(" Tension Vc: %.2f ", *Vc);
        printf(" Tension Vn: %.2f ", *Vn);
        printf(" courant ia: %.2f ", *ia);
        printf(" courant ib: %.2f ", *ib);
        printf(" courant ic: %.2f ", *ic);
        printf(" courant in: %.2f\n",*in);
*/
        *n+=1; // j'incrémente la valeur de mon échantillon
        /*déverouillage du mutex*/
        pthread_mutex_unlock(&(param->mutex));
        /*reveil de l'horloge */
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME, &t, NULL);
        /* attente le temps d'avoir 250us entre chaque échantillon*/
        time_add_ns(&t,250000);
        }

        /*mesure du temps final*/
        tfinal = clock();
        time_used = (double)(tfinal-tinit)/(double) CLOCKS_PER_SEC ;
        /*calcul du temps de cycle*/
        /* Affichage des différents temps et temps consommes */
        printf("Nb ticks/seconde = %ld,  Nb ticks depart : %ld, " "Nb ticks final : %ld\n",CLOCKS_PER_SEC, (long)tinit, (long)tfinal);
        printf("Temps consommé (s) : %lf \n",time_used);

       if (time_used < time_theoric - marge || time_used > time_theoric + marge)
          printf(" erreur : non respect de la durée limite \n");

        /* réinitialisation --- nouveau cycle */
        *n=0;
        SVPublisher_ASDU_setSmpCnt(asdu1,0);
    }

    SVPublisher_destroy(svPublisher); //destruction publisher

    pthread_exit(NULL);
}

void *create_signal(void *donnees)
{
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
    double* Va = malloc(sizeof(double));
    Va = (*param).Va;
    double* ia = malloc(sizeof(double));
    ia = (*param).ia;
    double* Vb = malloc(sizeof(double));
    Vb = (*param).Vb;
    double* ib = malloc(sizeof(double));
    ib = (*param).ib;
    double* Vc = malloc(sizeof(double));
    Vc = (*param).Vc;
    double* ic = malloc(sizeof(double));
    ic = (*param).ic;
    double* Vn = malloc(sizeof(double));
    Vn = (*param).Vn;
    double* in = malloc(sizeof(double));
    in = (*param).in;

    while(1){ /* Boucle infinie */

       /*verrouillage du mutex*/
       pthread_mutex_lock(&(*param).mutex);
        *theta = *w * (((double) *n) * 1/(*fech));
        *Va =  *Vamp * sin(*theta);
        *Vb =  *Vamp * sin(*theta - TWO_PI_OVER_THREE);
        *Vc =  *Vamp * sin(*theta + TWO_PI_OVER_THREE);
        *Vn =  *Va + *Vb + *Vc;
        *ia =  *Iamp * sin(*theta - *phase);
        *ib =  *Iamp * sin(*theta - TWO_PI_OVER_THREE -  *phase);
        *ic =  *Iamp * sin(*theta + TWO_PI_OVER_THREE -  *phase);
        *in =  *ia + *ib + *ic;
        /* Deverouillage du mutex */
        pthread_mutex_unlock(&(*param).mutex);

    }
        pthread_exit(NULL);
}


int
main(int argc, char** argv)
{
    char* Interface;
    float n = 0.0;
    float fech=4000.0;
    double Va,Vb,Vc,Vn,ia,ib,ic,in,theta;
    float f_nominal =50.0,samplesPerCycle=80.0,f=50.0,w= 2*M_PI*f,Vamp =11000.0,Zmag = 80.0,phase=0.0;
    float Iamp =Vamp/Zmag;

    if (argc > 1)
        Interface = argv[1];
    else
        Interface = "eth0";
    printf("Using Interface : %s \n",Interface);
    // création d'un bloc de données de thread de types data_
    data_ thread_data;

    // affectation de l'adresse ou doivent pointer les variables de la structure
    thread_data.fech = &fech;
    thread_data.n = &n;
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
    thread_data.in = &in;
    thread_data.theta = &theta;
    thread_data.interface = Interface;

    //intalisation du mutex
    pthread_mutex_init(&thread_data.mutex,NULL);

    // déclaration des threads
    pthread_t thread_publish;
    pthread_t thread_create_signal;

    // priorité thread

    pthread_attr_t attr1,attr2; 		//attributs pour gérer les priorités
	struct sched_param sched1,sched2;	//gestionnaire de priorité
    sched1.sched_priority = 99;
    sched2.sched_priority = 1;

	pthread_attr_init(&attr1); 		//initialise les attributs par défauts
	pthread_attr_init(&attr2);

	sched_setscheduler(0, SCHED_FIFO, &sched1); //on instancie les gestionnaires
	sched_setscheduler(0, SCHED_FIFO, &sched2);

	pthread_attr_setschedparam(&attr1,&sched1); //on initialise les attributs en fonction des gestionnaires
	pthread_attr_setschedparam(&attr2,&sched2);


    // création des threads de publication et création de signaux;
    pthread_create(&thread_publish,&attr1,publish,&thread_data);
    pthread_create(&thread_create_signal,&attr2,create_signal,&thread_data);

    // le programme principal attend la fin des deux tâches
    pthread_join(thread_publish,NULL);
    pthread_join(thread_create_signal,NULL);

    return 0;
} /* main() */








