#define _GNU_SOURCE
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
#include <time.h>
#include <errno.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#define _USE_MATH_DEFINES

#define gettid() syscall(__NR_gettid)

#define TWO_PI_OVER_THREE  2*M_PI/3

#define SCHED_DEADLINE	6

 /*  use the proper syscall numbers */
 #ifdef __x86_64__
 #define __NR_sched_setattr	314
 #define __NR_sched_getattr	315
 #endif

 #ifdef __i386__
 #define __NR_sched_setattr		351
 #define __NR_sched_getattr		352
 #endif

 #ifdef __arm__
 #define __NR_sched_setattr		380
 #define __NR_sched_getattr		381
 #endif


#define SUCCESS_MSG "Successfully set thread %lu to affinity to CPU %d\n"
#define FAILURE_MSG "Failed to set thread %lu to affinity to CPU %d\n"

#define print_error_then_terminate(en, msg) \
  do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)
#define print_perror_then_terminate(msg) \
  do { perror(msg); exit(EXIT_FAILURE); } while (0)

static volatile int done;
long int TableauTimeStamp[4000*21];
 int min,max;
sem_t semaphore;
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
     char* filename;
     char* svid;
    uint16_t* appid;
    float* f_nominal,* samplesPerCycle,* fech,* f,* w,* phase,* n, *Zmag;
    float* Iamp_init,* Vamp_init;
    float* Iamp, *Vamp;
    int*   Va,* Vb,* Vc,* Vn,* ia,* ib,* ic,* in;
    double* theta ;
  // pthread_mutex_t mutex;

} data_;

static uint64_t TimeValToUSeconds(struct timeval* ts)
{
   return(( ts->tv_sec*1000000) + ts->tv_usec)/1000;
}

void min_max (long int* duree,int *min,int *max)
{
	long int val_min,val_max,i;
	val_min = duree[12000]; //3 secondes après le lancement
	val_max = duree[12000];
	int nb = 4000*21;
  printf(" la taille de durée est %i",nb);
	for (i=0; i<nb; i++){

      if (duree[i] < val_min)
      {
        val_min = duree[i];
      }
      else if (duree[i] > val_max)
      {
        val_max = duree[i];
      }
  }
  *min=val_min;
  *max=val_max; 
}

/*
    Fonction ajout délai
*/
void time_add_ns( struct timespec *t,int ns)
{

    t->tv_nsec += ns;

    /* surveille qu'il n' y ait pas de dépassement dans le champs des nanosecondes, si
    on dépasse 1 secondes, le champs nsec est remis à 0 et le champs en seconde est incrémenté */

    if(t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }

}
void *publish (void *donnees){

 const pthread_t pid = pthread_self();
  const int core_id = 6;  // CPU6
  // cpu_set_t: This data set is a bitset where each bit represents a CPU.
  cpu_set_t cpuset;
  // CPU_ZERO: This macro initializes the CPUset to be the empty set.
  CPU_ZERO(&cpuset);
  // CPU_SET: This macro adds cpu to the CPU set set.
  CPU_SET(core_id, &cpuset);

  // pthread_setaffinity_np: The pthread_setaffinity_np() function sets the CPU affinity mask of the thread  to the CPU set pointed to by cpuset. If the call is successful, and the thread is not currently running on one of the CPUs in cpuset, then it is migrated to one of those CPUs.
  const int set_result = pthread_setaffinity_np(pid, sizeof(cpu_set_t), &cpuset);
  if (set_result != 0) {
    print_error_then_terminate(set_result, "pthread_setaffinity");
  }

/*============= Deadline scheduler =============== */
/* période de la tâche publication de 1s */
 /* paramètres du scheduler deadline */
    unsigned int flags =0;
    struct sched_attr attr;
    attr.size = sizeof(struct sched_attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_flags = 0;
    attr.sched_nice = -20;
    attr.sched_priority = 0;
      /* nanosecondes*/
    //attr.sched_runtime = 0.2e9;

      /* nanosecondes*/
    attr.sched_deadline = 0.25e9;
    attr.sched_period  = 0.25e9;
    sched_setattr(gettid(), &attr, flags);

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
     float* w = malloc(sizeof(float));
     w     = (*param).w;
     float* fech = malloc(sizeof(float));
     fech  = (*param).fech;
     double* theta = malloc(sizeof(double));
     theta = (*param).theta;
     float* Vamp = malloc(sizeof(float));
     Vamp  = (*param).Vamp;
     float* Iamp = malloc(sizeof(float));
     Iamp  = (*param).Iamp;
    float* Vamp_init = malloc(sizeof(float));
    Vamp_init  = (*param).Vamp_init;
    float* Iamp_init = malloc(sizeof(float));
    Iamp_init  = (*param).Iamp_init;

    float* phase = malloc(sizeof(float));
    phase = (*param).phase;
    uint16_t* appid = malloc(sizeof(uint16_t));
    appid = (*param).appid;

    char* interface = malloc(sizeof(char));
     interface =(*param).interface ;
    char* svid = malloc(sizeof(char));
     svid =(*param).svid ;
    const char* filename = malloc(sizeof(char));
    filename =(*param).filename ;

    Quality q = QUALITY_VALIDITY_GOOD;

    struct timeval maintenant;
    struct timeval debut_thread;
    struct timeval debut_programme;
    long int duration = 0;
    int i=0,taille=20;
    long int duree[taille];
    long long int A=0;
    int position =0;
    int min,max;
      /* Paramètres communication*/
    CommParameters SVCommParameters;
    SVCommParameters.appId = *appid;
     FILE * fichier_timestamp;
    fichier_timestamp = fopen(filename, "w+r");

    SVCommParameters.dstAddress[0] = 0x01;
    SVCommParameters.dstAddress[1] = 0x0c;
    SVCommParameters.dstAddress[2] = 0xcd;
    SVCommParameters.dstAddress[3] = 0x04;
    SVCommParameters.dstAddress[4] = 0x00;
    SVCommParameters.dstAddress[5] = 0x01;
    SVCommParameters.vlanId = 0;
    SVCommParameters.vlanPriority = 4;

    /* Création d'un publisheur avec une interface spécifiée*/
    SVPublisher svPublisher  = SVPublisher_create(&SVCommParameters,interface); // Crée un nouveau publisher de sampled values selon l'IEC61850-9-2
    /* Ajout d'un bloc de données ASDU dans le publisher*/
    SVPublisher_ASDU asdu1 = SVPublisher_addASDU(svPublisher, svid, NULL, 1);

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
    SVPublisher_ASDU_enableRefrTm(asdu1);
    SVPublisher_ASDU_setSmpCntWrap(asdu1, 4000);
    SVPublisher_ASDU_setRefrTm(asdu1, 0);
    SVPublisher_setupComplete(svPublisher);
    /* Le délai tempo n'est pas en temps absolu. Pour
     * former un temps absolu, il faut obtenir l'heure actuelle avec clock_gettime()
     * puis ajouter notre intervalle de sommeil tempo (250 µs) ** plus bas **
     */
    struct timespec tempo;
    Timestamp ts;
    Timestamp_clearFlags(&ts);
    struct timeval timer_usec,before_sending,after_sending;
    uint64_t timestamp_usec; /* timestamp in microsecond */
    int def=0,slp=0,cmpst=0;
    clock_gettime(CLOCK_MONOTONIC, &tempo);
    gettimeofday(&debut_programme,NULL);
    while (!done) {  /* Boucle infinie */
        gettimeofday(&debut_thread,NULL);
        while(*n<4000){

              gettimeofday(&timer_usec,NULL);
              timestamp_usec = TimeValToUSeconds(&timer_usec);
              /* bloquer tant que la tâche n'a pas le jeton*/
            //  sem_wait(&semaphore);
             *theta = *w *(double)(*n * 1/(*fech));
             *Va =  Vamp[0] * cos(*theta)*100;
             *Vb =  Vamp[1] * cos(*theta - TWO_PI_OVER_THREE)*100 ;
             *Vc =  Vamp[2] * cos(*theta + TWO_PI_OVER_THREE)*100 ;
             *Vn =  *Va + *Vb + *Vc;
             *ia =  Iamp[0] * cos(*theta - *phase)*1000;
             *ib =  Iamp[1] * cos(*theta - TWO_PI_OVER_THREE -  *phase)*1000 ;
             *ic =  Iamp[2] * cos(*theta + TWO_PI_OVER_THREE -  *phase)*1000 ;
             *in =  *ia + *ib + *ic;
              SVPublisher_ASDU_setSmpSynch(asdu1,0);
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
              SVPublisher_ASDU_setRefrTm(asdu1,timestamp_usec);
              SVPublisher_ASDU_increaseSmpCnt(asdu1);
              long long int Ts_us = (long long int) timestamp_usec;
              //printf("Timestamp : %lld\n",timestamp_usec);
              //printf("smpcount %f :",*n);
              //printf("time µs since the last sample : %lld\n",Ts_us-A);
              TableauTimeStamp[position] = (long int) (Ts_us-A);
              position+= 1;
              fprintf(fichier_timestamp,"%lld\n",Ts_us-A);
              A = Ts_us;
              gettimeofday(&before_sending,NULL);
              SVPublisher_publish(svPublisher);
              /*déverouillage du semaphore*/
             // sem_post(&semaphore);
              /* ajout du  temps de sommeil*/
              time_add_ns(&tempo,250000); /* ns */
              /*reveil de l'horloge */
              clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME, &tempo, NULL);

              gettimeofday(&after_sending,NULL);
               /* incrémentation du numéro de la donnée à publier */
              *n+=1;
        }
        /*mesure du temps final*/
        gettimeofday(&maintenant,NULL);
        /*calcul du temps de cycle*/
        duration  = (maintenant.tv_sec*1000000   + maintenant.tv_usec);
        duration -= (debut_thread.tv_sec*1000000 + debut_thread.tv_usec);
        
        /* remplissage dans un tableau des temps d'éxecution de la tâche de publication*/
        duree [i] =  duration;
        /*
        if (maintenant.tv_sec - debut_programme.tv_sec > taille){ // supérieur à une durée fixé dans la variable taille
          int j;
          FILE *fichier;
          fichier = fopen("./temps_cycle.csv", "w+");
        // fichier = fopen("/log/temps_cycle.csv", "w+");
            if(fichier != NULL) {
              //-- affichage après un certains temps des temps de cycle --//
              for (j=0;j<taille;j++){
                 // printf ("\t%ld\n",duree[j]);
                  fprintf(fichier,"%i",j);
                  fprintf(fichier,"\t%ld\n",duree[j]);
              }
           fclose(fichier_timestamp);
           fclose(fichier);
           }
            break;
        */
         /* réinitialisation --- nouveau cycle */
          *n=0;
          i+=1;
    }
    min_max(TableauTimeStamp,&min,&max);
    printf(" \n min : %i\t  max : %i\t",min,max);
    printf("jitter max : %i\n µs",max-250);
    /* Nettoyage - libéaration des ressources */
    SVPublisher_destroy(svPublisher);
    /* fin du thread */
    pthread_exit(NULL);

}


int main(int argc, char** argv) {
    char* Interface;
    char * filename = 0;
    char* svid = 0;
    uint16_t appid;
    float n=0.0,fech=4000.0;
    int Va,Vb,Vc,Vn,ia,ib,ic,in;
    float Vamp_init =  11000.0*sqrt(2);
    float Iamp_init =  10.0*sqrt(2);
    float Vamp [3] = {Vamp_init,Vamp_init,Vamp_init}, Iamp [3] = {Iamp_init,Iamp_init,Iamp_init};
    float *Vamp_p = Vamp;
    float *Iamp_p = Iamp;
    double theta = 0.0;
    float phase =0.0,f_nominal =50.0,samplesPerCycle=80.0,f=50.0,w= 2*M_PI*f;

    if (argc > 1){

      Interface = argv[1];
      appid = strtol(argv[2],NULL,16);
      filename = argv[3];
      svid = argv[4];
    }
    else{
      Interface = "eth0";
      //printf("Using Interface : %s \t",Interface);
     //printf("Using appid %s\n",argv[2]);
    }

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
    thread_data.Vamp = Vamp_p;
    thread_data.Iamp = Iamp_p;
    thread_data.Vamp_init = &Vamp_init;
    thread_data.Iamp_init = &Iamp_init;
    thread_data.f_nominal = &f_nominal;
    thread_data.f = &f;
    thread_data.Vamp_init = &Vamp_init;
    thread_data.Iamp_init = &Iamp_init;
    thread_data.f_nominal = &f_nominal;
    thread_data.f = &f;
    thread_data.w =&w;
    thread_data.n =&n;
    thread_data.phase=&phase;
    thread_data.samplesPerCycle = &samplesPerCycle;
    thread_data.fech = &fech;
    thread_data.theta = &theta;
    thread_data.interface = Interface;
    thread_data.filename = filename;
    thread_data.appid = &appid;
    thread_data.svid = svid;
    /* décalaration des threads */
    pthread_t thread_publish;

    /* création des threads de publication et création de signaux */
    pthread_create(&thread_publish,NULL,publish,&thread_data);
    /* le programme principal attend la fin des deux tâches */
    pthread_join(thread_publish,NULL);

    return 0;
} 
/* main() */
