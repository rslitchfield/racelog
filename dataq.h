#ifndef _def_bool
#define _def_bool
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif

typedef struct DataQData
{
    // admin
    int running;
    pthread_mutex_t lock;

    // from DataQ
    int analog[4];
    int digital;
} DataQData;

void * run_dataq(void *);
DataQData * dataq_data(void);
void stop_dataq(void);
