/* ======================================================================= *\
 *  EJERCICIO ENTREGABLE:
 *  En este ejercicio, tomando como punto de partida el ejempolo del EJERCICIO 3, en
 *  el que se muestra cómo estimar la carga computacional de cada hebra, se desarrolla
 *  un esquema de monitorización que permite balancear la carga computacional en el 
 *  microprocesador y llevarla a una condición “óptima”, definida como aquella en la que
 *  la utilización del procesador no supera el 85% (15% del tiempo en la tarea idle/loop) 
 *  y las tareas se ejecutan respetando el periodo establecido, pero con el máximo posible
 *  de intensidad computacional.
 *  
 *  IMPORTANTE: en ChRt/src/rt/templates/chconf.h
 *    - CH_DBG_THREADS_PROFILING debe activarse (TRUE) 
 *    - CH_CFG_NO_IDLE_THREAD debe activarse (TRUE)
 *    
 *  Requiere el uso de la librería ChRt de Bill Greiman
 *    https://github.com/greiman/ChRt
 *    
 *  AUTORES:
 *   - Antonio Aparicio González
 *   - Daniel Díaz Martel
 *
 *  Asignatura (GII-IoT)
\* ======================================================================= */ 
#include <ChRt.h>
#include <math.h>
//#include <../ChibiOS-master/src/ChRt.h>
//------------------------------------------------------------------------------
// Parametrization
//------------------------------------------------------------------------------
#define EXTRA         true    // Cambiar a TRUE para activar el apartado extra (carga aleatoria)
#define USE_DOUBLE    false   // Change to TRUE to use double precision (heavier)

#define CYCLE_MS      1000
#define NUM_THREADS   5  // Three working threads + loadEstimator (top) + 
                         // loop (as the idle thread)
                         // TOP thread is thread with id 0


#define LOAD_THRESHOLD 85.0  // Umbral de trabajo a 85%

#define BALANCED_STATUS LOAD_THRESHOLD / (NUM_THREADS-2)

#define TOTAL_SYSTEM_LOAD 100 - sysLoad.threadLoad[4].loadPerCycle_per // Carga total de trabajo del sistema

char thread_name[NUM_THREADS][15] = { "top", 
                                      "worker_1", "worker_2", "worker_3",
                                      "idle" };

volatile uint32_t threadPeriod_ms[NUM_THREADS] = { CYCLE_MS, 200, 100, 200, 0 };

// Define un valor mínimo y máximo de carga para cada hebra
volatile int threadMinLoad[NUM_THREADS] = {0, 50, 50, 50, 0}; // Por ejemplo, estos valores pueden ser ajustados según las necesidades.
volatile int threadMaxLoad[NUM_THREADS] = {0, 290, 290, 290, 0}; // Establece un valor máximo arbitrario. Se puede ajustar según las necesidades.

// Comenzamos definiendo la carga inicial de las hebras como el mínimo
volatile int threadLoad[NUM_THREADS] = {0, 50, 50, 50, 0};

volatile uint32_t threadEffectivePeriod_ms[NUM_THREADS] = { 0, 0, 0, 0, 0 };
volatile uint32_t threadCycle_ms[NUM_THREADS] = { 0, 0, 0, 0, 0 };


volatile int32_t period_difference[NUM_THREADS] = {0, 0, 0, 0, 0}; // Array que define el estado de conformidad de las hebras con su periodo establecido

// Struct to measure the cpu load using the ticks consumed by each thread
typedef struct {
  thread_t * thd;
  systime_t lastSampleTime_i;
  sysinterval_t lastPeriod_i;
  sysinterval_t ticksTotal;
  sysinterval_t ticksPerCycle;
  float loadPerCycle_per;
} threadLoad_t;


typedef struct {
  threadLoad_t threadLoad[NUM_THREADS];
  uint32_t idling_per;
} systemLoad_t;

systemLoad_t sysLoad;

//------------------------------------------------------------------------------
// Print thread Load And Period Stability
//------------------------------------------------------------------------------
static void printThreadLoad(){
    Serial.print("THREAD_LOAD: ");
    Serial.print("worker_1 = " );
    Serial.print(threadLoad[1]);
    Serial.print((period_difference[1] == 0) ? "S" : (period_difference[1] > 0)? "U+":"U-");
    Serial.print(", worker_2 = " );
    Serial.print(threadLoad[2]);
    Serial.print((period_difference[2] == 0) ? "S" : (period_difference[2] > 0)? "U+":"U-");
    Serial.print(", worker_3 = " );
    Serial.print(threadLoad[3]);
    Serial.println((period_difference[3] == 0) ? "S" : (period_difference[3] > 0)? "U+":"U-");
}

//------------------------------------------------------------------------------
// Print thread stats
//------------------------------------------------------------------------------
static void printThreadStats(uint32_t accumTicks) {
    // Mostramos la carga de cada una de las hebras
    printThreadLoad();
    for (int tid = 1; tid < NUM_THREADS; tid++) {
        threadLoad_t * thdLoad = &sysLoad.threadLoad[tid];
        thdLoad->loadPerCycle_per = (100 * (float)thdLoad->ticksPerCycle) / accumTicks;
        SerialUSB.print(thread_name[tid]);
        SerialUSB.print("  ticks(last cycle): "); SerialUSB.print(thdLoad->ticksPerCycle);
        SerialUSB.print("  CPU(%): "); SerialUSB.print(thdLoad->loadPerCycle_per);
        SerialUSB.print("   Cycle duration(ms): "); SerialUSB.print(threadCycle_ms[tid]);
        SerialUSB.print("  period(ms): "); SerialUSB.println(threadEffectivePeriod_ms[tid]);
    }
    SerialUSB.println();
}


static void checkAndCorrectPeriodDifference() {
  for (int tid = 1; tid < NUM_THREADS-1; tid++) {
    period_difference[tid] = threadEffectivePeriod_ms[tid] - threadPeriod_ms[tid]; // Obtenemos: {-2, 5, -3}  => Aumentamos 1 y 3 | Disminuimos 2 // Obtenemos: {1, -5, 4}   =>  Aumentamos 2     | Disminuimos 1 y 3        
    if (period_difference[tid] != 0){
      threadLoad[tid] = (period_difference[tid] > 0) // Si el periodo es superior al establecido, se disminuye la carga, si no se aumenta
                        ? max(threadLoad[tid] - 1, threadMinLoad[tid])
                        : min(threadLoad[tid] + 1, threadMaxLoad[tid]); 
    }
  }                                                                 
}

static void tryIncreaseAllThreadsLoad() {
  for (int tid = 1; tid < NUM_THREADS - 1; tid++) {
    threadLoad[tid] = (sysLoad.threadLoad[tid].loadPerCycle_per > BALANCED_STATUS) 
                      ? max(threadLoad[tid] - 1, threadMinLoad[tid]) 
                      : min(threadLoad[tid] + (EXTRA ? random(1, 16) : 1), threadMaxLoad[tid]);
  }
}

static void decreaseAllThreadsLoad(){
  for (int tid = 1; tid < NUM_THREADS-1; tid++) 
      threadLoad[tid] = max(threadLoad[tid] - 2, threadMinLoad[tid]);
}
//------------------------------------------------------------------------------
// Load Balancer (for top)
//------------------------------------------------------------------------------
static void balanceLoad() {

  // Determina la estabilidad entre los periodos y realiza las correcciones necesarias
  //checkAndCorrectPeriodDifference();                                           

  // Si la carga total es inferior a la óptima se aumenta
  if (TOTAL_SYSTEM_LOAD < LOAD_THRESHOLD) tryIncreaseAllThreadsLoad();
  else decreaseAllThreadsLoad();
     
}

//------------------------------------------------------------------------------
// Initialize LED
//------------------------------------------------------------------------------
static void initializeLed() {
    bool ledState = LOW;
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, ledState);
}

//------------------------------------------------------------------------------
// Toggle LED state
//------------------------------------------------------------------------------
static bool toggleLedState(bool ledState) {
    ledState = (ledState == HIGH) ? LOW : HIGH;
    digitalWrite(LED_BUILTIN, ledState);
    return ledState;
}

//------------------------------------------------------------------------------
// Collect ticks for each thread
//------------------------------------------------------------------------------
static uint32_t collectThreadTicks(systime_t lastTime_i) {

    uint32_t accumTicks = 0;
    
    for (int tid = 1; tid < NUM_THREADS; tid++) {
        threadLoad_t * thdLoad = &(sysLoad.threadLoad[tid]);
        thdLoad->lastSampleTime_i = lastTime_i;
        systime_t ticks = chThdGetTicksX(thdLoad->thd);
        thdLoad->ticksPerCycle = ticks - thdLoad->ticksTotal;
        thdLoad->ticksTotal = ticks;
        accumTicks += thdLoad->ticksPerCycle;
    }

    return accumTicks;
}

//------------------------------------------------------------------------------
// Load estimator (top)
// High priority thread that executes periodically
//------------------------------------------------------------------------------
BSEMAPHORE_DECL(top_sem, true);
static THD_WORKING_AREA(waTop, 256);

static THD_FUNCTION(top, arg) {
    (void)arg;
    
    initializeLed();

    memset(&sysLoad, 0, sizeof(sysLoad));
    systime_t lastTime_i = 0;
    systime_t period_i = TIME_MS2I(CYCLE_MS);
    chBSemReset(&top_sem, true);

    bool ledState = LOW;

    while (!chThdShouldTerminateX()) {
        // Wait for the specified cycle time
        systime_t deadline_i = lastTime_i + period_i;
        if (deadline_i > chVTGetSystemTimeX()) {
            chBSemWaitTimeout(&top_sem, sysinterval_t(deadline_i - chVTGetSystemTimeX()));
        }
        
        lastTime_i = chVTGetSystemTimeX();

        // Collect and process thread ticks
        uint32_t accumTicks = collectThreadTicks(lastTime_i);

        // En este momento se realiza el balanceo de carga en caso necesario
        balanceLoad();
        
        // Print statistics
        printThreadStats(accumTicks);

        // Toggle LED state
        ledState = toggleLedState(ledState);
    }
}


//------------------------------------------------------------------------------
// Worker thread executes periodically
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waWorker1, 256);
static THD_WORKING_AREA(waWorker2, 256);
static THD_WORKING_AREA(waWorker3, 256);

static THD_FUNCTION(worker, arg) 
{
  int worker_ID = (int)arg;
  sysinterval_t period_i = TIME_MS2I(threadPeriod_ms[worker_ID]);
  systime_t deadline_i = chVTGetSystemTimeX();
  systime_t lastBeginTime_i = 0;
  
  while (!chThdShouldTerminateX()) {
    systime_t beginTime_i = chVTGetSystemTimeX();
    threadEffectivePeriod_ms[worker_ID] = TIME_I2MS(beginTime_i - lastBeginTime_i);
    
    #if USE_DOUBLE
      double num = 10;
    #else
      float num = 10;
    #endif
    
    int niter = threadLoad[worker_ID];
    for (int iter = 0; iter < niter; iter++) {
      #if USE_DOUBLE
        num = exp(num) / (1 + exp(num));
      #else
        num = expf(num) / (1 + expf(num));
      #endif
    }
    
    deadline_i += period_i;
    
    lastBeginTime_i = beginTime_i;
    threadCycle_ms[worker_ID] = TIME_I2MS(chVTGetSystemTimeX() - beginTime_i);
    if (deadline_i > chVTGetSystemTimeX()) {
      chThdSleepUntil(deadline_i);
    }
  }
}

//------------------------------------------------------------------------------
// Continue setup() after chBegin() and create the two threads
//------------------------------------------------------------------------------
void chSetup() 
{
  // Here we assume that CH_CFG_ST_TIMEDELTA is set to zero
  // All SAMD-based boards are only supported in “tick mode”
  
  // Check first if ChibiOS configuration is compatible
  // with a non-cooperative scheme checking the value of CH_CFG_TIME_QUANTUM
  if (CH_CFG_TIME_QUANTUM == 0) {
    SerialUSB.println("You must set CH_CFG_TIME_QUANTUM to a non-zero value in");
    #if defined(__arm__)
        SerialUSB.print("src/<board type>/chconfig<board>.h");
    #elif defined(__AVR__)
        SerialUSB.print("src/avr/chconfig_avr.h"); 
    #endif 
    SerialUSB.println(" to enable round-robin scheduling.");
    while (true) {}
  } 
  SerialUSB.print("CH_CFG_TIME_QUANTUM: ");
  SerialUSB.println(CH_CFG_TIME_QUANTUM);

  // Check we do not spawn the idle thread
  if (CH_CFG_NO_IDLE_THREAD == FALSE) {
    SerialUSB.println("You must set CH_CFG_NO_IDLE_THREAD to TRUE");
  }
  
  // Start top thread
  sysLoad.threadLoad[0].thd = chThdCreateStatic(waTop, sizeof(waTop),
    NORMALPRIO + 2, top, (void *)threadPeriod_ms[0]);

  // Start working threads.
  sysLoad.threadLoad[1].thd = chThdCreateStatic(waWorker1, sizeof(waWorker1),
    NORMALPRIO + 1, worker, (void *)1);
  
  sysLoad.threadLoad[2].thd = chThdCreateStatic(waWorker2, sizeof(waWorker2),
    NORMALPRIO + 1, worker, (void *)2);

  sysLoad.threadLoad[3].thd = chThdCreateStatic(waWorker3, sizeof(waWorker3),
    NORMALPRIO + 1, worker, (void *)3);

  // This thread ID
  sysLoad.threadLoad[4].thd = chThdGetSelfX();
}

//------------------------------------------------------------------------------
// setup() function
//------------------------------------------------------------------------------
void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  SerialUSB.begin(115200);
  while(!SerialUSB) { ; }
  
  SerialUSB.println("Hit any key + ENTER to start ...");
  while(!SerialUSB.available()) { delay(10); }
  
  // Initialize OS and then call chSetup.
  // chBegin() never returns. Loop() is invoked directly from chBegin()
  chBegin(chSetup);
}

//------------------------------------------------------------------------------
// loop() function. It is considered here as the idle thread
//------------------------------------------------------------------------------
void loop() { }
