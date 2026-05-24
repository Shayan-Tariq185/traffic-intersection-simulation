#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/wait.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#define defaultVehicles 15
#define maximumVehicles 100
#define totalParkingSpots 10
#define waitingQueueSize 5
#define emergencyMsgF10 "Emergency_from_F10"
#define emergencyMsgF11 "Emergency_from_F11"
#define shutdownMsg "Shutdown"
#define msgLength 32
#define maximumLog 14

#define WIN_W 1600
#define WIN_H 900
#define ROAD_Y 285.0f
#define ROAD_H 90.0f
#define ROAD_TOP 240.0f
#define ROAD_BOT 330.0f
#define LANE_A_Y 265.0f  
#define LANE_B_Y 305.0f  
#define F10_CX 380.0f
#define F10_CY 285.0f
#define F11_CX 920.0f
#define F11_CY 285.0f
#define INT_R 55.0f  
#define LOT_F10_X 120.0f
#define LOT_F10_Y 355.0f
#define LOT_F11_X 980.0f
#define LOT_F11_Y 355.0f
#define LOT_W 200.0f
#define LOT_H 270.0f
#define SPOT_W 76.0f
#define SPOT_H 38.0f
#define DASH_X 1205.0f
#define LOG_Y 728.0f
#define HEADER_H 36.0f
#define VEHICLE_SPEED 120.0f  

typedef enum
{
    AMBULANCE,
    FIRETRUCK,
    BUS,
    CAR,
    BIKE,
    TRACTOR
} VehicleType;

typedef enum
{
    F10,
    F11
} Intersection;

typedef enum
{
    STRAIGHT,
    LEFT,
    RIGHT
} Direction;

typedef enum
{
    PRIORITY_NORMAL=1,
    PRIORITY_MEDIUM=2,
    PRIORITY_HIGH=3
} Priority;

typedef enum
{
    VRS_INACTIVE,
    VRS_APPROACHING,
    VRS_PARK_QUEUE,
    VRS_PARKING,
    VRS_EMERG_WAIT,
    VRS_CROSSING,
    VRS_TRAVELING,
    VRS_DONE
} VehicleRenderState;

typedef enum
{
    SCREEN_HOME,
    SCREEN_SIM
} ScreenState;

typedef struct
{
    Intersection id;
    sem_t parkingSpots;
    sem_t waiting_slots;
    int spotOccupied[totalParkingSpots];
} ParkingLot;

typedef struct
{
    int id;
    VehicleType type;
    Intersection origin;
    Intersection destination;
    Direction direction;
    Priority priority;
    int wantsToPark;
    int arrivalTime;
} Vehicle;

typedef struct
{
    int active;
    int id;
    VehicleType type;
    Intersection origin;
    Priority priority;
    float rx, ry;    
    float tx, ty;    
    VehicleRenderState vrs;
    int flash;    
    int park_spot;
    char status[80];
} VRI;

typedef struct
{
    char msg[128];
    Color col;
} LogEntry;

pthread_mutex_t mutex_F10 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_F11 = PTHREAD_MUTEX_INITIALIZER;

volatile int emergencyActiveF10 = 0;
volatile int emergencyActiveF11 = 0;
volatile sig_atomic_t stopFlag = 0;

ParkingLot lotF10, lotF11;
int pipeF10ToF11[2];
int pipeF11ToF10[2];
pid_t pid_F10_ctrl = -1;
pid_t pid_F11_ctrl = -1;
pthread_t vehicleThreads[maximumVehicles];
Vehicle vehicles[maximumVehicles];
int totalVehicles = defaultVehicles;

pthread_mutex_t g_render_mutex;
VRI g_vri[maximumVehicles];
LogEntry g_log[maximumLog];
int g_logCount = 0;
int g_vehiclesDone = 0;
int g_emergencyCount = 0;
int g_parkingCount = 0;
int g_pipeMsgs = 0;

int sigF10 = 0;  
int sigF11 = 2;

ScreenState currentScreen = SCREEN_HOME;
double simStartTime = 0.0;
int numCreated = 0;
int simInitDone = 0;  

// Returns the string representation of a VehicleType
const char *getVehicleTypeName(VehicleType t)
{
    switch(t)
    {
        case AMBULANCE: 
            return "Ambulance";
        case FIRETRUCK: 
            return "Firetruck";
        case BUS:       
            return "Bus";
        case CAR:       
            return "Car";
        case BIKE:      
            return "Bike";
        case TRACTOR:   
            return "Tractor";
    }
    return "Unknown";
}

// Returns the string representation of an Intersection
const char *getIntersectionName(Intersection i)
{
    if (i == F10)
    {
        return "F10";
    }
    return "F11";
}

// Returns the string representation of a Direction
const char *getDirectionName(Direction d)
{
    switch(d)
    {
        case STRAIGHT: 
            return "Straight";
        case LEFT:     
            return "Left";
        case RIGHT:    
            return "Right";
    }
    return "Unknown";
}

// Returns the string representation of a Priority
const char *getPriorityName(Priority p)
{
    switch(p)
    {
        case PRIORITY_NORMAL: 
            return "Normal";
        case PRIORITY_MEDIUM: 
            return "Medium";
        case PRIORITY_HIGH:   
            return "HIGH (Emergency)";
    }
    return "Unknown";
}

// Returns a random VehicleType
VehicleType randomVehicleType(void)  
{
    return (VehicleType)(rand() % 6);
}

// Returns a random Intersection
Intersection randomIntersection(void)
{
    return (Intersection)(rand() % 2);
}

// Returns a random Direction
Direction randomDirection(void)    
{
    return (Direction)(rand() % 3);
}

// Returns the Priority level based on VehicleType
Priority getPriority(VehicleType t)
{
    if (t == AMBULANCE || t == FIRETRUCK)
    {
        return PRIORITY_HIGH;
    }
    if (t == BUS)
    {
        return PRIORITY_MEDIUM;
    }
    return PRIORITY_NORMAL;
}

// Returns the rendering color associated with a VehicleType
Color getVehicleColor(VehicleType t)
{
    switch(t)
    {
        case AMBULANCE: 
            return (Color){220, 30,  60, 255};
        case FIRETRUCK: 
            return (Color){255, 90,   0, 255};
        case BUS:       
            return (Color){ 30,150, 255, 255};
        case CAR:       
            return (Color){ 60,210,  80, 255};
        case BIKE:      
            return (Color){255,215,   0, 255};
        case TRACTOR:   
            return (Color){170, 90,  40, 255};
    }
    return WHITE;
}

// Calculates coordinate waypoints based on the vehicle's origin
void getPositions(Intersection origin, float *sx, float *sy, float *apx, float *apy, float *crx, float *cry, float *trx, float *try_, float *ex, float *ey)
{
    if (origin == F10)
    {
        *sx = -(70);
        *sy = LANE_A_Y;
        *apx = F10_CX - INT_R - 35;
        *apy = LANE_A_Y;
        *crx = F10_CX + INT_R + 20;
        *cry = LANE_A_Y;
        *trx = F11_CX - INT_R - 35;
        *try_ = LANE_A_Y;
        *ex = (float)(WIN_W + 70);
        *ey = LANE_A_Y;
    }
    else
    {
        *sx = (float)(WIN_W + 70);
        *sy = LANE_B_Y;
        *apx = F11_CX + INT_R + 35;
        *apy = LANE_B_Y;
        *crx = F11_CX - INT_R - 20;
        *cry = LANE_B_Y;
        *trx = F10_CX + INT_R + 35;
        *try_ = LANE_B_Y;
        *ex = -(70);
        *ey = LANE_B_Y;
    }
}

// Calculates the pixel coordinates for a specific parking spot index
void getParkSpotPos(Intersection lotId, int spotIdx, float *px, float *py)
{
    float lx = (lotId == F10) ? LOT_F10_X : LOT_F11_X;
    float ly = (lotId == F10) ? LOT_F10_Y : LOT_F11_Y;
    int col = spotIdx % 2;
    int row = spotIdx / 2;
    *px = lx + 10.0f + col * (SPOT_W + 9.0f) + SPOT_W / 2.0f;
    *py = ly + 28.0f + row * (SPOT_H + 8.0f) + SPOT_H / 2.0f;
}

// Sleeps for a given number of seconds but checks the stop flag periodically
void interruptible_sleep(int seconds)
{
    for (int i = 0; i < seconds * 5 && !stopFlag; i++)
    {
        usleep(200000);
    }
}

// Adds an event string and color to the global log buffer
void addEvent(const char *msg, Color col)
{
    pthread_mutex_lock(&g_render_mutex);
    if (g_logCount < maximumLog)
    {
        g_logCount++;
    }
    for (int i = maximumLog - 1; i > 0; i--)
    {
        g_log[i] = g_log[i - 1];
    }
    strncpy(g_log[0].msg, msg, 127);
    g_log[0].msg[127] = '\0';
    g_log[0].col = col;
    pthread_mutex_unlock(&g_render_mutex);
   
    printf("Event: %s\n", msg);
    fflush(stdout);
}

// Writes a message string to the specified pipe file descriptor
void sendMessage(int write_fd, const char *msg)
{
    char buf[msgLength];
    memset(buf, 0, msgLength);
    strncpy(buf, msg, msgLength - 1);
    if (write(write_fd, buf, msgLength) == -1)
    {
        if (errno != EPIPE && errno != EBADF)
        {
            perror("sendMessage write failed");
        }
    }
}

// Process function for the traffic controllers running in separate processes
void controllerProcess(Intersection ctrl_id, int read_fd, int write_fd)
{
    const char *name = getIntersectionName(ctrl_id);
    printf("%s Controller Process (PID: %d) started and listening for messages.\n", name, getpid());
    fflush(stdout);
   
    char buf[msgLength];
    while (1)
    {
        memset(buf, 0, msgLength);
        ssize_t n = read(read_fd, buf, msgLength);
        if (n <= 0)
        {
            break;
        }
        if (strcmp(buf, shutdownMsg) == 0)
        {
            printf("%s Controller received shutdown message. Exiting safely.\n", name);
            fflush(stdout);
            break;
        }
        if (strcmp(buf, emergencyMsgF10) == 0)
        {
            printf("%s Controller: Emergency vehicle approaching from F10! Clearing route.\n", name);
            fflush(stdout);
        }
        else if (strcmp(buf, emergencyMsgF11) == 0)
        {
            printf("%s Controller: Emergency vehicle approaching from F11! Clearing route.\n", name);
            fflush(stdout);
        }
    }
    close(read_fd);
    close(write_fd);
    exit(EXIT_SUCCESS);
}

// Handles the synchronization logic for a vehicle attempting to park
void attemptParking(Vehicle *v, ParkingLot *lot, int vid)
{
    if (v->priority == PRIORITY_HIGH || !v->wantsToPark)
    {
        return;
    }

    char msg[128];
    snprintf(msg, sizeof(msg), "V%d (%s) wants to park at %s lot", v->id, getVehicleTypeName(v->type), getIntersectionName(lot->id));
    addEvent(msg, getVehicleColor(v->type));

    pthread_mutex_lock(&g_render_mutex);
    g_vri[vid].vrs = VRS_PARK_QUEUE;
    snprintf(g_vri[vid].status, 80, "Seeking park at %s", getIntersectionName(lot->id));
    pthread_mutex_unlock(&g_render_mutex);

    if (sem_trywait(&lot->parkingSpots) == 0)
    {
        int spot = -1;
        pthread_mutex_lock(&g_render_mutex);
        for (int i = 0; i < totalParkingSpots; i++)
        {
            if (!lot->spotOccupied[i])
            {
                spot = i;
                lot->spotOccupied[i] = 1;
                break;
            }
        }
        g_vri[vid].vrs = VRS_PARKING;
        g_vri[vid].park_spot = spot;
        g_parkingCount++;
        pthread_mutex_unlock(&g_render_mutex);

        if (spot >= 0)
        {
            float px, py;
            getParkSpotPos(lot->id, spot, &px, &py);
            pthread_mutex_lock(&g_render_mutex);
            g_vri[vid].tx = px;
            g_vri[vid].ty = py;
            pthread_mutex_unlock(&g_render_mutex);
        }
        snprintf(msg, sizeof(msg), "V%d parked at %s spot %d", v->id, getIntersectionName(lot->id), spot + 1);
        addEvent(msg, (Color){60, 220, 80, 255});

        int parkTime = (rand() % 3) + 1;
        interruptible_sleep(parkTime);

        sem_post(&lot->parkingSpots);
        pthread_mutex_lock(&g_render_mutex);
        if (spot >= 0)
        {
            lot->spotOccupied[spot] = 0;
        }
        g_vri[vid].park_spot = -1;
        pthread_mutex_unlock(&g_render_mutex);
        snprintf(msg, sizeof(msg), "V%d left %s parking", v->id, getIntersectionName(lot->id));
        addEvent(msg, (Color){150, 150, 150, 255});
        return;
    }

    if (sem_trywait(&lot->waiting_slots) == 0)
    {
        snprintf(msg, sizeof(msg), "V%d in wait queue at %s", v->id, getIntersectionName(lot->id));
        addEvent(msg, (Color){255, 200, 50, 255});

        sem_wait(&lot->parkingSpots);
        sem_post(&lot->waiting_slots);

        int spot = -1;
        pthread_mutex_lock(&g_render_mutex);
        for (int i = 0; i < totalParkingSpots; i++)
        {
            if (!lot->spotOccupied[i])
            {
                spot = i;
                lot->spotOccupied[i] = 1;
                break;
            }
        }
        g_vri[vid].vrs = VRS_PARKING;
        g_vri[vid].park_spot = spot;
        g_parkingCount++;
        pthread_mutex_unlock(&g_render_mutex);

        if (spot >= 0)
        {
            float px, py;
            getParkSpotPos(lot->id, spot, &px, &py);
            pthread_mutex_lock(&g_render_mutex);
            g_vri[vid].tx = px;
            g_vri[vid].ty = py;
            pthread_mutex_unlock(&g_render_mutex);
        }
        snprintf(msg, sizeof(msg), "V%d parked(queue) at %s spot %d", v->id, getIntersectionName(lot->id), spot + 1);
        addEvent(msg, (Color){60, 220, 80, 255});

        int parkTime = (rand() % 3) + 1;
        interruptible_sleep(parkTime);

        sem_post(&lot->parkingSpots);
        pthread_mutex_lock(&g_render_mutex);
        if (spot >= 0)
        {
            lot->spotOccupied[spot] = 0;
        }
        g_vri[vid].park_spot = -1;
        pthread_mutex_unlock(&g_render_mutex);
        snprintf(msg, sizeof(msg), "V%d left %s parking", v->id, getIntersectionName(lot->id));
        addEvent(msg, (Color){150, 150, 150, 255});
    }
    else
    {
        snprintf(msg, sizeof(msg), "V%d: %s lot FULL, skipping park", v->id, getIntersectionName(lot->id));
        addEvent(msg, (Color){200, 80, 80, 255});
    }
}

// Handles locking the intersection mutex and crossing safely
void crossIntersection(Vehicle *v, int vid)
{
    pthread_mutex_t *mtx = (v->origin == F10) ? &mutex_F10 : &mutex_F11;
    const char *intersectionName = getIntersectionName(v->origin);

    if (v->priority < PRIORITY_HIGH)
    {
        volatile int *ef = (v->origin == F10) ? &emergencyActiveF10 : &emergencyActiveF11;
        if (*ef)
        {
            pthread_mutex_lock(&g_render_mutex);
            g_vri[vid].vrs = VRS_EMERG_WAIT;
            snprintf(g_vri[vid].status, 80, "Emergency wait at %s", intersectionName);
            pthread_mutex_unlock(&g_render_mutex);
            char msg[128];
            snprintf(msg, sizeof(msg), "V%d waiting: emergency at %s", v->id, intersectionName);
            addEvent(msg, (Color){255, 200, 80, 255});
            interruptible_sleep(2);
        }
    }

    pthread_mutex_lock(mtx);
    float sx, sy, apx, apy, crx, cry, trx, try_, ex, ey;
    getPositions(v->origin, &sx, &sy, &apx, &apy, &crx, &cry, &trx, &try_, &ex, &ey);

    pthread_mutex_lock(&g_render_mutex);
    g_vri[vid].vrs = VRS_CROSSING;
    g_vri[vid].tx = crx;
    g_vri[vid].ty = cry;
    snprintf(g_vri[vid].status, 80, "Crossing %s", intersectionName);
    pthread_mutex_unlock(&g_render_mutex);

    char msg[128];
    snprintf(msg, sizeof(msg), "V%d crossing %s (%s)", v->id, intersectionName, getDirectionName(v->direction));
    addEvent(msg, getVehicleColor(v->type));

    interruptible_sleep(1);
    pthread_mutex_unlock(mtx);
}

// The main routine executed by each vehicle thread
void *vehicleThread(void *arg)
{
    Vehicle *v = (Vehicle*)arg;
    int vid = v->id - 1;
    ParkingLot *lot = (v->origin == F10) ? &lotF10 : &lotF11;

    float sx, sy, apx, apy, crx, cry, trx, try_, ex, ey;
    getPositions(v->origin, &sx, &sy, &apx, &apy, &crx, &cry, &trx, &try_, &ex, &ey);

    pthread_mutex_lock(&g_render_mutex);
    g_vri[vid].active = 1;
    g_vri[vid].id = v->id;
    g_vri[vid].type = v->type;
    g_vri[vid].origin = v->origin;
    g_vri[vid].priority = v->priority;
    g_vri[vid].rx = sx;
    g_vri[vid].ry = sy;
    g_vri[vid].tx = sx;
    g_vri[vid].ty = sy;
    g_vri[vid].vrs = VRS_INACTIVE;
    g_vri[vid].flash = 0;
    g_vri[vid].park_spot = -1;
    snprintf(g_vri[vid].status, 80, "Arriving.");
    pthread_mutex_unlock(&g_render_mutex);

    interruptible_sleep(v->arrivalTime);
    if (stopFlag)
    {
        pthread_exit(NULL);
    }

    char msg[128];
    snprintf(msg, sizeof(msg), "V%d (%s) arrived at %s (%s Priority)", v->id, getVehicleTypeName(v->type), getIntersectionName(v->origin), getPriorityName(v->priority));
    addEvent(msg, getVehicleColor(v->type));

    pthread_mutex_lock(&g_render_mutex);
    g_vri[vid].vrs = VRS_APPROACHING;
    g_vri[vid].tx = apx;
    g_vri[vid].ty = apy;
    snprintf(g_vri[vid].status, 80, "Approaching %s", getIntersectionName(v->origin));
    pthread_mutex_unlock(&g_render_mutex);
    interruptible_sleep(2);
   
    if (stopFlag)
    {
        pthread_exit(NULL);
    }

    if (v->priority == PRIORITY_HIGH)
    {
        pthread_mutex_lock(&g_render_mutex);
        g_vri[vid].flash = 1;
        snprintf(g_vri[vid].status, 80, "Emergency Preemption..!");
        pthread_mutex_unlock(&g_render_mutex);

        if (v->origin == F10)
        {
            emergencyActiveF10 = 1;
        }
        else              
        {
            emergencyActiveF11 = 1;
        }

        g_emergencyCount++;
        snprintf(msg, sizeof(msg), "Emergency: V%d (%s) is clearing %s..!", v->id, getVehicleTypeName(v->type), getIntersectionName(v->origin));
        addEvent(msg, (Color){255, 60, 60, 255});

        if (v->origin == F10 && v->destination == F11)
        {
            sendMessage(pipeF10ToF11[1], emergencyMsgF10);
            addEvent("Pipe Msg: Emergency_from_F10 sent to F11 Controller", (Color){255, 120, 120, 255});
            g_pipeMsgs++;
        }
        else if (v->origin == F11 && v->destination == F10)
        {
            sendMessage(pipeF11ToF10[1], emergencyMsgF11);
            addEvent("Pipe Msg: Emergency_from_F11 sent to F10 Controller", (Color){255, 120, 120, 255});
            g_pipeMsgs++;
        }
       
        usleep(300000);
        crossIntersection(v, vid);

        if (v->origin == F10)
        {
            emergencyActiveF10 = 0;
        }
        else              
        {
            emergencyActiveF11 = 0;
        }

        pthread_mutex_lock(&g_render_mutex);
        g_vri[vid].flash = 0;
        pthread_mutex_unlock(&g_render_mutex);
        snprintf(msg, sizeof(msg), "Emergency cleared at %s", getIntersectionName(v->origin));
        addEvent(msg, (Color){80, 255, 120, 255});
    }
    else
    {
        attemptParking(v, lot, vid);
       
        if (stopFlag)
        {
            pthread_exit(NULL);
        }

        pthread_mutex_lock(&g_render_mutex);
        g_vri[vid].vrs = VRS_APPROACHING;
        g_vri[vid].tx = apx;
        g_vri[vid].ty = apy;
        snprintf(g_vri[vid].status, 80, "Waiting to cross %s", getIntersectionName(v->origin));
        pthread_mutex_unlock(&g_render_mutex);
       
        interruptible_sleep(1);
        crossIntersection(v, vid);
    }

    pthread_mutex_lock(&g_render_mutex);
    g_vri[vid].vrs = VRS_TRAVELING;
    g_vri[vid].tx = trx;
    g_vri[vid].ty = try_;
    snprintf(g_vri[vid].status, 80, "Traveling to %s", getIntersectionName(v->destination));
    pthread_mutex_unlock(&g_render_mutex);
   
    snprintf(msg, sizeof(msg), "V%d traveling to %s", v->id, getIntersectionName(v->destination));
    addEvent(msg, (Color){160, 220, 255, 255});
    interruptible_sleep(3);

    pthread_mutex_lock(&g_render_mutex);
    g_vri[vid].vrs = VRS_DONE;
    g_vri[vid].tx = ex;
    g_vri[vid].ty = ey;
    snprintf(g_vri[vid].status, 80, "Done");
    g_vehiclesDone++;
    pthread_mutex_unlock(&g_render_mutex);
   
    snprintf(msg, sizeof(msg), "V%d (%s) finished routing", v->id, getVehicleTypeName(v->type));
    addEvent(msg, (Color){120, 120, 120, 255});

    pthread_exit(NULL);
}

// Updates the global stop flag when a signal is received
void signalHandler(int sig)
{
    (void)sig;
    stopFlag = 1;
}

// Safely joins threads, shuts down IPC, and frees system resources
void cleanupResources(int n)
{
    printf("\nStarting simulation cleanup.\n");
    printf("Waiting for %d vehicle threads to finish.\n", n);
    for (int i = 0; i < n; i++)
    {
        pthread_join(vehicleThreads[i], NULL);
    }
    printf("All vehicle threads have been joined.\n");

    printf("Sending shutdown signals to intersection controllers.\n");
    sendMessage(pipeF10ToF11[1], shutdownMsg);
    sendMessage(pipeF11ToF10[1], shutdownMsg);

    printf("Waiting for controller processes to exit.\n");
    if (pid_F10_ctrl > 0)
    {
        waitpid(pid_F10_ctrl, NULL, 0);
    }
    if (pid_F11_ctrl > 0)
    {
        waitpid(pid_F11_ctrl, NULL, 0);
    }
    printf("Controller processes have been terminated.\n");

    printf("Closing communication pipes.\n");
    close(pipeF10ToF11[0]);
    close(pipeF10ToF11[1]);
    close(pipeF11ToF10[0]);
    close(pipeF11ToF10[1]);

    printf("Destroying semaphores.\n");
    sem_destroy(&lotF10.parkingSpots);
    sem_destroy(&lotF10.waiting_slots);
    sem_destroy(&lotF11.parkingSpots);
    sem_destroy(&lotF11.waiting_slots);

    printf("Releasing intersection locks.\n");
    pthread_mutex_destroy(&mutex_F10);
    pthread_mutex_destroy(&mutex_F11);
    pthread_mutex_destroy(&g_render_mutex);
   
    printf("Cleanup complete. Simulation finished successfully.\n");
}

// Draws a solid rectangle using float coordinates
static void drect(float x, float y, float w, float h, Color c)
{
    DrawRectangle((int)x, (int)y, (int)w, (int)h, c);
}

// Draws a rounded rectangle panel with an outline
static void drectR(float x, float y, float w, float h, Color fill, Color border, float thick)
{
    Rectangle r = {x, y, w, h};
    DrawRectangleRounded(r, 0.08f, 8, fill);
    DrawRectangleRoundedLines(r, 0.08f, 8, border);
}

// Draws horizontally centered text at a given x coordinate
static void dtxtC(const char *s, int cx, int y, int sz, Color c)
{
    int tw = MeasureText(s, sz);
    DrawText(s, cx - tw / 2, y, sz, c);
}

// Draws a rectangle with layered semi-transparent borders for a glow effect
static void glowRect(float x, float y, float w, float h, Color c, int layers)
{
    for (int i = layers; i >= 1; i--)
    {
        Color gc = c;
        gc.a = (unsigned char)(35 * (layers - i + 1));
        drect(x - i * 4, y - i * 4, w + i * 8, h + i * 8, gc);
    }
    drect(x, y, w, h, c);
}

// Renders the initial title screen menu
void drawHomeScreen(void)
{
    float t = (float)GetTime();

    ClearBackground((Color){8, 12, 28, 255});

    for (int gx = 0; gx < WIN_W; gx += 50)
    {
        DrawLine(gx, 0, gx, WIN_H, (Color){18, 24, 50, 255});
    }
    for (int gy = 0; gy < WIN_H; gy += 50)
    {
        DrawLine(0, gy, WIN_W, gy, (Color){18, 24, 50, 255});
    }

    drect(0, WIN_H - 130, WIN_W, 90, (Color){38, 44, 60, 255});
    drect(0, WIN_H - 130, WIN_W, 2, (Color){180, 180, 180, 80});
    drect(0, WIN_H - 42, WIN_W, 2, (Color){180, 180, 180, 80});
    for (int dx = 0; dx < WIN_W; dx += 54)
    {
        drect((float)dx, WIN_H - 89, 34, 4, (Color){220, 185, 0, 140});
    }

    VehicleType vtypes[] = {CAR, BUS, AMBULANCE, BIKE, FIRETRUCK, TRACTOR};
    for (int i = 0; i < 6; i++)
    {
        float spd = 70.0f + i * 28.0f;
        float offset = (float)(i * 260);
        float x = (float)fmod(t * spd + offset, (double)(WIN_W + 120)) - 60.0f;
        float vy = (i % 2 == 0) ? WIN_H - 115.0f : WIN_H - 75.0f;
        float vw = (vtypes[i] == BUS) ? 50 : 40;
        Color vc = getVehicleColor(vtypes[i]);
       
        drect(x, vy, vw, 20, vc);
        drect(x + vw - 10, vy + 3, 8, 14, (Color){255, 255, 255, 90});
        DrawCircle((int)(x + 7), (int)(vy + 20), 4, (Color){40, 40, 40, 255});
        DrawCircle((int)(x + vw - 8), (int)(vy + 20), 4, (Color){40, 40, 40, 255});
    }

    drect(WIN_W / 2 - 330, WIN_H / 2 - 150, 24, 72, (Color){22, 26, 44, 255});
    DrawCircle(WIN_W / 2 - 318, WIN_H / 2 - 138, 8, (Color){200, 30, 30, 255});
    DrawCircle(WIN_W / 2 - 318, WIN_H / 2 - 114, 8, (Color){50, 50, 50, 255});
    DrawCircle(WIN_W / 2 - 318, WIN_H / 2 - 90, 8, (Color){50, 50, 50, 255});
   
    drect(WIN_W / 2 + 306, WIN_H / 2 - 150, 24, 72, (Color){22, 26, 44, 255});
    DrawCircle(WIN_W / 2 + 318, WIN_H / 2 - 138, 8, (Color){50, 50, 50, 255});
    DrawCircle(WIN_W / 2 + 318, WIN_H / 2 - 114, 8, (Color){50, 50, 50, 255});
    DrawCircle(WIN_W / 2 + 318, WIN_H / 2 - 90, 8, (Color){30, 190, 50, 255});

    float pulse = 0.6f + 0.4f * sinf(t * 2.0f);
    const char *title = "F10 & F11 TRAFFIC SIMULATION";
    int tsz = 46;
    int tw = MeasureText(title, tsz);
    int tx = WIN_W / 2 - tw / 2;
    int ty = WIN_H / 2 - 200;
   
    for (int i = 4; i >= 1; i--)
    {
        Color gc = {(unsigned char)(80 * pulse), (unsigned char)(140 * pulse), 255, (unsigned char)(18 * i * (int)pulse + 10)};
        DrawText(title, tx - i, ty - i, tsz, gc);
        DrawText(title, tx + i, ty + i, tsz, gc);
    }
   
    Color titleCol = {(unsigned char)(120 + 80 * pulse), (unsigned char)(180 + 50 * pulse), 255, 255};
    DrawText(title, tx, ty, tsz, titleCol);

    dtxtC("Operating Systems Project FAST NUCES", WIN_W / 2, WIN_H / 2 - 140, 20, (Color){130, 160, 220, 230});
    drect(WIN_W / 2 - 280, WIN_H / 2 - 112, 560, 2, (Color){60, 80, 150, 200});
    drectR(WIN_W / 2 - 280, WIN_H / 2 - 100, 560, 100, (Color){15, 20, 50, 220}, (Color){55, 85, 160, 255}, 2);
    dtxtC("Shayan Tariq       24I-3024", WIN_W / 2, WIN_H / 2 - 80, 20, (Color){100, 200, 255, 255});
    dtxtC("Mahrukh Masroor    24I-0778", WIN_W / 2, WIN_H / 2 - 48, 20, (Color){180, 220, 255, 255});

    Vector2 mouse = GetMousePosition();
    Rectangle btnR = {(float)(WIN_W / 2 - 170), (float)(WIN_H / 2 + 30), 340, 65};
    int hov = CheckCollisionPointRec(mouse, btnR);

    if (hov)
    {
        for (int i = 3; i >= 1; i--)
        {
            Color gc = {30, (unsigned char)(200 + 30 * pulse), 70, (unsigned char)(25 * i)};
            Rectangle gr = {btnR.x - i * 5, btnR.y - i * 5, btnR.width + i * 10, btnR.height + i * 10};
            DrawRectangleRounded(gr, 0.3f, 12, gc);
        }
    }

    Color btnFill = hov ? (Color){35, 195, 75, 255} : (Color){25, 160, 60, 255};
    Color btnBorder = hov ? (Color){80, 255, 140, 255} : (Color){50, 200, 90, 255};
    drectR(btnR.x, btnR.y, btnR.width, btnR.height, btnFill, btnBorder, 2.5f);

    const char *btnTxt = "RUN SIMULATION";
    int btw = MeasureText(btnTxt, 22);
    DrawText(btnTxt, WIN_W / 2 - btw / 2, (int)(btnR.y + 21), 22, hov ? (Color){255, 255, 255, 255} : (Color){200, 255, 210, 255});

    dtxtC("Click to launch the simulation", WIN_W / 2, (int)(btnR.y + 75), 14, (Color){100, 120, 170, 180});
    DrawText("OS Project 2025", (int)(WIN_W - 160), (int)(WIN_H - 28), 13, (Color){60, 70, 110, 180});
}

// Renders the static road, grid, and background elements
void drawSimBackground(void)
{
    ClearBackground((Color){8, 12, 28, 255});
   
    for (int gx = 0; gx < WIN_W; gx += 50)
    {
        DrawLine(gx, 0, gx, WIN_H, (Color){17, 22, 46, 255});
    }
    for (int gy = 0; gy < WIN_H; gy += 50)
    {
        DrawLine(0, gy, WIN_W, gy, (Color){17, 22, 46, 255});
    }
   
    drect(0, ROAD_TOP, (float)WIN_W, ROAD_H, (Color){40, 46, 64, 255});
    drect(0, ROAD_TOP, (float)WIN_W, 2.5f, (Color){190, 190, 190, 90});
    drect(0, ROAD_BOT - 2.5f, (float)WIN_W, 2.5f, (Color){190, 190, 190, 90});
   
    for (int dx = 0; dx < WIN_W; dx += 54)
    {
        drect((float)dx, ROAD_Y - 1.5f, 34, 3, (Color){220, 185, 0, 150});
    }
   
    DrawText(">> F10 to F11 >>", (int)(WIN_W / 2 - 80), (int)(LANE_A_Y - 11), 10, (Color){100, 190, 255, 70});
    DrawText("<< F11 to F10 <<", (int)(WIN_W / 2 - 80), (int)(LANE_B_Y + 3), 10, (Color){255, 200, 100, 70});
}

// Renders a pulsing red overlay when an emergency flag is active
void drawEmergencyGlow(void)
{
    if (!emergencyActiveF10 && !emergencyActiveF11)
    {
        return;
    }
    float pulse = 0.35f + 0.35f * sinf((float)GetTime() * 7.0f);
    drect(0, ROAD_TOP - 10, (float)WIN_W, ROAD_H + 20, (Color){255, 30, 30, (unsigned char)(55 * pulse)});
    drect(0, ROAD_Y - 2, (float)WIN_W, 4, (Color){255, 80, 80, (unsigned char)(110 * pulse)});
}

// Renders the visual representation of an intersection box
void drawIntersection(float cx, float cy, int emergency, float t)
{
    float x = cx - INT_R;
    float y = cy - INT_R;
    float sz = INT_R * 2;
    if (emergency)
    {
        float p = 0.5f + 0.5f * sinf(t * 6);
        drect(x - 22, y - 22, sz + 44, sz + 44, (Color){255, 40, 40, (unsigned char)(65 * p)});
    }
    drectR(x, y, sz, sz, (Color){50, 56, 80, 255}, (Color){85, 100, 150, 200}, 2);
    for (int i = 0; i < 5; i++)
    {
        drect(x + 3, y + 4 + i * 20, sz - 6, 11, (Color){190, 190, 190, 45});
    }
}

// Renders a traffic light pole and colored bulbs based on state
void drawTrafficLight(float px, float py, int state)
{
    float bw = 24;
    float bh = 72;
   
    drect(px + 10, py + bh, 4, ROAD_TOP - py - bh + 8, (Color){55, 60, 75, 255});
    drectR(px, py, bw, bh, (Color){22, 25, 42, 255}, (Color){55, 65, 100, 255}, 1.5f);

    Color off = {35, 35, 35, 255};
    Color redC = (state == 2) ? (Color){220, 30, 30, 255} : off;
    Color yelC = (state == 1) ? (Color){220, 190, 30, 255} : off;
    Color grnC = (state == 0) ? (Color){30, 200, 50, 255} : off;

    float cx2 = px + bw / 2;
   
    if (state == 2)
    {
        DrawCircle((int)cx2, (int)(py + 12), 14, (Color){220, 30, 30, 35});
    }
    DrawCircle((int)cx2, (int)(py + 12), 8, redC);
   
    if (state == 1)
    {
        DrawCircle((int)cx2, (int)(py + 36), 14, (Color){220, 190, 30, 35});
    }
    DrawCircle((int)cx2, (int)(py + 36), 8, yelC);
   
    if (state == 0)
    {
        DrawCircle((int)cx2, (int)(py + 60), 14, (Color){30, 200, 50, 35});
    }
    DrawCircle((int)cx2, (int)(py + 60), 8, grnC);
}

// Renders a parking lot, including capacity spots and the waiting queue
void drawParkingLot(ParkingLot *lot, float lx, float ly)
{
    drectR(lx, ly, LOT_W, LOT_H, (Color){16, 20, 40, 230}, (Color){52, 78, 145, 200}, 2);
   
    const char *ttl = (lot->id == F10) ? "F10 PARKING" : "F11 PARKING";
    DrawText(ttl, (int)(lx + 8), (int)(ly + 5), 12, (Color){130, 190, 255, 255});

    for (int i = 0; i < totalParkingSpots; i++)
    {
        int col = i % 2;
        int row = i / 2;
        float sx = lx + 10 + col * (SPOT_W + 9);
        float sy = ly + 26 + row * (SPOT_H + 8);
        Color bg = lot->spotOccupied[i] ? (Color){150, 32, 32, 255} : (Color){22, 105, 50, 255};
        Color fg = lot->spotOccupied[i] ? (Color){255, 180, 180, 255} : (Color){155, 255, 180, 255};
        drectR(sx, sy, SPOT_W, SPOT_H, bg, (Color){70, 100, 140, 255}, 1);
        DrawText(TextFormat("P%d%s", i + 1, lot->spotOccupied[i] ? " TAKEN" : " FREE"), (int)(sx + 3), (int)(sy + 13), 10, fg);
    }

    int wv = 0;
    sem_getvalue(&lot->waiting_slots, &wv);
    int inQueue = waitingQueueSize - wv;
    float qy = ly + LOT_H - 34;
    DrawText("Queue:", (int)(lx + 5), (int)qy, 10, (Color){180, 180, 210, 255});
    for (int i = 0; i < waitingQueueSize; i++)
    {
        float qbx = lx + 8 + i * 38.0f;
        Color qc = (i < inQueue) ? (Color){255, 185, 0, 255} : (Color){35, 40, 65, 255};
        drectR(qbx, qy + 14, 34, 16, qc, (Color){80, 90, 120, 255}, 1);
    }
}

// Renders an individual vehicle, reading from its visual render state
void drawVehicle(VRI *vr)
{
    if (!vr->active || vr->vrs == VRS_INACTIVE)
    {
        return;
    }
    if (vr->vrs == VRS_DONE && (vr->rx < -130 || vr->rx > WIN_W + 130))
    {
        return;
    }

    float t = (float)GetTime();
    float vw, vh;
    switch(vr->type)
    {
        case BUS:       
            vw = 52; 
            vh = 26; 
            break;
        case AMBULANCE:
        case FIRETRUCK: 
            vw = 46; 
            vh = 24; 
            break;
        case TRACTOR:   
            vw = 44; 
            vh = 22; 
            break;
        case BIKE:      
            vw = 34;
            vh = 16;
            break;
        default:        
            vw = 40; 
            vh = 20; 
            break;
    }
    float x = vr->rx - vw / 2;
    float y = vr->ry - vh / 2;
    Color vc = getVehicleColor(vr->type);

    if (vr->flash)
    {
        float phase = sinf(t * 9.0f);
        Color fc = (phase > 0) ? (Color){255, 50, 50, 255} : (Color){50, 100, 255, 255};
        for (int layer = 3; layer >= 1; layer--)
        {
            drect(x - layer * 5, y - layer * 5, vw + layer * 10, vh + layer * 10, (Color){fc.r, fc.g, fc.b, (unsigned char)(22 * (4 - layer))});
        }
        DrawCircle((int)(x - 6), (int)(y + vh / 2), 5, (phase > 0) ? (Color){255, 60, 60, 255} : (Color){50, 80, 255, 255});
        DrawCircle((int)(x + vw + 6), (int)(y + vh / 2), 5, (phase > 0) ? (Color){50, 80, 255, 255} : (Color){255, 60, 60, 255});
    }

    drectR(x, y, vw, vh, vc, (Color){255, 255, 255, 45}, 1);

    int goRight = (vr->origin == F10);
    float wx = goRight ? x + vw - 10 : x + 1;
    drect(wx, y + 2, 9, vh - 4, (Color){255, 255, 255, 85});

    DrawCircle((int)(x + 7), (int)(y + vh), 4, (Color){30, 30, 30, 255});
    DrawCircle((int)(x + vw - 7), (int)(y + vh), 4, (Color){30, 30, 30, 255});

    DrawText(TextFormat("V%d", vr->id), (int)(x + 3), (int)(y + 4), 10, (Color){0, 0, 0, 210});

    if (vr->flash)      
    {
        DrawText("EMRG!", (int)(x - 2), (int)(y - 15), 10, (Color){255, 80, 80, 255});
    }
    if (vr->vrs == VRS_PARKING)
    {
        DrawText("PARK", (int)(x + 2), (int)(y - 14), 10, (Color){60, 230, 80, 255});
    }
    if (vr->vrs == VRS_CROSSING)
    {
        drect(x - 4, y - 4, vw + 8, vh + 8, (Color){255, 255, 255, 18});
    }
}

// Renders the side dashboard showing live simulation statistics
void drawDashboard(VRI *snap, int snapCount, float simTime)
{
    float x = DASH_X, y = 0, w = (float)(WIN_W) - DASH_X, h = (float)WIN_H;

    drectR(x, y, w, h, (Color){12, 15, 35, 248}, (Color){50, 80, 150, 255}, 2);

    float ty = 10, pw = w - 16;

    dtxtC("TRAFFIC CTRL SYSTEM", (int)(x + w / 2), (int)ty, 16, (Color){100, 180, 255, 255});
    ty += 26;
    dtxtC("F10 & F11 Monitor", (int)(x + w / 2), (int)ty, 11, (Color){120, 145, 215, 220});
    ty += 22;
    drect(x + 8, ty, pw, 1.5f, (Color){50, 80, 145, 255});
    ty += 8;

    int mins = (int)(simTime / 60);
    int secs = (int)fmodf(simTime, 60);
    DrawText(TextFormat("Uptime: %02d:%02d", mins, secs), (int)(x + 10), (int)ty, 12, (Color){160, 255, 190, 255});
    ty += 20;
    drect(x + 8, ty, pw, 1.5f, (Color){50, 80, 145, 255});
    ty += 8;

    DrawText("Statistics:", (int)(x + 10), (int)ty, 11, (Color){100, 120, 205, 255});
    ty += 18;
    DrawText(TextFormat("%-16s %d", "Total Vehicles:", totalVehicles), (int)(x + 12), (int)ty, 11, (Color){200, 200, 200, 255});
    ty += 16;
    DrawText(TextFormat("%-16s %d", "Completed:", g_vehiclesDone), (int)(x + 12), (int)ty, 11, (Color){70, 240, 110, 255});
    ty += 16;
    DrawText(TextFormat("%-16s %d", "Emergencies:", g_emergencyCount), (int)(x + 12), (int)ty, 11, (Color){255, 100, 100, 255});
    ty += 16;
    DrawText(TextFormat("%-16s %d", "Parking Events:", g_parkingCount), (int)(x + 12), (int)ty, 11, (Color){255, 200, 80, 255});
    ty += 16;
    DrawText(TextFormat("%-16s %d", "Pipe Messages:", g_pipeMsgs), (int)(x + 12), (int)ty, 11, (Color){100, 200, 255, 255});
    ty += 18;
    drect(x + 8, ty, pw, 1.5f, (Color){50, 80, 145, 255});
    ty += 8;

    DrawText("-- Intersections --", (int)(x + 10), (int)ty, 11, (Color){100, 120, 205, 255});
    ty += 18;
    const char *sigNames[] = {"GREEN", "YELLOW", "RED"};
    Color sigCols[] = {{50, 220, 50, 255}, {220, 195, 50, 255}, {220, 50, 50, 255}};

    Color f10c = emergencyActiveF10 ? (Color){255, 80, 80, 255} : (Color){80, 230, 80, 255};
    DrawText(TextFormat("F10  %s", emergencyActiveF10 ? "[EMERGENCY]" : "[Normal]"), (int)(x + 12), (int)ty, 12, f10c);
    ty += 17;
    DrawText(TextFormat("  Signal: %s", sigNames[sigF10]), (int)(x + 12), (int)ty, 11, sigCols[sigF10]);
    ty += 17;
   
    Color f11c = emergencyActiveF11 ? (Color){255, 80, 80, 255} : (Color){80, 230, 80, 255};
    DrawText(TextFormat("F11  %s", emergencyActiveF11 ? "[EMERGENCY]" : "[Normal]"), (int)(x + 12), (int)ty, 12, f11c);
    ty += 17;
    DrawText(TextFormat("  Signal: %s", sigNames[sigF11]), (int)(x + 12), (int)ty, 11, sigCols[sigF11]);
    ty += 19;
    drect(x + 8, ty, pw, 1.5f, (Color){50, 80, 145, 255});
    ty += 8;

    DrawText("-- Parking Semaphores --", (int)(x + 10), (int)ty, 11, (Color){100, 120, 205, 255});
    ty += 18;
    int sv0 = 0, sv1 = 0, wv0 = 0, wv1 = 0;
    sem_getvalue(&lotF10.parkingSpots, &sv0);
    sem_getvalue(&lotF10.waiting_slots, &wv0);
    sem_getvalue(&lotF11.parkingSpots, &sv1);
    sem_getvalue(&lotF11.waiting_slots, &wv1);
    Color lc0 = (sv0 == 0) ? (Color){255, 100, 100, 255} : (Color){100, 240, 150, 255};
    Color lc1 = (sv1 == 0) ? (Color){255, 100, 100, 255} : (Color){100, 240, 150, 255};
    DrawText(TextFormat("F10: %d/10 free  Q:%d/%d", sv0, waitingQueueSize - wv0, waitingQueueSize), (int)(x + 12), (int)ty, 11, lc0);
    ty += 16;
    DrawText(TextFormat("F11: %d/10 free  Q:%d/%d", sv1, waitingQueueSize - wv1, waitingQueueSize), (int)(x + 12), (int)ty, 11, lc1);
    ty += 18;
    drect(x + 8, ty, pw, 1.5f, (Color){50, 80, 145, 255});
    ty += 8;

    DrawText("Active Vehicles:", (int)(x + 10), (int)ty, 11, (Color){100, 120, 205, 255});
    ty += 18;
    DrawText("ID  Type        State", (int)(x + 12), (int)ty, 10, (Color){110, 125, 185, 255});
    ty += 14;
    const char *sn[] = {"IDLE", "APPROACH", "PK_QUEUE", "PARKING", "EM_WAIT", "CROSS", "TRAVEL", "DONE"};
    for (int i = 0; i < snapCount && ty < h - 26; i++)
    {
        if (!snap[i].active)
        {
            continue;
        }
        int si = snap[i].vrs;
        if (si > 7)
        {
            si = 0;
        }
        Color vc = (snap[i].vrs == VRS_DONE) ? (Color){80, 80, 80, 255} : getVehicleColor(snap[i].type);
        DrawText(TextFormat("%-3d %-11s %s", snap[i].id, getVehicleTypeName(snap[i].type), sn[si]), (int)(x + 12), (int)ty, 10, vc);
        ty += 13;
    }
}

// Renders the scrolling array of events printed by the pthreads
void drawEventLog(LogEntry *logs, int count)
{
    float lx = 0, ly = LOG_Y, lw = DASH_X, lh = (float)WIN_H - LOG_Y;
    drectR(lx, ly, lw, lh, (Color){8, 11, 26, 230}, (Color){38, 62, 125, 255}, 2);
    DrawText("EVENT LOG", (int)(lx + 8), (int)(ly + 4), 12, (Color){100, 180, 255, 255});
    int n = (count < maximumLog) ? count : maximumLog;
    for (int i = 0; i < n; i++)
    {
        unsigned char alpha = (unsigned char)(255 * (1.0f - (float)i / maximumLog));
        Color c = logs[i].col;
        c.a = alpha;
        DrawText(logs[i].msg, (int)(lx + 8), (int)(ly + 22 + i * 11), 10, c);
    }
}

// Renders the final completion screen summarizing the run
void drawSummary(float elapsed)
{
    float ox = (WIN_W - 520) / 2;
    float oy = (WIN_H - 250) / 2;
    drectR(ox, oy, 520, 250, (Color){6, 12, 42, 252}, (Color){55, 115, 210, 255}, 3);

    dtxtC("SIMULATION COMPLETE", (int)(ox + 260), (int)(oy + 14), 24, (Color){70, 255, 130, 255});
    drect(ox + 10, oy + 48, 500, 2, (Color){55, 115, 210, 255});

    int m = (int)(elapsed / 60);
    int s = (int)fmodf(elapsed, 60);
    DrawText(TextFormat("Total time    : %02d:%02d", m, s), (int)(ox + 24), (int)(oy + 60), 14, (Color){200, 215, 255, 255});
    DrawText(TextFormat("Vehicles      : %d", totalVehicles), (int)(ox + 24), (int)(oy + 84), 14, (Color){200, 215, 255, 255});
    DrawText(TextFormat("Emergencies   : %d", g_emergencyCount), (int)(ox + 24), (int)(oy + 108), 14, (Color){255, 150, 150, 255});
    DrawText(TextFormat("Parking events: %d", g_parkingCount), (int)(ox + 24), (int)(oy + 132), 14, (Color){255, 210, 100, 255});
    DrawText(TextFormat("Pipe messages : %d", g_pipeMsgs), (int)(ox + 24), (int)(oy + 156), 14, (Color){120, 200, 255, 255});

    drect(ox + 10, oy + 180, 500, 2, (Color){55, 115, 210, 255});
    dtxtC("Press ESC to exit", (int)(ox + 260), (int)(oy + 216), 12, (Color){150, 155, 210, 255});
}

// Spawns the required number of pthreads to start the simulation
void startSimulation(void)
{
    printf("\nStarting F10 and F11 Traffic Simulation.\n");
    addEvent("Simulation Started", (Color){100, 220, 255, 255});
    numCreated = 0;
   
    for (int i = 0; i < totalVehicles && !stopFlag; i++)
    {
        Vehicle *v = &vehicles[i];
        v->id = i + 1;
        v->type = randomVehicleType();
        v->origin = randomIntersection();
        v->destination = (v->origin == F10) ? F11 : F10;
        v->direction = randomDirection();
        v->priority = getPriority(v->type);
        v->arrivalTime = i + (rand() % 3);
        v->wantsToPark = (v->priority == PRIORITY_HIGH) ? 0 : (rand() % 2);

        if (stopFlag)
        {
            break;
        }
        if (pthread_create(&vehicleThreads[i], NULL, vehicleThread, v) != 0)
        {
            perror("pthread_create");
            break;
        }
        numCreated++;
        printf("Created Vehicle %d (%s) at %s going to %s (Priority: %s)\n", v->id, getVehicleTypeName(v->type), getIntersectionName(v->origin), getIntersectionName(v->destination), getPriorityName(v->priority));
    }
    printf("\nSuccessfully created %d vehicle threads. Simulation is now running.\n\n", numCreated);
    simInitDone = 1;
    simStartTime = GetTime();
}

// Primary entrypoint handling initialization and the render loop
int main(int argc, char *argv[])
{
    srand((unsigned int)time(NULL));

    if (argc >= 2)
    {
        totalVehicles = atoi(argv[1]);
        if (totalVehicles < 1 || totalVehicles > maximumVehicles)
        {
            fprintf(stderr, "Vehicle count must be 1-%d. Using default %d.\n", maximumVehicles, defaultVehicles);
            totalVehicles = defaultVehicles;
        }
    }

    signal(SIGINT, signalHandler);

    lotF10.id = F10;
    memset(lotF10.spotOccupied, 0, sizeof(lotF10.spotOccupied));
    sem_init(&lotF10.parkingSpots, 0, totalParkingSpots);
    sem_init(&lotF10.waiting_slots, 0, waitingQueueSize);
   
    lotF11.id = F11;
    memset(lotF11.spotOccupied, 0, sizeof(lotF11.spotOccupied));
    sem_init(&lotF11.parkingSpots, 0, totalParkingSpots);
    sem_init(&lotF11.waiting_slots, 0, waitingQueueSize);

    if (pipe(pipeF10ToF11) == -1 || pipe(pipeF11ToF10) == -1)
    {
        perror("pipe");
        exit(1);
    }

    printf("Setting up F10 and F11 Traffic Controllers.\n");

    pid_F10_ctrl = fork();
    if (pid_F10_ctrl < 0)
    {
        perror("fork F10");
        exit(EXIT_FAILURE);
    }
    if (pid_F10_ctrl == 0)
    {
        close(pipeF10ToF11[0]);
        close(pipeF11ToF10[1]);
        controllerProcess(F10, pipeF11ToF10[0], pipeF10ToF11[1]);
    }

    pid_F11_ctrl = fork();
    if (pid_F11_ctrl < 0)
    {
        perror("fork F11");
        exit(EXIT_FAILURE);
    }
    if (pid_F11_ctrl == 0)
    {
        close(pipeF11ToF10[0]);
        close(pipeF10ToF11[1]);
        controllerProcess(F11, pipeF10ToF11[0], pipeF11ToF10[1]);
    }

    close(pipeF10ToF11[0]);
    close(pipeF11ToF10[0]);
   
    printf("F10 Controller started successfully (PID: %d)\n", pid_F10_ctrl);
    printf("F11 Controller started successfully (PID: %d)\n", pid_F11_ctrl);
    sleep(1);

    pthread_mutex_init(&g_render_mutex, NULL);
    memset(g_vri, 0, sizeof(g_vri));

    SetTraceLogLevel(LOG_WARNING);
    InitWindow(WIN_W, WIN_H, "F10 & F11 Traffic Simulation | OS Project");
    SetTargetFPS(60);
    SetExitKey(0);  

    VRI snap[maximumVehicles];
    LogEntry  logSnap[maximumLog];
    int logCountSnap = 0;

    while (!WindowShouldClose() && !stopFlag)
    {
        if (IsKeyPressed(KEY_ESCAPE))
        {
            stopFlag = 1;
            break;
        }

        float dt = (float)GetFrameTime();
        float simTime = (currentScreen == SCREEN_SIM) ? (float)(GetTime() - simStartTime) : 0;

        float phase = fmodf((float)GetTime(), 7.0f);
        sigF10 = (phase < 3.0f) ? 0 : (phase < 4.0f) ? 1 : 2;
        sigF11 = (phase < 3.0f) ? 2 : (phase < 4.0f) ? 1 : 0;

        if (currentScreen == SCREEN_HOME)
        {
            Vector2 mouse = GetMousePosition();
            Rectangle btnR = {(float)(WIN_W / 2 - 170), (float)(WIN_H / 2 + 30), 340, 65};
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, btnR))
            {
                currentScreen = SCREEN_SIM;
                startSimulation();
            }
            BeginDrawing();
            drawHomeScreen();
            EndDrawing();
            continue;
        }

        pthread_mutex_lock(&g_render_mutex);
        int allDone = (g_vehiclesDone >= numCreated && numCreated > 0);
        for (int i = 0; i < totalVehicles; i++)
        {
            float dx = g_vri[i].tx - g_vri[i].rx;
            float dy = g_vri[i].ty - g_vri[i].ry;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist > 0.5f)
            {
                float move = VEHICLE_SPEED * dt;
                if (move > dist)
                {
                    move = dist;
                }
                g_vri[i].rx += (dx / dist) * move;
                g_vri[i].ry += (dy / dist) * move;
            }
        }
        memcpy(snap, g_vri, sizeof(VRI) * totalVehicles);
        memcpy(logSnap, g_log, sizeof(LogEntry) * maximumLog);
        logCountSnap = g_logCount;
        pthread_mutex_unlock(&g_render_mutex);

        BeginDrawing();

        drawSimBackground();
        drawEmergencyGlow();

        drect(F10_CX - 2, ROAD_BOT, 4, LOT_F10_Y - ROAD_BOT, (Color){48, 54, 75, 255});
        drect(F11_CX - 2, ROAD_BOT, 4, LOT_F11_Y - ROAD_BOT, (Color){48, 54, 75, 255});

        drawParkingLot(&lotF10, LOT_F10_X, LOT_F10_Y);
        drawParkingLot(&lotF11, LOT_F11_X, LOT_F11_Y);

        drawIntersection(F10_CX, F10_CY, (int)emergencyActiveF10, (float)GetTime());
        drawIntersection(F11_CX, F11_CY, (int)emergencyActiveF11, (float)GetTime());

        drawTrafficLight(F10_CX - INT_R - 38, ROAD_TOP - 82, sigF10);
        drawTrafficLight(F11_CX + INT_R + 14, ROAD_TOP - 82, sigF11);

        dtxtC("F10", (int)F10_CX, (int)(F10_CY - 12), 18, (Color){220, 235, 255, 255});
        dtxtC("F11", (int)F11_CX, (int)(F11_CY - 12), 18, (Color){220, 235, 255, 255});

        for (int i = 0; i < totalVehicles; i++)
        {
            drawVehicle(&snap[i]);
        }

        drect(0, 0, DASH_X, HEADER_H, (Color){9, 12, 35, 225});
        DrawText("F10 & F11 Traffic Sim  |  fork / pipe / pthread / semaphore / mutex / SIGINT", 10, 10, 13, (Color){110, 185, 255, 255});
        DrawText(TextFormat("Vehicles: %d", totalVehicles), (int)(DASH_X - 130), 10, 13, (Color){160, 160, 210, 255});

        drawDashboard(snap, totalVehicles, simTime);

        drawEventLog(logSnap, logCountSnap);

        if (allDone)
        {
            drawSummary(simTime);
        }

        EndDrawing();
    }

    CloseWindow();
    stopFlag = 1;
   
    if (simInitDone)
    {
        cleanupResources(numCreated);
    }
    else
    {
        printf("Sending shutdown signals to intersection controllers.\n");
        sendMessage(pipeF10ToF11[1], shutdownMsg);
        sendMessage(pipeF11ToF10[1], shutdownMsg);
       
        if (pid_F10_ctrl > 0)
        {
            waitpid(pid_F10_ctrl, NULL, 0);
        }
        if (pid_F11_ctrl > 0)
        {
            waitpid(pid_F11_ctrl, NULL, 0);
        }
       
        close(pipeF10ToF11[1]);
        close(pipeF11ToF10[1]);
       
        sem_destroy(&lotF10.parkingSpots);
        sem_destroy(&lotF10.waiting_slots);
        sem_destroy(&lotF11.parkingSpots);
        sem_destroy(&lotF11.waiting_slots);
       
        pthread_mutex_destroy(&g_render_mutex);
        printf("Cleanup complete. Simulation finished successfully.\n");
    }
}
