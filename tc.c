#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>


// Time constants (microseconds)

#define STOP_TIME 2000000     // 2-second stop
#define DELTA_L   5000000     // left turn time
#define DELTA_S   4000000     // straight time
#define DELTA_R   3000000     // right turn time


// Direction indices

#define DIR_N 0
#define DIR_S 1
#define DIR_E 2
#define DIR_W 3


// Quadrant indices

#define Q_NW 0
#define Q_NE 1
#define Q_SW 2
#define Q_SE 3
#define NUM_QUADS 4
#define NUM_CARS 8


// Direction pair for each car

typedef struct {
    char dir_original;
    char dir_target;
} directions;


// Per-car state tracking

typedef struct {
    int cid;                // car ID
    double arrival_time;    // scheduled arrival
    directions dir;         // original + target directions
    double stop_complete_time;
    int at_front;           // is at front of lane?
    int waiting;            // waiting at stop sign?
    int crossing;           // currently in intersection?
    int done;               // finished crossing?
} car_info;


// Quadrant lock supporting same-direction sharing

typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int owner_dir;   // -1 = free, otherwise direction ID
    int count;       // number of cars from same direction
} quadrant_t;


// Global synchronization objects

quadrant_t quads[NUM_QUADS];
pthread_mutex_t dir_lock[4];     // ensures head-of-line behavior
pthread_mutex_t state_lock;      // protects shared car state
pthread_cond_t state_cond;       // wake waiting cars
pthread_mutex_t print_lock;      // serializes output

struct timeval start_time;
car_info cars[NUM_CARS];


// Returns seconds since simulation start

double get_sim_time() {
    struct timeval now;
    gettimeofday(&now, NULL);
    return (now.tv_sec - start_time.tv_sec) +
           (now.tv_usec - start_time.tv_usec) / 1000000.0;
}


// Sleep helper

void Spin(int usec) { usleep(usec); }


// Safe printing function

void print_event(int cid, char orig, char target, const char* event) {
    pthread_mutex_lock(&print_lock);
    printf("Time %.1f: Car %d (%c %c) %s\n",
           get_sim_time(), cid, orig, target, event);
    fflush(stdout);
    pthread_mutex_unlock(&print_lock);
}


// Converts character direction to index

int dir_to_index(char d) {
    switch (d) {
        case '^': return DIR_N;
        case 'v': return DIR_S;
        case '>': return DIR_E;
        case '<': return DIR_W;
    }
    return -1;
}


// Turn type: 0=straight, 1=left, 2=right

int get_turn_type(char orig, char target) {
    if (orig == target) return 0;
    if ((orig=='^' && target=='<') ||
        (orig=='v' && target=='>') ||
        (orig=='>' && target=='^') ||
        (orig=='<' && target=='v'))
        return 1;
    return 2;
}


// Returns DELTA_L, DELTA_S, DELTA_R

int get_crossing_time(int turn) {
    if (turn == 1) return DELTA_L;
    if (turn == 0) return DELTA_S;
    return DELTA_R;
}


// Quadrant bitmask needed for movement

int get_quadrant_mask(char orig, char target) {
    int turn = get_turn_type(orig, target);
    switch (orig) {
        case '^':
            if (turn == 2) return (1<<Q_SW);
            if (turn == 0) return (1<<Q_SW) | (1<<Q_NW);
            return (1<<Q_SW) | (1<<Q_NW) | (1<<Q_NE);
        case 'v':
            if (turn == 2) return (1<<Q_NE);
            if (turn == 0) return (1<<Q_NE) | (1<<Q_SE);
            return (1<<Q_NE) | (1<<Q_SE) | (1<<Q_SW);
        case '>':
            if (turn == 2) return (1<<Q_NW);
            if (turn == 0) return (1<<Q_NW) | (1<<Q_NE);
            return (1<<Q_NW) | (1<<Q_NE) | (1<<Q_SE);
        case '<':
            if (turn == 2) return (1<<Q_SE);
            if (turn == 0) return (1<<Q_SE) | (1<<Q_SW);
            return (1<<Q_SE) | (1<<Q_SW) | (1<<Q_NW);
    }
    return 0;
}


// Quadrant acquire supporting same-direction sharing

void acquire_quad(int q, int dir) {
    pthread_mutex_lock(&quads[q].lock);
    while (quads[q].owner_dir != -1 && quads[q].owner_dir != dir)
        pthread_cond_wait(&quads[q].cond, &quads[q].lock);
    quads[q].owner_dir = dir;
    quads[q].count++;
    pthread_mutex_unlock(&quads[q].lock);
}


// Release quadrant ownership

void release_quad(int q) {
    pthread_mutex_lock(&quads[q].lock);
    quads[q].count--;
    if (quads[q].count == 0) {
        quads[q].owner_dir = -1;
        pthread_cond_broadcast(&quads[q].cond);
    }
    pthread_mutex_unlock(&quads[q].lock);
}


// Check if earlier-arriving cars are stuck

int earlier_car_waiting(car_info *car) {
    double my_stop = car->stop_complete_time;
    int my_dir = dir_to_index(car->dir.dir_original);

    for (int i = 0; i < NUM_CARS; i++) {
        if (cars[i].cid == car->cid) continue;
        if (cars[i].done) continue;
        int odir = dir_to_index(cars[i].dir.dir_original);
        if (odir == my_dir) continue;

        if (cars[i].stop_complete_time > 0 &&
            cars[i].stop_complete_time < my_stop &&
            cars[i].at_front && cars[i].waiting && !cars[i].crossing)
            return 1;
    }
    return 0;
}


// Car arriving and waiting logic

void ArriveIntersection(car_info *car) {
    int dir = dir_to_index(car->dir.dir_original);
    print_event(car->cid, car->dir.dir_original, car->dir.dir_target, "arriving");

    Spin(STOP_TIME);

    pthread_mutex_lock(&state_lock);
    car->stop_complete_time = get_sim_time();
    pthread_mutex_unlock(&state_lock);

    pthread_mutex_lock(&dir_lock[dir]);

    pthread_mutex_lock(&state_lock);
    car->at_front = 1;
    car->waiting = 1;
    pthread_cond_broadcast(&state_cond);
    pthread_mutex_unlock(&state_lock);

    while (1) {
        pthread_mutex_lock(&state_lock);
        int wait = earlier_car_waiting(car);
        if (!wait) {
            pthread_mutex_unlock(&state_lock);
            break;
        }
        pthread_cond_wait(&state_cond, &state_lock);
        pthread_mutex_unlock(&state_lock);
    }
}


// Car crossing intersection

void CrossIntersection(car_info *car) {
    int dir = dir_to_index(car->dir.dir_original);
    int turn = get_turn_type(car->dir.dir_original, car->dir.dir_target);
    int cross_time = get_crossing_time(turn);
    int mask = get_quadrant_mask(car->dir.dir_original, car->dir.dir_target);

    for (int q = 0; q < NUM_QUADS; q++)
        if (mask & (1<<q)) acquire_quad(q, dir);

    pthread_mutex_lock(&state_lock);
    car->waiting = 0;
    car->crossing = 1;
    pthread_cond_broadcast(&state_cond);
    pthread_mutex_unlock(&state_lock);

    pthread_mutex_unlock(&dir_lock[dir]);

    print_event(car->cid, car->dir.dir_original, car->dir.dir_target, "crossing");
    Spin(cross_time);

    pthread_mutex_lock(&state_lock);
    car->crossing = 0;
    pthread_mutex_unlock(&state_lock);

    for (int q = NUM_QUADS - 1; q >= 0; q--)
        if (mask & (1<<q)) release_quad(q);
}


// Car exiting intersection

void ExitIntersection(car_info *car) {
    print_event(car->cid, car->dir.dir_original, car->dir.dir_target, "exiting");

    pthread_mutex_lock(&state_lock);
    car->done = 1;
    car->at_front = 0;
    pthread_cond_broadcast(&state_cond);
    pthread_mutex_unlock(&state_lock);
}


// Car thread function

void* car_thread(void *arg) {
    car_info *car = (car_info*)arg;

    while (get_sim_time() < car->arrival_time)
        usleep(1000);

    ArriveIntersection(car);
    CrossIntersection(car);
    ExitIntersection(car);

    return NULL;
}


// Initialize all locks and cars

void init_system() {
    pthread_mutex_init(&print_lock, NULL);
    pthread_mutex_init(&state_lock, NULL);
    pthread_cond_init(&state_cond, NULL);

    for (int i = 

 0; i < 4; i++)
        pthread_mutex_init(&dir_lock[i], NULL);

    for (int q = 0; q < NUM_QUADS; q++) {
        pthread_mutex_init(&quads[q].lock, NULL);
        pthread_cond_init(&quads[q].cond, NULL);
        quads[q].owner_dir = -1;
        quads[q].count = 0;
    }
}


// Hardcoded test cars from P3

void init_cars() {
    cars[0] = (car_info){1, 1.1, {'^','^'}, 0,0,0,0,0};
    cars[1] = (car_info){2, 2.2, {'^','^'}, 0,0,0,0,0};
    cars[2] = (car_info){3, 3.3, {'^','<'}, 0,0,0,0,0};
    cars[3] = (car_info){4, 4.4, {'v','v'}, 0,0,0,0,0};
    cars[4] = (car_info){5, 5.5, {'v','>'}, 0,0,0,0,0};
    cars[5] = (car_info){6, 6.6, {'^','^'}, 0,0,0,0,0};
    cars[6] = (car_info){7, 7.7, {'>','^'}, 0,0,0,0,0};
    cars[7] = (car_info){8, 8.8, {'<','^'}, 0,0,0,0,0};
}

// Main entry

int main() {
    pthread_t threads[NUM_CARS];

    init_cars();
    init_system();
    gettimeofday(&start_time, NULL);

    printf("Traffic Control Simulation Started\n");
    printf("===================================\n");

    for (int i = 0; i < NUM_CARS; i++)
        pthread_create(&threads[i], NULL, car_thread, &cars[i]);

    for (int i = 0; i < NUM_CARS; i++)
        pthread_join(threads[i], NULL);

    printf("===================================\n");
    printf("Simulation Complete\n");

    return 0;
}
