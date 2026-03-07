#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <stdarg.h>

#define BLUE   "\x1b[34m"
#define RED    "\x1b[31m"
#define GREEN  "\x1b[32m"
#define YELLOW "\x1b[33m"
#define RESET  "\x1b[0m"
#define PI     3.14159265358979323846

#define MOTOR1_STEP_PIN  17
#define MOTOR1_DIR_PIN   27
#define MOTOR2_STEP_PIN  22
#define MOTOR2_DIR_PIN   23
#define MOTOR3_STEP_PIN  24
#define MOTOR3_DIR_PIN   25
#define MOTOR4_STEP_PIN   5
#define MOTOR4_DIR_PIN    6

#ifndef GPIO_BASE_PATH
#define GPIO_BASE_PATH "/sys/class/gpio"
#endif

#define STEPS_PER_REV      200
#define STEP_DELAY_US      15000
#define MAX_MOTORS         4
#define DISPLAY_LINE_WIDTH 64

static const int ALL_PINS[] = {
    MOTOR1_STEP_PIN, MOTOR1_DIR_PIN,
    MOTOR2_STEP_PIN, MOTOR2_DIR_PIN,
    MOTOR3_STEP_PIN, MOTOR3_DIR_PIN,
    MOTOR4_STEP_PIN, MOTOR4_DIR_PIN
};
#define NUM_PINS  ((int)(sizeof(ALL_PINS) / sizeof(ALL_PINS[0])))

static const int motorPins[4][2] = {
    {MOTOR1_STEP_PIN, MOTOR1_DIR_PIN},
    {MOTOR2_STEP_PIN, MOTOR2_DIR_PIN},
    {MOTOR3_STEP_PIN, MOTOR3_DIR_PIN},
    {MOTOR4_STEP_PIN, MOTOR4_DIR_PIN}
};

void menuStructure(void);
void getTargetCoords(void);
void getCoords(void);

int choice;
int lengths      [2] = {500, 525};
int initcoords   [3] = {0};
int targetcoords [3] = {0};
int angles       [4] = {0};

static void clearScreen(void) {
    printf("\x1b[3J\x1b[H\x1b[2J");
    fflush(stdout);
}

static void clearInputBuffer(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}

void log_activity(const char *fmt, ...) {
    FILE *f = fopen(GPIO_BASE_PATH "/activity.log", "a");
    if (!f) return;
    va_list args;
    va_start(args, fmt);
    vfprintf(f, fmt, args);
    va_end(args);
    fprintf(f, "\n");
    fclose(f);
}

void gpio_export(int pin) {
    char path[64];
    snprintf(path, sizeof(path), GPIO_BASE_PATH "/gpio%d", pin);
    if (access(path, F_OK) == 0) return;
    FILE *f = fopen(GPIO_BASE_PATH "/export", "w");
    if (f) { fprintf(f, "%d", pin); fclose(f); }
}

void gpio_unexport(int pin) {
    FILE *f = fopen(GPIO_BASE_PATH "/unexport", "w");
    if (f) { fprintf(f, "%d", pin); fclose(f); }
}

void gpio_set_direction(int pin, const char *dir) {
    char path[64];
    snprintf(path, sizeof(path), GPIO_BASE_PATH "/gpio%d/direction", pin);
    FILE *f = fopen(path, "w");
    if (f) { fprintf(f, "%s", dir); fclose(f); }
}

void gpio_write(int pin, int value) {
    char path[64];
    snprintf(path, sizeof(path), GPIO_BASE_PATH "/gpio%d/value", pin);
    FILE *f = fopen(path, "w");
    if (f) { fprintf(f, "%d", value); fclose(f); }
}

int gpio_read(int pin) {
    char path[64];
    snprintf(path, sizeof(path), GPIO_BASE_PATH "/gpio%d/value", pin);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int value = -1;
    fscanf(f, "%d", &value);
    fclose(f);
    return value;
}

void gpio_init(void) {
    log_activity("INIT: Exporting and configuring %d GPIO pins", NUM_PINS);
    for (int i = 0; i < NUM_PINS; i++) {
        gpio_export(ALL_PINS[i]);
        gpio_set_direction(ALL_PINS[i], "out");
        gpio_write(ALL_PINS[i], 0);
    }
    log_activity("INIT: All pins configured as OUTPUT, set LOW");
}

void gpio_cleanup(void) {
    for (int i = 0; i < NUM_PINS; i++) {
        gpio_write(ALL_PINS[i], 0);
        gpio_unexport(ALL_PINS[i]);
    }
    log_activity("CLEANUP: All pins reset to LOW and unexported");
}

static int gcd2(int a, int b) {
    if (a < 0) a = -a;
    if (b < 0) b = -b;
    while (b) { int t = b; b = a % b; a = t; }
    return a;
}

static int gcd_nonzero(int *arr, int n) {
    int g = 0;
    for (int i = 0; i < n; i++)
        if (arr[i] > 0) g = gcd2(g, arr[i]);
    return (g == 0) ? 1 : g;
}

static void fill_bresenham(int *pat, int n, int k, int error_init) {
    int error = error_init;
    for (int i = 0; i < n; i++) {
        error -= k;
        if (error < 0) { pat[i] = 1; error += n; }
        else            { pat[i] = 0; }
    }
}

static int count_pattern(int *pat, int n) {
    int c = 0;
    for (int i = 0; i < n; i++) c += pat[i];
    return c;
}

static int buildPatterns(int *step_counts, int n_motors,
                         int **patterns, int *section_len_out, int *reps_out) {
    int max_steps = 0;
    for (int m = 0; m < n_motors; m++)
        if (step_counts[m] > max_steps) max_steps = step_counts[m];

    if (max_steps == 0) { *section_len_out = 0; *reps_out = 0; return 1; }

    int g           = gcd_nonzero(step_counts, n_motors);
    int section_len = max_steps / g;
    int reps        = g;

    for (int m = 0; m < n_motors; m++) {
        patterns[m] = calloc(section_len, sizeof(int));
        if (!patterns[m]) {
            for (int j = 0; j < m; j++) free(patterns[j]);
            return 0;
        }
    }

    int error_inits[] = { section_len / 2, 0, section_len - 1 };
    int n_inits = (int)(sizeof(error_inits) / sizeof(error_inits[0]));

    for (int attempt = 0; attempt < n_inits; attempt++) {
        int ok = 1;
        for (int m = 0; m < n_motors; m++) {
            fill_bresenham(patterns[m], section_len, step_counts[m] / g, error_inits[attempt]);
            if (count_pattern(patterns[m], section_len) != step_counts[m] / g) { ok = 0; break; }
        }
        if (ok) break;
    }

    for (int m = 0; m < n_motors; m++) {
        if (count_pattern(patterns[m], section_len) * reps != step_counts[m]) {
            for (int j = 0; j < n_motors; j++) free(patterns[j]);
            return 0;
        }
    }

    *section_len_out = section_len;
    *reps_out        = reps;
    return 1;
}

static void printRuler(int tick_start, int tick_end, int prefix_len) {
    int show_hundreds = (tick_end > 99);

    if (show_hundreds) {
        printf("%*s", prefix_len, "");
        for (int t = tick_start; t < tick_end; t++) {
            int col = t + 1;
            if (col % 100 == 0) printf(YELLOW "%d" RESET, (col / 100) % 10);
            else                 printf(" ");
        }
        printf("\n");
    }

    printf("%*s", prefix_len, "");
    for (int t = tick_start; t < tick_end; t++) {
        int col = t + 1;
        if      (col % 10 == 0) printf(YELLOW "%d" RESET, (col / 10) % 10);
        else if (col % 5  == 0) printf(YELLOW "+" RESET);
        else                    printf(".");
    }
    printf("\n");

    printf("%*s", prefix_len, "");
    for (int t = tick_start; t < tick_end; t++) {
        int col = t + 1;
        if (col % 10 == 0) printf(YELLOW "%d" RESET, col % 10);
        else                printf(" ");
    }
    printf("\n");
}

int showTickPatternScreen(int *step_counts, int n_motors) {
    static const char *names[] = {"Base", "Shoulder", "Elbow", "End Effector"};
    int *patterns[MAX_MOTORS];
    int section_len, reps;

    if (!buildPatterns(step_counts, n_motors, patterns, &section_len, &reps)) {
        printf("    %sPattern generation failed.%s\n", RED, RESET);
        return 0;
    }

    clearScreen();
    printf("%sTICK PATTERN%s\n", BLUE, RESET);

    if (section_len == 0) {
        printf("    %sAll motors at 0 steps — no movement.%s\n\n", YELLOW, RESET);
        printf("    %s1)%s Execute anyway   %s2)%s Cancel\n    ", RED, RESET, RED, RESET);
        int conf;
        scanf("%d", &conf);
        return conf == 1;
    }

    printf("    Section: %d ticks  x  %d repetitions\n\n", section_len, reps);

    int rows       = (section_len + DISPLAY_LINE_WIDTH - 1) / DISPLAY_LINE_WIDTH;
    int prefix_len = 24;

    for (int row = 0; row < rows; row++) {
        int tick_start = row * DISPLAY_LINE_WIDTH;
        int tick_end   = tick_start + DISPLAY_LINE_WIDTH;
        if (tick_end > section_len) tick_end = section_len;

        printRuler(tick_start, tick_end, prefix_len);

        for (int m = 0; m < n_motors; m++) {
            printf("    Motor %d  %-13s ", m + 1, names[m]);
            for (int t = tick_start; t < tick_end; t++) {
                if (patterns[m][t])
                    printf(GREEN "\xe2\x96\x88" RESET);
                else
                    printf(RED   "\xe2\x96\x88" RESET);
            }
            printf("  %d steps\n", step_counts[m]);
        }
        if (row < rows - 1) printf("\n");
    }

    printf("\n    %s1)%s Execute   %s2)%s Cancel\n    ", RED, RESET, RED, RESET);

    int conf;
    scanf("%d", &conf);

    for (int m = 0; m < n_motors; m++) free(patterns[m]);
    return conf == 1;
}

void driveMotorsSimultaneous(int *step_counts, int *dirs, int n_motors,
                             int *step_pins, int *dir_pins) {
    static const char *names[] = {"Base", "Shoulder", "Elbow", "End Effector"};
    int *patterns[MAX_MOTORS];
    int section_len, reps;

    if (!buildPatterns(step_counts, n_motors, patterns, &section_len, &reps)) {
        printf("    %sPattern generation failed. Aborting.%s\n", RED, RESET);
        return;
    }

    if (section_len == 0) {
        printf("    %sNo movement required.%s\n", YELLOW, RESET);
        return;
    }

    char log_buf[256];
    int  pos = 0;
    pos += snprintf(log_buf + pos, sizeof(log_buf) - pos, "DRIVE: Simultaneous |");
    for (int m = 0; m < n_motors; m++)
        pos += snprintf(log_buf + pos, sizeof(log_buf) - pos,
                        " M%d %s %dsteps |", m + 1, dirs[m] ? "CW" : "CCW", step_counts[m]);
    log_activity("%s", log_buf);

    for (int m = 0; m < n_motors; m++)
        gpio_write(dir_pins[m], dirs[m]);
    usleep(STEP_DELAY_US);

    for (int rep = 0; rep < reps; rep++) {
        for (int tick = 0; tick < section_len; tick++) {
            for (int m = 0; m < n_motors; m++)
                if (patterns[m][tick]) gpio_write(step_pins[m], 1);
            usleep(STEP_DELAY_US);
            for (int m = 0; m < n_motors; m++)
                if (patterns[m][tick]) gpio_write(step_pins[m], 0);
            usleep(STEP_DELAY_US);
        }
    }

    pos = 0;
    pos += snprintf(log_buf + pos, sizeof(log_buf) - pos, "DONE: Simultaneous |");
    for (int m = 0; m < n_motors; m++)
        pos += snprintf(log_buf + pos, sizeof(log_buf) - pos,
                        " M%d(%s) %dsteps |", m + 1, names[m], step_counts[m]);
    log_activity("%s", log_buf);

    for (int m = 0; m < n_motors; m++) free(patterns[m]);
}

void passAnglesToDriver(void) {
    int step_counts[3], dirs[3], step_pins[3], dir_pins[3];
    for (int m = 0; m < 3; m++) {
        dirs[m]        = (angles[m] >= 0) ? 1 : 0;
        step_counts[m] = (int)((double)abs(angles[m]) * STEPS_PER_REV / 360.0);
        step_pins[m]   = motorPins[m][0];
        dir_pins[m]    = motorPins[m][1];
    }

    clearScreen();
    printf("%sDRIVER PASSING%s\n", BLUE, RESET);
    printf("    Initializing GPIO pins...\n");
    gpio_init();

    printf("    Driving motors simultaneously...\n");
    driveMotorsSimultaneous(step_counts, dirs, 3, step_pins, dir_pins);

    gpio_cleanup();

    printf("\n    %sAll motors driven to target!%s\n", GREEN, RESET);
    printf("    %sPress any key to return to Main Menu...%s\n", YELLOW, RESET);
    clearInputBuffer();
    getchar();
    menuStructure();
}

void InverseKinematicsCalculations(void) {
    clearScreen();
    printf("%sINVERSE KINEMATICS CALCULATIONS%s\n", BLUE, RESET);
    printf("    Calculating the joint angles for the given target coordinates...\n\n");

    double L1        = (double)lengths[0];
    double L2        = (double)lengths[1];
    double x         = (double)targetcoords[0];
    double baseAngle = (double)targetcoords[1];
    double z         = (double)targetcoords[2];

    double dist = sqrt(x * x + z * z);

    if (dist > (L1 + L2) || dist < fabs(L1 - L2)) {
        clearScreen();
        printf("%sOOR EXCEPTION%s\n", RED, RESET);
        printf("%sERROR:%s Target (%.0f, %.0f) out of reach!\n", RED, RESET, x, z);
        printf("    Max reach: %.0f mm, Target distance: %.2f mm\n", L1 + L2, dist);
        printf("    Press any key to return...");
        clearInputBuffer();
        getchar();
        getTargetCoords();
        return;
    }

    double cosQ2 = (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (cosQ2 >  1.0) cosQ2 =  1.0;
    if (cosQ2 < -1.0) cosQ2 = -1.0;

    double q2 = acos(cosQ2);
    double q1 = atan2(z, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));

    angles[0] = (int)baseAngle;
    angles[1] = ((int)(q1 * 180.0 / PI)) * 16;
    angles[2] = ((int)(q2 * 180.0 / PI)) * 16;

    clearScreen();
    printf("%sINVERSE KINEMATICS CALCULATIONS%s\n", BLUE, RESET);
    printf("    %sCalculated Angles:%s\n", GREEN, RESET);
    printf("    Base (Rot): %d deg\n",   angles[0]);
    printf("    Shoulder:   %d deg\n",   angles[1]);
    printf("    Elbow:      %d deg\n\n", angles[2]);
    printf("    %sJoint angles are within limits.%s\n", GREEN, RESET);
    printf("    %sArm can reach the target!%s\n",       GREEN, RESET);
    printf("    %sPress any key to see tick pattern...%s\n", YELLOW, RESET);
    clearInputBuffer();
    getchar();

    int step_counts[3];
    for (int m = 0; m < 3; m++)
        step_counts[m] = (int)((double)abs(angles[m]) * STEPS_PER_REV / 360.0);

    if (showTickPatternScreen(step_counts, 3))
        passAnglesToDriver();
    else
        menuStructure();
}

void getTargetCoords(void) {
    clearScreen();
    printf("%sTARGET COORDINATES MENU%s\n", BLUE, RESET);
    printf("    Please enter coordinates (x, y, z):\n");
    printf("    x,z = mm, y = degrees base rotation\n    Input format: x, y, z\n    ");

    if (scanf("%d, %d, %d", &targetcoords[0], &targetcoords[1], &targetcoords[2]) != 3) {
        clearInputBuffer();
        printf("    %sInvalid input format. Try again.%s\n", RED, RESET);
        getTargetCoords();
        return;
    }

    printf("\n    %sX (Horizontal):%s %d mm\n", RED, RESET, targetcoords[0]);
    printf("    %sBase Angle (Y):%s %d deg\n",  RED, RESET, targetcoords[1]);
    printf("    %sZ (Height):    %s %d mm\n",   RED, RESET, targetcoords[2]);
    printf("\n    %s1)%s Confirm and continue\n    %s2)%s Re-enter\n    ",
           RED, RESET, RED, RESET);

    scanf("%d", &choice);
    if (choice == 1) InverseKinematicsCalculations();
    else if (choice == 2) getTargetCoords();
    else getTargetCoords();
}

void getCoords(void) {
    clearScreen();
    printf("%sINITIAL COORDINATES MENU%s\n", BLUE, RESET);
    printf("    Please enter coordinates (x, y, z):\n");
    printf("    x,z = mm, y = degrees base rotation\n    Input format: x, y, z\n    ");

    if (scanf("%d, %d, %d", &initcoords[0], &initcoords[1], &initcoords[2]) != 3) {
        clearInputBuffer();
        printf("    %sInvalid input format. Try again.%s\n", RED, RESET);
        getCoords();
        return;
    }

    printf("\n    %sX (Horizontal):%s %d mm\n", RED, RESET, initcoords[0]);
    printf("    %sBase Angle (Y):%s %d deg\n",  RED, RESET, initcoords[1]);
    printf("    %sZ (Height):    %s %d mm\n",   RED, RESET, initcoords[2]);
    printf("\n    %s1)%s Confirm and continue\n    %s2)%s Re-enter\n    ",
           RED, RESET, RED, RESET);

    scanf("%d", &choice);
    if (choice == 1) getTargetCoords();
    else if (choice == 2) getCoords();
    else getCoords();
}

void customSecLenMenu(void) {
    clearScreen();
    int tempL1, tempL2;
    printf("%sCUSTOM SECTION LENGTH MENU%s\n", BLUE, RESET);
    printf("    Lower Section (L1) in mm: ");
    if (scanf("%d", &tempL1) != 1) { clearInputBuffer(); customSecLenMenu(); return; }
    printf("    Upper Section (L2) in mm: ");
    if (scanf("%d", &tempL2) != 1) { clearInputBuffer(); customSecLenMenu(); return; }
    lengths[0] = tempL1;
    lengths[1] = tempL2;
    getCoords();
}

void inverseKinematicsMenu(void) {
    clearScreen();
    printf("%sINVERSE KINEMATICS MENU%s:\n", BLUE, RESET);
    printf("    %s1)%s Use default lengths (500/525)\n", RED, RESET);
    printf("    %s2)%s Use custom lengths\n    ",         RED, RESET);

    if (scanf("%d", &choice) != 1) { clearInputBuffer(); inverseKinematicsMenu(); return; }
    if (choice == 1) getCoords();
    else if (choice == 2) customSecLenMenu();
    else inverseKinematicsMenu();
}

void menuStructure(void) {
    clearScreen();
    printf("%sMAIN MENU%s\n", BLUE, RESET);
    printf("    %s1)%s Inverse Kinematics Mode\n", RED, RESET);
    printf("    %s2)%s Raw Movement Data (Exit)\n    ", RED, RESET);

    if (scanf("%d", &choice) == 1 && choice == 1) {
        inverseKinematicsMenu();
    } else {
        printf("Exiting...\n");
    }
}

int main(void) {
    menuStructure();
    clearScreen();
    return 0;
}