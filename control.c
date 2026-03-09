#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <dirent.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BABY_BLUE "\x1b[38;2;137;207;240m"  /* RGB(137,207,240) baby blue */
#define RED       "\x1b[31m"
#define GREEN     "\x1b[32m"
#define YELLOW    "\x1b[33m"
#define DIM       "\x1b[2m"
#define RESET     "\x1b[0m"

#define MOTOR1_STEP_PIN  17
#define MOTOR1_DIR_PIN   27
#define MOTOR2_STEP_PIN  22
#define MOTOR2_DIR_PIN   23
#define MOTOR3_STEP_PIN  24
#define MOTOR3_DIR_PIN   25
#define MOTOR4_STEP_PIN   5
#define MOTOR4_DIR_PIN    6
#define ENABLE           12

#ifndef GPIO_BASE_PATH
#define GPIO_BASE_PATH "/sys/class/gpio"
#endif

#define PI                 3.14159265358979323846
#define STEPS_PER_REV      200
#define STEP_DELAY_US      15000
#define MAX_MOTORS         4
#define DISPLAY_LINE_WIDTH 64
#define GEARBOX_RATIO      16
#define RULE_WIDTH         48
#define SCRIPT_DIR         "scripts"

static const int ALL_PINS[] = {
    MOTOR1_STEP_PIN, MOTOR1_DIR_PIN,
    MOTOR2_STEP_PIN, MOTOR2_DIR_PIN,
    MOTOR3_STEP_PIN, MOTOR3_DIR_PIN,
    MOTOR4_STEP_PIN, MOTOR4_DIR_PIN
};
#define NUM_PINS ((int)(sizeof(ALL_PINS) / sizeof(ALL_PINS[0])))

static const int motorPins[4][2] = {
    {MOTOR1_STEP_PIN, MOTOR1_DIR_PIN},
    {MOTOR2_STEP_PIN, MOTOR2_DIR_PIN},
    {MOTOR3_STEP_PIN, MOTOR3_DIR_PIN},
    {MOTOR4_STEP_PIN, MOTOR4_DIR_PIN}
};

static const int HAS_GEAR[3] = {0, 1, 1};

enum {
    SCRIPT_MODE_IK = 0,
    SCRIPT_MODE_RAW_JOINT = 1,
    SCRIPT_MODE_RAW_MOTOR = 2
};

void menuStructure(void);
void getTargetCoords(void);
void getCoords(void);
void getRawMotorData(void);
void getRawAngles(void);
void run_selftest(void);
void scriptLibraryMenu(void);

int choice;
int lengths      [2] = {500, 525};
int initcoords   [3] = {0};
int targetcoords [3] = {0};
int angles       [4] = {0};
int rawSteps     [3] = {0};
int rawDirs      [3] = {0};


static void clearScreen(void) {
    printf("\x1b[3J\x1b[H\x1b[2J");
    fflush(stdout);
}

static void clearInputBuffer(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}

static double monotonic_seconds(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void format_ttm(char *buf, size_t n, double seconds) {
    if (seconds < 0) seconds = 0;

    if (seconds < 1.0) {
        snprintf(buf, n, "%.0f ms", seconds * 1000.0);
        return;
    }
    if (seconds < 60.0) {
        snprintf(buf, n, "%.3f s", seconds);
        return;
    }

    int minutes = (int)(seconds / 60.0);
    double rest = seconds - (double)minutes * 60.0;
    snprintf(buf, n, "%dm %.3fs", minutes, rest);
}

static void printHeader(const char *title) {
    int i;
    printf("\n  " BABY_BLUE "%s" RESET "\n  " BABY_BLUE, title);
    for (i = 0; i < RULE_WIDTH; i++) printf("\xe2\x94\x80");
    printf(RESET "\n\n");
}

static void printDivider(void) {
    int i;
    printf("\n  " DIM);
    for (i = 0; i < RULE_WIDTH; i++) printf("\xe2\x94\x80");
    printf(RESET "\n\n");
}

static void printPrompt(void) {
    printf("  " BABY_BLUE ">" RESET " ");
    fflush(stdout);
}

static void printOk(const char *msg) {
    printf("  " GREEN "[+]" RESET "  %s\n", msg);
}

static void printErr(const char *msg) {
    printf("\n  " RED "[!]" RESET "  %s\n\n", msg);
}
static void printSplash(void) {
    clearScreen();
    printf(BABY_BLUE);
    printf("\n");
    printf("    ██  ██  ██████  ██      ██       █████  \n");
    printf("    ██  ██  ██      ██      ██      ██   ██ \n");
    printf("    ██████  ██████  ██      ██      ██   ██ \n");
    printf("    ██  ██  ██      ██      ██      ██   ██ \n");
    printf("    ██  ██  ██████  ██████  ██████   █████  \n");
    printf("\n");
    printf("\n" RESET);
    fflush(stdout);
    sleep(2);
}

static void printGoodbye(void) {
    clearScreen();
    printf(BABY_BLUE);
    printf("\n");
    printf("             ██████   ██  ██  ██████ \n");
    printf("             ██   ██   ████   ██     \n");
    printf("             ██████     ██    ██████ \n");
    printf("             ██   ██    ██    ██     \n");
    printf("             ██████     ██    ██████ \n");
    printf("\n");
    printf("\n" RESET);
    fflush(stdout);
    sleep(2);
    clearScreen();
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
    if (tick_end > 99) {
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
    int  section_len, reps;

    if (!buildPatterns(step_counts, n_motors, patterns, &section_len, &reps)) {
        printErr("Pattern generation failed.");
        return 0;
    }

    clearScreen();
    printHeader("TICK PATTERN");

    if (section_len == 0) {
        printf("  " YELLOW "All motors at 0 steps — no movement." RESET "\n\n");
        printf("  " DIM "[1]" RESET "  Execute anyway\n");
        printf("  " DIM "[2]" RESET "  Cancel\n\n");
        printPrompt();
        int conf;
        scanf("%d", &conf);
        return conf == 1 ? 1 : 0;
    }

    printf("  Section  " BABY_BLUE "%d ticks" RESET "  \xc3\x97  " BABY_BLUE "%d repetitions" RESET "\n\n",
           section_len, reps);

    int rows       = (section_len + DISPLAY_LINE_WIDTH - 1) / DISPLAY_LINE_WIDTH;
    int prefix_len = 24;

    for (int row = 0; row < rows; row++) {
        int tick_start = row * DISPLAY_LINE_WIDTH;
        int tick_end   = tick_start + DISPLAY_LINE_WIDTH;
        if (tick_end > section_len) tick_end = section_len;

        printRuler(tick_start, tick_end, prefix_len);

        for (int m = 0; m < n_motors; m++) {
            printf("  Motor %d  %-13s ", m + 1, names[m]);
            for (int t = tick_start; t < tick_end; t++) {
                if (patterns[m][t]) printf(GREEN  "\xe2\x96\x88" RESET);
                else                printf(RED    "\xe2\x96\x88" RESET);
            }
            printf("  " DIM "%d steps" RESET "\n", step_counts[m]);
        }
        if (row < rows - 1) printf("\n");
    }

    printf("\n");
    printf("  " DIM "[1]" RESET "  Execute " DIM "(simultaneous)" RESET "\n");
    printf("  " DIM "[2]" RESET "  Execute " DIM "(sequential)" RESET "\n");
    printf("  " DIM "[3]" RESET "  Cancel\n\n");
    printPrompt();

    int conf;
    scanf("%d", &conf);

    for (int m = 0; m < n_motors; m++) free(patterns[m]);

    if (conf == 1) return 1;
    if (conf == 2) return 2;
    return 0;
}


void driveMotorsSimultaneous(int *step_counts, int *dirs, int n_motors,
                             int *step_pins, int *dir_pins) {
    static const char *names[] = {"Base", "Shoulder", "Elbow", "End Effector"};
    int *patterns[MAX_MOTORS];
    int  section_len, reps;

    if (!buildPatterns(step_counts, n_motors, patterns, &section_len, &reps)) {
        printErr("Pattern generation failed. Aborting.");
        return;
    }
    if (section_len == 0) {
        printf("  " YELLOW "No movement required." RESET "\n");
        log_activity("DONE: Simultaneous | no steps | TTM=0 ms");
        printf("  " DIM "TTM (total, simultaneous)" RESET "  " BABY_BLUE "0 ms" RESET "\n");
        return;
    }

    char log_buf[256];
    int  pos = 0;
    pos += snprintf(log_buf + pos, sizeof(log_buf) - pos, "DRIVE: Simultaneous |");
    for (int m = 0; m < n_motors; m++)
        pos += snprintf(log_buf + pos, sizeof(log_buf) - pos,
                        " M%d %s %dsteps |", m + 1, dirs[m] ? "CW" : "CCW", step_counts[m]);
    log_activity("%s", log_buf);

    double t0 = monotonic_seconds();

    for (int m = 0; m < n_motors; m++) gpio_write(dir_pins[m], dirs[m]);
    usleep(STEP_DELAY_US);

    for (int rep = 0; rep < reps; rep++)
        for (int tick = 0; tick < section_len; tick++) {
            for (int m = 0; m < n_motors; m++)
                if (patterns[m][tick]) gpio_write(step_pins[m], 1);
            usleep(STEP_DELAY_US);
            for (int m = 0; m < n_motors; m++)
                if (patterns[m][tick]) gpio_write(step_pins[m], 0);
            usleep(STEP_DELAY_US);
        }

    double elapsed = monotonic_seconds() - t0;
    char ttm_buf[32];
    format_ttm(ttm_buf, sizeof(ttm_buf), elapsed);

    pos = 0;
    pos += snprintf(log_buf + pos, sizeof(log_buf) - pos, "DONE: Simultaneous |");
    for (int m = 0; m < n_motors; m++)
        pos += snprintf(log_buf + pos, sizeof(log_buf) - pos,
                        " M%d(%s) %dsteps |", m + 1, names[m], step_counts[m]);
    pos += snprintf(log_buf + pos, sizeof(log_buf) - pos, " TTM=%s", ttm_buf);
    log_activity("%s", log_buf);

    printf("\n  " DIM "TTM (total, simultaneous)" RESET "  " BABY_BLUE "%s" RESET "\n", ttm_buf);

    for (int m = 0; m < n_motors; m++) free(patterns[m]);
}

void driveMotorsSequential(int *step_counts, int *dirs, int n_motors,
                           int *step_pins, int *dir_pins) {
    static const char *names[] = {"Base", "Shoulder", "Elbow", "End Effector"};

    int any = 0;
    for (int m = 0; m < n_motors; m++) if (step_counts[m] != 0) { any = 1; break; }
    if (!any) {
        printf("  " YELLOW "No movement required." RESET "\n");
        log_activity("DONE: Sequential | no steps | TTM=0 ms");
        printf("  " DIM "TTM (segments, sequential)" RESET "  " BABY_BLUE "0 ms" RESET "\n");
        return;
    }

    for (int m = 0; m < n_motors; m++) {
        if (step_counts[m] == 0) continue;

        log_activity("DRIVE: Motor %d (%s) | DIR=%s | Steps=%d",
                     m + 1, names[m], dirs[m] ? "CW" : "CCW", step_counts[m]);

        printf("  Motor %d  %-9s  %s  \xe2\x86\x92  %d steps\n",
               m + 1, names[m], dirs[m] ? "CW " : "CCW", step_counts[m]);

        double t0 = monotonic_seconds();

        gpio_write(dir_pins[m], dirs[m]);
        usleep(STEP_DELAY_US);
        for (int i = 0; i < step_counts[m]; i++) {
            gpio_write(step_pins[m], 1);
            usleep(STEP_DELAY_US);
            gpio_write(step_pins[m], 0);
            usleep(STEP_DELAY_US);
        }
        double elapsed = monotonic_seconds() - t0;
        char ttm_buf[32];
        format_ttm(ttm_buf, sizeof(ttm_buf), elapsed);

        log_activity("DONE: Motor %d (%s) | %d steps complete | TTM=%s",
                     m + 1, names[m], step_counts[m], ttm_buf);
        printf("  " DIM "TTM (segment, sequential)" RESET "  %-9s  " BABY_BLUE "%s" RESET "\n",
               names[m], ttm_buf);
    }
}

static void executeDriver(int *step_counts, int *dirs, int n_motors,
                          int *step_pins, int *dir_pins, int mode,
                          int return_to_menu) {
    clearScreen();
    printHeader("EXECUTING");

    printf("  Initializing GPIO pins...\n");
    gpio_init();

    if (mode == 1) {
        printf("  Driving motors simultaneously...\n\n");
        driveMotorsSimultaneous(step_counts, dirs, n_motors, step_pins, dir_pins);
    } else {
        printf("  Driving motors sequentially...\n\n");
        driveMotorsSequential(step_counts, dirs, n_motors, step_pins, dir_pins);
    }

    gpio_cleanup();

    printf("\n");
    printOk("All motors driven to target!");
    if (return_to_menu) {
        printf("\n  Press any key to return to Main Menu...\n");
        clearInputBuffer();
        getchar();
        menuStructure();
    }
}

void passAnglesToDriver(int mode) {
    int step_counts[3], dirs[3], step_pins[3], dir_pins[3];
    for (int m = 0; m < 3; m++) {
        dirs[m]        = (angles[m] >= 0) ? 1 : 0;
        step_counts[m] = (int)((double)abs(angles[m]) * STEPS_PER_REV / 360.0);
        step_pins[m]   = motorPins[m][0];
        dir_pins[m]    = motorPins[m][1];
    }
    executeDriver(step_counts, dirs, 3, step_pins, dir_pins, mode, 1);
}

void passRawToDriver(int mode) {
    int step_pins[3], dir_pins[3];
    for (int m = 0; m < 3; m++) {
        step_pins[m] = motorPins[m][0];
        dir_pins[m]  = motorPins[m][1];
    }
    executeDriver(rawSteps, rawDirs, 3, step_pins, dir_pins, mode, 1);
}


/* Core IK: returns 0 if out of reach, 1 on success. Fills angles_out[3]: base, shoulder_motor_deg, elbow_motor_deg. */
static int compute_ik(double L1, double L2, double x, double base_angle, double z, int angles_out[3]) {
    double dist = sqrt(x * x + z * z);
    if (dist > (L1 + L2) || dist < fabs(L1 - L2))
        return 0;

    double cosQ2 = (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (cosQ2 >  1.0) cosQ2 =  1.0;
    if (cosQ2 < -1.0) cosQ2 = -1.0;

    double q2 = acos(cosQ2);
    double q1 = atan2(z, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));

    angles_out[0] = (int)base_angle;
    angles_out[1] = ((int)(q1 * 180.0 / PI)) * GEARBOX_RATIO;
    angles_out[2] = ((int)(q2 * 180.0 / PI)) * GEARBOX_RATIO;
    return 1;
}

void InverseKinematicsCalculations(void) {
    clearScreen();
    printHeader("CALCULATING");
    printf("  Computing joint angles...\n\n");

    double L1   = (double)lengths[0];
    double L2   = (double)lengths[1];
    double x    = (double)targetcoords[0];
    double z    = (double)targetcoords[2];
    double dist = sqrt(x * x + z * z);

    if (!compute_ik(L1, L2, x, (double)targetcoords[1], z, angles)) {
        clearScreen();
        printHeader("OUT OF REACH");
        printErr("Target coordinates are out of reach.");
        printf("  Max reach      " BABY_BLUE "%.0f mm" RESET "\n", L1 + L2);
        printf("  Target dist    " RED       "%.0f mm" RESET "\n", dist);
        printf("\n  Press any key to re-enter coordinates...\n");
        clearInputBuffer();
        getchar();
        getTargetCoords();
        return;
    }

    clearScreen();
    printHeader("IK RESULTS");
    printOk("Arm can reach the target.");
    printf("\n");
    printf("  %-24s  " BABY_BLUE "%4d" RESET " deg\n",
           "Base (rotation)", angles[0]);
    printf("  %-24s  " BABY_BLUE "%4d" RESET " deg motor  "
           DIM "(%d deg joint)" RESET "\n",
           "Shoulder", angles[1], angles[1] / GEARBOX_RATIO);
    printf("  %-24s  " BABY_BLUE "%4d" RESET " deg motor  "
           DIM "(%d deg joint)" RESET "\n",
           "Elbow", angles[2], angles[2] / GEARBOX_RATIO);
    printf("\n  Press any key to see tick pattern...\n");
    clearInputBuffer();
    getchar();

    int step_counts[3];
    for (int m = 0; m < 3; m++)
        step_counts[m] = (int)((double)abs(angles[m]) * STEPS_PER_REV / 360.0);

    int mode = showTickPatternScreen(step_counts, 3);
    if (mode > 0) passAnglesToDriver(mode);
    else          menuStructure();
}

void getTargetCoords(void) {
    clearScreen();
    printHeader("TARGET COORDINATES");
    printf("  Enter target position of the end effector.\n");
    printf("  " DIM "x, z in mm  \xc2\xb7  y = base rotation in degrees  \xc2\xb7  format: x, y, z" RESET "\n\n");
    printPrompt();

    if (scanf("%d, %d, %d", &targetcoords[0], &targetcoords[1], &targetcoords[2]) != 3) {
        clearInputBuffer();
        printErr("Invalid format. Expected: x, y, z");
        getTargetCoords();
        return;
    }

    printDivider();
    printf("  %-18s  " BABY_BLUE "%d mm" RESET "\n",  "X (horizontal)",  targetcoords[0]);
    printf("  %-18s  " BABY_BLUE "%d\xc2\xb0" RESET "\n", "Base angle (Y)",   targetcoords[1]);
    printf("  %-18s  " BABY_BLUE "%d mm" RESET "\n",  "Z (height)",      targetcoords[2]);
    printf("\n");
    printf("  " DIM "[1]" RESET "  Confirm\n");
    printf("  " DIM "[2]" RESET "  Re-enter\n\n");
    printPrompt();

    scanf("%d", &choice);
    if (choice == 1) InverseKinematicsCalculations();
    else             getTargetCoords();
}

void getCoords(void) {
    clearScreen();
    printHeader("INITIAL POSITION");
    printf("  Enter the current arm position.\n");
    printf("  " DIM "x, z in mm  \xc2\xb7  y = base rotation in degrees  \xc2\xb7  format: x, y, z" RESET "\n\n");
    printPrompt();

    if (scanf("%d, %d, %d", &initcoords[0], &initcoords[1], &initcoords[2]) != 3) {
        clearInputBuffer();
        printErr("Invalid format. Expected: x, y, z");
        getCoords();
        return;
    }

    printDivider();
    printf("  %-18s  " BABY_BLUE "%d mm" RESET "\n",  "X (horizontal)", initcoords[0]);
    printf("  %-18s  " BABY_BLUE "%d\xc2\xb0" RESET "\n", "Base angle (Y)",  initcoords[1]);
    printf("  %-18s  " BABY_BLUE "%d mm" RESET "\n",  "Z (height)",     initcoords[2]);
    printf("\n");
    printf("  " DIM "[1]" RESET "  Confirm\n");
    printf("  " DIM "[2]" RESET "  Re-enter\n\n");
    printPrompt();

    scanf("%d", &choice);
    if (choice == 1) getTargetCoords();
    else             getCoords();
}

void customSecLenMenu(void) {
    clearScreen();
    printHeader("CUSTOM ARM LENGTHS");
    printf("  Enter segment lengths in millimetres.\n\n");

    int tempL1, tempL2;
    printf("  Lower segment (L1):  ");
    if (scanf("%d", &tempL1) != 1 || tempL1 <= 0) {
        clearInputBuffer();
        printErr("Invalid length.");
        customSecLenMenu();
        return;
    }
    printf("  Upper segment (L2):  ");
    if (scanf("%d", &tempL2) != 1 || tempL2 <= 0) {
        clearInputBuffer();
        printErr("Invalid length.");
        customSecLenMenu();
        return;
    }
    lengths[0] = tempL1;
    lengths[1] = tempL2;

    printDivider();
    printf("  L1  " BABY_BLUE "%d mm" RESET "\n", lengths[0]);
    printf("  L2  " BABY_BLUE "%d mm" RESET "\n\n", lengths[1]);

    getCoords();
}

void inverseKinematicsMenu(void) {
    clearScreen();
    printHeader("INVERSE KINEMATICS");
    printf("  " DIM "Default segment lengths:  L1 = 500 mm  \xc2\xb7  L2 = 525 mm" RESET "\n\n");
    printf("  " DIM "[1]" RESET "  Use default lengths\n");
    printf("  " DIM "[2]" RESET "  Set custom lengths\n\n");
    printPrompt();

    if (scanf("%d", &choice) != 1) { clearInputBuffer(); inverseKinematicsMenu(); return; }
    if (choice == 1)      getCoords();
    else if (choice == 2) customSecLenMenu();
    else                  inverseKinematicsMenu();
}

void getRawAngles(void) {
    static const char *names[]   = {"Base", "Shoulder", "Elbow"};

    clearScreen();
    printHeader("RAW ANGLE ENTRY");
    printf("  Choose which side of the gearbox to specify angles from:\n\n");
    printf("  " DIM "[1]" RESET "  Joint side   " DIM "(physical arm output — what you see)" RESET "\n");
    printf("  " DIM "[2]" RESET "  Motor side   " DIM "(motor shaft input — before gearbox)" RESET "\n\n");
    printPrompt();

    int angle_mode;
    if (scanf("%d", &angle_mode) != 1 || (angle_mode != 1 && angle_mode != 2)) {
        clearInputBuffer();
        getRawAngles();
        return;
    }

    clearScreen();
    printHeader("RAW ANGLE ENTRY");
    printf("  Mode  " BABY_BLUE "%s" RESET "\n",
           angle_mode == 1 ? "joint side" : "motor side");
    printf("  " DIM "Enter signed degrees  (negative = CCW)" RESET "\n\n");

    int raw_angle[3];
    for (int m = 0; m < 3; m++) {
        if (angle_mode == 1 && HAS_GEAR[m])
            printf("  Motor %d  %-9s  joint angle (deg):  ", m + 1, names[m]);
        else
            printf("  Motor %d  %-9s  motor angle (deg):  ", m + 1, names[m]);

        if (scanf("%d", &raw_angle[m]) != 1) {
            clearInputBuffer();
            printErr("Invalid input. Try again.");
            getRawAngles();
            return;
        }
    }

    for (int m = 0; m < 3; m++) {
        int motor_angle = (angle_mode == 1 && HAS_GEAR[m])
                          ? raw_angle[m] * GEARBOX_RATIO
                          : raw_angle[m];
        rawDirs[m]  = (motor_angle >= 0) ? 1 : 0;
        rawSteps[m] = (int)((double)abs(motor_angle) * STEPS_PER_REV / 360.0);
    }

    printDivider();
    for (int m = 0; m < 3; m++) {
        if (angle_mode == 1 && HAS_GEAR[m])
            printf("  Motor %d  %-9s  %4d\xc2\xb0 joint  \xe2\x86\x92  %4d\xc2\xb0 motor  \xe2\x86\x92  "
                   BABY_BLUE "%d steps" RESET "  " DIM "%s" RESET "\n",
                   m + 1, names[m], raw_angle[m],
                   raw_angle[m] * GEARBOX_RATIO,
                   rawSteps[m], rawDirs[m] ? "CW" : "CCW");
        else
            printf("  Motor %d  %-9s  %4d\xc2\xb0 motor  \xe2\x86\x92  "
                   BABY_BLUE "%d steps" RESET "  " DIM "%s" RESET "\n",
                   m + 1, names[m], raw_angle[m],
                   rawSteps[m], rawDirs[m] ? "CW" : "CCW");
    }
    printf("\n");
    printf("  " DIM "[1]" RESET "  Confirm\n");
    printf("  " DIM "[2]" RESET "  Re-enter\n\n");
    printPrompt();

    scanf("%d", &choice);
    if (choice != 1) { getRawAngles(); return; }

    int mode = showTickPatternScreen(rawSteps, 3);
    if (mode > 0) passRawToDriver(mode);
    else          menuStructure();
}

void getRawMotorData(void) {
    static const char *names[] = {"Base", "Shoulder", "Elbow"};

    clearScreen();
    printHeader("RAW STEP ENTRY");
    printf("  Enter step count and direction for each motor.\n");
    printf("  " DIM "Direction:  1 = CW  \xc2\xb7  0 = CCW" RESET "\n\n");

    for (int m = 0; m < 3; m++) {
        printf("  Motor %d  %-9s  steps:  ", m + 1, names[m]);
        if (scanf("%d", &rawSteps[m]) != 1 || rawSteps[m] < 0) {
            clearInputBuffer();
            printErr("Invalid step count. Must be a non-negative integer.");
            getRawMotorData();
            return;
        }
        printf("  Motor %d  %-9s  dir:    ", m + 1, names[m]);
        if (scanf("%d", &rawDirs[m]) != 1 || (rawDirs[m] != 0 && rawDirs[m] != 1)) {
            clearInputBuffer();
            printErr("Invalid direction. Enter 1 (CW) or 0 (CCW).");
            getRawMotorData();
            return;
        }
        printf("\n");
    }

    printDivider();
    for (int m = 0; m < 3; m++)
        printf("  Motor %d  %-9s  " BABY_BLUE "%d steps" RESET "  " DIM "%s" RESET "\n",
               m + 1, names[m], rawSteps[m], rawDirs[m] ? "CW" : "CCW");
    printf("\n");
    printf("  " DIM "[1]" RESET "  Confirm\n");
    printf("  " DIM "[2]" RESET "  Re-enter\n\n");
    printPrompt();

    scanf("%d", &choice);
    if (choice != 1) { getRawMotorData(); return; }

    int mode = showTickPatternScreen(rawSteps, 3);
    if (mode > 0) passRawToDriver(mode);
    else          menuStructure();
}

void rawMovementMenu(void) {
    clearScreen();
    printHeader("RAW MOVEMENT");
    printf("  Drive motors directly, bypassing inverse kinematics.\n\n");
    printf("  " DIM "[1]" RESET "  Enter step counts\n");
    printf("  " DIM "[2]" RESET "  Enter angles\n");
    printf("  " DIM "[3]" RESET "  Back to Main Menu\n\n");
    printPrompt();

    if (scanf("%d", &choice) != 1) { clearInputBuffer(); rawMovementMenu(); return; }
    if (choice == 1)      getRawMotorData();
    else if (choice == 2) getRawAngles();
    else if (choice == 3) menuStructure();
    else                  rawMovementMenu();
}

static void ensureScriptDir(void) {
    struct stat st;
    if (stat(SCRIPT_DIR, &st) != 0) {
        mkdir(SCRIPT_DIR, 0755);
    }
}

static void passAnglesToDriver_batch(int mode) {
    int step_counts[3], dirs[3], step_pins[3], dir_pins[3];
    for (int m = 0; m < 3; m++) {
        dirs[m]        = (angles[m] >= 0) ? 1 : 0;
        step_counts[m] = (int)((double)abs(angles[m]) * STEPS_PER_REV / 360.0);
        step_pins[m]   = motorPins[m][0];
        dir_pins[m]    = motorPins[m][1];
    }
    executeDriver(step_counts, dirs, 3, step_pins, dir_pins, mode, 0);
}

static void loadScriptConfig(const char *path, int *exec_mode_out, int *loop_forever_out) {
    *exec_mode_out    = SCRIPT_MODE_IK;
    *loop_forever_out = 0;

    FILE *f = fopen(path, "r");
    if (!f) return;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (isspace((unsigned char)*p)) p++;
        if (*p != '#')
            break;

        char mode_token[32];
        char loop_token[32];

        if (sscanf(p, "# MODE: %31s", mode_token) == 1) {
            if (strcmp(mode_token, "IK") == 0)
                *exec_mode_out = SCRIPT_MODE_IK;
            else if (strcmp(mode_token, "RAW_JOINT") == 0)
                *exec_mode_out = SCRIPT_MODE_RAW_JOINT;
            else if (strcmp(mode_token, "RAW_MOTOR") == 0)
                *exec_mode_out = SCRIPT_MODE_RAW_MOTOR;
        } else if (sscanf(p, "# LOOP: %31s", loop_token) == 1) {
            if (strcmp(loop_token, "ON") == 0)
                *loop_forever_out = 1;
            else
                *loop_forever_out = 0;
        }
    }

    fclose(f);
}

static int runScriptFile_once(const char *path, int exec_mode) {
    FILE *f = fopen(path, "r");
    if (!f) {
        printErr("Could not open script file.");
        printf("  " DIM "%s" RESET "\n", path);
        return 0;
    }

    clearScreen();
    printHeader("SCRIPT RUNNER");
    printf("  Script file: " BABY_BLUE "%s" RESET "\n", path);
    if (exec_mode == SCRIPT_MODE_IK)
        printf("  Mode: Inverse kinematics (x, y, z)\n");
    else if (exec_mode == SCRIPT_MODE_RAW_JOINT)
        printf("  Mode: Raw angles (joint side)\n");
    else
        printf("  Mode: Raw angles (motor side)\n");
    printf("  Using segment lengths: L1 = %d mm  \xc2\xb7  L2 = %d mm\n\n",
           lengths[0], lengths[1]);

    char line[256];
    int  line_no   = 0;
    int  step_idx  = 0;

    while (fgets(line, sizeof(line), f)) {
        line_no++;
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\0' || *p == '\n') continue;

        int vals[3];
        if ((exec_mode == SCRIPT_MODE_IK) &&
            (p[0] == 'I' || p[0] == 'i') &&
            (p[1] == 'K' || p[1] == 'k')) {
            p += 2;
            while (isspace((unsigned char)*p)) p++;
        }

        if (sscanf(p, "%d, %d, %d", &vals[0], &vals[1], &vals[2]) != 3) {
            printf("  " RED "[!]" RESET "  Line %d: could not parse. Skipping.\n", line_no);
            continue;
        }

        step_idx++;
        printf("\n");

        int step_counts[3];
        int dirs[3];
        int step_pins[3], dir_pins[3];
        for (int m = 0; m < 3; m++) {
            step_pins[m] = motorPins[m][0];
            dir_pins[m]  = motorPins[m][1];
        }

        if (exec_mode == SCRIPT_MODE_IK) {
            int x = vals[0], y = vals[1], z = vals[2];
            printf("  Step %d: target (x = %d mm, base = %d\xc2\xb0, z = %d mm)\n",
                   step_idx, x, y, z);

            int local_angles[3];
            double L1 = (double)lengths[0];
            double L2 = (double)lengths[1];
            double xx = (double)x;
            double zz = (double)z;
            double dist = sqrt(xx * xx + zz * zz);

            if (!compute_ik(L1, L2, xx, (double)y, zz, local_angles)) {
                printf("  " RED "[!]" RESET "  Target out of reach (dist = %.0f mm). Skipping.\n", dist);
                continue;
            }

            angles[0] = local_angles[0];
            angles[1] = local_angles[1];
            angles[2] = local_angles[2];

            printf("  Angles: base " BABY_BLUE "%d" RESET " deg  \xc2\xb7  "
                   "shoulder " BABY_BLUE "%d" RESET " deg motor  \xc2\xb7  "
                   "elbow " BABY_BLUE "%d" RESET " deg motor\n",
                   angles[0], angles[1], angles[2]);

            for (int m = 0; m < 3; m++) {
                dirs[m]        = (angles[m] >= 0) ? 1 : 0;
                step_counts[m] = (int)((double)abs(angles[m]) * STEPS_PER_REV / 360.0);
            }
        } else {
            static const char *names[] = {"Base", "Shoulder", "Elbow"};
            int raw_angle[3];
            raw_angle[0] = vals[0];
            raw_angle[1] = vals[1];
            raw_angle[2] = vals[2];

            printf("  Step %d: raw angles (deg): ", step_idx);
            printf("%s %d, %s %d, %s %d\n",
                   names[0], raw_angle[0],
                   names[1], raw_angle[1],
                   names[2], raw_angle[2]);

            for (int m = 0; m < 3; m++) {
                int motor_angle = (exec_mode == SCRIPT_MODE_RAW_JOINT && HAS_GEAR[m])
                                  ? raw_angle[m] * GEARBOX_RATIO
                                  : raw_angle[m];
                dirs[m]        = (motor_angle >= 0) ? 1 : 0;
                step_counts[m] = (int)((double)abs(motor_angle) * STEPS_PER_REV / 360.0);
            }

            printDivider();
            for (int m = 0; m < 3; m++) {
                int motor_angle = (exec_mode == SCRIPT_MODE_RAW_JOINT && HAS_GEAR[m])
                                  ? raw_angle[m] * GEARBOX_RATIO
                                  : raw_angle[m];
                if (exec_mode == SCRIPT_MODE_RAW_JOINT && HAS_GEAR[m])
                    printf("  Motor %d  %-9s  %4d\xc2\xb0 joint  \xe2\x86\x92  %4d\xc2\xb0 motor  \xe2\x86\x92  "
                           BABY_BLUE "%d steps" RESET "  " DIM "%s" RESET "\n",
                           m + 1, names[m], raw_angle[m],
                           motor_angle,
                           step_counts[m], dirs[m] ? "CW" : "CCW");
                else
                    printf("  Motor %d  %-9s  %4d\xc2\xb0 motor  \xe2\x86\x92  "
                           BABY_BLUE "%d steps" RESET "  " DIM "%s" RESET "\n",
                           m + 1, names[m], raw_angle[m],
                           step_counts[m], dirs[m] ? "CW" : "CCW");
            }
        }

        int mode = showTickPatternScreen(step_counts, 3);
        if (mode == 0) {
            printf("\n  Script cancelled by user.\n");
            fclose(f);
            return 0;
        }

        executeDriver(step_counts, dirs, 3, step_pins, dir_pins, mode, 0);
        printf("\n  Completed step %d.\n", step_idx);
    }

    fclose(f);

    printf("\n");
    printDivider();
    printf("  Script run complete. Total executed steps: " BABY_BLUE "%d" RESET "\n", step_idx);
    return 1;
}

static void runScriptFile(const char *filename) {
    char path[256];
    snprintf(path, sizeof(path), "%s/%s", SCRIPT_DIR, filename);

    int exec_mode, loop_forever;
    loadScriptConfig(path, &exec_mode, &loop_forever);

    clearScreen();
    printHeader("SCRIPT RUN");
    printf("  Script file: " BABY_BLUE "%s" RESET "\n", path);
    printf("  Mode: ");
    if (exec_mode == SCRIPT_MODE_IK)
        printf("Inverse kinematics (x, y, z)\n");
    else if (exec_mode == SCRIPT_MODE_RAW_JOINT)
        printf("Raw angles (joint side)\n");
    else
        printf("Raw angles (motor side)\n");
    printf("  Loop: %s\n\n", loop_forever ? "forever" : "once");

    int ok = 1;
    do {
        ok = runScriptFile_once(path, exec_mode);
        if (!ok) break;
        if (loop_forever) {
            printf("\n  " YELLOW "Restarting script from beginning (loop mode)." RESET "\n");
        }
    } while (loop_forever);

    printf("\n  Press any key to return to Script Library...\n");
    clearInputBuffer();
    getchar();
    scriptLibraryMenu();
}

static void createScriptInteractive(void) {
    clearScreen();
    printHeader("NEW SCRIPT");
    ensureScriptDir();

    char name[64];
    printf("  Enter script name (no spaces):  ");
    if (scanf("%63s", name) != 1) {
        clearInputBuffer();
        scriptLibraryMenu();
        return;
    }

    char filename[128];
    const char *ext = ".robo";
    size_t len = strlen(name);
    if (len >= 5 && strcmp(name + len - 5, ext) == 0) {
        snprintf(filename, sizeof(filename), "%s", name);
    } else {
        snprintf(filename, sizeof(filename), "%s%s", name, ext);
    }

    char path[256];
    snprintf(path, sizeof(path), "%s/%s", SCRIPT_DIR, filename);

    FILE *f = fopen(path, "w");
    if (!f) {
        printErr("Unable to create script file.");
        printf("  " DIM "%s" RESET "\n", path);
        printf("\n  Press any key to return to Script Library...\n");
        clearInputBuffer();
        getchar();
        scriptLibraryMenu();
        return;
    }

    fprintf(f, "# Roboscript (.robo) generated via UI\n");
    fprintf(f, "# Script name: %s\n", name);

    clearScreen();
    printHeader("NEW SCRIPT MODE");
    printf("  Script name: " BABY_BLUE "%s" RESET "\n\n", name);
    printf("  Choose how this script should be interpreted when run:\n\n");
    printf("  " DIM "[1]" RESET "  IK positions        (each line: x, y, z)\n");
    printf("  " DIM "[2]" RESET "  Raw angles (joint)  (each line: base, shoulder, elbow degrees)\n");
    printf("  " DIM "[3]" RESET "  Raw angles (motor)  (each line: base, shoulder, elbow motor degrees)\n\n");
    printPrompt();

    int mode_choice;
    if (scanf("%d", &mode_choice) != 1 || mode_choice < 1 || mode_choice > 3) {
        clearInputBuffer();
        printErr("Invalid mode.");
        fclose(f);
        remove(path);
        printf("\n  Press any key to return to Script Library...\n");
        clearInputBuffer();
        getchar();
        scriptLibraryMenu();
        return;
    }

    int exec_mode = SCRIPT_MODE_IK;
    if (mode_choice == 1)      exec_mode = SCRIPT_MODE_IK;
    else if (mode_choice == 2) exec_mode = SCRIPT_MODE_RAW_JOINT;
    else                       exec_mode = SCRIPT_MODE_RAW_MOTOR;

    clearScreen();
    printHeader("NEW SCRIPT LOOP MODE");
    printf("  Script name: " BABY_BLUE "%s" RESET "\n\n", name);
    printf("  Choose how this script should repeat when run:\n\n");
    printf("  " DIM "[1]" RESET "  Run once\n");
    printf("  " DIM "[2]" RESET "  Loop forever\n\n");
    printPrompt();

    int loop_choice;
    if (scanf("%d", &loop_choice) != 1 || (loop_choice != 1 && loop_choice != 2)) {
        clearInputBuffer();
        printErr("Invalid loop option.");
        fclose(f);
        remove(path);
        printf("\n  Press any key to return to Script Library...\n");
        clearInputBuffer();
        getchar();
        scriptLibraryMenu();
        return;
    }

    int loop_forever = (loop_choice == 2);

    fprintf(f, "# MODE: %s\n",
            exec_mode == SCRIPT_MODE_IK       ? "IK" :
            exec_mode == SCRIPT_MODE_RAW_JOINT ? "RAW_JOINT" : "RAW_MOTOR");
    fprintf(f, "# LOOP: %s\n", loop_forever ? "ON" : "OFF");

    if (exec_mode == SCRIPT_MODE_IK)
        fprintf(f, "# Each line: IK x, y, z\n\n");
    else
        fprintf(f, "# Each line: base_deg, shoulder_deg, elbow_deg\n\n");

    int count;
    if (exec_mode == SCRIPT_MODE_IK)
        printf("  How many points in this script?  ");
    else
        printf("  How many moves in this script?   ");
    if (scanf("%d", &count) != 1 || count <= 0) {
        clearInputBuffer();
        printErr("Invalid count.");
        fclose(f);
        remove(path);
        printf("\n  Press any key to return to Script Library...\n");
        clearInputBuffer();
        getchar();
        scriptLibraryMenu();
        return;
    }

    for (int i = 0; i < count; i++) {
        int x, y, z;
        if (exec_mode == SCRIPT_MODE_IK) {
            printf("  Point %d (x, y, z):  ", i + 1);
            if (scanf("%d, %d, %d", &x, &y, &z) != 3) {
                clearInputBuffer();
                printErr("Invalid format. Expected: x, y, z");
                fclose(f);
                remove(path);
                printf("\n  Press any key to return to Script Library...\n");
                clearInputBuffer();
                getchar();
                scriptLibraryMenu();
                return;
            }
            fprintf(f, "IK %d, %d, %d\n", x, y, z);
        } else {
            printf("  Target %d (base, shoulder, elbow degrees):  ", i + 1);
            if (scanf("%d, %d, %d", &x, &y, &z) != 3) {
                clearInputBuffer();
                printErr("Invalid format. Expected: base, shoulder, elbow");
                fclose(f);
                remove(path);
                printf("\n  Press any key to return to Script Library...\n");
                clearInputBuffer();
                getchar();
                scriptLibraryMenu();
                return;
            }
            fprintf(f, "%d, %d, %d\n", x, y, z);
        }
    }

    fclose(f);
    printDivider();
    printOk("Script saved.");
    printf("\n  Saved to: " BABY_BLUE "%s" RESET "\n", path);
    printf("\n  Press any key to return to Script Library...\n");
    clearInputBuffer();
    getchar();
    scriptLibraryMenu();
}

void scriptLibraryMenu(void) {
    ensureScriptDir();

    DIR *dir = opendir(SCRIPT_DIR);
    if (!dir) {
        clearScreen();
        printHeader("SCRIPT LIBRARY");
        printErr("Could not open script directory.");
        printf("  Expected at: " BABY_BLUE "%s" RESET "\n", SCRIPT_DIR);
        printf("\n  Press any key to return to Main Menu...\n");
        clearInputBuffer();
        getchar();
        menuStructure();
        return;
    }

    char filenames[64][256];
    int  count = 0;

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && count < 64) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        const char *dot = strrchr(entry->d_name, '.');
        if (!dot || strcmp(dot, ".robo") != 0)
            continue;

        snprintf(filenames[count], sizeof(filenames[count]), "%s", entry->d_name);
        count++;
    }
    closedir(dir);

    clearScreen();
    printHeader("SCRIPT LIBRARY");
    printf("  Standard script directory: " BABY_BLUE "%s" RESET "\n\n", SCRIPT_DIR);

    if (count == 0) {
        printf("  " YELLOW "No .robo scripts found." RESET "\n");
        printf("  Use the option below to create one.\n\n");
    } else {
        printf("  Available scripts:\n\n");
        for (int i = 0; i < count; i++) {
            printf("  " DIM "[%d]" RESET "  %s\n", i + 1, filenames[i]);
        }
        printf("\n");
    }

    int create_opt = count + 1;
    int delete_opt = count + 2;
    int back_opt   = count + 3;

    printf("  " DIM "[%d]" RESET "  New script (UI)\n", create_opt);
    printf("  " DIM "[%d]" RESET "  Delete a script\n", delete_opt);
    printf("  " DIM "[%d]" RESET "  Back to Main Menu\n\n", back_opt);
    printPrompt();

    if (scanf("%d", &choice) != 1) {
        clearInputBuffer();
        scriptLibraryMenu();
        return;
    }

    if (choice >= 1 && choice <= count) {
        runScriptFile(filenames[choice - 1]);
    } else if (choice == create_opt) {
        createScriptInteractive();
    } else if (choice == delete_opt) {
        if (count == 0) {
            printErr("No scripts to delete.");
            printf("  Press any key to return...\n");
            clearInputBuffer();
            getchar();
            scriptLibraryMenu();
            return;
        }
        printf("  Enter script number to delete (0 to cancel): ");
        int del;
        if (scanf("%d", &del) != 1) {
            clearInputBuffer();
            scriptLibraryMenu();
            return;
        }
        if (del <= 0 || del > count) {
            scriptLibraryMenu();
            return;
        }
        char path[256];
        snprintf(path, sizeof(path), "%s/%s", SCRIPT_DIR, filenames[del - 1]);
        if (remove(path) != 0) {
            printErr("Failed to delete script file.");
            printf("  " DIM "%s" RESET "\n", path);
        } else {
            printOk("Script deleted.");
        }
        printf("\n  Press any key to refresh Script Library...\n");
        clearInputBuffer();
        getchar();
        scriptLibraryMenu();
    } else if (choice == back_opt) {
        menuStructure();
    } else {
        scriptLibraryMenu();
    }
}

/* ----- Selftest: IK and tick patterns ----- */
#define SELFTEST_IK_TOLERANCE 20  /* allow ±20 motor deg (~1 joint deg) from rounding */

static int selftest_ik_one(double L1, double L2, int x, int base_angle, int z,
                           int expect_reach, int exp_base, int exp_shoulder, int exp_elbow,
                           const char *label, int *passed, int *failed) {
    int got[3];
    int reach = compute_ik(L1, L2, (double)x, (double)base_angle, (double)z, got);
    if (expect_reach && !reach) {
        printf("  " RED "[FAIL]" RESET "  %s  (expected reachable, got out of reach)\n", label);
        (*failed)++;
        return 0;
    }
    if (!expect_reach && reach) {
        printf("  " RED "[FAIL]" RESET "  %s  (expected out of reach, got angles)\n", label);
        (*failed)++;
        return 0;
    }
    if (!expect_reach && !reach) {
        printf("  " GREEN "[PASS]" RESET "  %s\n", label);
        (*passed)++;
        return 1;
    }
    int ok = (got[0] == exp_base)
          && (abs(got[1] - exp_shoulder) <= SELFTEST_IK_TOLERANCE)
          && (abs(got[2] - exp_elbow) <= SELFTEST_IK_TOLERANCE);
    if (!ok) {
        printf("  " RED "[FAIL]" RESET "  %s  got [%d,%d,%d] expected [%d,%d,%d]\n",
               label, got[0], got[1], got[2], exp_base, exp_shoulder, exp_elbow);
        (*failed)++;
        return 0;
    }
    printf("  " GREEN "[PASS]" RESET "  %s\n", label);
    (*passed)++;
    return 1;
}

static int selftest_tick_one(int *steps, int n_motors, const char *label,
                             int *passed, int *failed) {
    int *patterns[MAX_MOTORS];
    int section_len, reps;
    int ok = buildPatterns(steps, n_motors, patterns, &section_len, &reps);
    if (!ok) {
        printf("  " RED "[FAIL]" RESET "  %s  buildPatterns failed\n", label);
        (*failed)++;
        return 0;
    }
    if (section_len == 0 && reps == 0) {
        int all_zero = 1;
        for (int m = 0; m < n_motors; m++) if (steps[m] != 0) { all_zero = 0; break; }
        /* buildPatterns does not allocate when max_steps==0; do not free patterns[] */
        if (all_zero) {
            printf("  " GREEN "[PASS]" RESET "  %s  (all zero)\n", label);
            (*passed)++;
            return 1;
        }
    }
    int err = 0;
    for (int m = 0; m < n_motors; m++) {
        int sum = 0;
        for (int t = 0; t < section_len; t++) sum += patterns[m][t];
        if (sum * reps != steps[m]) {
            err = 1;
            printf("  " RED "[FAIL]" RESET "  %s  motor %d: pattern sum*reps=%d expected %d\n",
                   label, m + 1, sum * reps, steps[m]);
            break;
        }
    }
    if (section_len > 0) {
        for (int m = 0; m < n_motors; m++) free(patterns[m]);
    }
    if (err) { (*failed)++; return 0; }
    printf("  " GREEN "[PASS]" RESET "  %s  (section_len=%d reps=%d)\n", label, section_len, reps);
    (*passed)++;
    return 1;
}

void run_selftest(void) {
    clearScreen();
    printHeader("SELFTEST");
    printf("  Verifying inverse kinematics and tick pattern calculations.\n\n");

    int passed = 0, failed = 0;
    double L1 = 500.0, L2 = 525.0;

    printf("  " BABY_BLUE "Inverse kinematics (L1=500, L2=525 mm)" RESET "\n");
    /* Reachable cases: (x, z, base) -> (base, shoulder_motor, elbow_motor) */
    selftest_ik_one(L1, L2, 1025, 0, 0, 1, 0, 0, 0,
                   "Full stretch +x", &passed, &failed);
    selftest_ik_one(L1, L2, -1025, 0, 0, 1, 0, 2880, 0,
                   "Full stretch -x", &passed, &failed);
    selftest_ik_one(L1, L2, 0, 0, 1025, 1, 0, 1440, 0,
                   "Straight up (z only)", &passed, &failed);
    selftest_ik_one(L1, L2, 0, 0, -1025, 1, 0, -1440, 0,
                   "Straight down (negative z)", &passed, &failed);
    selftest_ik_one(L1, L2, 500, 0, 525, 1, 0, 0, 1440,
                   "90° elbow at (500,525)", &passed, &failed);
    selftest_ik_one(L1, L2, 25, 0, 0, 1, 0, -2880, 2880,
                   "Min reach folded (25,0)", &passed, &failed);  /* shoulder ±tolerance from truncation */
    selftest_ik_one(L1, L2, 1025, 90, 0, 1, 90, 0, 0,
                   "Base angle 90°", &passed, &failed);
    selftest_ik_one(L1, L2, 725, 0, 0, 1, 0, -736, 1440,
                   "Horizontal 725 mm (elbow 90°)", &passed, &failed);
    /* Out of reach */
    selftest_ik_one(L1, L2, 1500, 0, 0, 0, 0, 0, 0,
                   "Out of reach (too far)", &passed, &failed);
    selftest_ik_one(L1, L2, 0, 0, 0, 0, 0, 0, 0,
                   "Out of reach (origin)", &passed, &failed);
    selftest_ik_one(L1, L2, 1026, 0, 0, 0, 0, 0, 0,
                   "Out of reach (just beyond max)", &passed, &failed);
    selftest_ik_one(L1, L2, 24, 0, 0, 0, 0, 0, 0,
                   "Out of reach (just inside min)", &passed, &failed);
    /* Edge: equal segments */
    selftest_ik_one(400.0, 400.0, 800, 0, 0, 1, 0, 0, 0,
                   "Equal segments L1=L2=400 full stretch", &passed, &failed);
    selftest_ik_one(400.0, 400.0, 0, 0, 0, 1, 0, -1440, 2880,
                   "Equal segments origin (folded, L1=L2)", &passed, &failed);
    selftest_ik_one(525.0, 500.0, 1025, 0, 0, 1, 0, 0, 0,
                   "L1>L2 full stretch (525,500)", &passed, &failed);

    printf("\n  " BABY_BLUE "Tick patterns (Bresenham)" RESET "\n");
    int s0[3] = { 0, 0, 0 };
    selftest_tick_one(s0, 3, "All zeros", &passed, &failed);
    int s1[3] = { 100, 0, 0 };
    selftest_tick_one(s1, 3, "Single motor (100,0,0)", &passed, &failed);
    int s2[3] = { 0, 100, 0 };
    selftest_tick_one(s2, 3, "Single motor (0,100,0)", &passed, &failed);
    int s2b[3] = { 0, 0, 100 };
    selftest_tick_one(s2b, 3, "Single motor (0,0,100)", &passed, &failed);
    int s3[3] = { 100, 100, 100 };
    selftest_tick_one(s3, 3, "Equal (100,100,100)", &passed, &failed);
    int s4[3] = { 200, 200, 200 };
    selftest_tick_one(s4, 3, "Equal (200,200,200)", &passed, &failed);
    int s5[3] = { 7, 11, 13 };
    selftest_tick_one(s5, 3, "Coprime (7,11,13)", &passed, &failed);
    int s6[3] = { 16, 32, 48 };
    selftest_tick_one(s6, 3, "GCD=16 (16,32,48)", &passed, &failed);
    int s7[3] = { 1, 1, 1 };
    selftest_tick_one(s7, 3, "Minimal (1,1,1)", &passed, &failed);
    int s8[3] = { 1, 0, 0 };
    selftest_tick_one(s8, 3, "Single step (1,0,0)", &passed, &failed);
    int s8b[3] = { 0, 0, 1 };
    selftest_tick_one(s8b, 3, "Single step (0,0,1)", &passed, &failed);
    int s9[3] = { 1000, 500, 250 };
    selftest_tick_one(s9, 3, "Large (1000,500,250)", &passed, &failed);
    int s10[3] = { 17, 17, 17 };
    selftest_tick_one(s10, 3, "Equal prime (17,17,17)", &passed, &failed);
    int s11[3] = { 64, 128, 192 };
    selftest_tick_one(s11, 3, "GCD=64 (64,128,192)", &passed, &failed);
    int s12[3] = { 3, 6, 9 };
    selftest_tick_one(s12, 3, "Small GCD=3 (3,6,9)", &passed, &failed);

    /* Step conversion consistency: IK angles -> steps -> pattern total */
    printf("\n  " BABY_BLUE "IK -> steps -> pattern consistency" RESET "\n");
    int angles_test[3] = { 0, 1440, 720 };  /* base 0, shoulder 90° motor, elbow 45° motor */
    int step_counts[3];
    for (int m = 0; m < 3; m++)
        step_counts[m] = (int)((double)abs(angles_test[m]) * STEPS_PER_REV / 360.0);
    int *pat[MAX_MOTORS];
    int sec_len, reps;
    if (buildPatterns(step_counts, 3, pat, &sec_len, &reps)) {
        int cons_ok = 1;
        for (int m = 0; m < 3; m++) {
            int sum = 0;
            for (int t = 0; t < sec_len; t++) sum += pat[m][t];
            if (sum * reps != step_counts[m]) cons_ok = 0;
            free(pat[m]);
        }
        if (cons_ok) {
            printf("  " GREEN "[PASS]" RESET "  IK angles -> steps -> pattern totals match\n");
            passed++;
        } else {
            printf("  " RED "[FAIL]" RESET "  IK angles -> steps -> pattern totals mismatch\n");
            failed++;
        }
    } else {
        printf("  " RED "[FAIL]" RESET "  IK angles -> steps -> pattern build failed\n");
        failed++;
    }

    printDivider();
    if (failed == 0)
        printf("  " GREEN "All %d tests passed." RESET "\n", passed);
    else
        printf("  " RED "%d failed" RESET ", " GREEN "%d passed" RESET "\n", failed, passed);
    printf("\n  Press any key to return to Main Menu...\n");
    clearInputBuffer();
    getchar();
    menuStructure();
}

void menuStructure(void) {
    clearScreen();
    printHeader("MAIN MENU");
    printf("  " DIM "[1]" RESET "  Inverse Kinematics\n");
    printf("  " DIM "[2]" RESET "  Raw Movement\n");
    printf("  " DIM "[3]" RESET "  Self-test\n");
    printf("  " DIM "[4]" RESET "  Script Library / Batch Mode\n");
    printf("  " DIM "[5]" RESET "  Exit\n\n");
    printPrompt();

    if (scanf("%d", &choice) != 1) { clearInputBuffer(); menuStructure(); return; }
    if (choice == 1)      inverseKinematicsMenu();
    else if (choice == 2) rawMovementMenu();
    else if (choice == 3) run_selftest();
    else if (choice == 4) scriptLibraryMenu();
    else if (choice == 5) printGoodbye();
    else                  menuStructure();
}

int main(void) {
    printSplash();
    menuStructure();
    return 0;
}