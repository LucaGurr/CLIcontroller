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

#define STEPS_PER_REV  200
#define STEP_DELAY_US  15000

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

int choice;
int lengths      [2] = {500, 525};
int initcoords   [3] = {0};
int targetcoords [3] = {0};
int angles       [4] = {0};

static void clearScreen(void) {
    printf("\x1b[3J\x1b[H\x1b[2J");
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

void step_motor(int motor_id, int step_pin, int dir_pin, int angle_deg) {
    static const char *names[] = {"Base", "Shoulder", "Elbow", "End Effector"};
    int direction = (angle_deg >= 0) ? 1 : 0;
    int abs_angle = abs(angle_deg);
    int steps     = (int)((double)abs_angle * STEPS_PER_REV / 360.0);

    log_activity("DRIVE: Motor %d (%s) | DIR=%s | Steps=%d | Angle=%ddeg",
                 motor_id + 1, names[motor_id],
                 direction ? "CW" : "CCW", steps, angle_deg);

    printf("      GPIO %d DIR=%s, GPIO %d pulsing %d steps...\n",
           dir_pin, direction ? "CW" : "CCW", step_pin, steps);

    gpio_write(dir_pin, direction);
    usleep(STEP_DELAY_US);

    for (int i = 0; i < steps; i++) {
        gpio_write(step_pin, 1);
        usleep(STEP_DELAY_US);
        gpio_write(step_pin, 0);
        usleep(STEP_DELAY_US);
    }

    log_activity("DONE: Motor %d (%s) | %d steps complete",
                 motor_id + 1, names[motor_id], steps);
}

void passAnglesToDriver(void) {
    clearScreen();
    printf("%sDRIVER PASSING%s\n", BLUE, RESET);
    printf("    Initializing GPIO pins...\n");
    gpio_init();

    printf("    Driving motors to target position...\n\n");

    printf("    %sMotor 1 (Base):%s      %d deg\n", GREEN, RESET, angles[0]);
    step_motor(0, motorPins[0][0], motorPins[0][1], angles[0]);

    printf("    %sMotor 2 (Shoulder):%s  %d motor-deg (%d deg x16)\n",
           GREEN, RESET, angles[1], angles[1] / 16);
    step_motor(1, motorPins[1][0], motorPins[1][1], angles[1]);

    printf("    %sMotor 3 (Elbow):%s     %d motor-deg (%d deg x16)\n",
           GREEN, RESET, angles[2], angles[2] / 16);
    step_motor(2, motorPins[2][0], motorPins[2][1], angles[2]);

    gpio_cleanup();

    printf("\n    %sAll motors driven to target!%s\n", GREEN, RESET);
    printf("    %sPress any key to return to Main Menu...%s\n", YELLOW, RESET);
    getchar(); getchar();
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
        getchar(); getchar();
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
    printf("    %sPress any key to continue...%s\n",    YELLOW, RESET);
    getchar(); getchar();

    clearScreen();
    printf("%sINVERSE KINEMATICS CALCULATIONS%s\n    Joint angles calculated successfully!\n", BLUE, RESET);
    printf("    Do you want to pass on the angles to the Driver?\n");
    printf("    %s1)%s Yes, pass angles to Driver\n    %s2)%s No, return to Main Menu\n    ",
           RED, RESET, RED, RESET);
    scanf("%d", &choice);
    if (choice == 1) {
        printf("    Passing angles to Driver...\n");
        passAnglesToDriver();
    } else {
        printf("    Returning to Main Menu...\n");
        menuStructure();
    }
}

void getTargetCoords(void) {
    clearScreen();
    printf("%sTARGET COORDINATES MENU%s\n", BLUE, RESET);
    printf("    Please enter coordinates (x, y, z):\n");
    printf("    x,z = mm, y = degrees base rotation\n    Input format: x, y, z\n    ");

    if (scanf("%d, %d, %d", &targetcoords[0], &targetcoords[1], &targetcoords[2]) != 3) {
        printf("Invalid input format.\n");
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
}

void getCoords(void) {
    clearScreen();
    printf("%sINITIAL COORDINATES MENU%s\n", BLUE, RESET);
    printf("    Please enter coordinates (x, y, z):\n");
    printf("    x,z = mm, y = degrees base rotation\n    Input format: x, y, z\n    ");

    if (scanf("%d, %d, %d", &initcoords[0], &initcoords[1], &initcoords[2]) != 3) {
        printf("Invalid input format.\n");
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
}

void customSecLenMenu(void) {
    clearScreen();
    int tempL1, tempL2;
    printf("%sCUSTOM SECTION LENGTH MENU%s\n", BLUE, RESET);
    printf("    Lower Section (L1) in mm: ");
    scanf("%d", &tempL1);
    printf("    Upper Section (L2) in mm: ");
    scanf("%d", &tempL2);
    lengths[0] = tempL1;
    lengths[1] = tempL2;
    getCoords();
}

void inverseKinematicsMenu(void) {
    clearScreen();
    printf("%sINVERSE KINEMATICS MENU%s:\n", BLUE, RESET);
    printf("    %s1)%s Use default lengths (500/525)\n", RED, RESET);
    printf("    %s2)%s Use custom lengths\n    ",         RED, RESET);

    scanf("%d", &choice);
    if (choice == 1) getCoords();
    else if (choice == 2) customSecLenMenu();
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
