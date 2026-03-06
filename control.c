#include <stdio.h>
#include <math.h>

#define BLUE "\x1b[34m"
#define RED "\x1b[31m"
#define GREEN "\x1b[32m"
#define YELLOW "\x1b[33m"
#define RESET "\x1b[0m"
#define PI 3.14159265358979323846

void menuStructure();
void getTargetCoords();

int choice;

int lengths         [2] = {[0] = 500, [1] = 525};
int initcoords      [3] = {0};
int targetcoords    [3] = {0};
int motors          [4] = {0}; // replacement for bool 0 = false/nC 1 = true/iC
int angles          [4] = {0}; // [0]: Base, [1]: Shoulder, [2]: Elbow ,[3]: End effector

void clearScreen(){
    printf("\x1b[3J\x1b[H\x1b[2J");
}

void passAnglesToDriver() {
    clearScreen();
    printf("%sDRIVER PASSING%s", BLUE, RESET);
    printf("    (Simulated) Passing angles to Driver...\n");
    printf("    Base: %d deg, Shoulder: %d deg, Elbow: %d deg\n", angles[0], angles[1], angles[2]);
    printf("    %sPress any key to return to Main Menu...%s\n", YELLOW, RESET);
    getchar(); getchar();
    menuStructure();
}

void InverseKinematicsCalculations(){
    clearScreen();
    printf("%sINVERSE KINEMATICS CALCULATIONS%s\n", BLUE, RESET);
    printf("    Calculating the joint angles for the given target coordinates...\n\n");

    double L1 =             (double)lengths[0];
    double L2 =             (double)lengths[1];
    
    double x =              (double)targetcoords[0];
    double baseAngle =      (double)targetcoords[1];
    double z =              (double)targetcoords[2];

    double dist = sqrt(x * x + z * z);


    //OOR handler
    if (dist > (L1 + L2) || dist < fabs(L1 - L2)) {
        clearScreen();
        printf("%sOOR EXCEPTION%s", RED, RESET);
        printf("%sERROR:%s Target (%.0f, %.0f) out of reach!\n", RED, RESET, x, z);
        printf("    Max reach: %.0f mm, Target distance: %.2f mm\n", L1 + L2, dist);
        printf("    Press any key to return...");
        getchar(); getchar();
        getTargetCoords();
        return;
    }

    // cos(q2) = (dist^2 - L1^2 - L2^2) / (2 * L1 * L2)
    double cosQ2 = (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    // Clamp to valid acos domain [-1, 1] to guard against floating-point drift
    if (cosQ2 > 1.0)  cosQ2 =  1.0;
    if (cosQ2 < -1.0) cosQ2 = -1.0;

    double q2 = acos(cosQ2);

    double q1 = atan2(z, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));

    angles[0] = (int)baseAngle;                     //gearbox scaling(not needed for base)      
    angles[1] = ((int)(q1 * 180.0 / PI)) * 16;      //shoulder needs 16:1 gearbox scaling
    angles[2] = ((int)(q2 * 180.0 / PI)) * 16;      //elbow needs 16:1 gearbox scaling

    clearScreen();
    printf("%sINVERSE KINEMATICS CALCULATIONS%s", BLUE, RESET);
    printf("    %sCalculated Angles:%s\n", GREEN, RESET);
    printf("    Base (Rot): %d deg\n", angles[0]);
    printf("    Shoulder:   %d deg\n", angles[1]);
    printf("    Elbow:      %d deg\n\n", angles[2]);
    printf("    %sJoint angles are within limits.%s\n", GREEN, RESET);
    printf("    %sArm can reach the target!%s\n", GREEN, RESET);
    printf("    %sPress any key to continue...%s\n", YELLOW, RESET);
    getchar(); getchar();
    
    clearScreen();
    printf("%sINVERSE KINEMATICS CALCULATIONS%s\n    Joint angles calculated successfully!\n", BLUE, RESET);
    printf("    Do you want to pass on the angles to the Driver?\n");
    printf("    %s1)%s Yes, pass angles to Driver\n    %s2)%s No, return to Main Menu\n    ", RED, RESET, RED, RESET);
    scanf("%d", &choice);
    if (choice == 1) {
        printf("    Passing angles to Driver...\n");
        passAnglesToDriver();
    } 
    else {
        printf("    Returning to Main Menu...\n");
        menuStructure();
    }
}

void getTargetCoords(){
    clearScreen();
    printf("%sTARGET COORDINATES MENU%s\n", BLUE, RESET);
    printf("    Please enter coordinates (x, y, z):\n");
    printf("    x,z = mm, y = degrees base rotation\n    Input format: x, y, z\n    ");
    
    if(scanf("%d, %d, %d", &targetcoords[0], &targetcoords[1], &targetcoords[2]) != 3) {
        printf("Invalid input format.\n");
        return;
    }

    printf("\n    %sX (Horizontal):%s %d mm\n", RED, RESET, targetcoords[0]);
    printf("    %sBase Angle (Y):%s %d deg\n", RED, RESET, targetcoords[1]);
    printf("    %sZ (Height):    %s %d mm\n", RED, RESET, targetcoords[2]);
    printf("\n    %s1)%s Confirm and continue\n    %s2)%s Re-enter\n    ", RED, RESET, RED, RESET);
    
    scanf("%d", &choice);
    if (choice == 1) InverseKinematicsCalculations();
    else if (choice == 2) getTargetCoords();
}

void getCoords(){
    clearScreen();
    printf("%sINITIAL COORDINATES MENU%s\n", BLUE, RESET);
    printf("    Please enter coordinates (x, y, z):\n");
    printf("    x,z = mm, y = degrees base rotation\n    Input format: x, y, z\n    ");

    if(scanf("%d, %d, %d", &initcoords[0], &initcoords[1], &initcoords[2]) != 3) {
        printf("Invalid input format.\n");
        return;
    }

    printf("\n    %sX (Horizontal):%s %d mm\n", RED, RESET, initcoords[0]);
    printf("    %sBase Angle (Y):%s %d deg\n", RED, RESET, initcoords[1]);
    printf("    %sZ (Height):    %s %d mm\n", RED, RESET, initcoords[2]);
    printf("\n    %s1)%s Confirm and continue\n    %s2)%s Re-enter\n    ", RED, RESET, RED, RESET);

    scanf("%d", &choice);
    if (choice == 1) getTargetCoords();
    else if (choice == 2) getCoords();
}

void customSecLenMenu() {
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

void inverseKinematicsMenu() {
    clearScreen();
    printf("%sINVERSE KINEMATICS MENU%s:\n", BLUE, RESET);
    printf("    %s1)%s Use default lengths (500/525)\n", RED, RESET);
    printf("    %s2)%s Use custom lengths\n    ", RED, RESET);
    
    scanf("%d", &choice);
    if (choice == 1) getCoords();
    else if (choice == 2) customSecLenMenu();
}

void menuStructure(){
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

int main() {
    menuStructure();
    clearScreen();
    return 0;
}