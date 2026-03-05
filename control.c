#include "stdio.h"

#define BLUE "\x1b[34m"
#define RED "\x1b[31m"
#define GREEN "\x1b[32m"
#define YELLOW "\x1b[33m"
#define RESET "\x1b[0m"

int choice;

int lengths[2] = {500, 525};

void clearScreen(){
    printf("\x1b[3J\x1b[H\x1b[2J");
}

void defaultSecLenMenu(){
    clearScreen();
}

void customSecLenMenu() {
    clearScreen();

    int tempLengths[2] = {0};

    printf("%sCUSTOM SECTION LENGTH MENU%s\n    Please enter the lower section length below (mm):\n    ", BLUE, RESET);
    scanf("%d", &tempLengths[0]);

    clearScreen();

    printf("%sCUSTOM SECTION LENGTH MENU%s\n    Please enter the upper section length below (mm):\n    ", BLUE, RESET);
    scanf("%d", &tempLengths[1]);

    clearScreen();
    printf("%sCUSTOM SECTION LENGTH MENU%s\n    You have entered the following section lengths:\n    %sLower Section Length:%s %d mm\n    %sUpper Section Length:%s %d mm\n    %s1)%s Confirm and continue\n    %s2)%s Re-enter section lengths\n    ", BLUE, RESET, RED, RESET, tempLengths[0], RED, RESET, tempLengths[1], RED, RESET);
    if (scanf("%d", &choice) == 1 && choice == 1) {
        lengths[0] = tempLengths[0];
        lengths[1] = tempLengths[1];
    }
    else if (choice == 2) {
        customSecLenMenu();
    }
    else {
        clearScreen();
        printf("navn\n");
    }
}


void inverseKinematicsMenu() {

    clearScreen();

    printf("%sINVERSE KINEMATICS MENU%s:\n    %s1)%s Use defauls section Lengths\n    %s2)%s Use custom section Lengths\n    ", BLUE, RESET, RED, RESET, RED, RESET);
    if (scanf("%d", &choice) == 1 && choice == 1) {
        defaultSecLenMenu();
    } 
    else if (choice == 2) {
        customSecLenMenu();
    }
    else {
        printf("navn\n");
    }

}

void rawDataMenu(){

    clearScreen();

    printf("%sRAW DATA MENU%s:\n    %s1)%s Use All Motors\n    %s2)%s Use custom Motors\n    ", BLUE, RESET, RED, RESET, RED, RESET);

    if (scanf("%d", &choice) == 1 && choice == 1) {
        
    } 
    else if (choice == 2) {

    }
    else {
        clearScreen();
        printf("navn\n");
    }

}


void menuStructure(){

    clearScreen();

    printf("%sMAIN MENU%s\n    %s1)%s choose inverse Kinematics mode\n    %s2)%s choose raw movement data mode\n    ", BLUE, RESET, RED, RESET, RED, RESET);
    if (scanf("%d", &choice) == 1 && choice == 1) {
        inverseKinematicsMenu();
    } 
    else if (choice == 2) {
        rawDataMenu();
    }
    else {
        printf("navn\n");
    }
}

int main() {
    
    menuStructure();

    return 0;
}