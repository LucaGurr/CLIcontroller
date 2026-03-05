#include "stdio.h"

void clearScreen(){
    printf("\x1b[3J\x1b[H\x1b[2J");
}


void inverseKinematicsMenu() {

    clearScreen();

    printf("INVERSE KINEMATICS MENU:\n    1) Use defauls section Lengths\n    2) Use custom section Lengths\n");

}

void rawDataMenu(){

    clearScreen();

    printf("RAW DATA MENU:\n    1) Use All Motors\n    2) Use custom Motors\n");

}


void menuStructure(){
    int choice;

    clearScreen();

    printf("MAIN MENU\n    1) choose inverse Kinematics mode\n    2) choose raw movement data mode\n");
    if (scanf("%d", &choice) == 1) {
        inverseKinematicsMenu();
    } 
    else if (scanf("%d", &choice) == 2) {
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