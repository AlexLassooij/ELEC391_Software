#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "ISR_operations.h"

int msec = 0, trigger = 10; /* 10ms */
clock_t before = clock();


void ISR(void)
{ 
   flag=1; 
}


while (1) {
    print("SCARA Robot Running");
    clock_t difference = clock() - before;
    msec = difference * 1000 / CLOCKS_PER_SEC;
    if (msec > trigger) {
        ISR();
        msec = 0;
    }
}


int flag = 0;

#pragma interrupt_handler ISR

void main() 
{
   while(1) 
   { 
      /* Wait for the ISR to set the 
      * flag; reset it before 
      * taking any action. */ 
      if (flag) 
      { 
         ISR();
         flag = 0; 
         /* Perform the required action here */ 
      } 
   } 
} 