//
// Created by ltyridium on 2023/4/22.
//

#include "State_machine.h"
#include "../BoardHAL/BoardHAL.h"
#include "../APP/PFC.h"



//background while task
void (*alphaTaskPtr)(void);
//1000Hz
void (*aTaskPtr)(void);
//200Hz
void (*bTaskPtr)(void);


// 1000hz
void PFC_fsmATask(void)
{
  {
    //A task code here
#if LAB == 5
    PFC_autoStartPFC();
#endif

#if LAB == 6
    PFC_autoStartPFC();
#endif

#if LAB == 7
    PFC_autoStartPFC();
#endif

  }
  aTaskPtr = PFC_fsmATask;
}

void PFC_fsmBTask(void)
{
  {
    //B task code here

  }
  bTaskPtr = PFC_fsmBTask;
}

