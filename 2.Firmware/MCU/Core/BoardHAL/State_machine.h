//
// Created by ltyridium on 2023/4/22.
//

#ifndef APF_STATE_MACHINE_H
#define APF_STATE_MACHINE_H


//background while task
extern void (*alphaTaskPtr)(void);
//1000Hz
extern void (*aTaskPtr)(void);
//200Hz
extern void (*bTaskPtr)(void);

extern void PFC_fsmBTask(void);
extern void PFC_fsmATask(void);

#endif //APF_STATE_MACHINE_H
