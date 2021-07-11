#ifndef _BOARD_DEF_H_
#define _BOARD_DEF_H_

#include "main.h"

#define ABQ_D10_SPI_MODULE      (5)     //IOM module used for communicate with External DPS D10
#define ABQ_D10_MOSI_PIN        (47)    //P13A_11   --DPS_PIN  MOSI-GPIO4 
#define ABQ_D10_MISO_PIN        (49)    //P13A_7    --DPS_PIN  MISO-GPIO5  
#define ABQ_D10_SCK_PIN         (48)    //P13A_9    --DPS_PIN  SCK-GPIO3
#define ABQ_D10_CS_PIN          (46)    //P13A_13   --DPS_PIN  CS-GPIO2
#define ABQ_D10_INT_PIN         (58)    //P13B_16   --DPS_PIN  INT-GPIO14 
#define ABQ_D10_WAKEUP_PIN      (59)    //P13B_14   --DPS_PIN  WAKE-GPIO15 
#define ABQ_D10_RDY_PIN         (60)    //P13B_12   --DPS_PIN  DSPRDY-GPIO10 
#define ABQ_D10_RST_PIN         (61)    //P11B_10   --DPS_PIN  RST 
#define ABQ_D10_MCLK_PIN        (7)     //P11A_7    --DPS_PIN  MSTCLK

#define ABQ_TEST_TASK_PIN_1     (44)    
#define ABQ_TEST_TASK_PIN_2     (43)    


#define ABQ_DEBUG_1_PIN         (36)    //P17A_13
#define ABQ_DEBUG_2_PIN         (37)    ///P17A_11
#define ABQ_DEBUG_3_PIN         (38)    ///P17A_9


#endif //_BOARD_DEF_H_