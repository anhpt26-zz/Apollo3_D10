#ifndef _TEST_TASK_H_
#define _TEST_TASK_H_

typedef enum 
{
  TESTTASK_SIGNAL_EVT_HW_TIMEOUT = 0,
  TESTTASK_SIGNAL_EVT_PIN_TOGGLE,
  TESTTASK_SIGNAL_EVT_MAX,
}testtask_signal_evt_t;

void TestTask_Init(void);
void TestTask_Task(void *pvParameter);


#endif //_TEST_TASK_H_