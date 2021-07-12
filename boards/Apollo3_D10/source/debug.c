/************************************************************************************************************
 Module:       xxxx.c
 
 Description:  Contains debug utility functions
 
 Notes:        ---
 
 History:
 Date          Name      Changes
 -----------   ----      -------------------------------------------------------------------------------------
 3/18/2018     xxx       Began coding

 ************************************************************************************************************/

#include "am_util.h"
#include "board_def.h"
#include "debug.h"
#include "main.h"

//###########################################################################################################
//      CONSTANT DEFINITION
//###########################################################################################################
#define ABQ_UART_DEBUG_MODULE             (0)
//###########################################################################################################
//      TYPEDEF DEFINITION
//###########################################################################################################



//###########################################################################################################
//      Module Level Variables
//###########################################################################################################

uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[2];

const am_hal_uart_config_t uart_debug_cfg =
{
  //
  // Standard UART settings: 115200-8-N-1
  //
  .ui32BaudRate = 115200,
  .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
  .ui32Parity = AM_HAL_UART_PARITY_NONE,
  .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
  .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

  //
  // Set TX and RX FIFOs to interrupt at half-full.
  //
  .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                      AM_HAL_UART_RX_FIFO_1_2),

  //
  // Buffers
  //
  .pui8TxBuffer = g_pui8TxBuffer,
  .ui32TxBufferSize = sizeof(g_pui8TxBuffer),
  .pui8RxBuffer = g_pui8RxBuffer,
  .ui32RxBufferSize = sizeof(g_pui8RxBuffer),
};

void *Uart_Debug_Handler;

void uart_print(char *pcStr);
//###########################################################################################################
//      PUBLIC FUNCTION DEFINITION
//###########################################################################################################
void Debug_Init() {
#if ENABLE_DEBUG_PRINTF
  uint32_t ui32Status, ui32Idle;
  am_hal_gpio_pincfg_t uart_pincfg;
  am_hal_uart_initialize(ABQ_UART_DEBUG_MODULE, &Uart_Debug_Handler);
  am_hal_uart_power_control(Uart_Debug_Handler, AM_HAL_SYSCTRL_WAKE, false);
  am_hal_uart_configure(Uart_Debug_Handler, &uart_debug_cfg);


  am_hal_uart_interrupt_status_get(Uart_Debug_Handler, &ui32Status, true);
  am_hal_uart_interrupt_clear(Uart_Debug_Handler, ui32Status);
  am_hal_uart_interrupt_service(Uart_Debug_Handler, ui32Status, &ui32Idle);
  NVIC_SetPriority((IRQn_Type)(UART0_IRQn + ABQ_UART_DEBUG_MODULE), 6);
  NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + ABQ_UART_DEBUG_MODULE));

  uart_pincfg = g_AM_BSP_GPIO_COM_UART_TX;
  uart_pincfg.uFuncSel = AM_HAL_PIN_39_UART0TX;
  am_hal_gpio_pinconfig(ABQ_UART_DEBUG_UART_TX_PIN, uart_pincfg);
  uart_pincfg = g_AM_BSP_GPIO_COM_UART_RX;
  uart_pincfg.uFuncSel = AM_HAL_PIN_40_UART0RX;
  am_hal_gpio_pinconfig(ABQ_UART_DEBUG_UART_RX_PIN, uart_pincfg);

  am_util_stdio_printf_init(uart_print);
#endif
  //Init IO pins for debugging
  am_hal_gpio_pinconfig(ABQ_DEBUG_1_PIN,g_AM_HAL_GPIO_OUTPUT);
  am_hal_gpio_pinconfig(ABQ_DEBUG_2_PIN,g_AM_HAL_GPIO_OUTPUT);
  am_hal_gpio_pinconfig(ABQ_DEBUG_3_PIN,g_AM_HAL_GPIO_OUTPUT);
  am_hal_gpio_output_clear(ABQ_DEBUG_1_PIN);
  am_hal_gpio_output_clear(ABQ_DEBUG_2_PIN);
  am_hal_gpio_output_clear(ABQ_DEBUG_3_PIN);
}

void Debug_SetDebugPin1(uint8_t val) {
  if(val) am_hal_gpio_output_set(ABQ_DEBUG_1_PIN);
  else am_hal_gpio_output_clear(ABQ_DEBUG_1_PIN);
}
void Debug_SetDebugPin2(uint8_t val) {
  if(val) am_hal_gpio_output_set(ABQ_DEBUG_2_PIN);
  else am_hal_gpio_output_clear(ABQ_DEBUG_2_PIN);
}
void Debug_SetDebugPin3(uint8_t val) {
  if(val) am_hal_gpio_output_set(ABQ_DEBUG_3_PIN);
  else am_hal_gpio_output_clear(ABQ_DEBUG_3_PIN);
}
//###########################################################################################################
//      PRIVATE FUNCTION DEFINITION
//###########################################################################################################
//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(Uart_Debug_Handler, &ui32Status, true);
    am_hal_uart_interrupt_clear(Uart_Debug_Handler, ui32Status);
    am_hal_uart_interrupt_service(Uart_Debug_Handler, ui32Status, &ui32Idle);
}

//*****************************************************************************
//
// UART print string
//
//*****************************************************************************
void
uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;
    int ret;

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    ret = am_hal_uart_transfer(Uart_Debug_Handler, &sUartWrite);
    if(ret)
    {
      while (1);
    }

    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while(1);
    }
}