/*
 *  ======== egh456_team15 ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/gates/GateHwi.h>
#include <ti/sysbios/gates/GateSwi.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Standard Header Files */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <inc/hw_ints.h>


/* driverlib headers */
#include <driverlib/fpu.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>

/* grlib headers */
#include <grlib/grlib.h>
#include <grlib/widget.h>
#include <grlib/canvas.h>
#include <grlib/container.h>
#include <grlib/pushbutton.h>

/* Touchscreen Headers */
#include <drivers/Kentec320x240x16_ssd2119_spi.h>
#include <drivers/touch.h>
#include <drivers/pinout.h>

/* Board Header file */
#include <Board.h>

/* Motor Library */
#include <motorlib.h>

#define TASKSTACKSIZE   1024

/* Function Declarations */
void OnNext(tWidget *psWidget);
void OnPrevious(tWidget *psWidget);
void DrawPlots();
void drawSpeedPoint();
void drawCurrentPoint();
void StartStopBttnPress(tWidget *psWidget);
void heartBeatFxn(UArg arg0, UArg arg1);
void UARTFxn(UART_Handle handle, void *rxBuf, size_t size);
void toggleMotor(UArg arg0, UArg arg1);
void SwiUp(UArg arg0, UArg arg1);
void SwiDown(UArg arg0, UArg arg1);
void SwiRight(UArg arg0, UArg arg1);
void SwiLeft(UArg arg0, UArg arg1);
void SwiEnter(UArg arg0, UArg arg1);
void initScreen();
void initUART();

/* Global Variables */
int motorSpeed = 0;
int prevMotorSpeed = 0;
int plotLeft = 0;
uint8_t motorOn = 0;

int motorCurrent = 0;
int prevMotorCurrent = 0;

tCanvasWidget     g_sBackground;
tPushButtonWidget g_sStartStopBttn;
tContext sContext;
tRectangle sRect;

UART_Params uartParams;
UART_Handle uart;
uint32_t wantedRxBytes = 1;
uint8_t rxBuf[8];        // Receive buffer

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

GateHwi_Handle gateHwi;
GateHwi_Params gHwiprms;

Hwi_Handle TouchScreenIntHandlerHandle;
Hwi_Params hwiParams;

Swi_Params swiParams;
Swi_Handle SwiDownHandle;
Swi_Handle SwiUpHandle;
Swi_Handle SwiRightHandle;
Swi_Handle SwiLeftHandle;
Swi_Handle SwiEnterHandle;
Swi_Handle SwiMotorToggle;

// The canvas widget acting as the background to the display.
Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sStartStopBttn,
       &g_sKentec320x240x16_SSD2119, 50, 190, 220, 50,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE, 0, 0, ClrSilver,
       &g_sFontCm20, 0, 0, 0);

RectangularButton(g_sStartStopBttn, &g_sBackground, 0, 0,
                  &g_sKentec320x240x16_SSD2119, 50, 200, 100, 25,
                  (PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT |
                   PB_STYLE_FILL | PB_STYLE_RELEASE_NOTIFY),
                   ClrDarkBlue, ClrBlue, ClrWhite, ClrWhite,
                   g_psFontCmss16b, "Start", 0, 0, 0, 0, StartStopBttnPress);

void OnNext(tWidget *psWidget){
    //
}

void OnPrevious(tWidget *psWidget){
    //
}

void DrawPlots() {
    sRect.i16XMin = 0;
    sRect.i16YMin = 23;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 103;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    sRect.i16XMin = 0;
    sRect.i16YMin = 103;
    sRect.i16XMax =GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 183;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "Current", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 29, 0);
    GrStringDrawCentered(&sContext, "Speed", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 111, 0);
}

void drawSpeedPoint(){
    GrLineDraw(&sContext, plotLeft,183 - (prevMotorSpeed/65),plotLeft + 5 ,183 - (motorSpeed/65));
    GrStringDrawCentered(&sContext, "Speed", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 111, 0);
}

void drawCurrentPoint(){
    GrLineDraw(&sContext, plotLeft,103 - (motorCurrent/65),plotLeft + 5 ,103 - (motorCurrent/65));
    GrStringDrawCentered(&sContext, "Speed", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 111, 0);
}

void StartStopBttnPress(tWidget *psWidget)
{
    motorOn = !motorOn;

    if(motorOn)
    {
        //
        // Change the button text to indicate the new function.
        //
        PushButtonTextSet(&g_sStartStopBttn, "Stop");

        //
        // Repaint the pushbutton and all widgets beneath it (in this case,
        // the welcome message).
        //
        WidgetPaint((tWidget *)&g_sStartStopBttn);
    }
    else
    {
        //
        // Change the button text to indicate the new function.
        //
        PushButtonTextSet(&g_sStartStopBttn, "Start");

        WidgetPaint((tWidget *)&g_sStartStopBttn);
    }
}

void heartBeatFxn(UArg arg0, UArg arg1)
{
    while (1) {

        if((prevMotorSpeed != motorSpeed) || (prevMotorCurrent != motorCurrent)){
            drawSpeedPoint();
            prevMotorSpeed = motorSpeed;
            prevMotorCurrent = motorCurrent;
            drawCurrentPoint();
            plotLeft =  plotLeft + 5;

        }


        if(motorOn){
            GPIO_write(Board_LED0, Board_LED_ON);
        }else{
            GPIO_write(Board_LED0, Board_LED_OFF);
        }


        if(plotLeft > 290){
            DrawPlots();
            plotLeft = 0;
        }

        WidgetMessageQueueProcess();
    }
}

void UARTFxn(UART_Handle handle, void *rxBuf, size_t size){
    UInt gateKey;

    gateKey = GateHwi_enter(gateHwi);

    uint8_t c = ((uint8_t*)rxBuf)[0];

    switch(c) {
        case 119: //w
            Swi_post(SwiUpHandle);
            break;
        case 115: //s
            Swi_post(SwiDownHandle);
            break;
        case 97:  //a
            Swi_post(SwiLeftHandle);
            break;
        case 100: //d
            Swi_post(SwiRightHandle);
            break;
        case 13:  //enter
            Swi_post(SwiEnterHandle);
            break;
        case 127:  //backspace
            Swi_post(SwiMotorToggle);
            break;
    }

    GateHwi_leave(gateHwi, gateKey);

    UART_read(uart, rxBuf, wantedRxBytes);
}

void toggleMotor(UArg arg0, UArg arg1){

    if (motorOn){
        motorOn = 0;
    }else{
        motorOn = 1;
    }

}
void SwiUp(UArg arg0, UArg arg1){
    if(motorSpeed <= 4900){
        motorSpeed = motorSpeed + 100;
    }
}

void SwiDown(UArg arg0, UArg arg1){
    if(motorSpeed >= 100){
        motorSpeed = motorSpeed - 100;
    }
}

void SwiRight(UArg arg0, UArg arg1){
     motorCurrent = motorCurrent  + 100;
}

void SwiLeft(UArg arg0, UArg arg1){
    if(motorCurrent >= 100){
        motorCurrent = motorCurrent  - 100;
    }
}

void SwiEnter(UArg arg0, UArg arg1){

}

void initScreen(){
    Types_FreqHz cpuFreq;
    BIOS_getCpuFreq(&cpuFreq);

    //
    // The FPU should be enabled because some compilers will use floating-
    // point registers, even for non-floating-point code.  If the FPU is not
    // enabled this will cause a fault.  This also ensures that floating-
    // point operations could be added to this application and would work
    // correctly and use the hardware floating-point unit.  Finally, lazy
    // stacking is enabled for interrupt handlers.  This allows floating-
    // point instructions to be used within interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(cpuFreq.lo);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //Create Hwi for Touchscreen interrupt
    Hwi_Params_init(&hwiParams);

    TouchScreenIntHandlerHandle = Hwi_create(INT_ADC0SS3_TM4C129, (Hwi_FuncPtr)TouchScreenIntHandler , &hwiParams, NULL);

    if (TouchScreenIntHandlerHandle == NULL) {
        System_abort("Hwi create failed");
    }

    TouchScreenInit(cpuFreq.lo);
    TouchScreenCallbackSet(WidgetPointerMessage);

    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sBackground);

    WidgetPaint(WIDGET_ROOT);

    DrawPlots();
}

void initUART(){
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = UARTFxn;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    const char echoPrompt[] = "\fKey Entered:\r\n";
    UART_write(uart, echoPrompt, sizeof(echoPrompt));

    UART_read(uart, rxBuf, wantedRxBytes);
}

int main(void){
    Error_Block eb;
    Error_init(&eb);

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();
    PinoutSet(false, false);

    Task_Params taskParams;

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    //Create the Swi threads
    Swi_Params_init(&swiParams);
    SwiDownHandle = Swi_create(SwiDown,&swiParams,NULL);

    Swi_Params_init(&swiParams);
    SwiUpHandle = Swi_create(SwiUp,&swiParams,NULL);

    Swi_Params_init(&swiParams);
    SwiRightHandle = Swi_create(SwiRight,&swiParams,NULL);

    Swi_Params_init(&swiParams);
    SwiLeftHandle = Swi_create(SwiLeft,&swiParams,NULL);

    Swi_Params_init(&swiParams);
    SwiEnterHandle = Swi_create(SwiEnter,&swiParams,NULL);

    Swi_Params_init(&swiParams);
    SwiMotorToggle = Swi_create(toggleMotor,&swiParams,NULL);

    //Create Hwi Gate Mutex
    GateHwi_Params_init(&gHwiprms);
    gateHwi = GateHwi_create(&gHwiprms, NULL);
    if (gateHwi == NULL) {
        System_abort("Gate Hwi create failed");
    }

    initUART();
    initScreen();
    bool motorStatus = initMotorLib(50, &eb);

    if (!motorStatus) {
        System_abort(eb.msg);
    }

    /* Start BIOS */
    BIOS_start();

    return (0);
}
