/*
 * Smart Parking Garage Gate System
 * CSE411 / CSE323 - Spring 2026
 *
 * Hardware:
 *   PF4 (SW1) = OPEN button  (active LOW, internal pull-up)
 *   PF0 (SW2) = CLOSE button (active LOW, internal pull-up)
 *   PF3       = Green LED    (gate OPENING)
 *   PF1       = Red LED      (gate CLOSING)
 *
 * Scenarios implemented:
 *   1. Manual Mode   - hold button -> gate moves, release -> STOPPED_MIDWAY
 *   2. One-Touch Auto- brief tap   -> gate moves for AUTO_DURATION_MS then IDLE
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* ---------------------------------------------
   PIN DEFINITIONS
--------------------------------------------- */
#define LED_RED        (1U << 1)   /* PF1 */
#define LED_GREEN      (1U << 3)   /* PF3 */
#define BTN_OPEN       (1U << 4)   /* PF4 - SW1 */
#define BTN_CLOSE      (1U << 0)   /* PF0 - SW2 */

/* ---------------------------------------------
   TIMING
--------------------------------------------- */
#define DEBOUNCE_MS         50U    /* button debounce window          */
#define SHORT_PRESS_MS     300U    /* max duration for a "tap"        */
#define AUTO_DURATION_MS  3000U    /* simulated travel to limit       */

/* ---------------------------------------------
   GATE STATES
--------------------------------------------- */
typedef enum {
    GATE_IDLE_CLOSED = 0,
    GATE_IDLE_OPEN,
    GATE_OPENING,
    GATE_CLOSING,
    GATE_STOPPED_MIDWAY
} GateState_t;

/* ---------------------------------------------
   BUTTON EVENTS sent through the queue
--------------------------------------------- */
typedef enum {
    EVT_OPEN_PRESS = 0,   /* button pressed  */
    EVT_OPEN_RELEASE,     /* button released */
    EVT_CLOSE_PRESS,
    EVT_CLOSE_RELEASE,
    EVT_AUTO_LIMIT        /* software timer fired = reached limit */
} ButtonEvent_t;

/* ---------------------------------------------
   RTOS HANDLES
--------------------------------------------- */
static QueueHandle_t    xEventQueue;
static SemaphoreHandle_t xGateStateMutex;
static TimerHandle_t    xAutoTimer;

/* Shared gate state – always access under mutex */
static volatile GateState_t gGateState = GATE_IDLE_CLOSED;

/* ---------------------------------------------
   GPIO INITIALISATION
--------------------------------------------- */
static void GPIO_Init(void)
{
    /* Enable clock for Port F */
    SYSCTL_RCGCGPIO_R |= (1U << 5);
    while ((SYSCTL_PRGPIO_R & (1U << 5)) == 0) {}

    /* Unlock PF0 (special lock register needed) */
    GPIO_PORTF_LOCK_R  = 0x4C4F434B;
    GPIO_PORTF_CR_R   |= BTN_CLOSE;

    /* LEDs as outputs */
    GPIO_PORTF_DIR_R  |= (LED_RED | LED_GREEN);
    GPIO_PORTF_DEN_R  |= (LED_RED | LED_GREEN);

    /* Buttons as inputs with internal pull-ups */
    GPIO_PORTF_DIR_R  &= ~(BTN_OPEN | BTN_CLOSE);
    GPIO_PORTF_DEN_R  |=  (BTN_OPEN | BTN_CLOSE);
    GPIO_PORTF_PUR_R  |=  (BTN_OPEN | BTN_CLOSE);

    /* LEDs off at start */
    GPIO_PORTF_DATA_R &= ~(LED_RED | LED_GREEN);
}

/* ---------------------------------------------
   LED HELPERS
--------------------------------------------- */
static void LED_SetOpening(void)
{
    GPIO_PORTF_DATA_R =  (GPIO_PORTF_DATA_R & ~(LED_RED | LED_GREEN)) | LED_GREEN;
}

static void LED_SetClosing(void)
{
    GPIO_PORTF_DATA_R =  (GPIO_PORTF_DATA_R & ~(LED_RED | LED_GREEN)) | LED_RED;
}

static void LED_SetOff(void)
{
    GPIO_PORTF_DATA_R &= ~(LED_RED | LED_GREEN);
}

/* ---------------------------------------------
   AUTO TIMER CALLBACK
   Fires when simulated gate travel time elapses.
   Sends EVT_AUTO_LIMIT into the queue from ISR context.
--------------------------------------------- */
static void AutoTimer_Callback(TimerHandle_t xTimer)
{
    ButtonEvent_t evt = EVT_AUTO_LIMIT;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xEventQueue, &evt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ---------------------------------------------
   TASK 1 – INPUT TASK  (Priority: High)
   Polls buttons, debounces, detects press/release
   and short-vs-long press, then posts events.
--------------------------------------------- */
static void InputTask(void *pvParameters)
{
    bool openWasPressed  = false;
    bool closeWasPressed = false;

    TickType_t openPressTime  = 0;
    TickType_t closePressTime = 0;

    for (;;)
    {
        /* Buttons are active LOW (pull-up, press -> 0) */
        bool openNow  = ((GPIO_PORTF_DATA_R & BTN_OPEN)  == 0);
        bool closeNow = ((GPIO_PORTF_DATA_R & BTN_CLOSE) == 0);

        /* -- OPEN button -- */
        if (openNow && !openWasPressed)
        {
            /* Just pressed */
            openWasPressed = true;
            openPressTime  = xTaskGetTickCount();
            ButtonEvent_t evt = EVT_OPEN_PRESS;
            xQueueSend(xEventQueue, &evt, 0);
        }
        else if (!openNow && openWasPressed)
        {
            /* Just released */
            openWasPressed = false;
            ButtonEvent_t evt = EVT_OPEN_RELEASE;
            xQueueSend(xEventQueue, &evt, 0);
        }

        /* -- CLOSE button -- */
        if (closeNow && !closeWasPressed)
        {
            closeWasPressed = true;
            closePressTime  = xTaskGetTickCount();
            ButtonEvent_t evt = EVT_CLOSE_PRESS;
            xQueueSend(xEventQueue, &evt, 0);
        }
        else if (!closeNow && closeWasPressed)
        {
            closeWasPressed = false;
            ButtonEvent_t evt = EVT_CLOSE_RELEASE;
            xQueueSend(xEventQueue, &evt, 0);
        }

        /* Suppress unused variable warnings */
        (void)openPressTime;
        (void)closePressTime;

        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    }
}

/* ---------------------------------------------
   TASK 2 – GATE CONTROL TASK  (Priority: Medium)
   Reads events from queue, runs the FSM,
   decides gate actions.

   Manual Mode logic:
     PRESS  -> start moving
     RELEASE-> stop (STOPPED_MIDWAY) unless limit reached

   One-Touch Auto logic:
     PRESS then RELEASE quickly -> start auto timer
     EVT_AUTO_LIMIT fires       -> transition to IDLE
--------------------------------------------- */
static void GateControlTask(void *pvParameters)
{
    ButtonEvent_t evt;

    /* Track when each button was pressed to detect short press */
    TickType_t openPressedAt  = 0;
    TickType_t closePressedAt = 0;
    bool       openHeld       = false;
    bool       closeHeld      = false;

    for (;;)
    {
        /* Block until an event arrives */
        if (xQueueReceive(xEventQueue, &evt, portMAX_DELAY) == pdTRUE)
        {
            xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
            GateState_t state = gGateState;
            xSemaphoreGive(xGateStateMutex);

            switch (evt)
            {
                /* ------------------------------
                   OPEN BUTTON PRESSED
                ------------------------------ */
                case EVT_OPEN_PRESS:
                    openPressedAt = xTaskGetTickCount();
                    openHeld      = true;

                    /* Start opening from any stopped state */
                    if (state == GATE_IDLE_CLOSED   ||
                        state == GATE_STOPPED_MIDWAY ||
                        state == GATE_CLOSING)
                    {
                        /* Stop auto-close timer if running */
                        xTimerStop(xAutoTimer, 0);

                        xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
                        gGateState = GATE_OPENING;
                        xSemaphoreGive(xGateStateMutex);
                    }
                    break;

                /* ------------------------------
                   OPEN BUTTON RELEASED
                ------------------------------ */
                case EVT_OPEN_RELEASE:
                    openHeld = false;
                    if (state == GATE_OPENING)
                    {
                        TickType_t held = xTaskGetTickCount() - openPressedAt;

                        if (held <= pdMS_TO_TICKS(SHORT_PRESS_MS))
                        {
                            /* -- One-Touch Auto Mode --
                               Short tap: keep moving, start timer */
                            xTimerStart(xAutoTimer, 0);
                            /* State stays GATE_OPENING */
                        }
                        else
                        {
                            /* -- Manual Mode --
                               Long hold then release: stop */
                            xTimerStop(xAutoTimer, 0);
                            xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
                            gGateState = GATE_STOPPED_MIDWAY;
                            xSemaphoreGive(xGateStateMutex);
                        }
                    }
                    break;

                /* ------------------------------
                   CLOSE BUTTON PRESSED
                ------------------------------ */
                case EVT_CLOSE_PRESS:
                    closePressedAt = xTaskGetTickCount();
                    closeHeld      = true;

                    if (state == GATE_IDLE_OPEN     ||
                        state == GATE_STOPPED_MIDWAY ||
                        state == GATE_OPENING)
                    {
                        xTimerStop(xAutoTimer, 0);

                        xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
                        gGateState = GATE_CLOSING;
                        xSemaphoreGive(xGateStateMutex);
                    }
                    break;

                /* ------------------------------
                   CLOSE BUTTON RELEASED
                ------------------------------ */
                case EVT_CLOSE_RELEASE:
                    closeHeld = false;
                    if (state == GATE_CLOSING)
                    {
                        TickType_t held = xTaskGetTickCount() - closePressedAt;

                        if (held <= pdMS_TO_TICKS(SHORT_PRESS_MS))
                        {
                            /* -- One-Touch Auto Mode -- */
                            xTimerStart(xAutoTimer, 0);
                        }
                        else
                        {
                            /* -- Manual Mode -- */
                            xTimerStop(xAutoTimer, 0);
                            xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
                            gGateState = GATE_STOPPED_MIDWAY;
                            xSemaphoreGive(xGateStateMutex);
                        }
                    }
                    break;

                /* ------------------------------
                   AUTO TIMER FIRED = LIMIT REACHED
                ------------------------------ */
                case EVT_AUTO_LIMIT:
                    xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
                    if (gGateState == GATE_OPENING)
                        gGateState = GATE_IDLE_OPEN;
                    else if (gGateState == GATE_CLOSING)
                        gGateState = GATE_IDLE_CLOSED;
                    xSemaphoreGive(xGateStateMutex);
                    break;

                default:
                    break;
            }

            /* Suppress unused variable warnings */
            (void)openHeld;
            (void)closeHeld;
        }
    }
}

/* ---------------------------------------------
   TASK 3 – LED TASK  (Priority: Medium)
   Reads gate state and drives LEDs accordingly.
   Polls every 50ms — low overhead.
--------------------------------------------- */
static void LEDTask(void *pvParameters)
{
    GateState_t lastState = GATE_IDLE_CLOSED;

    for (;;)
    {
        xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
        GateState_t state = gGateState;
        xSemaphoreGive(xGateStateMutex);

        if (state != lastState)
        {
            lastState = state;
            switch (state)
            {
                case GATE_OPENING:
                    LED_SetOpening();   /* Green ON, Red OFF  */
                    break;
                case GATE_CLOSING:
                    LED_SetClosing();   /* Red ON, Green OFF  */
                    break;
                case GATE_IDLE_OPEN:
                case GATE_IDLE_CLOSED:
                case GATE_STOPPED_MIDWAY:
                default:
                    LED_SetOff();       /* Both LEDs OFF      */
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------------------------------------------
   MAIN
--------------------------------------------- */
int main(void)
{
    /* Hardware init */
    GPIO_Init();

    /* -- Create RTOS primitives -- */

    /* Queue: holds up to 10 button events */
    xEventQueue = xQueueCreate(10, sizeof(ButtonEvent_t));

    /* Mutex: protects shared gGateState */
    xGateStateMutex = xSemaphoreCreateMutex();

    /* Software timer: one-shot, fires after AUTO_DURATION_MS */
    xAutoTimer = xTimerCreate(
        "AutoTimer",
        pdMS_TO_TICKS(AUTO_DURATION_MS),
        pdFALSE,              /* one-shot */
        NULL,
        AutoTimer_Callback
    );

    /* -- Create Tasks -- */

    /* Input Task – highest priority so buttons are always read */
    xTaskCreate(InputTask,       "Input",   128, NULL, 3, NULL);

    /* Gate Control Task – medium priority, runs the FSM */
    xTaskCreate(GateControlTask, "GateCtrl",256, NULL, 2, NULL);

    /* LED Task – medium priority, reflects state on LEDs */
    xTaskCreate(LEDTask,         "LED",     128, NULL, 2, NULL);

    /* Start the scheduler – never returns */
    vTaskStartScheduler();

    /* Should never reach here */
    while (1) {}
}