#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_uart_serial.h"
#include "maxTouch/maxTouch.h"
#include "tfont.h"
#include "digital521.h"
#include "icons/restart_icon_light.h"
#include "icons/restart_icon_dark.h"
#include "icons/pause_icon_dark.h"
#include "icons/pause_icon_light.h"
#include "icons/change_icon_dark.h"
#include "icons/change_icon_light.h"
#include "icons/dist_icon_dark.h"
#include "icons/hour_icon_dark.h"
#include "icons/velo_icon_dark.h"
#include "icons/dist_icon_light.h"
#include "icons/hour_icon_light.h"
#include "icons/velo_icon_light.h"
/************************************************************************/
/* prototypes                                                           */
/************************************************************************/


/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        3

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;
uint32_t n;

/************************************************************************/
/* Botoes lcd                                                           */
/************************************************************************/


/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (6*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (6*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

typedef struct {
  uint x;
  uint y;
} touchData;

typedef struct {
	uint32_t width;     // largura (px)
	uint32_t height;    // altura  (px)
	uint32_t colorOn;   // cor do bot�o acionado
	uint32_t colorOff;  // cor do bot�o desligado
	uint32_t x;         // posicao x
	uint32_t y;         // posicao y
	uint8_t status;
	void (*callback)(t_but);
	tImage image;
} t_but;

QueueHandle_t xQueueTouch;

/************************************************************************/
/* handler/callbacks                                                    */
/************************************************************************/


/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
  * identify which task has overflowed its stack.
  */
  for (;;) {
  }
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void)
{
  pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/

static void configure_lcd(void){
  /* Initialize display parameter */
  g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
  g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
  g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
  g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

  /* Initialize LCD */
  ili9488_init(&g_ili9488_display_opt);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void draw_screen(void) {
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
  ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

void draw_button_new(t_but but){
	uint32_t color;
	if(but.status)
	ili9488_draw_pixmap(but.x-but.image.width/2, but.y-but.image.height/2, but.image.width, but.image.height, but.image.data);
	else {
		color = but.colorOff;
		ili9488_set_foreground_color(COLOR_CONVERT(color));
		ili9488_draw_filled_rectangle(but.x-but.width/2, but.y-but.height/2,
		but.x+but.width/2, but.y+but.height/2);
	}
}

void but0_callback(t_but *but) {
	but->status = !but->status;
	draw_button_new(*but);
}

void but1_callback(t_but *but) {
	but->status = !but->status;
	draw_button_new(*but);
}

void but2_callback(t_but *but) {
	but->status = !but->status;
	draw_button_new(*but);
}

void but3_callback(t_but *but) {
	but->status = !but->status;
	draw_button_new(*but);
}

void but4_callback(t_but *but) {
	but->status = !but->status;
	draw_button_new(*but);
}

int process_touch(t_but botoes[], touchData touch, uint32_t n){
		for (int i = 0; i < n; i++) {
			if (touch.x >= botoes[i].x - botoes[i].width/2 && touch.x <= botoes[i].x + botoes[i].width/2) {
				if (touch.y >= botoes[i].y - botoes[i].height/2 && touch.y <= botoes[i].y + botoes[i].height/2) {
					return i;
				}
			}
		}
		
		return -1;
}

void draw_button(uint32_t clicked) {
  static uint32_t last_state = 255; // undefined
  if(clicked == last_state) return;
  
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
  ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2, BUTTON_Y-BUTTON_H/2, BUTTON_X+BUTTON_W/2, BUTTON_Y+BUTTON_H/2);
  if(clicked) {
    ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
    ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y+BUTTON_H/2-BUTTON_BORDER);
    } else {
    ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
    ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y-BUTTON_H/2+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y-BUTTON_BORDER);
  }
  last_state = clicked;
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
  // entrada: 4096 - 0 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
  // entrada: 0 - 4096 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void update_screen(uint32_t tx, uint32_t ty) {
  if(tx >= BUTTON_X-BUTTON_W/2 && tx <= BUTTON_X + BUTTON_W/2) {
    if(ty >= BUTTON_Y-BUTTON_H/2 && ty <= BUTTON_Y) {
      draw_button(1);
      } else if(ty > BUTTON_Y && ty < BUTTON_Y + BUTTON_H/2) {
      draw_button(0);
    }
  }
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
  char *p = text;
  while(*p != NULL) {
    char letter = *p;
    int letter_offset = letter - font->start_char;
    if(letter <= font->end_char) {
      tChar *current_char = font->chars + letter_offset;
      ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
      x += current_char->image->width + spacing;
    }
    p++;
  }
}

void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
  /* USART tx buffer initialized to 0 */
  uint8_t i = 0; /* Iterator */

  /* Temporary touch event data struct */
  struct mxt_touch_event touch_event;
  
  /* first touch only */
  uint first = 0;

  /* Collect touch events and put the data in a string,
  * maximum 2 events at the time */
  do {

    /* Read next next touch event in the queue, discard if read fails */
    if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
      continue;
    }
    
    /************************************************************************/
    /* Envia dados via fila RTOS                                            */
    /************************************************************************/
    if(first == 0 ){
      *x = convert_axis_system_x(touch_event.y);
      *y = convert_axis_system_y(touch_event.x);
      first = 1;
    }
    
    i++;

    /* Check if there is still messages in the queue and
    * if we have reached the maximum numbers of events */
  } while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}

/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){
  
  struct mxt_device device; /* Device data container */
  mxt_init(&device);       	/* Initialize the mXT touch device */
  touchData touch;          /* touch queue data type*/
  
  while (true) {
    /* Check for any pending messages and run message handler if any
    * message is found in the queue */
    if (mxt_is_message_pending(&device)) {
      mxt_handler(&device, &touch.x, &touch.y);
      xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
    }
    vTaskDelay(100);
  }
}

void task_lcd(void){
	xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
	configure_lcd();
	draw_screen();
	font_draw_text(&digital52, "DEMO - BUT", 0, 0, 1);

	t_but but0 = {.width = 120, .height = 75,
		.colorOn = COLOR_TOMATO, .colorOff = COLOR_BLACK,
	.x = ILI9488_LCD_WIDTH/2, .y = 40, .status = 1, .callback = &but0_callback, .image = hour_dark};
	
	t_but but1 = {.width = 120, .height = 75,
		.colorOn = COLOR_VIOLET, .colorOff = COLOR_BLACK,
	.x = ILI9488_LCD_WIDTH/2, .y = 140, .status = 1, .callback = &but1_callback, .image = pause_dark};
	
	t_but but2 = {.width = 120, .height = 75,
		.colorOn = COLOR_BLUE, .colorOff = COLOR_BLACK,
	.x = ILI9488_LCD_WIDTH/2, .y = 240, .status = 1, .callback = &but2_callback, .image = restart_dark};
	
	t_but but3 = {.width = 120, .height = 75,
		.colorOn = COLOR_YELLOW, .colorOff = COLOR_BLACK,
	.x = ILI9488_LCD_WIDTH/2, .y = 340, .status = 1, .callback = &but3_callback, .image = velo_dark};
	
	t_but but4 = {.width = 120, .height = 75,
		.colorOn = COLOR_MAGENTA, .colorOff = COLOR_BLACK,
	.x = ILI9488_LCD_WIDTH/2, .y = 440, .status = 1, .callback = &but4_callback, .image = restart_light};
	
	draw_button_new(but0);
	draw_button_new(but1);
	draw_button_new(but2);
	draw_button_new(but3);
	draw_button_new(but4);
	
	//numero de botoes e lista deles
	n = 5;
	t_but botoes[] = {but0, but1, but2, but3, but4};

	// struct local para armazenar msg enviada pela task do mxt
	touchData touch;

	while (true) {
		if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
			//update_screen(touch.x, touch.y);
			int b = process_touch(botoes, touch, n);
			if(b >= 0){
				botoes[b].callback(&botoes[b]);
				//botoes[b].status = !botoes[b].status;
				//draw_button_new(botoes[b]);
			}
			printf("x:%d y:%d\n", touch.x, touch.y);
		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
  /* Initialize the USART configuration struct */
  const usart_serial_options_t usart_serial_options = {
    .baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
    .charlength   = USART_SERIAL_CHAR_LENGTH,
    .paritytype   = USART_SERIAL_PARITY,
    .stopbits     = USART_SERIAL_STOP_BIT
  };

  sysclk_init(); /* Initialize system clocks */
  board_init();  /* Initialize board */
  
  /* Initialize stdio on USART */
  stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
  
  /* Create task to handler touch */
  if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  
  /* Create task to handler LCD */
  if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  
  /* Start the scheduler. */
  vTaskStartScheduler();

  while(1){

  }


  return 0;
}
