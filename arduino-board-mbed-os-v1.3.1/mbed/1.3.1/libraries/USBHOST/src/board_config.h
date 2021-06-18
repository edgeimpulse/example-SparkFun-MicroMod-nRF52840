/*       
 *         _______                    _    _  _____ ____  
 *        |__   __|                  | |  | |/ ____|  _ \ 
 *           | | ___  ___ _ __  _   _| |  | | (___ | |_) |
 *           | |/ _ \/ _ \ '_ \| | | | |  | |\___ \|  _ < 
 *           | |  __/  __/ | | | |_| | |__| |____) | |_) |
 *           |_|\___|\___|_| |_|\__, |\____/|_____/|____/ 
 *                               __/ |                    
 *                              |___/                     
 *
 * TeenyUSB - light weight usb stack for STM32 micro controllers
 * 
 * Copyright (c) 2019 XToolBox  - admin@xtoolbox.org
 *                         www.tusb.org
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/* Use external phy for high speed core */
#define  OTG_HS_EXTERNAL_PHY

/* Use embedded phy for high speed core */
//#define  OTG_HS_EMBEDDED_PHY

/* Use embedded phy for full speed core */ 
#define  OTG_FS_EMBEDDED_PHY

/* Enable DMA for High speed phy */
#define  OTG_HS_ENABLE_DMA

/* Support for other speed config and device qualifier descriptor */
#define  SUPPORT_OTHER_SPEED

/* Setup descriptor buffer size, used for other speed config and DMA */
#define  DESCRIPTOR_BUFFER_SIZE  256

/* USB core ID used in test app, 0 - FS core, 1 - HS core */
#define  USB_CORE_ID_FS             0
#define  USB_CORE_ID_HS             1


    /**USB_OTG_HS GPIO Configuration    
    PA3     ------> USB_OTG_HS_ULPI_D0
    PB0     ------> USB_OTG_HS_ULPI_D1
    PB1     ------> USB_OTG_HS_ULPI_D2
    PB10     ------> USB_OTG_HS_ULPI_D3
    PB11     ------> USB_OTG_HS_ULPI_D4
    PB12     ------> USB_OTG_HS_ULPI_D5
    PB13     ------> USB_OTG_HS_ULPI_D6
    PB5     ------> USB_OTG_HS_ULPI_D7 
    PI11     ------> USB_OTG_HS_ULPI_DIR
    PC0     ------> USB_OTG_HS_ULPI_STP
    PH4     ------> USB_OTG_HS_ULPI_NXT
    PA5     ------> USB_OTG_HS_ULPI_CK
    */
#define  OTG_HS_ULPI_IO_CLK_ENABLE() \
do {} while(0U)

#define  OTG_HS_ULPI_D0    GPIOA,  3, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_D1    GPIOB,  0, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_D2    GPIOB,  1, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_D3    GPIOB, 10, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_D4    GPIOB, 11, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_D5    GPIOB, 12, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_D6    GPIOB, 13, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_D7    GPIOB,  5, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_DIR   GPIOI, 11, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_STP   GPIOC,  0, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_NXT   GPIOH,  4, GPIO_AF10_OTG_HS
#define  OTG_HS_ULPI_CK    GPIOA,  5, GPIO_AF10_OTG_HS


#define  HOST_PORT_POWER_ON()

#define  HOST_PORT_POWER_ON_HS()

#define  HOST_PORT_POWER_ON_FS() \
do{}while(0)

#define RTOS_INTERRUPT_ENTER    core_util_critical_section_enter
#define RTOS_INTERRUPT_LEAVE    core_util_critical_section_exit

// init the stdio hardware
void stdio_init(void);
// stdin recv char handler
void stdin_recvchar(int ch);
// stdout send char handler 
void stdout_sendchar(int ch);

#endif

