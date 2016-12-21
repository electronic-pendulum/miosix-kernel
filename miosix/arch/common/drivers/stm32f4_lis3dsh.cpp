/***************************************************************************
 *   Copyright (C) 2016                                                    *
 *   by Fioratto Raffaele, Cardinale Claudio                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/ 

#include "stm32f4_lis3dsh.h"

#include <errno.h>
#include "miosix.h"

namespace miosix {
    
/* ----------------- SPI Defines ----------------- */
    
#define SPI_CR1_BAUDRATE_FPCLK_DIV_2	(0x00 << 3)
#define SPI_CR1_BAUDRATE_FPCLK_DIV_4	(0x01 << 3)
#define SPI_CR1_BAUDRATE_FPCLK_DIV_8	(0x02 << 3)
#define SPI_CR1_BAUDRATE_FPCLK_DIV_16	(0x03 << 3)
#define SPI_CR1_BAUDRATE_FPCLK_DIV_32	(0x04 << 3)
#define SPI_CR1_BAUDRATE_FPCLK_DIV_64	(0x05 << 3)
#define SPI_CR1_BAUDRATE_FPCLK_DIV_128	(0x06 << 3)
#define SPI_CR1_BAUDRATE_FPCLK_DIV_256	(0x07 << 3)
  
#define SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE	(0 << 1)
#define SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE	(1 << 1)
    
#define SPI_CR1_CPHA_CLK_TRANSITION_1	(0 << 0)
#define SPI_CR1_CPHA_CLK_TRANSITION_2	(1 << 0)
    
#define SPI_CR1_DFF_8BIT                (0 << 11)
#define SPI_CR1_DFF_16BIT               (1 << 11)
    
#define SPI_CR1_MSBFIRST		(0 << 7)
    
/* ------------------- Local Typedefs ------------------------*/
    
typedef Gpio<GPIOA_BASE,5> lis3dsh_CLK;
typedef Gpio<GPIOA_BASE,6> lis3dsh_MOSI;
typedef Gpio<GPIOA_BASE,7> lis3dsh_MISO;
typedef Gpio<GPIOE_BASE,3> lis3dsh_CS;

/* ------------ Local functions implementation -------------- */

static void spi_enable(SPI_TypeDef* SPI)
{
    SPI->CR1 |= SPI_CR1_SPE; /* Enable SPI. */
}


static int spi_init_master(SPI_TypeDef* SPI, uint32_t br, uint32_t cpol, uint32_t cpha,
		    uint32_t dff, uint32_t lsbfirst)
{
    uint32_t reg32 = SPI->CR1;

    /* Reset all bits omitting SPE, CRCEN and CRCNEXT bits. */
    reg32 &= SPI_CR1_SPE | SPI_CR1_CRCEN | SPI_CR1_CRCNEXT;

    reg32 |= SPI_CR1_MSTR;	/* Configure SPI as master. */

    reg32 |= br;		/* Set baud rate bits. */
    reg32 |= cpol;		/* Set CPOL value. */
    reg32 |= cpha;		/* Set CPHA value. */
    reg32 |= dff;		/* Set data format (8 or 16 bits). */
    reg32 |= lsbfirst;          /* Set frame format (LSB- or MSB-first). */

    SPI->CR2 |= SPI_CR2_SSOE;   /* common case */
    SPI->CR1 = reg32;

    return 0;
}

/**
 * \internal
 * Initialize GPIO Pins
 */
static void gpio_init()
{
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    
    /* Configure CLK, MOSI, MISO pins, Alternate Function 5, speed 100MHz*/
    lis3dsh_CLK::mode(Mode::ALTERNATE);
    lis3dsh_CLK::alternateFunction(5);
    lis3dsh_CLK::speed(Speed::_100MHz);
    
    lis3dsh_MOSI::mode(Mode::ALTERNATE);
    lis3dsh_MOSI::alternateFunction(5);
    lis3dsh_MOSI::speed(Speed::_100MHz);
    
    lis3dsh_MISO::mode(Mode::ALTERNATE);
    lis3dsh_MISO::alternateFunction(5);
    lis3dsh_MISO::speed(Speed::_100MHz);
    
    /* Enable GPIOE clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; 
    
    /* Configure CS pin, Output, speed 100MHz */
    lis3dsh_CS::mode(Mode::OUTPUT);
    lis3dsh_CS::speed(Speed::_100MHz);
    lis3dsh_CS::high();
}

/**
 * \internal
 * Initialize SPI
 */
static void spi_init()
{
    /* Enable SPI1 clock. */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Pulse SPI1 reset line */
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
            
    /* Initialize SPI1 master */
    spi_init_master(SPI1,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);
    
    /* Enable SPI1 first */
    spi_enable(SPI1);
}

SPILIS3DSHDriver::SPILIS3DSHDriver() : Device(Device::TTY) {
    
    gpio_init();
    
    spi_init();
}

intrusive_ref_ptr<SPILIS3DSHDriver> SPILIS3DSHDriver::instance() {
    static FastMutex m;
    static intrusive_ref_ptr<SPILIS3DSHDriver> instance;
    Lock<FastMutex> l(m);
    if(!instance) instance= new SPILIS3DSHDriver();
    return instance;
}
    
ssize_t SPILIS3DSHDriver::readBlock(void *buffer, size_t size, off_t where) {
    return -ENOSYS;
}
    
ssize_t SPILIS3DSHDriver::writeBlock(const void *buffer, size_t size, off_t where) {
    return -ENOSYS;
}
    
int SPILIS3DSHDriver::ioctl(int cmd, void *arg) {
    return -ENOSYS;
}

}