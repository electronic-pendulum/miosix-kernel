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
#include <stdio.h>
#include "miosix.h"

namespace miosix {
    
/* ---------------- Local Defines ----------------- */

/* LIS3DSH registers addresses */
#define ADD_REG_WHO_AM_I			0x0F
#define ADD_REG_CTRL_4				0x20
#define ADD_REG_OUT_X_L				0x28
#define ADD_REG_OUT_X_H				0x29
#define ADD_REG_OUT_Y_L				0x2A
#define ADD_REG_OUT_Y_H				0x2B
#define ADD_REG_OUT_Z_L				0x2C
#define ADD_REG_OUT_Z_H				0x2D

#define ADD_REG_OFF_X                           0x10
#define ADD_REG_OFF_Y                           0x11
#define ADD_REG_OFF_Z                           0x12
    
/* WHO AM I register default value */
#define UC_WHO_AM_I_DEFAULT_VALUE		0x3F

/* ADD_REG_CTRL_4 register configuration value: X,Y,Z axis enabled and 400Hz of 
 * output data rate */
#define UC_ADD_REG_CTRL_4_CFG_VALUE		0x77

/* Sensitivity for 2G range [mg/digit] */
#define SENS_2G_RANGE_MG_PER_DIGIT		((float)0.06)    
    
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
    
/* ---------------- Local Macros ----------------- */

/* set read single command. Attention: command must be 0x3F at most */
#define SET_READ_SINGLE_CMD(x)			(x | 0x80)
/* set read multiple command. Attention: command must be 0x3F at most */
#define SET_READ_MULTI_CMD(x)			(x | 0xC0)
/* set write single command. Attention: command must be 0x3F at most */
#define SET_WRITE_SINGLE_CMD(x)			(x & (~(0xC0)))
/* set write multiple command. Attention: command must be 0x3F at most */
#define SET_WRITE_MULTI_CMD(x)			(x & (~(0x80))	\
                                                 x |= 0x40)
    
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

static void spi_write(SPI_TypeDef* SPI, uint16_t data)
{
    /* Write data into DR. */
    SPI->DR = data;
}

static uint16_t spi_read(SPI_TypeDef* SPI)
{
    /* Wait for transfer finished. */
    while (!(SPI->SR & SPI_SR_RXNE));

    /* Read the data from DR. */
    return SPI->DR;
}

static uint16_t spi_xfer(SPI_TypeDef* SPI, uint16_t data)
{
    spi_write(SPI, data);

    return spi_read(SPI);
}

/**
 * \internal
 * Function to write a register to LIS3DSH through SPI 
 */
static void spi_write_reg(uint8_t reg, uint8_t data)
{
    /* set CS low */
    lis3dsh_CS::low();
    
    /* discard returned value */
    spi_xfer(SPI1, SET_WRITE_SINGLE_CMD(reg));
    spi_xfer(SPI1, data);
    
    /* set CS high */
    lis3dsh_CS::high();
}

/**
 * \internal
 * Function to read a register from LIS3DSH through SPI
 */
static uint8_t spi_read_reg(uint8_t reg)
{
    uint8_t reg_value;
    /* set CS low */
    lis3dsh_CS::low();
    reg_value = spi_xfer(SPI1, SET_READ_SINGLE_CMD(reg));
    reg_value = spi_xfer(SPI1, 0xFF);
    /* set CS high */
    lis3dsh_CS::high();

    return reg_value;
}

/**
 * \internal
 * Transform a two's complement value to 16-bit int value 
 */
static inline int16_t high_low_to_int16(uint16_t high_low_value)
{
    int16_t int16_value = 0;

    /* conversion */
    if (high_low_value > 32768) {
        int16_value = -(((~high_low_value) & 0xFFFF) + 1);
    } else {
        int16_value = high_low_value;
    }

    return int16_value;
}

//
// class SPILIS3DSHDriver
//
const uint8_t SPILIS3DSHDriver::axis_reg_addr_array[AXES_NUM][2] = {
    {ADD_REG_OUT_X_L, ADD_REG_OUT_X_H},
    {ADD_REG_OUT_Y_L, ADD_REG_OUT_Y_H},
    {ADD_REG_OUT_Z_L, ADD_REG_OUT_Z_H}
};

SPILIS3DSHDriver::SPILIS3DSHDriver() : Device(Device::TTY),
    initialized(false), selected_axis(Axes::X) {
    
    int8_t reg_value;
    
    gpio_init();
    
    spi_init();
    
    /* get WHO AM I value */
    reg_value = spi_read_reg(ADD_REG_WHO_AM_I);

    /* if WHO AM I value is the expected one */
    if (reg_value == UC_WHO_AM_I_DEFAULT_VALUE) {
        /* set output data rate to 400 Hz and enable X,Y,Z axis */
        spi_write_reg(ADD_REG_CTRL_4, UC_ADD_REG_CTRL_4_CFG_VALUE);
        /* verify written value */
        reg_value = spi_read_reg(ADD_REG_CTRL_4);
        /* if written value is different */
        if (reg_value != UC_ADD_REG_CTRL_4_CFG_VALUE) {
            return;
        }
    } else {
        return;
    }
    
    spi_write_reg(ADD_REG_OFF_X, OFF_X);
    spi_write_reg(ADD_REG_OFF_Y, OFF_Y);
    spi_write_reg(ADD_REG_OFF_Z, OFF_Z);
    
    initialized = true;
}

intrusive_ref_ptr<SPILIS3DSHDriver> SPILIS3DSHDriver::instance() {
    static FastMutex m;
    static intrusive_ref_ptr<SPILIS3DSHDriver> instance;
    Lock<FastMutex> l(m);
    if(!instance) instance= new SPILIS3DSHDriver();
    return instance;
}
    

ssize_t SPILIS3DSHDriver::readBlock(void *buffer, size_t size, off_t where) 
{
    if (size < 2) {
        return -1;
    }
    
    ssize_t bytes_read = 0;
    int16_t* buffer_16 = (int16_t*) buffer;
    while(size > 0) {
        *buffer_16++ = _doReadAxis();
        bytes_read += 2;
        size -= 2;
    }
    
    return bytes_read;
}

ssize_t SPILIS3DSHDriver::writeBlock(const void *buffer, size_t size, 
        off_t where) 
{
    if (size == 0) {
        return -1;
    }
    
    ssize_t bytes_written = 0;
    const int8_t* buffer_8 = (const int8_t *) buffer;
    while (size > 0) {
        selected_axis = *buffer_8++;
        size--;
        bytes_written++;
    }
    iprintf("New selected axis = %d\n", selected_axis);
    return bytes_written;
}

int SPILIS3DSHDriver::ioctl(int cmd, void *arg) {
    return -ENOSYS;
}

int16_t SPILIS3DSHDriver::_doReadAxis() {
    int16_t mg_value = 0;

    if (selected_axis < AXES_NUM) {
        /* get 16-bit value */
        mg_value = ((spi_read_reg(axis_reg_addr_array[selected_axis][1]) << 8) |
                     spi_read_reg(axis_reg_addr_array[selected_axis][0]));

        /* transform X value from two's complement to 16-bit int */
        mg_value = high_low_to_int16(mg_value);

        /* convert X absolute value to mg value */
        mg_value = mg_value * SENS_2G_RANGE_MG_PER_DIGIT;
    }
    
    return mg_value;
}

}