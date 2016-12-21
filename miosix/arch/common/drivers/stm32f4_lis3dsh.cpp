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
    
typedef Gpio<GPIOA_BASE,5> lis3dsh_CLK;
typedef Gpio<GPIOA_BASE,6> lis3dsh_MOSI;
typedef Gpio<GPIOA_BASE,7> lis3dsh_MISO;
typedef Gpio<GPIOE_BASE,3> lis3dsh_CS;

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

SPILIS3DSHDriver::SPILIS3DSHDriver() : Device(Device::TTY) {
    
    gpio_init();
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