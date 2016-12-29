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

namespace miosix {

SPILIS3DSHDriver::SPILIS3DSHDriver() : Device(Device::TTY) {
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