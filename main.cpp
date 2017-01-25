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

#include "LengthCalculator.h"
#include <stdio.h>
#include <miosix.h>
#include <fcntl.h>
#include <filesystem/stringpart.h>
#include <filesystem/file_access.h>
#include <kernel/sync.h>
#include <miosix/arch/cortexM4_stm32f4/stm32f407vg_stm32f4discovery/drivers/stm32f4_lis3dsh.h>

using namespace miosix;

static const int8_t OFF_Y = 10;
static const int8_t OFF_X = 15;
static const int8_t OFF_Z = 0;

//best value to have precision and speed, it was found in experimentally way
static const unsigned int DELAY = 30; // ms
int main()
{
    LengthCalculator calculator;
    intrusive_ref_ptr<FileBase> accelFile;
    intrusive_ref_ptr<DevFs> devfs = FilesystemManager::instance().getDevFs();
    StringPart accel("accel");
    if(devfs->open(accelFile,accel,O_RDWR,0)<0) {
        iprintf("Cannot communicate with accelerometer!\n");
        for (;;);
    }

    SPILIS3DSHDriver::Ioctl ctl;
    ctl.axis_select.axis = SPILIS3DSHDriver::Y;
    accelFile->ioctl(SPILIS3DSHDriver::AXIS_SELECT, &ctl);

    ctl.offset.axes = SPILIS3DSHDriver::X | SPILIS3DSHDriver::Y | SPILIS3DSHDriver::Z;
    ctl.offset.X = OFF_X;
    ctl.offset.Y = OFF_Y;
    ctl.offset.Z = OFF_Z;
    accelFile->ioctl(SPILIS3DSHDriver::OFFSETS_WRITE, &ctl);

    int16_t value;
    float previousLength, length;
    previousLength = -1.0;

    //loop that retrieves value of acceleroemer every 1 ms and prints the length calculated
    for(;;) {
      Thread::sleep(DELAY);
      accelFile->read(&value, 2);
      length = calculator.getLength(value, getTick());
      if (length != previousLength) {
          printf("Last length read: %f\n", length);
          previousLength = length;
      }
    }

}
