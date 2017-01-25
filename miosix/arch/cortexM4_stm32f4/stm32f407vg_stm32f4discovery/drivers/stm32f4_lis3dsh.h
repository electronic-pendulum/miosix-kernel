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

#ifndef STM32F4_LIS3DSH_H
#define STM32F4_LIS3DSH_H

#include <filesystem/devfs/devfs.h>

namespace miosix {
    
/* ---------------- Local Defines ----------------- */
    
/**
 * Driver for interfacing to an SD card through SPI
 */
class SPILIS3DSHDriver : public Device
{
public:
    enum Axes {
        X = 1,
        Y = 2,
        Z = 4,
        AXES_NUM = 3
    };
    
    
    enum IoctlCmds {
        CMD_INVALID = 0,
        AXIS_SELECT,
        OFFSETS_WRITE,
        OFFSETS_READ
    };
    
    struct Ioctl {
        union {
           struct {
               Axes axis;
           } axis_select;
           struct {
               int8_t axes;
               union {
                   unsigned int offsets;
                   struct {
                       unsigned char X;
                       unsigned char Y;
                       unsigned char Z;
                       unsigned char padding;
                   };
               };
           } offset;
        };
    };
    
    /**
     * Constructor
     */
    SPILIS3DSHDriver();
    
    /**
     * \return an instance to this class, singleton
     */
    
    static intrusive_ref_ptr<SPILIS3DSHDriver> instance();
    
    virtual ssize_t readBlock(void *buffer, size_t size, off_t where);
    
    virtual ssize_t writeBlock(const void *buffer, size_t size, off_t where);
    
    virtual int ioctl(int cmd, void *arg);
    
private:    
    int16_t _doReadAxis();
    
    static const uint8_t axis_reg_addr_array[Axes::AXES_NUM][2];
     
    FastMutex mutex;
    
    bool initialized;
    int8_t selected_axis;
};

} //namespace miosix

#endif /* STM32F4_LIS3DSH_H */