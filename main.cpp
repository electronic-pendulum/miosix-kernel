#include "LengthCalculator.h"
#include <stdio.h>
#include <miosix.h>
#include <fcntl.h>
#include <filesystem/stringpart.h>
#include <filesystem/file_access.h>
#include <kernel/sync.h>
#include <miosix/arch/common/drivers/stm32f4_lis3dsh.h>

using namespace miosix;

static const int8_t OFF_Y = 10;
static const int8_t OFF_X = 15;
static const int8_t OFF_Z = 0;
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
    double length;
    int time = 0;
    // TODO: use getTick() instead of Timer
    miosix::Timer timer;
    timer.start();

    //loop that retrieves value of acceleroemer every 1 ms and prints the length calculated
    for(;;) {
        Thread::sleep(1);
        accelFile->read(&value, 2);
        timer.stop();
        time += timer.interval();
        timer.clear();
        timer.start();
        length = calculator.getLength(value, time);
        printf("Last length read: %f\n", length);
    }

}
