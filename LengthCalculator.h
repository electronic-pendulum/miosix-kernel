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

#ifndef LENGTHCALCULATOR_H
#define LENGTHCALCULATOR_H

#include "miosix.h"

class LengthCalculator {
public:
    LengthCalculator();
    LengthCalculator(const LengthCalculator& orig);
    virtual ~LengthCalculator();
    double getLength(int acc, long long now);

private:
    void calculateLength(int period, double theta);
    double calculateTheta();
    int calculatePeriod();

private:
    const double ALPHA_ANGLE = 0.5; //low pass filter factor for angle
    const double ALPHA_LENGTH = 0.8; //low pass filter factor for length
    const int SCALE = 100;
    const double G = 9.81;

    double lastLength;
    bool decrementingY;
    double previousY;
    double minY;
    long long zeroTime;
    long long previousTime;
};

#endif /* LENGTHCALCULATOR_H */
