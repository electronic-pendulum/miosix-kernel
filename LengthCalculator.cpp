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
#include <algorithm>
#include <math.h>

using namespace miosix;

LengthCalculator::LengthCalculator() :
    lastLength(-1.0), decrementingY(false), previousY(10000), minY(10000), zeroTime(0) {
}

LengthCalculator::LengthCalculator(const LengthCalculator& orig) {
    this->previousY = orig.previousY;
    this->minY = orig.minY;
    this->lastLength = orig.lastLength;
}

LengthCalculator::~LengthCalculator() {
}

/**
* return the last leght calculated
* it wants acceleromenter value of y and time in ms
**/
double LengthCalculator::getLength(int accY, int now) {
    return 0.0;
}

/* PRIVATE METHODS */

//calculate length of the pendulum and set in the private property
void LengthCalculator::calculateLength(int period, double theta) {
    //period is in ms
    //this is the inverse of the pendulum serie approximated to the second term
    lastLength = G * pow(period / 1000.0, 2.0)/(4 * pow(M_PI, 2) * pow((1 + pow(theta, 2) / 16), 2));
}

//calculate the max theta of the current period
double LengthCalculator::calculateTheta() {
    //0 is the vertical position
    //the purpose of min is avoid NaN result of asin
    //we don't need information about the side of the ependulum, for that reason we use abs
    return 90.0 - asin(std::min(1.0, std::abs((double) minY / (SCALE * G)))) / M_PI * 180;
}

//calculate the current period (not yet finished)
int LengthCalculator::calculatePeriod(int now) {
    //we have measured a quarter of a period
    return (now - zeroTime) * 4;
}
