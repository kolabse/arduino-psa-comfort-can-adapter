/*
Copyright 2019-2022, Ludwig V. <https://github.com/ludwig-v>
Copyright 2021, Nick V. (V3nn3tj3) <https://github.com/v3nn3tj3>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License at <http://www.gnu.org/licenses/> for
more details.

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
*/

/////////////////////
//    Libraries    //
/////////////////////

#include <PsaComfortCanAdapter.h>

/////////////////////
//  Configuration  //
/////////////////////

const uint8_t csPinCan0 {10};
const uint8_t csPinCan1 {9};


////////////////////
// Initialization //
////////////////////

//MCP2515 CAN0(csPinCan0); // CAN-BUS Shield N°1
//MCP2515 CAN1(csPinCan1); // CAN-BUS Shield N°2

//PsaComfortCanAdapter canAdapter = PsaComfortCanAdapter(CAN0, CAN1);

PsaComfortCanAdapter canAdapter = PsaComfortCanAdapter(csPinCan0, csPinCan1);

void setup() {
  canAdapter.adapterInit();
}

void loop() {
  canAdapter.checkAnalogButtonsPressed();
  canAdapter.transferCan2004Messages();
  canAdapter.transferCan2010Messages();
}