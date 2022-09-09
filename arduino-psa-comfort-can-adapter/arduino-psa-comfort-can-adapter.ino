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

const uint8_t csPinCan0 {10};
const uint8_t csPinCan1 {9};

////////////////////
// Initialization //
////////////////////

PsaComfortCanAdapter canAdapter = PsaComfortCanAdapter(csPinCan0, csPinCan1);

void setup() {

  /////////////////////
  //  Configuration  //
  /////////////////////

  canAdapter.setEmulateVIN(true);
  canAdapter.setVinNumber("VF7XXXXXXXXXXXXXX");
  canAdapter.setHasAnalogicButtons(true);
  canAdapter.setLanguageID(RU);

  canAdapter.adapterInit();
}

void loop() {
  canAdapter.checkAnalogButtonsPressed();
  canAdapter.transferCan2004Messages();
  canAdapter.transferCan2010Messages();
}