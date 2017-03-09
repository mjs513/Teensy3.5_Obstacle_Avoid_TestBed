//============================================================================
//    Sketch to test various technicques in robotic car design such as
//    obstacle detection and avoidance, compass as turn guide,
//    motor control, etc.
//    Copyright (C) 2015  Michael J Smorto
//    https://github.com/mjs513/Sainsmart_4WD_Obstacle_Avoid_TestBed.git
//    FreeIMU@gmail.com
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================
//

void AltIMU_Init() {
  
    compass.init();
    compass.enableDefault();

    int deviceType = compass.getDeviceType();

    if((int16_t) deviceType == 3) {  // LSM303D
        // Accelerometer
        //0x00:+/-2g,  0x08:+/- 4g,  0x10: +/- 6g, 0x18: +/- 8g,  0x20: +/- 16g
        compass.writeReg(LSM303::CTRL2, 0x00);

  // Magnetometer
      // MFS = 01 (+/- 4 gauss full scale),  MFS = 00(+/- 2 gauss full scale)
  // MFS = 10 (+/- 8 gauss full scale),  MFS = 11(+/- 12 gauss full scale)
  compass.writeReg(LSM303::CTRL6, 0x00);
  }
    else if((int16_t) deviceType == 2) {  // LSM303DLHC
  // Accelerometer
  // FS = 00  (+/- 2 g full scale); HR = 1 (high resolution enable)
  //                                  FS HR
  //    equates to 0b00 00  1    00 0
      // FS = 0x01  (+/- 4 g),  0x10: +/- 8 g, 0x11: +/- 16 g 
  //  or: 0x08, 0x18, 0x28, 0x38 respectfully
  compass.writeAccReg(LSM303::CTRL_REG4_A, 0x08);
  
  // Magnetometer Settings
  // Â±1.3ga  (0x20)
  // Â±1.9ga  (0x40)
  // Â±2.5ga  (0x60)
  // Â±4.0ga  (0x80)  
  // Â±4.7ga  (0xA0)  
  // Â±5.6ga  (0xC0)
  // Â±8.1ga  (0xE0)  
  compass.writeMagReg(LSM303::CRB_REG_M, 0x20);
  }
    else if((int16_t) deviceType == 0 || (int16_t) deviceType == 1) {  // LSM303DLH or LSM303DLM
  // Accelerometer
  // 0x00 = 0b00000000
  // FS = 00 (+/- 2 g full scale), 01 (+/- 4g), 11 (+/- 8g)
  // 0x00, 0x01, 0x03 respectfully
        compass.writeAccReg(LSM303::CTRL_REG4_A, 0x00);

  // Magnetometer Settings
  // Â±1.3ga  (0x20)
  // Â±1.9ga  (0x40)
  // Â±2.5ga  (0x60)
  // Â±4.0ga  (0x80)  
  // Â±4.7ga  (0xA0)  
  // Â±5.6ga  (0xC0)
  // Â±8.1ga  (0xE0)  
  compass.writeMagReg(LSM303::CRB_REG_M, 0x20);
    }
    
    /*
    Calibration values; the default values of +/-32767 for each axis
    lead to an assumed magnetometer bias of 0. Use the Calibrate example
    program to determine appropriate values for your particular unit.
    */
    compass.m_min = (LSM303::vector<int16_t>){-3061,  -2357,  -2424};
    compass.m_max = (LSM303::vector<int16_t>){+2839,  +3278,  +3135}; 
  
    for(int i = 0; i < (compass_avg_cnt); i++) compass_update();
    
}
