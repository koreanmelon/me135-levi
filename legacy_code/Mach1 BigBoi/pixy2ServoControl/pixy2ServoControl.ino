//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <Pixy2.h>
#include <PIDLoop.h>

Pixy2 pixy;
//PIDLoop panLoop(400, 0, 400, true);
//PIDLoop tiltLoop(500, 0, 500, true);

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
 
  // We need to initialize the pixy object 
  pixy.init();
  // Use color connected components program for the pan tilt to track 
  pixy.changeProg("color_connected_components");
}

void loop()
{  
//  static int i = 0;
//  int j;
//  char buf[64]; 
//  int32_t panOffset, tiltOffset;
  

  pixy.setServos(500, 800);

}
