#include "Arduino.h"
#include <Dynamixel2Arduino.h>
#include <stdarg.h>
#include <ojw.h>


// ===== define =====
#define ROTATE_LEFT   (1)
#define ROTATE_RIGHT  (2) 

#define DXL_SERIAL_SUNNY  Serial1



// ===== global =====
CProtocol2 m_CMot;
byte cnt = 0;
float gfDegree = 0.0;
int glocal_Degree = 0;

  Dynamixel2Arduino dxl_sunny(DXL_SERIAL_SUNNY, DXL_DIR_PIN);

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  m_CMot.Open(1000000); // serial open, 57600 bps


 // Dynamixel2Arduino
  dxl_sunny.begin(1000000);
  dxl_sunny.setPortProtocolVersion(2.0);

  //dxl_sunny.torqueOff(3);

  // 1.pwm linit
  //dxl_sunny.writeControlTableItem(36, 3, 600);

  // 2. current limit
  //dxl_sunny.writeControlTableItem(38, 3, 1000);

  // 3. position P Gain
  //dxl_sunny.writeControlTableItem(84, 3, 1200);

  //dxl_sunny.torqueOn(3);
  
  // CMot
  m_CMot.SetTorq(true); // Torq On
  // position read
  m_CMot.SyncRead("1");

 
  //m_CMot.Play("1000, 0, 1, 0");
  //m_CMot.Play("1000, 0, 2, 0");
  //m_CMot.Play("1000, 0, 3, 0");
  //m_CMot.Play("1000, 0, 4, 0");
  //m_CMot.Play("1000, 0, 5, 45");

  m_CMot.Play("1000, 0, 1, 0");
  gfDegree = dxl_sunny.getPresentPosition( 1, UNIT_DEGREE);           
  glocal_Degree = (int)gfDegree;
  glocal_Degree -= 180;
  Serial.print("1 Degree : ");
  Serial.println(glocal_Degree);
  
  m_CMot.Play("1000, 0, 2, 0");
  gfDegree = dxl_sunny.getPresentPosition( 2, UNIT_DEGREE);           
  glocal_Degree = (int)gfDegree;
  glocal_Degree -= 180;
  Serial.print("2 Degree : ");
  Serial.println(glocal_Degree);
  m_CMot.Play("1000, 0, 3, 0");
  gfDegree = dxl_sunny.getPresentPosition( 3, UNIT_DEGREE);           
  glocal_Degree = (int)gfDegree;
  glocal_Degree -= 180;
  Serial.print("3 Degree : ");
  Serial.println(glocal_Degree);
  m_CMot.Play("1000, 0, 4, 0");
  gfDegree = dxl_sunny.getPresentPosition( 4, UNIT_DEGREE);           
  glocal_Degree = (int)gfDegree;
  glocal_Degree -= 180;
  Serial.print("4 Degree : ");
  Serial.println(glocal_Degree);
  m_CMot.Play("1000, 0, 5, 45");
  gfDegree = dxl_sunny.getPresentPosition( 5, UNIT_DEGREE);           
  glocal_Degree = (int)gfDegree;
  glocal_Degree -= 180;
  Serial.print("5 Degree : ");
  Serial.println(glocal_Degree);

}

void moveMotor(int iMotorId, int iRotate, int iTargetStep, int iMaxDegree )
{
  int local_Degree = 0;
  // check MotorID
  if ((iMotorId <= 0) || (iMotorId >= 7))
  {
    Serial.print("Err : MotorId !");
    return;
  }
  // check Degree
  if ((iTargetStep >= 90)||(iMaxDegree >= 254))
  {
    Serial.print("Err : Degree!");
    return;
  }

  gfDegree = dxl_sunny.getPresentPosition( iMotorId, UNIT_DEGREE);
                    
  local_Degree = (int)gfDegree;
  
  local_Degree -= 180;

  Serial.print("local_Degree : ");
  Serial.println(local_Degree);

  //if (dxl_sunny.getLastLibErrCode() != DXL_LIB_ERROR_CRC)
  //{
  //  Serial.print("Err : getPresentPosition !");
  //  return;
  //}

  int target_Degree = local_Degree;

  if (ROTATE_LEFT == iRotate )
  {
    target_Degree = local_Degree - iTargetStep;
    if (target_Degree < iMaxDegree) target_Degree = iMaxDegree;
  }
  else // RIGHT
  {
    target_Degree = local_Degree + iTargetStep;
    if (target_Degree > iMaxDegree) target_Degree = iMaxDegree;
  }

  char cmdBuffer[50];
  sprintf(cmdBuffer, "1000, 0, %d, %d", iMotorId, target_Degree);
  

  Serial.print("Tx cmd : ");
  Serial.println(cmdBuffer);

  m_CMot.Play(cmdBuffer);
}

void loop() {

  //if (0 == cnt%10)
  //{
  //  Serial.print("Press : L then moter 45degree : ");
  //  Serial.println(cnt);
  //}
  if (Serial.available() > 0){    
    char incomingChar = Serial.read();
    Serial.println(incomingChar);

    switch(incomingChar)
    {
      //Left
      case 'l':
        moveMotor(1, ROTATE_LEFT, 10, -45 );
        break;

      //Right
      case 'r':
        moveMotor(1, ROTATE_RIGHT, 10, 45 );
        break;

      // Up
      case 'u':
        //moveMotor(2, ROTATE_RIGHT, 30, 0 );
        //delay(1000);
        
        //moveMotor(3, ROTATE_LEFT, 30, 0 );
        moveMotor(3, ROTATE_RIGHT, 25, 10 );

        //moveMotor(3, ROTATE_RIGHT, 10, 45);
        //delay(1000);
        break;

      // Down
      case 'd':
        //moveMotor(2, ROTATE_LEFT, 30, -90 );
        //delay(1000);
        
        //moveMotor(3, ROTATE_RIGHT, 30, 90 );
        moveMotor(3, ROTATE_LEFT, 10, -60 );

        //moveMotor(3, ROTATE_LEFT, 10, -45);
        //delay(1000);
        break;

      // Close
      case 'c':
        moveMotor(5, ROTATE_LEFT, 10, 0 );
        break;

      // Open
      case 'o':
        moveMotor(5, ROTATE_RIGHT, 10, 50 );
        break;

      // 4 : ID4 motor down
      case '4':
        moveMotor(4, ROTATE_LEFT, 10, -90);
        break;

      // 5 : ID4 motor up
      case '5':
        moveMotor(4, ROTATE_RIGHT, 10, 0);
        break;


      // init
      case 'i':
        m_CMot.SyncRead("1");
        m_CMot.Play("1000, 0, 1, 0");
        gfDegree = dxl_sunny.getPresentPosition( 1, UNIT_DEGREE);           
        glocal_Degree = (int)gfDegree;
        glocal_Degree -= 180;
        Serial.print("1 Degree : ");
        Serial.println(glocal_Degree);
        
        m_CMot.Play("1000, 0, 2, 0");
        gfDegree = dxl_sunny.getPresentPosition( 2, UNIT_DEGREE);           
        glocal_Degree = (int)gfDegree;
        glocal_Degree -= 180;
        Serial.print("2 Degree : ");
        Serial.println(glocal_Degree);

        m_CMot.Play("1000, 0, 3, 0");
        gfDegree = dxl_sunny.getPresentPosition( 3, UNIT_DEGREE);           
        glocal_Degree = (int)gfDegree;
        glocal_Degree -= 180;
        Serial.print("3 Degree : ");
        Serial.println(glocal_Degree);

        m_CMot.Play("1000, 0, 4, 0");
        gfDegree = dxl_sunny.getPresentPosition( 4, UNIT_DEGREE);           
        glocal_Degree = (int)gfDegree;
        glocal_Degree -= 180;
        Serial.print("4 Degree : ");
        Serial.println(glocal_Degree);

        m_CMot.Play("1000, 0, 5, 45");
        gfDegree = dxl_sunny.getPresentPosition( 5, UNIT_DEGREE);           
        glocal_Degree = (int)gfDegree;
        glocal_Degree -= 180;
        Serial.print("5 Degree : ");
        Serial.println(glocal_Degree);
        break;

      default:
        break;
    }


    //if (incomingChar == 'L')
    //{
    //  // ID 1, 2 -> move to 45 degree, (1000 ms, 0 ms delay)
    //  m_CMot.Play("1000, 0, 6, -45");
    //  // ID 1, 2 -> move to 0 degree, (1000 ms, 0 ms delay)
    //  m_CMot.Play("1000, 0, 6, 0");
    //}
  }
  else
  {
    cnt++;
    delay(1000);
  }
}
