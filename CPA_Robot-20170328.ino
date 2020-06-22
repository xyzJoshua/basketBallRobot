#include <L298Drv.h>

/*
 * 동양미래대학교 로봇자동화공학부 창의과제기초 로봇 프로그램
 * 2017년 3월 27일
 */

//- 모터0(7, 22 핀 이용)의 PWM 매크로 값 
#define MTR0_HIGH   254     // 좌측 모터 고속에 해당하는 값으로 최대 255
#define MTR0_MID    250     // 좌측 모터 중속에 해당하는 값으로 최대 255
#define MTR0_LOW    185     // 좌측 모터 저속에 해당하는 값으로 최대 255

//- 모터1(8, 23 핀 이용)의 PWM 매크로 값
#define MTR1_HIGH   254     // 우측 모터 고속에 해당하는 값으로 최대 255
#define MTR1_MID    250//우측 모터 중속에 해당하는 값으로 최대 255
#define MTR1_LOW    185   // 우측 모터 저속에 해당하는 값으로 최대 255

//- 모터2(9, 24 핀 이용)의 PWM 매크로 값
#define MTR2_HIGH   140     // 모터2 고속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR2_MID    140   // 모터2 중속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR2_LOW     70     // 모터2 저속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)

//- 모터3(10, 25 핀 이용)의 PWM 매크로 값
#define MTR3_HIGH   140     // 모터3 고속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR3_MID    140     // 모터3 중속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR3_LOW     70     // 모터3 저속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)

//- 모터4(10, 25 핀 이용)의 PWM 매크로 값
#define MTR4_HIGH   140     // 모터4 고속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR4_MID    100     // 모터4 중속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR4_LOW     70     // 모터4 저속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)

//- 모터5(10, 25 핀 이용)의 PWM 매크로 값
#define MTR5_HIGH   140     // 모터5 고속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR5_MID    100     // 모터5 중속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)
#define MTR5_LOW     70     // 모터5 저속에 해당하는 값으로 최대 255 (너무 크면 문제 됨)

//- L298Drv 클래스 인스턴스
L298Drv Motor0(7, 22);  // 방향을 바꾸려면 Motor0(7, 22, true);
L298Drv Motor1(8, 23);  // 방향을 바꾸려면 Motor0(8, 23, true);
L298Drv Motor2(9, 24);  // 방향을 바꾸려면 Motor0(9, 24, true);
L298Drv Motor3(10, 25);  // 방향을 바꾸려면 Motor0(10, 25, true);
L298Drv Motor4(11, 26);  // 방향을 바꾸려면 Motor0(11, 26, true);
L298Drv Motor5(12, 27);  // 방향을 바꾸려면 Motor0(12, 27, true);

//- 블루투스 모듈 통신 관련 매크로
#define BT_PACKET_START 0xf5
#define BT_PACKET_LEN   4
#define MTR_RF    0x1         // Robot Forward
#define MTR_RB    0x2         // Robot Backward
#define MTR_RL    0x4         // Robot Left
#define MTR_RR    0x8         // Robot Right
#define MTR_2F    0x10
#define MTR_2B    0x20
#define MTR_FS    0x40
#define MTR_3F    0x100
#define MTR_3B    0x200
#define MTR_4F    0x400
#define MTR_4B    0x800
#define MTR_5F    0x1000
#define MTR_5B    0x2000
#define MTR_SS    0x4000

//- 블루투스 모듈 통신 관련 변수
unsigned char btData[BT_PACKET_LEN];
unsigned char btPtr = 0;
unsigned int btCommand;

//- setup()
void setup() {
  Serial.begin(9600);
  Serial1.begin(57600);

  pinMode(13, OUTPUT);
}

//- 블루투스로 한 패킷이 전송되면 true 리턴, 그렇지 않으면 false 리턴
boolean btReadPacket(void) {
  unsigned char rdata;
  
  while (Serial1.available()) {
    rdata = Serial1.read();
    if (rdata == BT_PACKET_START) {
      btPtr = 0;
    }
    btData[btPtr] = rdata;
    if (++btPtr == BT_PACKET_LEN) {
      btPtr = 0;
      if (((btData[1] + btData[2]) & 0x7f) == btData[3]) {
        btCommand = btData[1] + (word(btData[2]) << 8);
        return true;
      }
    }
  }
  return false;
}

void mtrDrive() {
  //- 전진
  if (btCommand & MTR_RF) {
    if (btCommand & MTR_FS) {
      Motor0.drive(MTR0_HIGH);
      Motor1.drive(MTR1_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor0.drive(MTR0_LOW);
      Motor1.drive(MTR1_LOW);
    }
    else {
      Motor0.drive(MTR0_MID);
      Motor1.drive(MTR1_MID);
    }
  }
  //- 후진
  else if (btCommand & MTR_RB)
  {
    if (btCommand & MTR_FS) {
      Motor0.drive(-MTR0_HIGH);
      Motor1.drive(-MTR1_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor0.drive(-MTR0_LOW);
      Motor1.drive(-MTR1_LOW);
    }
    else {
      Motor0.drive(-MTR0_MID);
      Motor1.drive(-MTR1_MID);
    }
  }
  //- 좌회전
  else if (btCommand & MTR_RL)
  {
    if (btCommand & MTR_FS) {
      Motor0.drive(-MTR0_HIGH);
      Motor1.drive(MTR1_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor0.drive(-MTR0_LOW);
      Motor1.drive(MTR1_LOW);
    }
    else {
      Motor0.drive(-MTR0_MID);
      Motor1.drive(MTR1_MID);
    }
  }
  //- 우회전
  else if (btCommand & MTR_RR)  {
    if (btCommand & MTR_FS) {
      Motor0.drive(MTR0_HIGH);
      Motor1.drive(-MTR1_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor0.drive(MTR0_LOW);
      Motor1.drive(-MTR1_LOW);
    }
    else {
      Motor0.drive(MTR0_MID);
      Motor1.drive(-MTR1_MID);
    }
  }
  else {
      Motor0.drive(0);
      Motor1.drive(0);
  }

  //- CH2 모터
  if (btCommand & MTR_2F) {
    if (btCommand & MTR_FS) {
      Motor2.drive(MTR2_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor2.drive(MTR2_LOW);
    }
    else {
      Motor2.drive(MTR2_MID);
    }
  }
  else if (btCommand & MTR_2B) {
    if (btCommand & MTR_FS) {
      Motor2.drive(-MTR2_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor2.drive(-MTR2_LOW);
    }
    else {
      Motor2.drive(-MTR2_MID);
    }
  }
  else {
      Motor2.drive(0);
  }

  //- CH3 모터
  if (btCommand & MTR_2F) {
    if (btCommand & MTR_FS) {
      Motor3.drive(MTR3_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor3.drive(MTR3_LOW);
    }
    else {
      Motor3.drive(MTR3_MID);
    }
  }
  else if (btCommand & MTR_2B) {
    if (btCommand & MTR_FS) {
      Motor3.drive(-MTR3_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor3.drive(-MTR3_LOW);
    }
    else {
      Motor3.drive(-MTR3_MID);
    }
  }
  else {
    Motor3.drive(0);
  }

  //- CH4 모터
  if (btCommand & MTR_4F) {
    if (btCommand & MTR_FS) {
      Motor4.drive(MTR4_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor4.drive(MTR4_LOW);
    }
    else {
      Motor4.drive(MTR4_MID);
    }
  }
  else if (btCommand & MTR_4B) {
    if (btCommand & MTR_FS) {
      Motor4.drive(-MTR4_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor4.drive(-MTR4_LOW);
    }
    else {
      Motor4.drive(-MTR4_MID);
    }
  }
  else {
    Motor4.drive(0);
  }

  //- CH5 모터
  if (btCommand & MTR_5F) {
    if (btCommand & MTR_FS) {
      Motor5.drive(MTR5_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor5.drive(MTR5_LOW);
    }
    else {
      Motor5.drive(MTR5_MID);
    }
  }
  else if (btCommand & MTR_5B) {
    if (btCommand & MTR_FS) {
      Motor5.drive(-MTR5_HIGH);
    }
    else if (btCommand & MTR_SS) {
      Motor5.drive(-MTR5_LOW);
    }
    else {
      Motor5.drive(-MTR5_MID);
    }
  }
  else {
    Motor5.drive(0);
  }    
}

void loop() {
  static unsigned long t_prev = 0;
  
  if (btReadPacket()) {
    mtrDrive();
    t_prev = millis();
  }

  if ((millis() - t_prev) >= 250) {
    Motor0.drive(0);
    Motor1.drive(0);
    Motor2.drive(0);
    Motor3.drive(0);
    Motor4.drive(0);
    Motor5.drive(0);
  }

  digitalWrite(13, ((millis()/250)%2) ? HIGH : LOW);
}
