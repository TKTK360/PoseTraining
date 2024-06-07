#define LGFX_M5STACK    // M5Stack M5Stack Basic / Gray / Go / Fire
#define LGFX_AUTODETECT // 自動認識 (D-duino-32 XS, WT32-SC01, PyBadge はパネルID読取りが出来ないため自動認識の対象から外れています)

#include <M5Stack.h>
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>  // クラス"LGFX"を準備します
#include <Seeed_Arduino_SSCMA.h>
//#include <DFMiniMp3.h>
#include "DFRobotDFPlayerMini.h"

#define RXD2 17 // Connects to module's RX 
#define TXD2 16 // Connects to module's TX 

static LGFX lcd;                 // LGFXのインスタンスを作成。
static LGFX_Sprite canvas(&lcd); // スプライトを使う場合はLGFX_Spriteのインスタンスを作成。

SSCMA AI;

HardwareSerial dfSD(2); // Use UART channel 1
DFRobotDFPlayerMini player;

// Mode
const int modeWait = 1;
const int modeTraining = 2;
const int modeEndt = 3;
int g_mode = 0;

// Counter
int g_cnt = 4;
unsigned long g_previousMillis = 0;         // 前回の描画処理が行われた時間
const unsigned long g_startDelay = 4000;    // 描画処理を開始するまでの遅延時間 (3秒)
const unsigned long g_drawInterval = 1000;  // 1秒ごとの描画インターバル

// Training
int g_trainCnt = 10;

int g_cntLeft = 0;
int g_cntRight = 0;

int g_traLeft = -1;
int g_traRight = -1;


//----------------------------------------------------------------------------
// WaitFunctin
//----------------------------------------------------------------------------
void WaitFunctin()
{
  // 現在の時間を取得
  unsigned long currentMillis = millis();
  
  // 3秒経過後に描画処理を開始
  if (currentMillis >= g_startDelay) {
    // 次の描画時間が来たら描画処理を行う
    if (currentMillis - g_previousMillis >= g_drawInterval) {
      g_previousMillis = currentMillis;

      // 描画処理の呼び出し
      g_cnt -= 1;
      if (g_cnt <= 0) {
        g_mode = modeTraining;  // Training

        //mp3.playMp3FolderTrack(1);  // sd:/mp3/000[i].mp3が再生
        //mp3.start();
        player.play(1);

        Serial.println("=== TRAINING ===");
      }
      Serial.println(g_cnt);

      // draw
      canvas.fillScreen(BLACK); 

      canvas.setTextColor(WHITE);         // 文字色指定
      canvas.setTextSize(3.5);            // 文字倍率変更
      canvas.setCursor(100, 30, &Font7);  // 座標とフォントを指定（x, y, フォント）
      canvas.printf("%d", g_cnt);         // カウント数表示

      canvas.pushSprite(0, 0);            // メモリ内に描画したcanvasを座標を指定して表示する
    }
  }
}


//----------------------------------------------------------------------------
// ArcFunction
//----------------------------------------------------------------------------
float ArcFunction(int x1, int y1,
                  int x2, int y2,
                  int x3, int y3)
{
  // ベクトルの成分
  float BAx = x1 - x2;
  float BAy = y1 - y2;
  float BCx = x3 - x2;
  float BCy = y3 - y2;
  
  // 内積
  float dotProduct = BAx * BCx + BAy * BCy;
  
  // ベクトルの大きさ
  float magnitudeBA = sqrt(BAx * BAx + BAy * BAy);
  float magnitudeBC = sqrt(BCx * BCx + BCy * BCy);
  
  // cosθの計算
  float cosTheta = dotProduct / (magnitudeBA * magnitudeBC);
  
  // θの計算（ラジアン）
  float theta = acos(cosTheta);
  
  // θの計算（度）
  float thetaDegrees = theta * 180.0 / PI;

  return thetaDegrees;
}


//----------------------------------------------------------------------------
// TrainingFunctin
//----------------------------------------------------------------------------
void TrainingFunctin()
{
  if (AI.invoke(1, false, false)) { // invoke once, no filter, not contain image
    return;
  }

  //AI.perf().prepocess;
  //AI.perf().inference;
  //AI.perf().postprocess;

  int num = AI.keypoints().size();
  if (num < 1) {
    g_trainCnt -= 1;
    if (g_trainCnt > 0) {
      return;
    }
  }
  g_trainCnt = 10;
  String str;

  canvas.fillScreen(BLACK); 

  // HumanPose
  int colorType[] ={RED, GREEN, BLUE, YELLOW, WHITE};
  uint16_t left = 0;
  uint16_t top = 0;
  float leftArc = 0;
  float rightArc = 0;

  // Num
  for (int i = 0; i < num; i++) {
    str = i;
    // Bone
    int pose = AI.keypoints()[i].points.size();
    for (int j = 0; j < pose; j++) {
      /*
      "nose",  # 0
      "eye(L)",  # 1
      "eye(R)",  # 2
      "ear(L)",  # 3
      "ear(R)",  # 4
      "shoulder(L)",  # 5
      "shoulder(R)",  # 6
      "elbow(L)",  # 7
      "elbow(R)",  # 8
      "wrist(L)",  # 9
      "wrist(R)",  # 10
      "hip(L)",  # 11
      "hip(R)",  # 12
      "knee(L)",  # 13
      "knee(R)",  # 14
      "ankle(L)",  # 15
      "ankle(R)",  # 16
      //*/
      // Left: shoulder(L)# 5, elbow(L)# 7, hip(L)# 11
      // Right: shoulder(R)# 6, elbow(R)# 8, hip(R)# 12

      left = 320 - AI.keypoints()[i].points[j].x;
      top = AI.keypoints()[i].points[j].y;
/*
      str +=",[";
      str += left;
      str +=",";
      str += top;
      str += "]";
//*/      
      canvas.fillCircle(left, top, 3, colorType[i]);  // 円の塗り

      if (pose >= 12) {
        // 下げ１３０、上げ６０
        // Left
        int shoulderX = 320 - AI.keypoints()[i].points[5].x;
        int shoulderY = AI.keypoints()[i].points[5].y;
        int elbowX = 320 - AI.keypoints()[i].points[7].x;
        int elbowY = AI.keypoints()[i].points[7].y;
        int hipX = 320 - AI.keypoints()[i].points[11].x;
        int hipY = AI.keypoints()[i].points[11].y;

        canvas.drawLine(shoulderX, shoulderY,elbowX, elbowY, BLUE);
        canvas.drawLine(hipX, hipY,shoulderX, shoulderY, BLUE);
        leftArc = ArcFunction(shoulderX, shoulderY, elbowX, elbowY, hipX, hipY);

        // Right
        shoulderX = 320 - AI.keypoints()[i].points[6].x;
        shoulderY = AI.keypoints()[i].points[6].y;
        elbowX = 320 - AI.keypoints()[i].points[8].x;
        elbowY = AI.keypoints()[i].points[8].y;
        hipX = 320 - AI.keypoints()[i].points[12].x;
        hipY = AI.keypoints()[i].points[12].y;

        canvas.drawLine(shoulderX, shoulderY,elbowX, elbowY, BLUE);
        canvas.drawLine(hipX, hipY,shoulderX, shoulderY, BLUE);
        rightArc = ArcFunction(shoulderX, shoulderY, elbowX, elbowY, hipX, hipY);

        // Gage
        canvas.setTextColor(WHITE);         // 文字色指定
        canvas.setTextSize(1);            // 文字倍率変更

        int gauge = leftArc;
        gauge = min(gauge, 130);
        gauge = max(gauge, 60);
        int gaugeValue = map(gauge, 130, 60, 0, 250);
        canvas.fillRect( 50, 200, gaugeValue, 14, GREEN);

        if (gaugeValue > 100) {
          if (g_traLeft > 0) {
            g_traLeft = -1;
          }
        } else if (gaugeValue < 75) {
          if (g_traLeft < 0) {
            g_traLeft = 1;
            g_cntLeft++;
          }
        } 

        canvas.setCursor(5, 150, &Font7);  // 座標とフォントを指定（x, y, フォント）
        canvas.printf("%d", g_cntLeft);         // カウント数表示


        gauge = rightArc;
        gauge = min(gauge, 130);
        gauge = max(gauge, 60);
        gaugeValue = map(gauge, 130, 60, 0, 250);
        canvas.fillRect( 250, 220, -gaugeValue, 14, ORANGE);

        if (gaugeValue > 100) {
          if (g_traRight > 0) {
            g_traRight = -1;
          }
        } else if (gaugeValue < 75) {
          if (g_traRight < 0) {
            g_traRight = 1;
            g_cntRight++;
          }
        } 

        if (g_cntRight >= 10) {
          canvas.setCursor(245, 150, &Font7);
        } else {
          canvas.setCursor(270, 150, &Font7);
        }
        canvas.printf("%d", g_cntRight);         // カウント数表示
      }    
    }
/*    
    str += "]";
    Serial.println(str);
//*/
  }

  // Count
  canvas.setTextColor(WHITE);         // 文字色指定
  canvas.setTextSize(1.5);            // 文字倍率変更
  canvas.setCursor(4, 2, &Font2);     // 座標とフォントを指定（x, y, フォント）
  canvas.printf("NUM= %d", num);      // カウント数表示
  canvas.setCursor(60, 2, &Font2);
  canvas.printf("%d,  %d", (int)leftArc, (int)rightArc);
  
  canvas.pushSprite(0, 0);      // メモリ内に描画したcanvasを座標を指定して表示する
}


//----------------------------------------------------------------------------
// Setup
//----------------------------------------------------------------------------
void setup(void)
{
  M5.begin();               // 本体初期化
  Serial.begin(9600);

  //mp3初期化
  dfSD.begin(9600, SERIAL_8N1, RXD2, TXD2);
  if (player.begin(dfSD)) {
    Serial.println("OK");
    player.volume(14); //0~30で音量指定
  }

  
  AI.begin();

  lcd.init();
  lcd.setRotation(1);       // 回転方向を 0～3 の4方向から設定します。(4～7を使用すると上下反転になります。)  
  lcd.setBrightness(128);   // バックライトの輝度を 0～255 の範囲で設定します。
  //lcd.setColorDepth(16);  // RGB565の16ビットに設定
  //lcd.setColorDepth(24);  // RGB888の24ビットに設定(表示される色数はパネル性能によりRGB666の18ビットになります)

  canvas.setColorDepth(8);    // カラーモード設定（書かなければ初期値16bit。24bit（パネル性能によっては18bit）は対応していれば選択可）
                              // CORE2 GRAY のスプライトは16bit以上で表示されないため8bitに設定
  canvas.createSprite(lcd.width(), lcd.height());
  canvas.setPaletteColor(1, 0x0000FFU);    // パレット1番を青に設定
  canvas.setPaletteColor(2, 0x00FF00U);    // パレット2番を緑に設定
  canvas.setPaletteColor(3, 0xFF0000U);    // パレット3番を赤に設定

  canvas.fillScreen(WHITE); 
  canvas.setTextColor(BLACK);       // 文字色指定
  canvas.setTextSize(3);            // 文字倍率変更
  canvas.drawString("POSE TRAINING", 15, 100, &Font2);  
  canvas.pushSprite(0, 0);          // メモリ内に描画したcanvasを座標を指定して表示する

  Serial.println("=== START ===");
}


//----------------------------------------------------------------------------
// loop
//----------------------------------------------------------------------------
void loop(void)
{
  M5.update();

  // Button
  //  Counter
  if (M5.BtnB.isPressed()) {
    if (modeWait != g_mode) {
      g_cnt = 4;
      g_previousMillis = 0;         // 前回の描画処理が行われた時間
      g_cntLeft = 0;
      g_cntRight = 0;
      g_traLeft = -1;
      g_traRight = -1;

      g_mode = modeWait;
      Serial.println("=== COUNTER ===");
    }
  }
  //  Pose
  else if (M5.BtnA.isPressed()) {
    if (modeTraining != g_mode) {
      g_mode = modeTraining;
      Serial.println("=== TRAINING ===");
    }
  }

  switch(g_mode)
  {
    case modeWait:
      WaitFunctin();
      break;

    case modeTraining:
      TrainingFunctin();
      break;

    default:
      break;
  }

  delay(10);
}
