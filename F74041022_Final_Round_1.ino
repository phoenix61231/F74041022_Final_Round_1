#include <Ultrasonic.h>
#define front_trig 22
#define front_echo 32
#define left_trig 28
#define left_echo 30
#define right_trig 24
#define right_echo 26
#define motor_right_back 6
#define motor_right_for 3
#define motor_left_back 4
#define motor_left_for 5

#define front_dir 0
#define left_dir 1
#define right_dir 2

Ultrasonic front(front_trig, front_echo);
Ultrasonic left(left_trig, left_echo);
Ultrasonic right(right_trig, right_echo);

int v = 130,last_cross, mapping_front=0, mapping_cross=0;
float front_dis, right_dis, left_dis, temp=0.0;
bool back = false, front_sta, left_sta, right_sta;
bool mapping[6][6] = {true},inf_loop = false;

void for_back(int mot_1,int mot_2,int dur){
  analogWrite(mot_1, v);
  analogWrite(mot_2, v);
  delay(dur);
  analogWrite(mot_1,0);
  analogWrite(mot_2,0);
}

void turn(int mot,int dur){
  analogWrite(mot, v); 
  delay(dur);  
  analogWrite(mot,0);
}

void line(int front,int left,int right){
  if(front < 8){
    for_back(motor_right_back,motor_left_back,450);
    front_sta = true;
    left_sta = true;
    right_sta = true;
  }
  else if (left < 5) {
    turn(motor_right_back,190);
    front_sta = true;
    left_sta = true;
    right_sta = true;   
  }
  else if (right < 5) {
    turn(motor_left_back,190);
    front_sta = true;
    left_sta = true;
    right_sta = true;    
  }
}
void setup() {
  Serial.begin(9600);
  pinMode(motor_right_back, OUTPUT);
  pinMode(motor_right_for, OUTPUT);
  pinMode(motor_left_back, OUTPUT);
  pinMode(motor_left_for, OUTPUT);
  analogWrite(motor_right_for, 0);
  analogWrite(motor_right_back, 0);
  analogWrite(motor_left_for, 0);
  analogWrite(motor_left_back, 0);

}
//調整轉彎後前進距離

void loop() {
  //測距
  long front_sec = front.timing(), left_sec = left.timing(), right_sec = right.timing();
  front_dis = front.convert(front_sec, Ultrasonic::CM);
  left_dis = left.convert(left_sec, Ultrasonic::CM);
  right_dis = right.convert(right_sec, Ultrasonic::CM);
  //紀錄路徑
  if(temp==0.0){
    temp = front_dis;
  }
  else{
    if(front_dis-temp>28){
      temp=front_dis;
      mapping[mapping_cross][mapping_front] = false;
      mapping_front++;
    }
  }  
  //直線校正
  line(front_dis,left_dis,right_dis); 
  //路線判斷
  if(back!=true){
    if (front_dis <= 8) { front_sta = false;    
    }
    else {  front_sta = true;
    }   
    if (left_dis >= 30) { left_sta = true;
    }
    else {  left_sta = false;
    }
    if (right_dis >= 30) {  right_sta = true;
    }
    else {  right_sta = false;
    }  
  }
  else{
    if (left_dis >= 100) { left_sta = true;
    }
    else {  left_sta = false;
    }
    if (right_dis >= 100) {  right_sta = true;
    }
    else {  right_sta = false;
    }  
  }
  //
  Serial.println(front_dis);
  Serial.println(front_sta); 
  Serial.println(left_dis);
  Serial.println(left_sta); 
  Serial.println(right_dis);
  Serial.println(right_sta); 
  //不合理
  /*if(front_dis>500 || right_dis>500 || left_dis>500){
    front_sta = true;
    left_sta = true;
    right_sta = true;
  }*/
  //按照路線判斷走
  if (back == false) {
    switch (front_sta) {
      case true:
        switch (left_sta) {
          case true:
            switch (right_sta) {
              case true:
                //TTT
                //worst situation
                 for_back(motor_right_for,motor_left_for,500);
                break;
              case false:  
                //TTF 
                //if(mapping[mapping_cross][mapping_front]) { 
                  for_back(motor_right_for,motor_left_for,150);
                  turn(motor_right_for,560);
                  for_back(motor_right_for,motor_left_for,1000);                
                  last_cross = left_dir;
                  //mapping_cross++;                             
                //}               
                break;
            }
            break;
          case false:
            switch (right_sta) {
              case true:
                //TFT
                //if(mapping[mapping_cross][mapping_front]) {
                  for_back(motor_right_for,motor_left_for,150); 
                  turn(motor_left_for,560);                
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = right_dir ;
                  //mapping_cross++;                            
                //}                
                break;
              case false:
                //TFF
                for_back(motor_right_for,motor_left_for,100);
                break;
            }
            break;
        }
        break;
      case false:
        switch (left_sta) {
          case true:
            switch (right_sta) {
              case true:
                //FTT
                //worst situation
                if(inf_loop){
                  turn(motor_right_for,560);                
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = front_dir;
                  inf_loop = false;
                }
                else{
                  turn(motor_left_for,560);                
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = front_dir;
                  inf_loop = true;
                }
                turn(motor_left_for,560);                
                for_back(motor_right_for,motor_left_for,1000);
                last_cross = front_dir;
                //mapping_cross++;
                break;
              case false:
                //FTF
                for_back(motor_right_for,motor_left_for,150);
                turn(motor_right_for,560); 
                for_back(motor_right_for,motor_left_for,1000);
                //mapping_cross++;
                break;
            }
            break;
          case false:
            switch (right_sta) {
              case true:
                //FFT
                for_back(motor_right_for,motor_left_for,150);
                turn(motor_left_for,560); 
                for_back(motor_right_for,motor_left_for,1000);
                //mapping_cross++;
                break;
              case false:
                //FFF
                for_back(motor_right_for,motor_left_back,540);
                back = true;
                break;
            }
            break;
        }
        break;
    }
  }
  else {
    //倒退    
    if (left_sta != false && right_sta != false) {
      switch (last_cross) {
        case 0:
          for_back(motor_right_for,motor_left_for,1000);
          break;
        case 1:
          for_back(motor_right_for,motor_left_for,50);
          turn(motor_right_for,590);          
          for_back(motor_right_for,motor_left_for,1000);
          break;
        case 2:
          for_back(motor_right_for,motor_left_for,50);
          turn(motor_left_for,590);
          for_back(motor_right_for,motor_left_for,1000);
          break;        
      }
      back = false;
    }
    else {
      for_back(motor_right_for,motor_left_for,200);
    }
  }
}
