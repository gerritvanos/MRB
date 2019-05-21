 #include <Servo.h>
constexpr size_t array_size = 3;
Servo servo_A;
Servo servo_B;
Servo servo_C;

constexpr Servo* servos[array_size] = {&servo_A, &servo_B, &servo_C}; 
int degrees[array_size] ={110,110,110};

int max_servo = 120;
int min_servo = 90 ;

String data;
void setup() {
  servos[0]->attach(2);
  servos[1]->attach(3);
  servos[2]->attach(4);
  Serial.begin(115200); 
}

int check_degree(int input){
  if (input > max_servo){
    return max_servo;
  } else if (input < min_servo) {
    return min_servo;
  } 
  return input;
}

void get_servo_info(){
  if(Serial.available()){
    data = Serial.readStringUntil('\n');
    if (data.startsWith("Y")){
      degrees[0] = check_degree(degrees[0]-data.substring(1).toInt());
      degrees[2] = check_degree(degrees[2]+data.substring(1).toInt());
    }
    else if (data.startsWith("X")){
      degrees[1] = check_degree(degrees[1]-data.substring(1).toInt());
    }
  }
}

void update_servo_position(){
  for(size_t i = 0 ; i <array_size; ++i){
    if (i == 1){
      servos[i]->write(degrees[i]+7);
    } else{
      servos[i]->write(degrees[i]);
    }  
  }
}

void loop() {
  get_servo_info();
  update_servo_position();
}
