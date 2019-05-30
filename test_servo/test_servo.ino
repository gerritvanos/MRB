 #include <Servo.h>

//create an array for the 3 servo's
constexpr size_t array_size = 3;

//construct 3 servo objects
Servo servo_A;
Servo servo_B;
Servo servo_C;

//add servo objects to array
constexpr Servo* servos[array_size] = {&servo_A, &servo_B, &servo_C}; 
//create an array with the degree every servo needs to be set at
//degrees[0] corresponding to servo_A etc.
int degrees[array_size] ={110,110,110};

//setting max and min point for all the servo's to prevent them from over tilting the plate on top
//or from getting stuck on the wooden enclosure
int max_servo = 115;
int min_servo = 95 ;

// constructing the string object that will contain the data received from a PC
String data;

void setup() {
  //attaching digital pins to servo objects
  // Servo_A = pin 2, Servo_B = pin 3, Servo_C = pin 4
  servos[0]->attach(2);
  servos[1]->attach(3);
  servos[2]->attach(4);
  //setting serial baudrate to 115200
  Serial.begin(115200); 
}

//this function checks if the given degree value is below the max and above the minimal servo degrees
//this will prevent the servo's from overshooting
int check_degree(int input){
  if (input > max_servo){
    return max_servo;
  } else if (input < min_servo) {
    return min_servo;
  } 
  return input;
}

//this function gets the servo data from the serial input
//Also the Y input will be converted to 1 positive and 1 negative value to acomplish one servo going up and the other going down.
void get_servo_info(){
  if(Serial.available()){
    data = Serial.readStringUntil('\n');
    if (data.startsWith("Y")){
      degrees[0] = check_degree(degrees[0] - data.substring(1).toInt());
      degrees[2] = check_degree(degrees[2] + data.substring(1).toInt());
    }
    else if (data.startsWith("X")){
      degrees[1] = check_degree(degrees[1]-data.substring(1).toInt());
    }
  }
}

// this function sets the new position to the servo's
void update_servo_position(){
  for(size_t i = 0 ; i <array_size; ++i){
    if (i == 1){
      servos[i]->write(degrees[i]+7); //the 7 is needed because one of the 3 servo's has some weird offset
    } else{
      servos[i]->write(degrees[i]);
    }  
  }
}

void loop() {
  get_servo_info();
  update_servo_position();
}
