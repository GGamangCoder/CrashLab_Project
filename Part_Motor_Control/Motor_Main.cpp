#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <fstream>
// #include <wiringPi.h>

////////////////////////////////////
#define X_BOUND_1 260
#define X_BOUND_2 380       // sense where person is
#define BOUND_MID 320
#define BIG_SIZE 100000       // big enough to interact with target person
#define BIG_SIZE_P 500      // size of person
#define BIG_SIZE_S 250      // size of sign
#define SENSOR_LIMIT 50     // distance, need to interation with person
#define Kp 0.11
///////////////////////////////////// define value

/////////////////////////////
bool person_detect = false;
bool sign1_detect = false;
float person_x = 0;
float person_size = 0;  /////// cam data(need to make more sign data)

///////////////////////////
float sign1_x = 0;
float sign1_size = 0;
float sign2_x = 0;
float sign2_size = 0;
float sign_x = 0;
float sign_size = 0;
///////////////////////
float Sensor_front = 0;
float Sensor_left = 0;
float Sensor_right = 0;
float Sensor_Back = 0;
////////////////////////// sensor data

///////////////
int sequence = 0; // sequence 0 to 3
int dt = 0;     // target person detecting time
int dt_1 = 0;   // if detected person is gone away, check time(wait time)

bool state = false; // display data
///////////////


void Text_Input(void)
{
    int i = 0;
    std::size_t found;
    std::ifstream inFile;
    inFile.open("/home/ubuntu/catkin_ws/src/motor_pkg/motor_input.txt");
    for (std::string line; std::getline(inFile, line);)
    {
        found = line.find("=");

        switch (i)
        {
        case 0: PWM_range = atof(line.substr(found + 2).c_str()); break;
        case 1: PWM_frequency = atof(line.substr(found + 2).c_str()); break;
        case 2: PWM_limit = atof(line.substr(found + 2).c_str()); break;
        case 3: Control_cycle = atof(line.substr(found + 2).c_str()); break;
        case 4: Acceleration_ratio = atof(line.substr(found + 2).c_str()); break;
        case 5: Wheel_radius = atof(line.substr(found + 2).c_str()); break;
        case 6: Robot_radius = atof(line.substr(found + 2).c_str()); break;
        case 7: Encoder_resolution = atof(line.substr(found + 2).c_str()); break;
            //case :  = atof(line.substr(found+2).c_str()); break;
        }
        i += 1;
    }
    inFile.close();
}
int Motor_Setup(void)
{
    pinum = pigpio_start(NULL, NULL);

    if (pinum < 0)
    {
        ROS_INFO("Setup failed");
        ROS_INFO("pinum is %d", pinum);
        return 1;
    }

    set_mode(pinum, motor1_DIR, PI_OUTPUT);
    set_mode(pinum, motor2_DIR, PI_OUTPUT);
    set_mode(pinum, motor1_PWM, PI_OUTPUT);
    set_mode(pinum, motor2_PWM, PI_OUTPUT);
    set_mode(pinum, motor1_ENA, PI_INPUT);
    set_mode(pinum, motor1_ENB, PI_INPUT);
    set_mode(pinum, motor2_ENA, PI_INPUT);
    set_mode(pinum, motor2_ENB, PI_INPUT);
    ///////////////////////////////////////////////////
    set_mode(pinum, motor_sv, PI_INPUT);
    gpio_write(pinum, motor_sv, PI_HIGH);
    set_pull_up_down(pinum, motor_sv, PI_PUD_DOWN);

    // pinmode(motor_sv, OUTPUT);
    // digitalWrite(motor_sv, HIGH);
    ///////////////////////////////////////////////////

    gpio_write(pinum, motor1_DIR, PI_LOW);
    gpio_write(pinum, motor2_DIR, PI_LOW);

    set_PWM_range(pinum, motor1_PWM, PWM_range);
    set_PWM_range(pinum, motor2_PWM, PWM_range);
    set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
    set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
    set_PWM_dutycycle(pinum, motor1_PWM, 0);
    set_PWM_dutycycle(pinum, motor1_PWM, 0);

    set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

    current_PWM1 = 0;
    current_PWM2 = 0;

    current_Direction1 = true;
    current_Direction2 = true;

    acceleration = PWM_limit / (Acceleration_ratio);

    ROS_INFO("Setup Fin");
    return 0;
}
void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A++;
    else EncoderCounter1A--;
    EncoderSpeedCounter1++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B++;
    else EncoderCounter1B--;
    EncoderSpeedCounter1++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A--;
    else EncoderCounter2A++;
    EncoderSpeedCounter2++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B--;
    else EncoderCounter2B++;
    EncoderSpeedCounter2++;
}
int Motor1_Encoder_Sum()
{
    EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
    return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
    EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
    return EncoderCounter2;
}
void Init_Encoder(void)
{
    EncoderCounter1 = 0;
    EncoderCounter2 = 0;
    EncoderCounter1A = 0;
    EncoderCounter1B = 0;
    EncoderCounter2A = 0;
    EncoderCounter2B = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void)
{
    Text_Input();
    Motor_Setup();
    Init_Encoder();
    Interrupt_Setting();

    Wheel_round = 2 * PI * Wheel_radius;
    Robot_round = 2 * PI * Robot_radius;

    switch_direction = true;
    Theta_Distance_Flag = 0;

    ROS_INFO("PWM_range %d", PWM_range);
    ROS_INFO("PWM_frequency %d", PWM_frequency);
    ROS_INFO("PWM_limit %d", PWM_limit);
    ROS_INFO("Control_cycle %f", Control_cycle);
    ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
    ROS_INFO("Initialize Complete");

    printf("\033[2J");
}

void Motor_Controller(int motor_num, bool direction, int pwm)
{
    int local_PWM = Limit_Function(pwm);

    if (motor_num == 1)
    {
        if (direction == true)
        {
            gpio_write(pinum, motor1_DIR, PI_LOW);
            set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
            current_PWM1 = local_PWM;
            current_Direction1 = true;
        }
        else if (direction == false)
        {
            gpio_write(pinum, motor1_DIR, PI_HIGH);
            set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
            current_PWM1 = local_PWM;
            current_Direction1 = false;
        }
    }

    else if (motor_num == 2)
    {
        if (direction == true)
        {
            gpio_write(pinum, motor2_DIR, PI_LOW);
            set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
            current_PWM2 = local_PWM;
            current_Direction2 = true;
        }
        else if (direction == false)
        {
            gpio_write(pinum, motor2_DIR, PI_HIGH);
            set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
            current_PWM2 = local_PWM;
            current_Direction2 = false;
        }
    }
}

void Theta_Turn(double Theta, int PWM)
{
  double local_encoder;
  int local_PWM = Limit_Function(PWM);
  if(Theta_Distance_Flag == 1)
  {
      Init_Encoder();
      Theta_Distance_Flag = 2;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if(Theta > 0)
  {
    local_encoder = (Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, false, local_PWM);
    Motor_Controller(2, false, local_PWM);
  }
  else
  {
    local_encoder = -(Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, true, local_PWM);
    Motor_Controller(2, true, local_PWM);
  }

  if(EncoderCounter1 > local_encoder)
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    Theta_Distance_Flag = 3;
  }
}

void Distance_Go(double Distance, int PWM)
{
  double local_encoder = (Encoder_resolution*4*Distance)/Wheel_round;
  int local_PWM = Limit_Function(PWM);
  bool Direction = true;
  if(Distance < 0)
  {
    Direction = false;
    local_encoder = -local_encoder;
  }
  if(Theta_Distance_Flag == 3)
  {
      Init_Encoder();
      Theta_Distance_Flag = 4;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if(EncoderCounter1 < local_encoder)
  {
    if(Direction==true)
    {
      Motor_Controller(1, false, local_PWM);
      Motor_Controller(2, true, local_PWM);
    }
    else
    {
      Motor_Controller(1, true, local_PWM);
      Motor_Controller(2, false, local_PWM);
    }
  }
  else
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    Theta_Distance_Flag = 0;
  }
}

int Limit_Function(int pwm)
{
    int output;
    if (pwm > PWM_limit * 2)
    {
        output = PWM_limit;
        ROS_WARN("PWM too fast!!!");
    }
    else if (pwm > PWM_limit)output = PWM_limit;
    else if (pwm < 0)
    {
        output = 0;
        ROS_WARN("trash value!!!");
    }
    else output = pwm;
    return output;
}
void RPM_Calculator()
{
    RPM_Value1 = (EncoderSpeedCounter1 * (60 * Control_cycle)) / (Encoder_resolution * 4);
    EncoderSpeedCounter1 = 0;
    RPM_Value2 = (EncoderSpeedCounter2 * (60 * Control_cycle)) / (Encoder_resolution * 4);
    EncoderSpeedCounter2 = 0;
}
void Motor_View()
{
    RPM_Calculator();
    printf("\033[2J");
    printf("\033[1;1H");
    printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
    printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
    printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
    printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
    printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
    printf("Acc  :%10.0d\n", acceleration);
    printf("\n");
}

float X_error = 0;
float P_error = 0;
float PWM_R = 0;
float PWM_L = 0;

///////////////////////////////////////////////////
///////////////////////////////////////////////////
/*
void CamDataCallback(const my_msgs::CameraData &msg) { // function which subscribes from camera
    person_detect = msg.person;
    person_x = msg.p_x;
    person_size = msg.p_size;
   // sign_detect = msg.msg_sign;
   // sign_x = msg.msg_scx;
   // sign_size = msg.msg_ssize;
}*/
/*
void SensorDataCallback(const my_msgs::SensorData &msg) {
    Sensor_front = msg.front;
    Sensor_left = msg.left;
    Sensor_right = msg.right;
    Sensor_Back = msg.back;
}*/

void Init_P_control()
{
    X_error = 0;
    P_error = 0;
    PWM_R = 0;
    PWM_L = 0;
    dt = 0;
    dt_1 = 0;
}

void Init_CamData()
{
    person_detect = 0;
    person_x = 0;
    person_size = 0;
}

//////////////// Pub, Sub part

void Person_Detect_Callback(const std_msgs::Bool &msg) { // function which subscribes from camera
    person_detect = msg.data;
   // sign_detect = msg.msg_sign;
   // sign_x = msg.msg_scx;
   // sign_size = msg.msg_ssize;
}

void Person_X_Callback(const std_msgs::Float64 &msg) {
	person_x = msg.data;
}

void Person_Size_Callback(const std_msgs::Float64 &msg){
	person_size = msg.data;
}

void Chair_Detect_Callback(const std_msgs::Bool &msg) {
        sign_detect = msg.data;
}

void Chair_X_Callback(const std_msgs::Float64 &msg) {
        sign_x = msg.data;
}

void Chair_Size_Callback(const std_msgs::Float64 &msg) {
    sign_size = msg.data;
}

void Display_Callback(const std_msgs::Bool &msg) {
        state = msg.data;
}

void Front_Sensor_Callback(const std_msgs::Float64 &msg) {
    FrontSensor = msg.data;
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;
    Initialize();
    ros::Publisher signal_pub = nh.advertise<std_msgs::Bool>("/signal/topic", 10);
    ros::Subscriber signal_sub = nh.subscribe("/signal/topic2", 10, Display_Callback);
    ros::Subscriber front_sensor_sub = nh.subscribe("/sensor/topic", 10, Front_Sensor_Callback);
    ros::Subscriber person_detect_sub = nh.subscribe("/detectnet/personBool", 10, Person_Detect_Callback);
    ros::Subscriber person_x_sub = nh.subscribe("/detectnet/personX", 10, Person_X_Callback);
    ros::Subscriber person_size_sub = nh.subscribe("/detectnet/personSize", 10, Person_Size_Callback);
    ros::Subscriber sign_detect_sub = nh.subscribe("/detectnet/chairBool", 10, Chair_Detect_Callback);
    ros::Subscriber sign_x_sub = nh.subscribe("/detectnet/chairX", 10, Chair_X_Callback);
    ros::Subscriber sign_size_sub = nh.subscribe("/detectnet/chairSize", 10, Chair_Size_Callback);

    ros::Rate loop_rate(Control_cycle);
    std_msgs::Bool signal_msg;

    while (ros::ok())
    {
        person_detect_sub;
        person_x_sub;
        person_size_sub;

        if (sequence == 0)
        {
           // sensor_sub; //subscribe camera and sensor data
	        ROS_INFO("sub %d", person_detect);
 
            Motor_Controller(1, false, 60);
            Motor_Controller(2, true, 60);

            if(person_detect == true){
                Motor_Controller(1, true, 0);
                Motor_Controller(2, true, 0);
                sequence++;
            }
        }

        if (sequence == 1)
        {
            gpio_write(pinum, motor_sv, PI_LOW);
            ROS_INFO("time(dt): %f, time(dt_1): %f", PWM_L, PWM_R);

            X_error = BOUND_MID - person_x;
            P_error = X_error * Kp;

            PWM_R = 60 + P_error;
            PWM_L = 60 - P_error;

            Motor_Controller(1, false, PWM_L);
            Motor_Controller(2, true, PWM_R);
            ROS_INFO("PWM_L: %f, PWM_R: %f", PWM_L, PWM_R);
            ROS_INFO("time: %d, size: %f", dt, person_size);

            if(person_detect == true && person_size >= BIG_SIZE)     // Frontsensor on && size_person is big enough
            {
                dt++;
                dt_1 = 0;

                if(dt > 15)     // need to change
                    sequence = 9;   ///////////////////////////////////////
            }
            ///////////// general condition
            dt_1++;

            if(dt_1 > 5)
                dt = 0;
        }

        if(sequence == 2)     // Screen interaction
        {
            signal_sub;

            Motor_Controller(1, true, 0);
            Motor_Controller(2, true, 0);

            signal_msg.data = true; // when person is in front of robot, pub 1 to display node
                signal_pub.publish(signal_msg);
            
            Init_P_control();
            Init_CamData();
            gpio_write(pinum, motor_sv, PI_HIGH);

            if (state == true) // true = display's end_signal
            {
                sequence++;
                state = false;
                signal_msg.data = false;
            }
        }

        if(sequence ==3)
        {
            // before detecting sign1 or 2, turn ccw
            Motor_Controller(1, true, 30);
            Motor_Controller(2, true, 30);

            /////////////////////////////// return method
            if(sign1_x == BOUND_MID || sign2_x == BOUND_MID)
            {
                ////////////
                return_method();
                // delay?
                ////////////

                init_return();
                Init_Cal_Theta();
                sequence = 0;   // go to init sequence
            }
            /*
            //////////////////////
            //if we are in inside, run here
            // Motor_Controller(1, true, 0);
            // Motor_Controller(2, true, 0);
            // ros::Duration(10).sleep();
            // sequence = 0;    // go to init sequence
            //////////////////////
            */
        }

        if(sequence == 9)     // Screen interaction
        {
            //if we are in inside, run here
            Motor_Controller(1, true, 0);
            Motor_Controller(2, true, 0);
            ros::Duration(5).sleep();
            sequence = 0;    // go to init sequence
            
            Init_P_control();
            Init_CamData();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    return 0;
}

