#define distance_ab     // between c and c

float theta_1 = 0;      // between 1 and 2
float theta_2 = 0;      // between 2 and 3
float theta_3 = 0;      // between 3 and 4
float theta_4 = 0;      // between 4 and 1
float Max_angle = 0;    // Max(theta1~4)=

if(sequence ==3)
{
    // before detecting sign1, turn ccw
    Motor_Controller(1, true, 30);
    Motor_Controller(2, true, 30);

    if(sign1_detect == true && sign1_x == BOUND_MID)
    {
        ////////////
        return_method();
        //////////// init theta
        // wait for sec?
        theta_1 =0;
        theta_2 =0;
        theta_3 =0;
        theta_4 =0;
        Max_angle =0;

        sequence = 0;   // go to init
    }
}


void return_method()
{
    theta_1 = ?;
    theta_2 = ?;
    theta_3 = ?;
    theta_4 = ?;

    Max_angle = std::max(theta_1, theta_2, theta_3, theta_4);

    if(Max_angle == theta_1)    // into area1
    {
        Theta_Turn(theta_4 + theta_3 / 2);
        Distance_Go(distance_ab / 2);
    }
    else if(Max_angle == theta_2)    // into area2
    {
        Theta_Turn(theta_4 / 2);
        Distance_Go(distance_ab / 2);
    }
    else if(Max_angle == theta_3)    // into area3
    {
        Theta_Turn(theta_3 / 2);
        Distance_Go(distance_ab / 2);
    }
    else if(Max_angle == theta_4)    // into area4
    {
        Theta_Turn(theta_1 + theta_2 / 2);
        Distance_Go(distance_ab / 2);
    }
}


