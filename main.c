#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#define LED_RED 1
#define LED_GREEN (LED_RED << 1)
#define LED_BLUE (LED_RED << 2)

void systick_init(void){
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}

void systick_wait1ms(){
    SysTick->LOAD = 48000;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000) == 0) {};
}

void systick_waitXms(int count){ //count=1000->1s
    int i;

    for (i = 0; i < count; i++){
        systick_wait1ms();
    }
}

void systick_wait1s(){ //count=1000->1s
    int i;
    int count = 1000;
    for (i = 0; i < count; i++){
        systick_wait1ms();
    }
}




void sensor_init(){

    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08; //
    P5->DIR |= 0x08; //direction
    P5->OUT &= ~0X08; //output (led)

    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->OUT &= ~0X04;

    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= 0xFF;
}

void pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    //CCR0 period
    TIMER_A0->CCR[0] = period;

    //divide by 1
    TIMER_A0->EX0 = 0x0000;

    //toggle/reset
    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;

    //0x200 ->SMCLK
    //0b1100 0000 -> input divider /8
    //0b0011 0000 -> up/down mode
    TIMER_A0->CTL = 0x02F0;

    //set alternative //
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
}

void motor_init(void){

    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;

    pwm_init34(7500, 0, 0);

}

void move(uint16_t leftDuty, uint16_t rightDuty){
    P3->OUT |= 0xC0;
    TIMER_A0->CCR[3] = leftDuty;
    TIMER_A0->CCR[4] = rightDuty;
}

void left_forward(){
    P5->OUT &= ~0x10;
}

void left_backward(){
    P5->OUT |= 0x10;
}

void right_forward(){
    P5->OUT &= ~0x20;
}

void right_backward(){
    P5->OUT |= 0x20;
}

//timer interrupt
void(*TimerA2Task)(void);
void TimerA2_Init(void(*task)(void), uint16_t period){
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280; //
    TIMER_A2->CCTL[0] = 0x0010; //compare mode
    TIMER_A2->CCR[0] = (period-1); //compare match value when the counter meets CCR[0], interrupt occurs
    TIMER_A2->EX0 = 0x0005; //input divider2
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF00)|0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014; //0000000000010100 5,4bit = up mode, increment counter / 2bit = clear TA0R register
}

void TA2_0_IRQHandler(void){
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}

//timer interrupt end

//rotate_linetracer9
void timer_A3_capture_init(){
    //alternative mode
    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;
    P10->DIR &= ~0x30;

    //setup timer & input capture mode
    TIMER_A3->CTL &= ~0x0030;
    TIMER_A3->CTL = 0x0200;

    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    TIMER_A3->EX0 &= ~0x0007;

    //configure interrupts
    NVIC->IP[3] = (NVIC->IP[3]&0x0000FFFF) | 0x404000000;
    NVIC->ISER[0] = 0x0000C000;
    TIMER_A3->CTL |= 0x0024;
}

//Interrupt Handler
uint16_t first_left;
uint16_t first_right;

uint16_t period_left;
uint16_t period_right;


void TA3_0_IRQHandler(void){
    TIMER_A3->CCTL[0] &= ~0x0001;
    period_right = TIMER_A3->CCR[0] - first_right;
    first_right = TIMER_A3->CCR[0];
}

/*void TA3_N_IRQHandler(void){
    TIMER_A3->CCTL[1] &= ~0x0001;
    period_left = TIMER_A3->CCR[1] - first_left;
    first_left = TIMER_A3->CCR[1];
}*/

//Measure RPM :
uint32_t get_left_rpm(){
    return 2000000/period_left;
}

//180 wheel rotation
uint32_t left_count;
void TA3_N_IRQHandler(void){
    TIMER_A3->CCTL[1] &= ~0x0001;
    left_count++;
}
//rotate end

void turn_right(int speed){
    left_forward();
    right_backward();
    move(speed, speed);
}
void turn_right_angle(int speed, int angle){
    left_count = 0;
    left_forward();
    right_backward();

    move(speed, speed);
    while(1){
        if(left_count>2*angle+33){
            move(0,0);
            //systick_waitXms(500);
            break;
        }
    }

//    systick_wait1s();
//    systick_wait1s();
//    systick_wait1s();
//    systick_wait1s();
//    systick_wait1s();
}


void turn_left(int speed){
    right_forward();
    left_backward();
    move(speed, speed);
}
void turn_left_angle(int speed, int angle){
    left_count = 0;
    right_forward();
    left_backward();

    move(speed, speed);
    while(1){
        if(left_count>2*angle){
            move(0,0);
            //systick_waitXms(500);
            break;
        }
    }

}

void turn_straight(int speed){
    right_forward();
    left_forward();
    move(speed, speed);
}
void turn_backward(int speed){
    right_backward();
    left_backward();
    move(speed, speed);
}


void goal1(void){

    left_forward();
    right_backward();
    systick_waitXms(500);
    if(left_count>20){
        turn_straight(0);

        systick_wait1s();
        systick_wait1s();
        systick_wait1s();

        turn_straight(800);

        systick_wait1s();
        //systick_wait1s();
    }

//    systick_wait1s();

}

int num=0;
int i=0;
void goal2(void){

    //left_count = 0;
    //systick_waitXms(1000);
    if(num==0){ //한번만 회전
        left_backward();
        right_forward();
        systick_waitXms(370);
        num=1;
    }

    for(; i<2; i++){
        turn_straight(800);
        systick_wait1s();

        turn_backward(800);
        systick_wait1s();
    }

    turn_straight(800);
    systick_wait1s();
    systick_wait1s();
    systick_waitXms(250);

}

void goal3(int speed){ //점점 느려지기
    /*left_forward();
    right_backward();
    systick_waitXms(210);*/
    right_forward();
    left_forward();

    int minspeed =350;

    while(speed>minspeed){
        speed -=45;
        move(speed+50, speed);
        //systick_wait1s();
        systick_waitXms(355);


    }
    //move(0,0);
    //systick_wait1s();
}

void goal4(int speed){ //점점 빨라지기
     //한번만 회전
//
//    left_forward();
//    right_backward();
//    systick_waitXms(130);


    right_forward();
    left_forward();
    int maxspeed = 1200;
    //move(1500, 1500);
    //systick_waitXms(300);
    while(speed<maxspeed){
        speed +=40;
        move(speed+40, speed);

        systick_waitXms(143);
    }
}

void goal5(void){

    left_forward();
    right_backward();
    systick_waitXms(320);
    turn_straight(0);
    systick_wait1s();

    //turn_left(1000);
    turn_left_angle(1000, 355);

   //systick_waitXms(6500);
    left_forward();
    right_forward();
    move(800,800);
    //turn_straight(600);

    //systick_wait1s();
   // systick_wait1s();
    systick_waitXms(200);
    //systick_wait1s();

    turn_right_angle(1000, 360);
    //systick_wait1s();
    //systick_waitXms(6500);
    turn_left_angle(1000, 325);
    turn_straight(0);
    systick_wait1s();
}

void goal6(void){ //반바퀴씩 두번


    left_forward();
    right_forward();
    move(820,800);
    systick_wait1s();
    systick_waitXms(600);

    systick_waitXms(300);

    turn_right_angle(1000, 180);
    //systick_waitXms(3300);

    //turn_straight(600);
    left_forward();
    right_forward();
    move(820,800);
    systick_wait1s();
    systick_waitXms(600);



    turn_left_angle(1000, 195);
    turn_straight(0);
   systick_wait1s();
   left_forward();
   right_forward();
   move(820,800);
   systick_waitXms(1700);


}



void main(void)
   {


    Clock_Init48MHz();
    sensor_init();
    motor_init();
    timer_A3_capture_init();
    systick_init();


    int sensor0 = 0;
    int sensor1 = 0;
    int sensor2 = 0;
    //int sensor3 = 0;
    //int sensor4 = 0;
    int sensor5 = 0;
    int sensor6 = 0;
    int sensor7 = 0;
    int sensor = 0;
    int flag =1;




    while(1){

        P5->OUT |= 0x08;
        P9->OUT |= 0X04;

        P7->DIR = 0XFF;
        P7->OUT = 0XFF;

        Clock_Delay1us(1);

        P7->DIR = 0X00;
        Clock_Delay1us(800); //검은색은 빛을 흡수 흰색은 빛을 반사 -> 흰색이 더 빛이 많이 들어와서 소진시간이 훨씬 빠름 -> 검은색과 흰색의 소진시간의 가운데점을 넣어서 경계선을 찾는다. -> sensor가 흰색인지 검은색인지 구분.

        sensor7 = P7->IN & 0x80;
        sensor6 = P7->IN & 0x40;
        sensor5 = P7->IN & 0x20;
        //sensor4 = P7->IN & 0x10;
        //sensor3 = P7->IN & 0x8;
        sensor2 = P7->IN & 0x04;
        sensor1 = P7->IN & 0x02;
        sensor0 = P7->IN & 0x01;
        sensor = P7->IN & 0x18;



        if(sensor){  //검은색일때
            if(((sensor0&&sensor7) || (sensor2&&sensor6&&!sensor7&&!sensor5)) && (flag == 1)){ //goal1
                goal1();
                flag = 2;
            }
            else if(((sensor1&&sensor6) || (!sensor0&&!sensor1&&!sensor2&&sensor5&&!sensor6&&sensor7)) && (flag == 2)){ //goal2 : sensor1 & sensor6
                goal2();
                flag = 3;
            }
            else if((sensor2 && (P7->IN & 0x8)) && (flag == 3)){ //goal3
                left_forward();
                right_backward();
                systick_waitXms(115);

            }
            else if(((sensor0&&!sensor1&&!sensor2)) && (flag == 3)){ //goal3
                goal3(800);
                flag = 4;
            }
            else if((sensor2 && sensor6) && (!sensor1 && !sensor0 && !sensor7)  && (flag == 4)){ // turn before goal4
                left_forward();
                right_backward();
                systick_waitXms(100);
                flag=4;
            }
            else if(/*(!sensor1&&!sensor6&&sensor7)*/(sensor7) && (!sensor6&& !sensor1 && ! sensor0) && (flag == 4)){ //goal4
                goal4(800);
                flag = 5;
            }
            else if(((!sensor7&&!sensor6&&sensor5&&sensor2) || (!sensor5&sensor6&!sensor2))&& (flag == 5)){ //goal5

                goal5();

                flag = 6;
            }
            else if(((/*(sensor0 && sensor2)*/ sensor1 || sensor2)) && (flag == 6)){ //goal6
                goal6();
                flag = 7;
            }
            else if((sensor2&&sensor5 && sensor) &&flag == 7){ //goal7
                //goal7();

                move(0,0);
                break;
            }


           ///right && left///
           if(sensor2){ //right
                 turn_right(800);
            }
           if(sensor5){ //left로 돌기
                 turn_left(800);
            }

           turn_straight(800);

           //Clock_Delay1ms(1); //기다리기
        }



        else if(sensor2){ //right
              turn_right(800);
        }

        else if(sensor5){ //left로 돌기
              turn_left(800);
        }

        else{
           turn_backward(700);
        }

//        P5->OUT &= ~0x08;
//        P9->OUT &= ~0x04;
//        Clock_Delay1ms(10);
    }

}
