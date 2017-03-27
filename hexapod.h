/*
 * hexapod.h
 *
 * Created: 11/02/2013 22:13:49
 *  Author: yogesh
 */ 
#ifndef HEXAPOD_H_
#define HEXAPOD_H_

/**
 * Name: Hexapod.h 
 * @brief The project is about a six legged robot i.e. Hexapod a.k.a spidey
 * @author Yogesh Wani (yogeshwani(a)gmail.com)
 * @date 11/02/2013 22:13:49
 *
 *
 * @mainpage Hexapod
 *  The project comprises of a serial servo controller which can control up to 20 servos. The servos are controlled 
 *  using the port pins present on the ports of AVR(R) Atmega32. 
 *
 * @section Working
 *  This section describes the high level working of the serial servo controller and details the serial communication 
 *  protocol used to send commands etc. to the servo controller        
  
  
  Since the timer is running at 16Mhz clock each clock tick is 62.5ns so 
  to generate 2ms timer we need 31999 ticks which is given to the ICR reg.
  The min servo pulse is 500us and max is 2000us. So the OCR values will be 8000 and 32000 resp 
  whcih gives us a resolution of 24000 for the 18degress motion which 
  means we can move the servo by 7.5milidegrees or 7.5 x 10^-3 degrees.
  which is really miniscule and hece if we increment teh pwm reg values by 133/134 instead of 1 thenit
  means we will have a total of 180 different values to achive the motions and if needed less (??) 
  then we can reduce teh incremetn to 100 etc etc
  For every us we increment teh count by 16. So,write a amcro to convert the micor to timer tick. 
 B) 
  the servomask is as follows IN AA 10x 2 array where the rows give the bank index and teh coulns gives
  the servo index in the bank 
  -------
  |A1|A2|
  |A3|A4|
  |A5|A6| 
  |A7|A8|
  |B1|B2| 
  |B3|B4|
  |B5|B6| 
  |B7|B8| 
  |C1|C2| 
  |C3|C4|
  -------
  C)
  What happens in case where ocr1a and ocr1b have same values?
  D)
  port C was used before insgtead of portd for outputting teh last 4 pwm channels 
  however to use port c properly a ajtagg related fuse needs to be taken acare of 
  sice the portc pins have i/o pins as well as jtagg functionality multiplesed and 
  it causes problems(In my case no output was present on tha jtag pins). HEnce portd
  is now used instead of portc.

  E)Protocol for serial communication 
  data[0] = ---------------------------------------------------------
           | Mode | Sel0 | Sel1 | Sel2 | Sel3 | Sel4 | Type | notused|
            ---------------------------------------------------------
test data      0      0      1       0     1      1       1       0
            Mode:  mode 0 = single chanel control
                   mode 1 = GHait control 

            SEL0 - SEL4  = 5 bit value selectes servo channel if mode = 0 or selects the 
                           gait to be used if the mode = 1
            Type = Indicates the type of the drive command i.e either specified in 
                   the mountof degress to travel or the actual pulse length 
                   Type = 0 indicated degrees and Type = 1 indicated pulse wdth 
            notused: Unused bit maybe used in the future        
                   
  data[1] = degree to be travelled. if teh type is degree Sice degree is from 0 - 180 so 
            8 bit vaklue is sufficient 
            US byte 1 if the type is set to use the pulse width (type =1 )
            
  data[2] = not used for degree but will coantain the 2nd bute for het us tavel  
  data[3] = will containt the speed. currently this value will be ignored and it will not have any effect 
            on the arm motion. However it will be used to set the speed of travel for the arm so 
            this will dictate how fast or slow the arm motion would be.

  So, to have the effect of the serial command issued the transmitter has to make sure that 
  the packet is formatted in the above format and issed to eh servo controller. All 4 bytes must be present
  to have the effect of the desied command 
*/

/*HASHDEFINES */
/** Number of servos which are present in a bank */
#define NUM_OF_SERVOS_IN_1_BANK 2
/** Total number of servo banks */
#define NUM_OF_BANKS 10
/** Index for the first servo in a servo bank */
#define SERVO1 0
/** Index for the second servo in a servo bank */
#define SERVO2 1
/** Port A renamed for convenience to Servo set 1 */
#define SERVO_SET1_PORT PORTA
/** #define for the DDRA register for PORT A*/
#define SERVO_SET1_DDR  DDRA
/** Port B renamed for convenience to Servo set 2 */
#define SERVO_SET2_PORT PORTB
/** #define for the DDRB register for PORT B*/
#define SERVO_SET2_DDR  DDRB
/** Port D renamed for convenience to Servo set 2 */
#define SERVO_SET3_PORT PORTD
/** Port A renamed for convenience to Servo set 3*/
#define SERVO_SET3_DDR  DDRD
#define MAX_NUM_RS232_BYTES 12
#define MIN_SERVO_PULSE_WIDTH 400
/** Number of updates given to a Servo while moving 
from one position to another */
#define NUM_OF_POS_UPDATES 10  // 50

/* MACROS*/
//#define US2TICKS(val) ((((val) - MIN_SERVO_PULSE_WIDTH) * 16) + 8000)/* for 16MHZ*/
#define US2TICKS(val) ((((val) - MIN_SERVO_PULSE_WIDTH) * 8) + 3200)/* for 8MHZ*/
#define DEG2TICKS(val1) 
/* macros to get the index for the structure from the channel value passed in*/
#define GetIndx1(channel) ((((channel)+1)/2)-1)
#define GetIndx2(channel) (((channel)%2 == 0)? SERVO2:SERVO1)


//ENUMS
typedef enum BAUD_RATE
{
    BAUD_RATE_38400 = 38400,
    BAUD_RATE_19200 = 19200,
    BAUD_RATE_9600  = 9600, 
    BAUD_RATE_4800  = 4800, 
    BAUD_RATE_2400  = 2400, 
    BAUD_RATE_1200  = 1200
}BAUD_RATE;

typedef enum POSITION_TYPE 
{
    DEGREES,
    PULSEWIDTH
}POSITION_TYPE;

typedef enum MODE_OF_OPERATION 
{
    INDIVIDUAL_SERVO = 1,
    GAIT = 2
}MODE_OF_OPERATION;

/* Variables*/
static uint8_t servo_bank_index = 0;
///static uint8_t speed_compare_counter = 0;
static uint8_t num_of_pos_updates_trkr = 0;
static volatile uint8_t gait_id;
char *error_msg;
char *normal_msg;
static uint8_t flag_error = 0;
///static bool towards_start_val;

/* masks for the ports. used to set/clear pins */
/* NOTE : This needs changing to reflect the pins you are using otherwise the PWM wont work properly*/
uint8_t ServoMask[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{0b00000001,0b00000010},
                                                            {0b00000100,0b00001000},
                                                            {0b00010000,0b00100000},
                                                            {0b01000000,0b10000000},
                                                            {0b00000001,0b00000010},
                                                            {0b00000100,0b00001000},
                                                            {0b00010000,0b00100000},
                                                            {0b01000000,0b10000000},
                                                            {0b00000100,0b00001000},
                                                            {0b00010000,0b00100000}
                                                          };
														  
/* Structures*/                                                          
typedef struct rs232data
{
    char data[MAX_NUM_RS232_BYTES]; // duty cycle for a particular servo
    bool valid;
    uint8_t length;
}RS232DATA;

typedef struct controllerconfig
{
    MODE_OF_OPERATION mode; // used 
    POSITION_TYPE type;  // can be removed 
    uint8_t channel; // used 
    uint16_t servovalue; // used
    uint8_t speed; // used .. needs to be implemented properly 
	int8_t trimvalue; //used ..
}CONTROLLERCONFIG;

typedef struct servogait 
{
uint16_t start_val; // used
uint16_t curr_val;// used 
uint16_t end_val;// used 
uint16_t incr_val;//used 
uint8_t num_of_pos_updates; // unused 
int8_t trim_val; // needs to be implemented 
}SERVOGAIT;

volatile SERVOGAIT servogaitinfo[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK];
RS232DATA serialservodata;
volatile CONTROLLERCONFIG cntrlcnfg;


/*uint16_t const Fwd_start[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1500,1000},
                                                              {1225,1500},
                                                              {1500,1725},
                                                              {1500,1000},
                                                              {1225,1500},
                                                              {1500,1225},
                                                              {1500,1845},
                                                              {1725,1500},
                                                              {1500,1225},
                                                              {1500,1500}
                                                             };
uint16_t const Fwd_end[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1500,1500},
                                                              {1725,1500},
                                                              {1000,1225},
                                                              {1500,1500},
                                                              {1725,1500},
                                                              {1845,1725},
                                                              {1500,1500},
                                                              {1225,1500},
                                                              {1845,1725},
                                                              {1500,1500}
                                                             };
*/
/*The gait values*/
//rk1 = rk3 1000.. for start
//rh1=rh3 = 1225 i.e. 225 - centre value ofr servo
//lk2 =  1845
//lh2 = 1725 i.e. 225 + centre val of ervo
//lk1 = lk3 = 1845
// rk2 = 1000
#define RL1_STRT 1500
#define RK1_STRT 1000 //updated
#define RH1_START 1500 // updated
#define RL2_STRT 1500
#define RK2_STRT 1600 // updated
#define RH2_START 1725 // updated
#define RL3_STRT 1500 
#define RK3_STRT 1000 // updated
#define RH3_START 1500 // updated
#define LL1_START 1500 
#define LK1_STRT 1500
#define LH1_START 1225 //updated
#define LL2_START 1500
#define LK2_STRT 1970 // updated
#define LH2_START 1500 // updated
#define LL3_START 1500
#define LK3_STRT 1500
#define LH3_START 1225 // updated

#define RL1_END 1500 
#define RK1_END 1500 
#define RH1_END 1725 // updated
#define RL2_END 1500
#define RK2_END 1000 // updated
#define RH2_END 1500 // updated
#define RL3_END 1500
#define RK3_END 1500 
#define RH3_END 1725 // updated 
#define LL1_END 1500 
#define LK1_END 1975 //updated
#define LH1_END 1500 // updated
#define LL2_END 1500 
#define LK2_END 1500 
#define LH2_END 1225 // updated
#define LL3_END 1500
#define LK3_END 1975 // updated
#define LH3_END 1500 // updated



uint16_t const Fwd_start[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{RL1_STRT,RK1_STRT},
                                                              {RH1_START,RL2_STRT},
                                                              {RK2_STRT,RH2_START},
                                                              {RL3_STRT,RK3_STRT},
                                                              {RH3_START,LL1_START},
                                                              {LK1_STRT,LH1_START},
                                                              {LL2_START,LK2_STRT},
                                                              {LH2_START,LL3_START},
                                                              {LK3_STRT,LH3_START},
                                                              {1500,1500}
                                                             };
uint16_t const Fwd_end[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{RL1_END,RK1_END},
                                                              {RH1_END,RL2_END},
                                                              {RK2_END,RH2_END},
                                                              {RL3_END,RK3_END},
                                                              {RH3_END,LL1_END},
                                                              {LK1_END,LH1_END},
                                                              {LL2_END,LK2_END},
                                                              {LH2_END,LL3_END},
                                                              {LK3_END,LH3_END},
                                                              {1500,1500}
                                                             };
uint16_t const Bck_start[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500}
                                                             };
uint16_t const Bck_end[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700}
                                                             };
uint16_t const Lft_start[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1500,1500},
															{1500,1500},
															{1500,1500},
															{1500,1500},
															{1500,1500},
															{1500,1500},
															{1500,1500},
															{1500,1500},
															{1500,1500},
															{1500,1500}
															};
uint16_t const Lft_end[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{700,700},
															{700,700},
															{700,700},
															{700,700},
															{700,700},
															{700,700},
															{700,700},
															{700,700},
															{700,700},
															{700,700}
															};
uint16_t const Rght_start[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1500}
                                                             };
uint16_t const Rght_end[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700},
                                                              {700,700}
                                                             };															

// values for initial steps for wlking
/* 2 - 1025 , 8-1013 , 14 -1803 **16- 1779*/
/* 5- 1019 , 11- 1813 , 18 - 1837  */
uint16_t const default_gait[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1500,1500},
                                                              {1500,1500},
                                                              {1540,1500},
                                                              {1500,1600},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1425},
                                                              {1500,1440},
                                                              {1500,1500},
                                                              {1500,1500}
                                                             };

uint16_t const pushups_start[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1500,1500},
                                                              {1500,1500},
                                                              {1540,1500},
                                                              {1500,1600},
                                                              {1500,1500},
                                                              {1500,1500},
                                                              {1500,1425},
                                                              {1500,1440},
                                                              {1500,1500},
                                                              {1500,1500}
                                                             };

/* PUSH UPS
// righ edge legs 1895
// right middle legs 1984
//left middle legs 1100
//left edge legs 1100
*/
uint16_t const pushups_end[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{1895,1984},
                                                              {1500,1895},
                                                              {1984,1500},
                                                              {1895,1984},
                                                              {1500,1100},
                                                              {1100,1500},
                                                              {1100,1100},
                                                              {1500,1100},
                                                              {1100,1500},
                                                              {1500,1500}
                                                              };



uint16_t const default_gait2[NUM_OF_BANKS][NUM_OF_SERVOS_IN_1_BANK] = {{979,1396},
                                                              {832,944},
                                                              {1019,1420},
                                                              {1118,1350},
                                                              {1954,1861},
                                                              {1813,1954},
                                                              {1861,1443},
                                                              {1489,1965},
                                                              {932,1837},
                                                              {828,700}
                                                             };



#endif /* HEXAPOD_H_ */ 