/**
 * Name Hexapodusingspeed.c
 *
 * @date 20/09/2012 13:53:05
 * @author: Yogesh Wani yogeshwani(a)gmail(dot)com
 */ 

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "hexapod.h"

/** Enables the Timer0 overflow interrupt */
#define EnableTimer0OvfInt() TIMSK |= (1<< TOIE0);

/** Disables the Timer0 overflow interrupt */
#define DisbleTimer0OvfInt() TIMSK &= ~(1<< TOIE0); 

//bool comp_values_equal = FALSE;

/*Used only to debug certain things in the code*/
#ifndef DEBUG
static unsigned char data_dbg;
uint8_t indx1_dbg = 0;
uint8_t indx2_dbg = 0;
#endif

/** USART function to send a data byte on the serial (RS-232) port 
    @param data : 8 bit ASCII value of the data byte 
*/
void UsartSend(unsigned char data)
{
    data_dbg = data;
    /* Wait for empty transmit buffer */
    while ( !( UCSRA & (1<<UDRE)) );
    /* Put data into buffer, sends the data */
    UDR = data;
}

/** USART function to send an entire string on the serial (RS-232)port
    This utilizes the UsartSend function  
    @param Char * which points to the string 
*/
void UsartSendString(char *msg)
{
    while (*msg != '\0' )
    {
        UsartSend(*msg);
        msg++;    
    }  
}

/** Calcultates the incremental values to be added to a servo position 
    These incremental values then are added every 20ms as part of position update 
*/
void CalculateOffsets(uint8_t cnt1, uint8_t cnt2, uint8_t nm_pos_updates)
{
    uint16_t diff;
    uint8_t incr;
#ifdef ORIGINAL_CODE
    uint8_t incr,cnt1,cnt2;

    for(cnt1 = 0; cnt1 < 10; cnt1++)
    {
        for(cnt2 = 0; cnt2 < 2; cnt2++)
        {
#endif
            /* made sure the difference is always positive */
            if (servogaitinfo[cnt1][cnt2].start_val > servogaitinfo[cnt1][cnt2].end_val)
            {
                diff = servogaitinfo[cnt1][cnt2].start_val - servogaitinfo[cnt1][cnt2].end_val;
            }
            else
            {
                diff = servogaitinfo[cnt1][cnt2].end_val - servogaitinfo[cnt1][cnt2].start_val;
            }
            incr = diff / nm_pos_updates ;
            //incr = diff / NUM_OF_POS_UPDATES ;
            servogaitinfo[cnt1][cnt2].incr_val = incr;
#ifdef ORIGINAL_CODE
        }
    }
#endif
}
/** Sets up the 16-bit Timer 1 appropriately to follow the algorithm
*/
void SetupPWMTimer()
{
    /*  timer 1 in mode 12 with OC disconnected and ICR value equal to 2ms */
    TCCR1A = 0x00;
    TCCR1B |= ((1<<WGM13)|(1<<WGM12));
    TIMSK |= ((1<<OCIE1A)| (1<<OCIE1B)| (1<<TICIE1)); /*setup both the compare interrupts*/
    TCNT1 = 0;//initialize tvcnt1 count
    OCR1A = OCR1B = 0;//initialize ocr1a ocr1b count 
    //ICR1 = 31999;/*setup ICR to go off every 2 ms at 16mhz clock [top = (Fcpu/(N*Fpwm)) - 1]*/
    ICR1 = 15999;/*setup ICR to go off every 2 ms at 8mhz clock [top = (Fcpu/(N*Fpwm)) - 1]*/
}

/** Sets up all the ports required as outputs 
*/
void SetupPWMHardware()
{
    SERVO_SET1_DDR = 0xFF;
    SERVO_SET2_DDR = 0xFF;
    /*TODo : Here we dont needa ll the 8 pins as output since this is on port d which is also used for serial 
    communication we nned to just set those pins as output whichare going to be used foe the servo motors */
    SERVO_SET3_DDR = 0xFF;
}

/** Sets up the default gait or else if a value is passed, it will set all the servos
    to that value 
 */
void SetupPWMDefaults(uint16_t micros)
{
    uint8_t cnt1,cnt2;
    for(cnt1 = 0; cnt1 < 10; cnt1++)
    {
        for(cnt2 = 0; cnt2 < 2; cnt2++)
        {
#ifdef ORIGINAL_CODE
            /* Her the ides is tha th the tart end and the curr should have the same vlue . the 
            hexapod should respond to motions only when we have the fwd button being clicked */
            servogaitinfo[cnt1][cnt2].start_val = pushups_start[cnt1][cnt2];//Fwd_start[cnt1][cnt2];//default_gait[cnt1][cnt2];
            //servogaitinfo[cnt1][cnt2].curr_val = default_gait[cnt1][cnt2];
            servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;//default_gait[cnt1][cnt2];
            servogaitinfo[cnt1][cnt2].end_val = pushups_end[cnt1][cnt2];//Fwd_end[cnt1][cnt2];//pushups_end[cnt1][cnt2];
            servogaitinfo[cnt1][cnt2].num_of_pos_updates = 20;
#else 
            servogaitinfo[cnt1][cnt2].curr_val = default_gait[cnt1][cnt2];
            CalculateOffsets(cnt1,cnt2,NUM_OF_POS_UPDATES);
#endif
        }
    }
//    CalculateOffsets();
}

/** Function sets up the values for all the servos 
    Based on the type value passed in the servo values are filled up 
    to program up pre-loaded gaits.
*/
void SetupPWMValues(uint8_t type)
{
  uint8_t cnt1,cnt2;
 switch (type)
 {
     case 1: // for going forward
         for(cnt1 = 0; cnt1 < 10; cnt1++)
         {
             for(cnt2 = 0; cnt2 < 2; cnt2++)
             {
                 servogaitinfo[cnt1][cnt2].start_val = Fwd_start[cnt1][cnt2];// start values of a gait
                 servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;
                 servogaitinfo[cnt1][cnt2].end_val = Fwd_end[cnt1][cnt2]; // end values of a gait  
                 CalculateOffsets(cnt1,cnt2,NUM_OF_POS_UPDATES);
             }
         }
          break;
     case 2: // for going backward
     for(cnt1 = 0; cnt1 < 10; cnt1++)
     {
         for(cnt2 = 0; cnt2 < 2; cnt2++)
         {
             servogaitinfo[cnt1][cnt2].start_val = default_gait[cnt1][cnt2];// start values of a gait
             servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;
             servogaitinfo[cnt1][cnt2].end_val = default_gait2[cnt1][cnt2]; // end values of a gait  
             CalculateOffsets(cnt1,cnt2,NUM_OF_POS_UPDATES);
         }
     }
     break;
     
     case 3: // for goinf left
         for(cnt1 = 0; cnt1 < 10; cnt1++)
         {
             for(cnt2 = 0; cnt2 < 2; cnt2++)
             {
                 servogaitinfo[cnt1][cnt2].start_val = default_gait[cnt1][cnt2];// start values of a gait
                 servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;
                 servogaitinfo[cnt1][cnt2].end_val = default_gait2[cnt1][cnt2]; // end values of a gait         
                 CalculateOffsets(cnt1,cnt2,NUM_OF_POS_UPDATES);
             }
         }
         break;
         
    case 4: // for going right
        for(cnt1 = 0; cnt1 < 10; cnt1++)
        {
            for(cnt2 = 0; cnt2 < 2; cnt2++)
            {
                servogaitinfo[cnt1][cnt2].start_val = default_gait[cnt1][cnt2];// start values of a gait
                servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;
                servogaitinfo[cnt1][cnt2].end_val = default_gait2[cnt1][cnt2]; // end values of a gait
                CalculateOffsets(cnt1,cnt2,NUM_OF_POS_UPDATES);
            }
        }
        break;
        
    case 5: // for doing push ups 
        for(cnt1 = 0; cnt1 < 10; cnt1++)
        {
            for(cnt2 = 0; cnt2 < 2; cnt2++)
            {
                servogaitinfo[cnt1][cnt2].start_val = pushups_start[cnt1][cnt2];// start values of a gait
                servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;
                servogaitinfo[cnt1][cnt2].end_val = pushups_end[cnt1][cnt2]; // end values of a gait
                CalculateOffsets(cnt1,cnt2,NUM_OF_POS_UPDATES);
            }
        }
        break;
        
    default:
        break;     
 }   
    // CalculateOffsets();
}

/** Sets up the start up delay
 */
void SetStartupDelay(uint8_t speed)
{
    uint8_t j;
    for (j = 0; j < speed; j++)
    {
        _delay_ms(20);
    }
}

/* Sets up usart to start using serial comms*/
void SetupUsart(BAUD_RATE baud_rate)
{
    uint16_t ubrr_val = 0;

    /* USART initialization: 
       Set the baud rate in the UBBRs */ 
    ubrr_val = ( uint16_t ) ( (F_CPU/(16UL * baud_rate))- 1 );
    UBRRH = ubrr_val >> 8;
    UBRRL = ubrr_val;
    /* Set the usart in Asynchronous mode with No Parity, 1 Stop Bit 
       & character size of 8 bits */
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes 
    
    /* Enable the receiver and the transmitter (in UCSRB register) 
       and also enable the interrupts
       NOTE : here initially the  tx interrupt was also enbaled but there wa sno isr writteen and 
       hence it caused continous interrupts to be generatedthis causing grief
     */ 
    UCSRB |= ((1<<RXEN) | (1<<TXEN) | (1<<RXCIE));
    
}


/* Start the Timer */
void StartTimer()
{
    /* start with no prescalar*/
    TCCR1B |= (1<<CS10);
}

static void ParseServoData()
{
    volatile CONTROLLERCONFIG *pcfg = &cntrlcnfg;
    uint8_t indx1,indx2;
    uint8_t tens;
    uint8_t units; 
    uint16_t thousands;
    uint16_t hundreds;
    switch (serialservodata.data[0])
    {
     case 'M':
        if (serialservodata.data[1] == 'I')
        {
            pcfg->mode = INDIVIDUAL_SERVO;
            normal_msg = "ACK";
        }
        else if( serialservodata.data[1] == 'G' )
        {
            //EnableTimer0Int();
            pcfg->mode = GAIT;
            normal_msg = "ACK";
        }
        else
        {
            flag_error = 1 ;
            error_msg = "WRONG MODE";      
        }     
     break;
     /*
        implement for trim rather than type of drive signal .. 
        degres can be easily converted to pulse width using map function 
      */
     case 'T':
         if(cntrlcnfg.mode == INDIVIDUAL_SERVO)
         {
        tens = ((serialservodata.data[1] & 0x0f) * 10);
        units = (serialservodata.data[2] & 0x0f);
        pcfg->channel = tens + units;
    
        hundreds = ((serialservodata.data[3] & 0x0f) * 100);
        tens = ((serialservodata.data[4] & 0x0f) * 10);
        units = (serialservodata.data[5] & 0x0f);
        pcfg->trimvalue = hundreds+tens+units;
    
            indx1 = GetIndx1(pcfg->channel);
            indx1_dbg = indx1;

            indx2 = GetIndx2(pcfg->channel);
            indx2_dbg = indx2;

            if((indx1 > (NUM_OF_BANKS-1)) || (indx2 > (NUM_OF_SERVOS_IN_1_BANK-1)))
            {
            flag_error = 1;
            error_msg = "OUT OF RANGE";
            }
            else
            {
                servogaitinfo[indx1][indx2].trim_val = pcfg->trimvalue;
                normal_msg = "UPDATED";
            }
        }
break;    

     case 'C':
           if(cntrlcnfg.mode == INDIVIDUAL_SERVO)
           {
               tens = ((serialservodata.data[1] & 0x0f) * 10);
               units = (serialservodata.data[2] & 0x0f);
               pcfg->channel = tens + units;           

               thousands = ((serialservodata.data[3] & 0x0f) * 1000);
               hundreds = ((serialservodata.data[4] & 0x0f) * 100);
               tens = ((serialservodata.data[5] & 0x0f) * 10);
               units = (serialservodata.data[6] & 0x0f);
               pcfg->servovalue = thousands+hundreds+tens+units;
               
               indx1 = GetIndx1(pcfg->channel);
               indx1_dbg = indx1;

               indx2 = GetIndx2(pcfg->channel);
               indx2_dbg = indx2;
           
               if((indx1 > (NUM_OF_BANKS-1)) || (indx2 > (NUM_OF_SERVOS_IN_1_BANK-1)))
               {
                  flag_error = 1;
                  error_msg = "OUT OF RANGE";
               }
               else /* update the servo values */
               {
                   #ifdef ORIGINAL_CODE
                   servogaitinfo[indx1][indx2].curr_val = pcfg->servovalue;//--> original implementation 
                   #else
                   servogaitinfo[indx1][indx2].end_val = pcfg->servovalue;
                   #endif
                   CalculateOffsets(indx1,indx2,NUM_OF_POS_UPDATES);
                   normal_msg = "UPDATED";
               }
           }
           /* TODO THIS LOGIC NEEDS CHANGING.. WE NO LONGER USE 2 INDICES TO REFLECT WHICH GAIT TO USE . INSTEAD 
           NOW EVERYTHING HAPPENS IN ONE INDEX PASSED TO US. SEE SETPWMVALUES FUNCTION FOR IMPLEMENTAION AND CHANGE ACCORDINGLY
           */
           if(cntrlcnfg.mode == GAIT) 
           {
                /*Extract which gait needs to be used  */
                 gait_id = ((serialservodata.data[1] & 0x0f));
                hundreds = ((serialservodata.data[2] & 0x0f) * 100);
                tens = ((serialservodata.data[3] & 0x0f) * 10);
                units = (serialservodata.data[4] & 0x0f);
                pcfg->speed = hundreds+tens+units;
                // set the respective gaits only in gait mode
                SetupPWMValues(gait_id);
           }
        
        /* following comment NOT valid anymore after code was doen under hexapod using speed  
        for the gait .. we should be having the forward backward gait programed in the 
        eeprom or ram and then the said gait will be reflected in the sel field og teh structure for cntrlcnfg a
        we ca hava an array of pointer point int o teh gais and wehn the at is passed in we can just
        say array[gaittype]. This will fetch teh pointer and it will passed to teh setup pwmvalues funtion which would then populate the 
        servoinfo structure. The main thing in the gait is looping and hnce we can either use a timer interrup weith a adefautkl delay to switch between 
        arm up down etc .. we cant use delays in the while loop because that will simply not work */
     break; 

     default:
       flag_error = 1;
       error_msg = "SCREWED PLEASE RESET BOARD!!";
     break;
        }    
}
void UpdateServoCurrValuesSingleMode()
{
    uint8_t cnt1, cnt2;
    cli();
    for(cnt1 = 0; cnt1 < 10; cnt1++)
    {
        for(cnt2 = 0; cnt2 < 2; cnt2++)
        {
            if (servogaitinfo[cnt1][cnt2].end_val > servogaitinfo[cnt1][cnt2].curr_val)
            {
                servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].curr_val + servogaitinfo[cnt1][cnt2].incr_val;
                if (servogaitinfo[cnt1][cnt2].curr_val >= servogaitinfo[cnt1][cnt2].end_val)
                {
                    servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].end_val;
                    if(num_of_pos_updates_trkr > NUM_OF_POS_UPDATES)
                    {
                        num_of_pos_updates_trkr = 0;
                    }
                }    
            }
            else if (servogaitinfo[cnt1][cnt2].curr_val > servogaitinfo[cnt1][cnt2].end_val)
            {
                servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].curr_val - servogaitinfo[cnt1][cnt2].incr_val;
                if (servogaitinfo[cnt1][cnt2].curr_val <= servogaitinfo[cnt1][cnt2].end_val)
                {
                    servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].end_val;
                    if(num_of_pos_updates_trkr > NUM_OF_POS_UPDATES)
                    {
                        num_of_pos_updates_trkr = 0;
                    }
                }
            }     
        }
    }            
    sei();
}

void UpdateServoCurrValues() // called only every 20 ms 
{
    uint8_t cnt1, cnt2;
    static uint8_t towards_final = 1;// flag for going towards final position 

    for(cnt1 = 0; cnt1 < 10; cnt1++)
    {
        for(cnt2 = 0; cnt2 < 2; cnt2++)
        {
            if (servogaitinfo[cnt1][cnt2].end_val > servogaitinfo[cnt1][cnt2].start_val)
            {
                if (towards_final == 1)
                {
                    servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].curr_val + servogaitinfo[cnt1][cnt2].incr_val;
                    if (servogaitinfo[cnt1][cnt2].curr_val >= servogaitinfo[cnt1][cnt2].end_val)
                     {
                        servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].end_val;
                        if(num_of_pos_updates_trkr > NUM_OF_POS_UPDATES)
                        {
                            num_of_pos_updates_trkr = 0;
                            towards_final = 0;                           
                        }
                    }
                }
                else
                {
                    servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].curr_val - servogaitinfo[cnt1][cnt2].incr_val;
                    if (servogaitinfo[cnt1][cnt2].curr_val <= servogaitinfo[cnt1][cnt2].start_val)
                     {
                        servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;        
                        if(num_of_pos_updates_trkr > NUM_OF_POS_UPDATES)
                        {
                            num_of_pos_updates_trkr = 0;
                            towards_final = 1 ;
                        }
                    }
                        
                }
            }
            else
            {                
                if(towards_final == 1)
                {
                    servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].curr_val - servogaitinfo[cnt1][cnt2].incr_val;
                    if (servogaitinfo[cnt1][cnt2].curr_val <= servogaitinfo[cnt1][cnt2].end_val)
                     {
                        servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].end_val;
                        if(num_of_pos_updates_trkr > NUM_OF_POS_UPDATES)
                        {
                            num_of_pos_updates_trkr = 0;
                            towards_final = 0; 
                        }
                     }
                }
                else
                {
                    servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].curr_val + servogaitinfo[cnt1][cnt2].incr_val;
                    if (servogaitinfo[cnt1][cnt2].curr_val >= servogaitinfo[cnt1][cnt2].start_val)
                     {
                        servogaitinfo[cnt1][cnt2].curr_val = servogaitinfo[cnt1][cnt2].start_val;
                        if(num_of_pos_updates_trkr > NUM_OF_POS_UPDATES)
                        {
                            num_of_pos_updates_trkr = 0;
                            towards_final = 1 ;
                        }
                     }
                }
            }                                        
        }  
    }
    sei(); 
} 

ISR(TIMER1_COMPA_vect)
{ 
    /*clear the active banks first servoi.e clear the pin it is connected to */
    if(servo_bank_index <= 3 )
    { 
        SERVO_SET1_PORT &= ~(ServoMask[servo_bank_index][SERVO1]);
    }
    else if (servo_bank_index <= 7 )
    {
        SERVO_SET2_PORT &= ~(ServoMask[servo_bank_index][SERVO1]);
    } 
     else if (servo_bank_index <= 9)
    {
        SERVO_SET3_PORT &= ~(ServoMask[servo_bank_index][SERVO1]);        
    }
}

/* This interrupt will be used to clear a pwm pin channel based on
 * which bank is active
 */
ISR(TIMER1_COMPB_vect)
{
    /*clear the active banks second servo */
    if(servo_bank_index <= 3 )
    {
       SERVO_SET1_PORT &= ~(ServoMask[servo_bank_index][SERVO2]);
    }
    else if (servo_bank_index <= 7 )
    {
       SERVO_SET2_PORT &= ~(ServoMask[servo_bank_index][SERVO2]);
    }
    else if (servo_bank_index <= 9)
    {
       SERVO_SET3_PORT &= ~(ServoMask[servo_bank_index][SERVO2]);
    }
}

/* This interrupt will go off every 2ms. In each slot of 2 ms it
 * will service 2 servos and increment the index 
 */
ISR(TIMER1_CAPT_vect)
{
    //static uint8_t gait_flag = 1; 
    static uint8_t update_values_flag = 0;
    servo_bank_index++; 
    if (servo_bank_index == NUM_OF_BANKS)
    {
        servo_bank_index = 0;
        /* after 20 ms are done i.e. the refresh rate of the servos lets see if we want to update the servo values*/ 
        /* by setting this flag here and doing the real thing after toggling teh pios ensures that at the start of
        every 2ms interval the pios are toggles first and the function to update the servo values is done later,
        This makes sure that the setting of pios is not hapered by the function processing time and hence minimum latency 
        on the pulse width*/
        update_values_flag = 1;
    }        
        
    if(servo_bank_index <= 3 )
    {
        SERVO_SET1_PORT |= ((ServoMask[servo_bank_index][SERVO1]) | (ServoMask[servo_bank_index][SERVO2]));
    }
    else if (servo_bank_index <= 7 )
    {
        SERVO_SET2_PORT |= ((ServoMask[servo_bank_index][SERVO1]) | (ServoMask[servo_bank_index][SERVO2]));
    }
    else if (servo_bank_index <= 9)
    {
        SERVO_SET3_PORT |= ((ServoMask[servo_bank_index][SERVO1]) | (ServoMask[servo_bank_index][SERVO2]));
    } 
    
    if ((update_values_flag==1) && (cntrlcnfg.mode == GAIT))
    {
        UpdateServoCurrValues();
        update_values_flag = 0;
        num_of_pos_updates_trkr++;
    }
    #ifndef ORIGINAL_CODE
    else if ((update_values_flag==1) && (cntrlcnfg.mode == INDIVIDUAL_SERVO))
    {
        UpdateServoCurrValuesSingleMode();
        update_values_flag = 0;
        num_of_pos_updates_trkr++;
    }
    #endif
         
    #if SAME_VALUE_IMPLEMENTATION
    if (servoinfo[servo_bank_index][SERVO1].compare == servoinfo[servo_bank_index][SERVO2].compare)
    {
        comp_values_equal = TRUE;
    }
    #endif
    OCR1A = US2TICKS(servogaitinfo[servo_bank_index][SERVO1].curr_val + servogaitinfo[servo_bank_index][SERVO1].trim_val);
    OCR1B = US2TICKS(servogaitinfo[servo_bank_index][SERVO2].curr_val + servogaitinfo[servo_bank_index][SERVO2].trim_val);    
}

/* interrupt routine to handle rx serial data 
   Each byte of data received is buffered until the 
   MAX_NUM_RS232_BYTES is reached or the character is 
   an end of message character which in this case is a $
   */
ISR(USART_RXC_vect)
{    
    static uint8_t validdatacounter;

    /* if serial data goes above the max allowed limit then output E = ERROR 
        this checks needs to be done before we store the incomming data 
        other wise it will corrupt teh next byte in teh sram data structure which in 
        this case happens to be the valid variable !!DANGEROUS !!
    */
    if(validdatacounter >= MAX_NUM_RS232_BYTES)
    {
       flag_error = 1;
       error_msg = "Buff OVFl";
       validdatacounter = 0;
    }

    serialservodata.data[validdatacounter] = UDR;

    if (serialservodata.data[validdatacounter] == '$')
    {
        serialservodata.length = validdatacounter;
        validdatacounter = 0;
        serialservodata.valid = true;
    }

    if (serialservodata.valid == true)
    {
       ParseServoData();
       //Acknowledge the receipt of all 4 bytes 
       normal_msg = "ACK";
    }

    /* Increment the counter to receive the next Serial byte at the 
     * next interrupt
     */
    if (!(serialservodata.valid == true))
    {
        validdatacounter++;
    }    
    /* More elaborate way of sending error messages */
    /* NOTE:The latest error or no error message will be dispalyed
            rest of them will be overwritten 
     */
    if (flag_error == 1)
    {
        UsartSendString(error_msg);
        flag_error = 0;
    }
    else if (serialservodata.valid == true)
    { 
        UsartSendString(normal_msg);
        serialservodata.valid = false;
    }    
}

int main()
{ 
    SetStartupDelay(200);
    SetupPWMTimer();
    SetupPWMHardware();
    SetupPWMDefaults(1990);
    // set the servo travel direction 
    /*SetupUsart(BAUD_RATE_9600);*/
    //SetupUsart(BAUD_RATE_19200);
    SetupUsart(BAUD_RATE_38400);
    /* set the valid flag to false since we don't have any serial data yet !!*/
    serialservodata.valid = false;
    /* The start of the first 2 servo will be done in main and then the 
     * interrupts will be fired
     */
    SERVO_SET1_PORT |= ((ServoMask[servo_bank_index][SERVO1]) | (ServoMask[servo_bank_index][SERVO2]));
    OCR1A = US2TICKS(servogaitinfo[servo_bank_index][SERVO1].curr_val + servogaitinfo[servo_bank_index][SERVO1].trim_val);
    OCR1B = US2TICKS(servogaitinfo[servo_bank_index][SERVO2].curr_val + servogaitinfo[servo_bank_index][SERVO2].trim_val);
    StartTimer();
    /* enable global interrupt */
    sei();
    UsartSendString("\rServo Controller Initialized!!");
    while(1)
    {
    // infinite loop 
    }
    return 0;
}


