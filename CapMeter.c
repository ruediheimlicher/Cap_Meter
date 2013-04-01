//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#define F_CPU 4000000UL  // 4 MHz
#include <avr/delay.h>

#include "def.h"
#include "lcd.c"
#include "adc.c"

uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;


volatile uint8_t					Programmstatus=0x00;
	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
volatile uint16_t					Manuellcounter=0; // Counter fuer Timeout	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;



volatile uint16_t					intervallcounter=0; // Counter fuer Messintervall
volatile uint16_t					messwert=0; // Counter fuer Messintervall
volatile uint32_t					messwertsumme=0; // Counter fuer Messintervall
volatile uint16_t					messwertmittel=0;
volatile uint8_t              eventcounter=0;
volatile uint8_t              messwertcounter=0;

volatile uint8_t              ablaufstatus=0;

volatile uint8_t              errcounta=0;
volatile uint8_t              errcountb=0;
volatile uint8_t              impulscount=0;



uint8_t Tastenwahl(uint8_t Tastaturwert)
{
//lcd_gotoxy(0,1);
//lcd_putint(Tastaturwert);
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}

void initAnalogComp(void)
{
   ACO_DDR &= ~(1<<ACO_I0);
   ACO_DDR &= ~(1<<ACO_I1);         // AIN1 als Eingang fuer Messwert
   
   ADC_DDR &= ~(1<<ADMUX_L); // ADC Eingaenge
   ADC_DDR &= ~(1<<ADMUX_H);
   ADC_PORT &= ~(1<<ADMUX_L);
   ADC_PORT &= ~(1<<ADMUX_H);
   
   ACO_DDR |= (1<<ACO_INVERS_PIN);   // Startpin als Ausgang fuer Steuerung Ladeverlauf
   ACO_PORT |= (1<<ACO_INVERS_PIN);  // HI, Sensor kurzgeschlossen
   ACO_DDR |= (1<<ACO_FENSTER_PIN);   // Endpin als Ausgang
   ACO_PORT |= (1<<ACO_FENSTER_PIN);  // HI, Sensor kurzgeschlossen
   
   ACSR &= ~(1<<ACD);
   
   ACSR |= (1<<ACIS0);
   ACSR |=(1<<ACIS1);  // steigende Flanke
   
   
   // ACSR |= (1<<ACBG);               // Interne Ref.Spannung
   
   // Interrupt ein
   ADCSRA &= ~(1<<ADEN);

   ACSR &= ~(1<<ACIC);
      
   SFIOR |= (1<<ACME); // Eingang von ADMUX
   //ADMUX = ADMUX_L;
   ADMUX = ADMUX_H; //nur oberen Werrt erfassen
   ACSR |= (1<<ACIE);
   
   ablaufstatus |= (1<<MESSUNG_O);
  
}



void slaveinit(void)
{
	//MANUELL_DDR |= (1<<MANUELLPIN);		//Pin 5 von PORT D als Ausgang fuer Manuell
	//MANUELL_PORT &= ~(1<<MANUELLPIN);
 	//DDRD |= (1<<CONTROL_A);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
	//DDRD |= (1<<CONTROL_B);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
	LOOPLED_DDR |= (1<<LOOPLED_PIN);
	//DDRD |= (1<<7);
	//PORTD &= ~(1<<7);

   DDRD |= (1<<2);
	PORTD &= ~(1<<2);

   DDRD |= (1<<5);
	PORTD &= ~(1<<5);

	//DDRB &= ~(1<<PORTB2);	//Bit 2 von PORT B als Eingang fuer Brennerpin
	//PORTB |= (1<<PORTB2);	//HI
	
   TAKT_DDR |= (1<<PORTB0);	//Bit 0 von PORT B als Ausgang fuer PWM
	TAKT_PORT &= ~(1<<PORTB1);	//LO

	TAKT_DDR |= (1<<TAKT_PIN);	//Bit 2 von PORT B als Ausgang fuer PWM
	TAKT_PORT &= ~(1<<TAKT_PIN);	//LO

	TAKT_DDR |= (1<<PORTB1);	//Bit 1 von PORT B als Ausgang fuer PWM
	TAKT_PORT &= ~(1<<PORTB1);	//LO
	
   TAKT_DDR |= (1<<PORTB3);	//Bit 1 von PORT B als Ausgang fuer PWM
	TAKT_PORT &= ~(1<<PORTB3);	//LO
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

	// TWI vorbereiten
	TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang für SDA
	TWI_PORT |= (1<<SDAPIN); // HI
	
	TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang für SCL
	TWI_PORT |= (1<<SCLPIN); // HI


	
	DDRC &= ~(1<<PORTC0);	//Pin 0 von PORT C als Eingang fuer Vorlauf 	
	PORTC &= ~(1<<DDC0); //Pull-up
	DDRC &= ~(1<<PORTC1);	//Pin 1 von PORT C als Eingang fuer Ruecklauf 	
	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<PORTC2);	//Pin 2 von PORT C als Eingang fuer Aussen	
//	PORTC |= (1<<DDC2); //Pull-up
	DDRC &= ~(1<<PORTC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
	//PORTC &= ~(1<<DDC3); //Pull-up
	
}

void timer2 (void)
{
   // Timer fuer Exp
  // 	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
   //	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
  	TCCR2 |= (1<<CS20);	//Takt /64 Intervall 64 us
	
   // 
	TIFR |= (1<<TOV2); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	
   TIMSK |= (1<<TOIE2);			//Overflow Interrupt aktivieren
   
   /*
   OCR0A = _BV(WGM1) | _BV(WGB0);
   TCCR0B = _BV(WGB02) | _BV(CS01) | _BV(CS00);
   OCR0A = 250;
    */
   TCNT2 = 0x00;					//Rücksetzen des Timers
	
}

void timer1(void)
{
   TCCR1B=0;
   TCCR1B |=  (1 << CS11);// div 8
   //TCCR1B |=  (1 << CS10);
   
   TCCR1B |=  (1<<ICNC1);
   unsigned char sreg;
   sreg = SREG;
   cli();
   TCNT1=0;                         // Counter von Timer1 reset
   
   SREG = sreg;
   
}

void timer1start(void)
{
   //TCCR1B |=  (1 << CS11);// div 8
   //TCCR1B |=  (0 << CS10)  | (1 << CS11);
   unsigned char sreg;
   sreg = SREG;
   cli();
   TCNT1=0;
   SREG = sreg;
   messwertsumme = 0,
   messwertcounter = 0;
   
}



ISR (TIMER2_OVF_vect)
{
   TAKT_PORT ^= (1<<PORTB0);
   intervallcounter++;
   
   if (impulscount%64==0)
   {
      if (TAKT_PORT & (1<<PORTB3))
      {
         TAKT_PORT |= (1<<PORTB2);
         //PORTD &= ~(1<<5);
         timer1start(); //timer1 starten
         PORTD |= (1<<5);
         TCNT1 = 0;
         ACO_PORT &= ~(1<<ACO_FENSTER_PIN);// Pin Hi, Messung fertig
         ACO_PORT |= (1<<ACO_INVERS_PIN);
         ablaufstatus |= (1<<MESSUNG_O);
      }
      else
      {
         TAKT_PORT &= ~(1<<PORTB2);
      }
      TAKT_PORT ^= (1<<PORTB3);
   }
   impulscount++;

   
}


ISR(ANA_COMP_vect)
{
   //errcountb++;
   /*
   if (ablaufstatus & (1<<MESSUNG_U)) // Messevent hat begonnen, unteren Wert erfassen
   {
      errcounta++;
      //ACO_PORT |= (1<<ACO_START_PIN); // erste Phase beendet
      ACO_PORT &= ~(1<<ACO_FENSTER_PIN);// Pin > LO, Messung start, Beginn des Messfensters
      
      ablaufstatus &= ~(1<<MESSUNG_U); // Ladespannung am unteren Wert, Bit U reset
    
      
      ADMUX = ADMUX_H; // Kanal auf hi umschalten
      
      ablaufstatus |= (1<<MESSUNG_O); // Bit Mesung_O set
      
     // timer1start(); //timer1 starten
  
   }
   
   else
   */
   if (ablaufstatus & (1<<MESSUNG_O)&& ANALOGCOMP) // Messevent ist beendet, oberen Wert erfassen
   {
      errcountb++;
      ACO_PORT |= (1<<ACO_FENSTER_PIN);// Pin Hi, Messung fertig
      ACO_PORT &= ~(1<<ACO_INVERS_PIN);
      
      // timer erfassen
      
      //ablaufstatus &= ~(1<<MESSUNG_O);

      unsigned char sreg;
      sreg = SREG;
      cli();
      
      messwert = TCNT1;
      {
         messwertcounter++;
         messwertsumme += messwert;
         if (messwertcounter==8)
         {
            messwertmittel = messwertsumme/8;
            messwertsumme = 0,
            messwertcounter = 0;
         }
      }
      SREG = sreg;
   }
    eventcounter++;
}

ISR( INT1_vect )
{
   if (ANALOGCOMP)
   {
      return;
   }
   
   unsigned char sreg;
   sreg = SREG;
   cli();
   
   messwert = TCNT1;
   PORTD &= ~(1<<5);
   //if (messwert>300)
   {
      messwertcounter++;
      messwertsumme += messwert;
      if (messwertcounter==8)
      {
         messwertmittel = messwertsumme/8;
         messwertsumme = 0,
         messwertcounter = 0;
      }
   }
   SREG = sreg;
   
}	// ISR

void InitAnalog(void)
{
   /*
    // interrupt on INT0 pin falling edge (sensor triggered)
    EICRA = (1<<ISC01) | (0<<ISC00);
    // turn on interrupts!
    EIMSK  |= (1<<INT0);
    */
   DDRD &= ~(1<<3);
	//PORTD &= ~(1<<3);

   
   // interrupt on INT1 pin falling edge
	MCUCR = (1<<ISC11) ;//| (0<<ISC10);
	// turn on interrupts!
	GICR  |= (1<<INT1);
    
	sei(); // Enable global interrupts
   
} 



int main (void)
{
	slaveinit();
	
   if (ANALOGCOMP)
   {
      initAnalogComp();
   }
   else
   {
      InitAnalog();
   }
   
   timer2();
   timer1();
  
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	_delay_ms(200);
	lcd_cls();
	lcd_puts("READY\0");
	

	_delay_ms(200);
	sei();
#pragma mark 
	while (1) 
	{
		
		loopCount0 ++;
		//_delay_ms(2);
		
		if (loopCount0 >=0x4FFF)
		{
			
			LOOPLED_PORT ^= (1<<LOOPLED_PIN);
			loopCount1++;
         //lcd_gotoxy(0,1);
         //lcd_puthex(loopCount1);
			if ((loopCount1 >0x0004) && (!(Programmstatus & (1<<MANUELL))))
			{
				{
               //ACO_PORT ^= (1<<ACO_START_PIN);
					//LOOPLED_PORT ^= (1<<LOOPLED_PIN);
					loopCount1=0;
               
               lcd_gotoxy(0,0);
               lcd_puthex(messwert>>8);
               lcd_puthex(messwert&0xFF);
               //lcd_putc(' ');
               //lcd_putint(messwert&0xFF);
               lcd_putc(' ');
               lcd_putint16(messwert);
               //lcd_putc('*');
               //lcd_putint(messwert);
               //lcd_putc('*');
               //lcd_gotoxy(0,1);
               //lcd_putint(errcountb);
               //lcd_putc('*');

               /*
               lcd_gotoxy(0,1);
               //lcd_putint16(messwertmittel);
               
               lcd_putc(' ');
               lcd_putint(intervallcounter);
               lcd_putc('*');
               lcd_putint(errcounta);
               lcd_putc('*');
               lcd_putint(errcountb);
               lcd_putc('*');
               */
				}
				
			}
			
			loopCount0 =0;
		}
		
#pragma mark Tastatur 
		/* ******************** */
//		initADC(TASTATURPIN);
//		Tastenwert=(readKanal(TASTATURPIN)>>2);
		
//		lcd_gotoxy(3,1);
//		lcd_putint(Tastenwert);
		Tastenwert=0;
		if (Tastenwert>5)
		{
			/*
			 0:											1	2	3
			 1:											4	5	6
			 2:											7	8	9
			 3:											x	0	y
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=200)
			{
				 lcd_gotoxy(17,1);
				 lcd_puts("T:  \0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				//Taste=0;
				 lcd_gotoxy(19,1);
				 lcd_putint1(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				
				switch (Taste)
				{
					case 0:// Schalter auf Null-Position
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
							Manuellcounter=0;
							Programmstatus |= (1<<MANUELLNEU);
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SI:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SP\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							*/
						}
						
					}break;
						
					case 1:	//	
					{ 
					if (Programmstatus & (1<<MANUELL))
						{
						uint8_t i=0;
						Manuellcounter=0;
						
						}
					}break;
						
						
						
					case 12: // # Normalbetrieb einschalten
					{
						Programmstatus &= ~(1<<MANUELL); // MANUELL OFF
						Programmstatus &= ~(1<<MANUELLNEU);
						MANUELL_PORT &= ~(1<<MANUELLPIN);
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}
	}
	
	
	return 0;
}
