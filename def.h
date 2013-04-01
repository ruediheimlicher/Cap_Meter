#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN             4
#define SCLPIN             5


#define LOOPLED_PORT       PORTD
#define LOOPLED_DDR        DDRD
#define LOOPLED_PIN        4

#define TAKT_PORT          PORTB
#define TAKT_DDR           DDRB

#define TAKT_PIN           2
#define TAKT_INVERS_PIN    3



#define ANALOGCOMP         1

#define ACO_PORT           PORTD
#define ACO_DDR            DDRD
#define ACO_I0             6
#define ACO_I1             7

#define ACO_INVERS_PIN      0
#define ACO_FENSTER_PIN        1

#define INTERVALL          0x08  // 
#define INTERVALL_MAX      0x00FFF

#define MESSUNG_DAUER      0x00FF

#define ADC_PORT   PORTC
#define ADC_DDR    DDRD

#define ADMUX_L 0
#define ADMUX_H 1

#define MESSUNG_U 0
#define MESSUNG_O 1

#define TASTE1		19
#define TASTE2		29
#define TASTE3		44
#define TASTE4		67
#define TASTE5		94
#define TASTE6		122
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		223
#define TASTE0		236
#define TASTER		248
#define TASTATURPORT PORTC
#define TASTATURPIN		3

#define MANUELL_PORT		PORTD
#define MANUELL_DDR		DDRD
#define MANUELL_PIN		PIND

#define MANUELL			7	// Bit 7 von Status
#define MANUELLPIN		6	// Pin 6 von PORT D fuer Anzeige Manuell
#define MANUELLNEU		7	// Pin 7 von Status. Gesetzt wenn neue Schalterposition eingestellt
#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s
