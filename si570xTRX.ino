/*
Revision 1.0 - Main code by Richard Visokey AD7C - www.ad7c.com
Revision 2.0 - November 6th, 2013...  ever so slight revision by  VK8BN for AD9851 chip Feb 24 2014
Revision 3.0 - April, 2016        - AD9851 + ARDUINO PRO NANO + integrate cw decoder (by LZ1DPN) (uncontinued version)
Revision 4.0 - May 31, 2016       - deintegrate cw decoder and add button for band change (by LZ1DPN)
Revision 5.0 - July 20, 2016      - change LCD with OLED display + IF --> ready to control transceiver RFT SEG-100 (by LZ1DPN)
Revision 6.0 - August 16, 2016    - serial control buttons from computer with USB serial (by LZ1DPN) (1 up freq, 2 down freq, 3 step increment change, 4 print state)
									for no_display work with DDS generator
Revision 7.0 - November 30, 2016  - added some things from Ashhar Farhan's Minima TRX sketch to control transceiver, keyer, relays and other ... (LZ1DPN mod)								
Revision 8.0 - December 12, 2016  - EK1A trx end revision. Setup last hardware changes ... (LZ1DPN mod)
Revision 9.0 - January 07, 2017   - EK1A trx last revision. Remove not worked bands ... trx work well on 3.5, 5, 7, 10, 14 MHz (LZ1DPN mod)
Revision 10.0 - March 13, 2017 	  - scan function (LZ1DPN mod)
Revision 11.0 - June 22, 2017 	  - RIT + other - other (LZ1DPN mod)
Revision 15.0 - Octomber 7, 2017  - support for Si570 for LZ1DPN-CW3 Transmitter

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Include the library code
//#include <SPI.h>
#include <Wire.h>
#include <rotary.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 5
Adafruit_SSD1306 display(OLED_RESET);

#include <avr/io.h>
#include "Si570.h"
#include "debug.h"
//#define SI570_I2C_ADDRESS 0x55
Si570 *vfo;

//Setup some items
#define CW_TIMEOUT (1200l) // in milliseconds, this is the parameter that determines how long the tx will hold between cw key downs
unsigned long cwTimeout = 0;     //keyer var - dead operator control

#define TX_RX (12)   //mute + (+12V) relay - antenna switch relay TX/RX, and +V in TX for PA - RF Amplifier (2 sided 2 possition relay)
#define CW_KEY (4)   // KEY output pin - in Q7 transistor colector (+5V when keyer down for RF signal modulation) (in Minima to enable sidetone generator on)
//#define BAND_HI (6)  // relay for RF output LPF  - (0) < 10 MHz , (1) > 10 MHz (see LPF in EK1A schematic)  
#define FBUTTON (A0)  // tuning step freq CHANGE from 1Hz to 1MHz step for single rotary encoder possition
#define ANALOG_KEYER (A1)  // KEYER input - for analog straight key
#define BTNDEC (A2)  // BAND CHANGE BUTTON from 1,8 to 29 MHz - 11 bands
//#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }
//#define RADIONO_VERSION "LZ1DPN-CW3"

Rotary r = Rotary(3,2); // sets the pins for rotary encoder uses.  Must be interrupt pins.

char inTx = 0;     // trx in transmit mode temp var
char keyDown = 0;   // keyer down temp vat  
int_fast32_t xit=1200; // TX offset
int_fast32_t rx=7000000; // Starting frequency of VFO
int_fast32_t rx2=1; // temp variable to hold the updated frequency
int_fast32_t rxof=700;  // RX offset//800
int_fast32_t freqIF=12000000; //crystal filter freq
int_fast32_t rxif=(freqIF - rxof); // IF freq, will be summed with vfo freq - rx variable

int_fast32_t rxRIT=0;
int RITon=0;
int_fast32_t increment = 100; // starting VFO update increment in HZ. tuning step
int buttonstate = 0;   // temp var
String hertz = "100 Hz";
int  hertzPosition = 0;

// byte ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders
String freq; // string to hold the frequency
// int_fast32_t timepassed = millis(); // int to hold the arduino miilis since startup
// int byteRead = 0;
// int var_i = 0;

// buttons temp var
int BTNdecodeON = 0;   
//int BTNlaststate = 0;
//int BTNcheck = 0;
//int BTNcheck2 = 0;
int BTNinc = 3; // set number of default band minus 1

void checkCW(){
  pinMode(TX_RX, OUTPUT);
  if (keyDown == 0 && analogRead(ANALOG_KEYER) < 50){
    //switch to transmit mode if we are not already in it
    digitalWrite(TX_RX, 1);
    delay(5);  //give the relays a few ms to settle the T/R relays
    inTx = 1;
    keyDown = 1;
    rxif = (-rxRIT);  // in tx freq +600Hz and minus +-RIT 
    sendFrequency(rx);
    digitalWrite(CW_KEY, 1); //start the side-tone
  }

//reset the timer as long as the key is down
  if (keyDown == 1){
     cwTimeout = CW_TIMEOUT + millis();
  }

//if we have a keyup
  if (keyDown == 1 && analogRead(ANALOG_KEYER) > 150){
    keyDown = 0;
  	inTx = 0;   
    digitalWrite(CW_KEY, 0);  // stop the side-tone
    delay(5);  //give the relays a few ms to settle the T/R relays
	  rxif = (freqIF - rxof);  
	  sendFrequency(rx); 
    digitalWrite(TX_RX, 0);
    cwTimeout = millis() + CW_TIMEOUT;
  }

//if we have keyuup for a longish time while in cw rx mode
  if ((inTx == 1) && (millis() > cwTimeout)){
    //move the radio back to receive
    digitalWrite(TX_RX, 0);
  	digitalWrite(CW_KEY, 0);
    inTx = 0;
    keyDown = 0;
    rxif = (freqIF - rxof);
    sendFrequency(rx);
    cwTimeout = 0;
  }
}

// start variable setup

void setup() {

 // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C address 0x3C (for oled 128x32)
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();

  // Clear the buffer.
  display.clearDisplay();  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(rx);
  display.setTextSize(1);
  display.setCursor(0,16);
  display.print("St:");display.print(hertz);
  display.setCursor(64,16);
  display.print("rit:");display.print(rxRIT);
  display.display();

//set up the pins in/out and logic levels
pinMode(TX_RX, OUTPUT);
digitalWrite(TX_RX, LOW);
pinMode(CW_KEY, OUTPUT);
digitalWrite(CW_KEY, LOW);
pinMode(BTNDEC,INPUT);    // band change button
digitalWrite(BTNDEC,HIGH);    // level
pinMode(FBUTTON,INPUT); // Connect to a button that goes to GND on push - rotary encoder push button - for FREQ STEP change
digitalWrite(FBUTTON,HIGH);  //level
    

// Initialize the Serial port so that we can use it for debugging
  Serial.begin(115200);
  Serial.println("Start VFO ver 15.0");
//  debug("Radiono starting - Version: %s", RADIONO_VERSION);

//#ifdef RUN_TESTS
//  run_tests();
//#endif

  vfo = new Si570(0x55, 56320000);

//  if (vfo->status == SI570_ERROR) {
//    Serial.println("Si570 comm error");
//    delay(10000);
//  }

  // This will print some debugging info to the serial console.
//  vfo->debugSi570();

  //set the initial frequency
  vfo->setFrequency(26150000L);
  vfo->setFrequency(rx+rxif+rxRIT);
  Serial.println(rx);
  
 //  rotary
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  
}

///// START LOOP - MAIN LOOP

void loop() {
	checkCW();   // when pres keyer
	checkBTNdecode();  // BAND change
	
// freq change 
  if ((rx != rx2) || (RITon == 1)){
	    showFreq();
      sendFrequency(rx);
      rx2 = rx;
      }

//  step freq change + RIT ON/OFF  
  buttonstate = digitalRead(FBUTTON);
  if(buttonstate == LOW) {
        setincrement();        
    }

}	  
/// END of main loop ///
/// ===================================================== END ============================================


/// START EXTERNAL FUNCTIONS

ISR(PCINT2_vect) {
  unsigned char result = r.process();
if (result) {  
	if (RITon==0){
		if (result == DIR_CW){rx=rx+increment;}
		else {rx=rx-increment;}
	}
	if (RITon==1){
		if (result == DIR_CW){
		  rxRIT=rxRIT+50;
		  }
		else {
		  rxRIT=rxRIT-50;
	 	  }
  } 
}
}

// frequency calc from datasheet page 8 = <sys clock> * <frequency tuning word>/2^32
void sendFrequency(double frequency) {  
      vfo->setFrequency(frequency + rxif + rxRIT);
//      Serial.println(frequency);
  }

// step increments for rotary encoder button
void setincrement(){
  if(increment == 0){increment = 10; hertz = "10Hz"; hertzPosition=0;RITon=0;} 
//  else if(increment == 1){increment = 10; hertz = "10Hz"; hertzPosition=0;RITon=0;}
  else if(increment == 10){increment = 50; hertz = "50Hz"; hertzPosition=0;RITon=0;}
  else if (increment == 50){increment = 100;  hertz = "100Hz"; hertzPosition=0;RITon=0;}
  else if (increment == 100){increment = 500; hertz="500Hz"; hertzPosition=0;RITon=0;}
//  else if (increment == 500){increment = 1000000; hertz="1MHz"; hertzPosition=0;RITon=0;}
//  else if (increment == 1000){increment = 2500; hertz="2.5Khz"; hertzPosition=0;RITon=0;}
//  else if (increment == 2500){increment = 5000; hertz="5Khz"; hertzPosition=0;RITon=0;}
//  else if (increment == 5000){increment = 10000; hertz="10Khz"; hertzPosition=0;RITon=0;}
//  else if (increment == 10000){increment = 100000; hertz="100Khz"; hertzPosition=0;RITon=0;}
  else if (increment == 500){increment = 1000000; hertz="1Mhz"; hertzPosition=0;RITon=0;} 
  else{increment = 0; hertz = "ritON"; hertzPosition=0; RITon=1;};  
  showFreq();
  delay(250); // Adjust this delay to speed up/slow down the button menu scroll speed.
}

// oled display functions
void showFreq(){
	display.clearDisplay();	
	display.setTextSize(2);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	display.println(rx);
	display.setTextSize(1);
	display.setCursor(0,16);
	display.print("St:");display.print(hertz);
	display.setCursor(64,16);
	display.print("rit:");display.print(rxRIT);
	display.display();
}

//  BAND CHANGE !!! band plan - change if need 
void checkBTNdecode(){
  
BTNdecodeON = digitalRead(BTNDEC);    
    if(BTNdecodeON == LOW){
         BTNinc = BTNinc + 1;
         
         if(BTNinc > 7){
              BTNinc = 4;
              }
              
          switch (BTNinc) {
//          case 2:
//            rx=3500000;
//            break;
//          case 3:
//            rx=5250000;
//            break;
          case 4:
            rx=7000000;
            break;
          case 5:
            rx=10100000;
            break;
          case 6:
            rx=14000000;
            break;
          case 7:
            rx=18068000;
            break;    
//          case 8:
//            rx=21000000;
//            break;        
          default:             
            break;
        }         
        delay(200);     
    }
}

///
/*
#ifdef RUN_TESTS
bool run_tests() {
  // Those tests check that the Si570 libary is able to understand the
  // register values provided and do the required math with them.
  //
  // Testing for thomas - si570
  {
    uint8_t registers[] = { 0xe1, 0xc2, 0xb5, 0x7c, 0x77, 0x70 };
    vfo = new Si570(registers, 56320000);
    assert(vfo->getFreqXtal() == 114347712);
    delete(vfo);
  }

  // Testing Jerry - si570
  {
    uint8_t registers[] = { 0xe1, 0xc2, 0xb6, 0x36, 0xbf, 0x42 };
    vfo = new Si570(registers, 56320000);
    assert(vfo->getFreqXtal() == 114227856);
    delete(vfo);
  }

  Serial.println("Tests successful!");
  return true;
}

// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  debug("ASSERT FAILED - %s (%s:%i): %s", __func, __file, __lineno, __sexp);
  Serial.flush();
  // Show something on the screen
//
  // abort program execution.
  abort();
}
#endif
*/



//// OK END OF PROGRAM
