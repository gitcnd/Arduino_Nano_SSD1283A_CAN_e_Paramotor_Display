/*

  This code runs on any 5v arduino (The CAN boards are unstable at 3.3v).  

  Uses this CAN Bus library: https://github.com/coryjfowler/MCP_CAN_lib
    => to install: cd Arduino/libraries; git clone https://github.com/coryjfowler/MCP_CAN_lib

  To drive these CAN_Bus boards: https://www.aliexpress.com/item/33041533951.html

  Using these scereens:  https://www.aliexpress.com/item/1005002378136214.html
  With this driver: https://github.com/ZinggJM/SSD1283A
    => to install: cd Arduino/libraries; git clone https://github.com/ZinggJM/SSD1283A

  Inside this 3D-printed case: https://a360.co/3JJTYie
  
  It reads incoming data from CAN_Bus

  It outputs the data onto upto 4 LCD screens.

  Except for the CS wires, all the LCDs are joined together (if you have more than 1)

  Wiring - LCD <=> ARDUINO
           LED  => Pin6  (PWM brightness)
           SCK  => PIN13 (SCK)
           SDA  => PIN11 (MOSI)
           A0   => PIN8
           RST  => PIN7
           CS   => Pins 5, 9, 4, and 3  (each screen has own pin)

           CAN_Bus <=> ARDUINO
           SO   => PIN12 (MISO)
           SI   => PIN11 (MOSI)
           SCK  => PIN13 (SCK)
           CS   => PIN10 (SS)
           INT  => PIN2
           
	   Optionally, I also wired 2 AUX pins:
	   AUX1 => PIN3	* Internally, I've wired both of these to a 4 pin plug with CANH and CANL
	   AUX2 => PIN4 * I can then output digital LOW to these pins to connect CAN GND id needed


  This also requires the custom LCDWIKI_GUI library;
   => to install: cd Arduino/libraries; git clone **TBD**
  And the SerialID library:
   => to install: cd Arduino/libraries; git clone https://github.com/gitcnd/SerialID.git 

  Inspired by https://github.com/lcdwiki/LCDWIKI_SPI/tree/master/Example/Example_03_display_string/display_string

*/



#include <SerialID.h>  // So we know what code and version is running inside our MCUs
SerialIDset("\n#\tv3.3 " __FILE__ "\t" __DATE__ " " __TIME__);


#include <mcp_can.h>
#include <SPI.h>

// CAN TX Variables and storage area
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};  // Generic CAN data to send
uint8_t can_ok = 0;                                               // Gets set to 1 if the CAN initialized OK, and 2 when we get CAN data

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// Serial Output String Buffer
char msgString[128];

uint8_t last_err=0;
uint8_t last_dve=0;
int last_amps=0;
int last_kwatts=0;
int last_soc=0;

unsigned char bright=0;
unsigned char degsym[]={246,0}; // 246 is the font dergees-symbol

// CAN0 INT and CS
#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

#define LCD_BACKLIGHT 6
#define LCD_CD_PIN_A0 8   // This pin controls whether data or control signals are being sent to the screen (bizarre non-SPI idea...)
#define LCD_RST_PIN 7
#define AUX_PINA 3
#define AUX_PINB 4


// Wire your 4 screens with CD(A0)=8, SDA=13, SCK=11, RST=8, LED=6, and CS= the below:-

#define CS_SCREEN1 5
//#define CS_SCREEN2 9
//#define CS_SCREEN3 4
//#define CS_SCREEN4 3


#include <LCDWIKI_GUI.h> //Core graphics library
#include <SSD1283A.h> //Hardware-specific library
#define min_sz 1      // text scale factor
#define FONT_W  6
#define FONT_H  8
#define FONT_SP 1

#if defined(__AVR)
SSD1283A_GUI scrn[]={ SSD1283A_GUI(CS_SCREEN1, LCD_CD_PIN_A0, LCD_RST_PIN, LCD_BACKLIGHT) }; //, 
//                      SSD1283A_GUI(CS_SCREEN2,8,7,6), 
//                      SSD1283A_GUI(CS_SCREEN3,8,7,6), 
//                      SSD1283A_GUI(CS_SCREEN4,8,7,6) };
#define NUM_SCREENS 1

//SSD1283A_GUI scrn[s]1(/*CS=10*/ CS_SCREEN1, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
//SSD1283A_GUI scrn[s]2(/*CS=10*/ CS_SCREEN2, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
//SSD1283A_GUI scrn[s]3(/*CS=10*/ CS_SCREEN3, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
//SSD1283A_GUI scrn[s]4(/*CS=10*/ CS_SCREEN4, /*DC=*/ 8, /*RST=*/ 7, /*LED=*/ 6); //hardware spi,cs,cd,reset,led
#endif


#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Pick one or more screens to write to
void sel_screen(int n) {
  if(n&1) digitalWrite(CS_SCREEN1,0); else digitalWrite(CS_SCREEN1,1);
//  if(n&2) digitalWrite(CS_SCREEN2,0); else digitalWrite(CS_SCREEN2,1);
//  if(n&4) digitalWrite(CS_SCREEN3,0); else digitalWrite(CS_SCREEN3,1);
//  if(n&8) digitalWrite(CS_SCREEN4,0); else digitalWrite(CS_SCREEN4,1);
}




// Function to display (P)ark, (N)eutral, or (D)rive on a display, and optionally the check-engine symbol
void pnd(uint8_t screen_number, uint8_t dve) { // dve is 0 for Park, 1 for Neutral, 2 for Drive
  sel_screen(1 << screen_number);
  pnd2(screen_number, last_dve, 0);   // un-draw old
  pnd2(screen_number, dve, 1);        // draw new
  last_dve=dve;                       // Remember what we just drew, so we can un-draw it later
} // pnd

void pnd2(uint8_t s,uint8_t dve, uint8_t draw) { // Draw (and un-draw) for the pnd() Function
  int x,y;
  char pnd[]={'P','N','D'};
  #define PND_SIZE 4
  
  // Draw P, N, or D to show the transmission state:-
  for(uint8_t c=0,x=26,y=1+(2*PND_SIZE+2);c<3;c++) {
    scrn[s].Set_Text_Back_colour(BLACK);
    if(c==dve) {scrn[s].Set_Text_Size( 1+PND_SIZE ); y=y-(2*PND_SIZE+2);} // Make the active letter bigger
    else scrn[s].Set_Text_Size( PND_SIZE );
    
    if(!draw) scrn[s].Set_Text_colour(BLACK);
    else if(c==dve)  scrn[s].Set_Text_colour(0,192,192);
    else      scrn[s].Set_Text_colour(WHITE);

    scrn[s].Set_Text_Cousur(x,y);
    scrn[s].writec(pnd[c]); // Print_String("", 45, 49);
    
    if(c==dve) {y=y+(2*PND_SIZE+2); x+=6+PND_SIZE*6;}
    else x+=PND_SIZE*6; 
  }
} // pnd2



// Function to show (or remove) the Check Engine symbol
void check_engine(uint8_t s, uint8_t err) { //s is screen_number, err=0 means no error (un-draw the logo) err>0 means error
  uint8_t sz=2; // Set the size of the symbol
  sel_screen(1 << s);

  if(err)  scrn[s].Set_Text_colour(RED);
  else     scrn[s].Set_Text_colour(BLACK);
  scrn[s].Set_Text_Back_colour(BLACK);
  scrn[s].Set_Text_Size(sz);
  for(uint8_t c=128,x=45,y=79;c<132;c++,x+=5*sz) { // These are 8 custom font characters that make up a check-engine symbol (4 x 2 characters of 6wide x 8high pixels each)
    scrn[s].Set_Text_Cousur(x,y);
    scrn[s].writec(c); // Print_String("", 45, 49);
    //scrn[s].Print_String("", 45, 49+8);
  }
  for(uint8_t c=132,x=45,y=79+8*sz;c<136;c++,x+=5*sz) { // The next 4 from the above 4 custom font chars
    scrn[s].Set_Text_Cousur(x,y);
    scrn[s].writec(c); // Print_String("", 45, 49);
    //scrn[s].Print_String("", 45, 49+8);
  }
} // check_engine




// Function to display the current AMPS being drawn
void amps(uint8_t screen_number, int val) { 
  sel_screen(1 << screen_number);
  amps2(screen_number, last_amps, 0);   // un-draw old
  amps2(screen_number, val, 1);         // draw new
  last_amps=val;                        // Remember what we just drew, so we can un-draw it later
} // amps

void amps2(uint8_t s,int val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal

  #define AMP_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( AMP_SIZE );  
  scrn[s].Print_Number_Int(val, 130/2 - (3 * 6 * AMP_SIZE)/2 , 1, 0, ' ',10); // "center" for 3 digits   ( num,  x, y, length, filler, base)

  if(draw) scrn[s].Set_Text_colour(WHITE);
  scrn[s].Set_Text_Size( AMP_SIZE-1 );  
  scrn[s].Print_String("AMPS", 130/2 - (3 * 6 * AMP_SIZE)/2, 8*AMP_SIZE);
} // amps2



// Function to display watts being drawn (or negative - regen - added)
void kwatts(uint8_t screen_number, int val) { 
  sel_screen(1 << screen_number);
  kwatts2(screen_number, last_kwatts, 0);   // un-draw old
  kwatts2(screen_number, val, 1);         // draw new
  last_kwatts=val;                        // Remember what we just drew, so we can un-draw it later
} // kwatts

void kwatts2(uint8_t s,int val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else if(val>=0) scrn[s].Set_Text_colour(0,192,192); // teal
  else  scrn[s].Set_Text_colour(GREEN); // make it green for regen :-)

  #define KW_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( KW_SIZE );  
  scrn[s].Print_Number_Int(val, 130/2 - (3 * 6 * KW_SIZE)/2 - KW_SIZE*3 , 66, 3, ' ',10); // "center" for 2 digits   ( num,  x, y, length, filler, base)

  if(draw) scrn[s].Set_Text_colour(WHITE);
  scrn[s].Set_Text_Size( KW_SIZE-1 );  
  scrn[s].Print_String(" kW", 130/2 - (3 * 6 * KW_SIZE)/2 , 66+8*KW_SIZE);
} // kwatts2



// Function to display the remaining battery capacity in numbers (%) 
void soc(uint8_t screen_number, int val) { 
  sel_screen(1 << screen_number);
  soc2(screen_number, last_soc, 0);   // un-draw old
  soc2(screen_number, val, 1);         // draw new
  last_soc=val;                        // Remember what we just drew, so we can un-draw it later
} // kwatts

void soc2(uint8_t s,int val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal

  #define SOC_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( SOC_SIZE-1 );  
  scrn[s].Print_String("%",  130/2 - (3 * 6 * SOC_SIZE)/2 + SOC_SIZE*3 + (2 * 6 * SOC_SIZE)   , 66+8); // Do this first, so big numbers overwrite the % instead of other way around

  scrn[s].Set_Text_Size( SOC_SIZE );  
  scrn[s].Print_Number_Int(val, 130/2 - (3 * 6 * SOC_SIZE)/2 + SOC_SIZE*3 , 66, 3, ' ',10); // "center" for 2 digits   ( num,  x, y, length, filler, base)

  if(draw) scrn[s].Set_Text_colour(WHITE);
  scrn[s].Set_Text_Size( SOC_SIZE-1 );  
  scrn[s].Print_String("SoC", 130/2 - (3 * 6 * SOC_SIZE)/2 + (SOC_SIZE*6/2) , 66+8*SOC_SIZE);
} // soc2

void socBar(uint8_t s,int percent) {
  int spc=130 * percent; spc/=100; // Scale the percentage to screen pixels.
  if(spc<130) scrn[s].Fill_Rect( 125,0,       129, 129-spc, BLACK); // Black top (overwrite prior green)
  if(spc>0)   scrn[s].Fill_Rect( 125,130-spc, 129,129,      GREEN); // Green bottom

    // Draw_Line(x0, y0, x1, y1);
    // Draw_Fast_HLine(int16_t x, int16_t y, int16_t w);
  // mylcd.Set_Draw_color (YELLOW);, mylcd.Draw_Round_Rectangle (70,70,100,100,6);, mylcd.Draw_Fast_VLine (80,80,30);, mylcd.Draw_Fast_HLine (80,80,30);
  // mylcd.Draw_Circle (70,40,16);  , mylcd.Fill_Circle (73,46,6);, mylcd.Draw_Triangle (5,110, 15,120,50,16);, mylcd.Fill_Triangle (90,125, 20,120,50,99);
} // socBar

// degrees symbol: (246 I think)  ö


// Function to display pretty temperature data
void tempC(uint8_t screen_number, int val, int last_val, int x_offset) { 
  sel_screen(1 << screen_number);
  tempC2(screen_number, last_val, 0, x_offset);   // un-draw old
  tempC2(screen_number, val, 1, x_offset);         // draw new
} // tempC

void tempC2(uint8_t s,int val, uint8_t draw, int x_offset) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal

  #define TEMP_SIZE 2  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( TEMP_SIZE );  
  
  sprintf(msgString, "%d%sC", val, degsym);
 
  scrn[s].Print_String(msgString, x_offset + 10, 130 - 8*TEMP_SIZE-10); // Do it all in teal, so the degrees-C is in the right spot
  if(draw) { // re-draw just the number, in white now
    scrn[s].Set_Text_colour(WHITE);
    sprintf(msgString, "%d", val);
    scrn[s].Print_String(msgString, x_offset + 10, 130 - 8*TEMP_SIZE-10); // Do it all in teal, so the degrees-C is in the right spot
  }

  // Draw a thermometer in graphics here...
  
} // tempC2



void signal(uint8_t s,bool has_sig) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Size(1);
  if(has_sig) {//	  1234567890123 = 13 = 78px wide
    scrn[s].Set_Text_Back_colour(BLACK);
    scrn[s].Set_Text_colour(RED);
    scrn[s].Print_String("               ", 20, 57);
  } else {
    scrn[s].Set_Text_Back_colour(RED);
    scrn[s].Set_Text_colour(WHITE);
    scrn[s].Print_String(" No CAN Signal ", 20, 57);
  }
}



void show_font(uint8_t s) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  scrn[s].Set_Text_colour(WHITE);
  uint8_t c=0;
  for(int y=0;y<128-1;y+=8) {
    for(int x=0;x<126-1;x+=6) {
      scrn[s].Set_Text_Cousur(x,y); // I did not write the lib - whoever did probably doesn't speak english as their first language :-)
      scrn[s].writec(c++); 
    }    
  }
} // show_font




// Light up all the instruments like analogue cars do, to check all guages are working...
void instrument_check() {
  check_engine(0,1);
  //pnd(0,1);
  amps(0,888);
  kwatts(0,-88);
  soc(0,88);
  socBar(0,66); // 66 percent
  //show_font(1);  
  tempC(0,88,0,0); // Caller (us here) needs to cache the previous temp numbers
  tempC(0,88,0,130/2);
  signal(0,false);
} // instrument_check



void demo_screen() {

  for(int s=0; s<NUM_SCREENS; s++) {
    int sbit=1<<s;
    sel_screen(sbit);
//    scrn[s].setRotation(r); // If you get an error in this line, you've forgotten to pick an Arduino board to compile to ( Tools => Board => Arduin AVR Boards => Arduino Nano )
//    scrn[s].setRotation(s); // If you get an error in this line, you've forgotten to pick an Arduino board to compile to ( Tools => Board => Arduin AVR Boards => Arduino Nano )
  
    scrn[s].Set_Text_Mode(0);
  
    scrn[s].Fill_Screen(0x0000);
    scrn[s].Set_Text_colour(RED);
    scrn[s].Set_Text_Back_colour(BLACK);
    scrn[s].Set_Text_Size(1);
    scrn[s].Print_String("Hello World!", 0, 0);
    scrn[s].Print_Number_Float(01234.56789, 2, 0, 8, '.', 0, ' ');  
    scrn[s].Print_Number_Int(0xDEADBEF, 0, 16, 0, ' ',16);

    scrn[s].Set_Text_colour(GREEN);
    scrn[s].Set_Text_Size(2);
    scrn[s].Print_String("Screen", 0, 32);
    scrn[s].Print_Number_Int(s, 100, 32, 0, ' ',32);
    scrn[s].Print_Number_Float(01234.56789, 2, 0, 48, '.', 0, ' ');  
    scrn[s].Print_Number_Int(0xDEADBEF, 0, 64, 0, ' ',16);

    scrn[s].Set_Text_colour(BLUE);
    scrn[s].Set_Text_Size(3);
    scrn[s].Print_String("Hello", 0, 86);
    scrn[s].Print_Number_Float(01234.56789, 2, 0, 110, '.', 0, ' ');  
    //scrn[s].Print_Number_Int(0xDEADBEF, 0, 134, 0, ' ',16);

    pnd(s, s%3); // 0 (P) ,1 (N), ,2 (D)

  } // s

  if(0) { // example of changing screen brightness
    analogWrite(LCD_BACKLIGHT,bright); // This is how you change the LCD brightness from no backlight (0) to full on (255)
    bright+=32;
    if(bright==128)bright--; // so we hit 255 later
    if(bright<32)bright=0;   // so we hit 0 as well
    // $c=0;while(1){$c=0 if($c<32); print "$c "; $c+=32; $c-- if($c==128); $c-=256 if($c>255);}
  }

  
} // demo_screen


void setup() 
{
  pinMode(LCD_BACKLIGHT,OUTPUT);
  digitalWrite(LCD_BACKLIGHT,1); // Turn on the backlight, full strength, at startup.
  // Alternatively - can dim it:- analogWrite(LCD_BACKLIGHT,255); // Turn on the backlight at startup (also inits this pin)  

  // Connect our AUX pins to ground:-
  pinMode(AUX_PINA,OUTPUT); digitalWrite(AUX_PINA,0);
  pinMode(AUX_PINB,OUTPUT); digitalWrite(AUX_PINB,0);
  
  SerialIDshow(115200); // starts Serial.

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  //if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
  if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK)  {
    can_ok=1;         
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515..."); // can_ok already=0
  }

  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);                           // Configuring pin for /INT input

  
  pinMode(CS_SCREEN1,OUTPUT);
//  pinMode(CS_SCREEN2,OUTPUT);
//  pinMode(CS_SCREEN3,OUTPUT);
//  pinMode(CS_SCREEN4,OUTPUT);
  sel_screen(0); // de-select all
  
  // MUST Init all at the same time
  sel_screen(1+2+4+8); // select all
  scrn[0].init();
  scrn[0].Fill_Screen(BLACK);
  sel_screen(0);

  if(!can_ok) {
    scrn[0].Set_Text_colour(WHITE);
    scrn[0].Set_Text_Back_colour(BLACK);
    scrn[0].Set_Text_Size(2);
    scrn[0].Print_String("CAN Init", 17, 55);
    scrn[0].Print_String( "failed",  29, 65);
    check_engine(0,1);
  } 

  sel_screen(0); // de-select them
  delay(10);
  sel_screen(1);
  scrn[0].setRotation(1); // Orient all the screens so "UP is UP" // 0 puts top at 9oclock // 1 puts top at noon

  instrument_check(); // Show 888 on everything (for testing)
}











void loop() 
{
  char buf[50]; // to assemble a big message
  char one[2];  // to output it one-byte-a-a-time
  int X=0;
  int Y=0;

  // demo_screen()

  //analogWrite(6,bright); // This is how you change the LCD brightness from no backlight (0) to full on (255)
  //bright+=32;
  //if(bright==128)bright--; // so we hit 255 later
  //if(bright<32)bright=0;   // so we hit 0 as well
  // $c=0;while(1){$c=0 if($c<32); print "$c "; $c+=32; $c-- if($c==128); $c-=256 if($c>255);}


  if(!digitalRead(CAN0_INT))  {                        // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)

    if ((rxId>0)||(len>0)) { // we were getting spurious zeros too much...
      if(can_ok!=2) { can_ok=2; signal(0,true); }

     if((rxId & 0x80000000) == 0x80000000) {            // Determine if ID is standard (11 bits) or extended (29 bits)
       sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
       sprintf(buf,"EX:%.8lX L:%1d d:", (rxId & 0x1FFFFFFF), len);
     } else {
       sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
       sprintf(buf,"ID:%.3lX L:%ld ", rxId, len);
     }
   
     Serial.print(msgString);
     
   
     if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
       //sprintf(msgString, " REMOTE REQUEST FRAME");
       Serial.print("RMT REQ");
       if(strlen(buf)<46) sprintf(&buf[strlen(buf)],"RMT"); // buf is 50 bytes
     } else {
       for(byte i = 0; i<len; i++){
         sprintf(msgString, " 0x%.2X", rxBuf[i]);				// 0        1         2         3         4         5
         Serial.print(msgString);						// 12345678901234567890123456789012345678901234567890
         if(strlen(buf)<46) sprintf(&buf[strlen(buf)],"%.2X ", rxBuf[i]);	// EX:12345678 L:123 d:12 34 56 78 90 12 34 56.
       }
     }


 
     // Send data to LCD
     #define CAN_SCRN 0
     sel_screen(1<<CAN_SCRN);
     scrn[CAN_SCRN].Set_Text_Back_colour(BLACK);
     scrn[CAN_SCRN].Set_Text_colour(YELLOW);
     scrn[CAN_SCRN].Set_Text_Size(1);
     one[1]=0;
     for(int i=0;i<strlen(buf);i++) { // Send data to LCD, wrapping as needed
       //one[0]=" ";    textPrint(0,X,Y,BLACK,BLACK,1,one);     // erase
       //one[0]=buf[i]; textPrint(0,X,Y,YELLOW,BLACK,1,one); // write char
 
       //scrn[CAN_SCRN].Set_Text_colour(BLACK);
       one[0]=" "; scrn[CAN_SCRN].Print_String(one, X, Y);
       one[0]=buf[i]; scrn[CAN_SCRN].Print_String(one, X, Y);
       
       X=X+6;  if(X>=126) { X=0; Y=Y+8; if(Y>=127)Y=0;}  // advance
     }	  

     Serial.println();

   } // nothing
  
  }else {
    //Serial.print("pin "); Serial.print(CAN0_INT); Serial.println(" is high: no data to read");
  } // CAN0_INT

  
  if(0){ //  example of how to send:-
    if(millis() - prevTX >= invlTX){                    // Send this at a one second interval. 
      CAN0.enOneShotTX(); delay(10);
      prevTX = millis();
      byte sndStat = CAN0.sendMsgBuf(0x111, 8, data); data[7]++;
      delay(10);CAN0.disOneShotTX();delay(10);
      if(sndStat == CAN_OK)
        Serial.println("Message Sent Successfully!");
      else
        Serial.println("Error Sending Message...");
    }
  } // send test

} // loop()
