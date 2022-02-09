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
SerialIDset("\n#\tv3.7 " __FILE__ "\t" __DATE__ " " __TIME__);


#include <mcp_can.h>
#include <SPI.h>

// CAN TX Variables and storage area
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};  // Generic CAN data to send
uint8_t can_ok = 0;                                               // Gets set to 1 if the CAN initialized OK, and 2 when we get CAN data, and 3 when we get valid CAN data
boolean flymode=true;						// Show just the important data

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
//unsigned char rxBuf[8];

#define BufSz 8
union Data {
  uint8_t b[BufSz/sizeof(uint8_t)];   // 8 bytes
  int8_t sb[BufSz/sizeof(uint8_t)];   // 8 bytes
  uint16_t i[BufSz/sizeof(uint16_t)]; // 4
  int16_t si[BufSz/sizeof(int16_t)];  // 4
  uint32_t l[BufSz/sizeof(uint32_t)]; // 2
  int32_t sl[BufSz/sizeof(int32_t)];  // 2
  float f[BufSz/sizeof(float)];
} rxBuf;


// Serial Output String Buffer
char msgString[128];

uint8_t last_err=0;
uint8_t last_dve=0;
float last_volts=0;
float last_amps=0;
long last_watts=0;
int last_soc=0;
int last_rpm=0;
int s_guess=8;			// Guess at startup if we probably have 14S or 15S attached.  int(volts/3.96)=S  (works upto 16s)
uint32_t last_temp_m=0;
uint32_t last_temp_e=0;
uint32_t last_temp_b=0;
uint32_t last_temp_f=0;
long unsigned int last_diag_e=0;
long unsigned int last_diag_w=0;
long unsigned int last_diag_n=0;
unsigned int last_diags_r=0;
unsigned int last_diags_i=0;
float last_volt_i=0;
float last_volt_e=0;
float last_volt_b=0;
int last_pc_t=0;
int last_pc_m=0;
int last_pc_c=0;
int last_phase_a=0;

unsigned char bright=0;
unsigned char degsym[]={246,'C',0}; // 246 is the font dergees-symbol

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
#define ORANGE  0xFF80
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



/*
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
*/


/*
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
*/


// Function to display the battery VOLTS
void Fvolts(uint8_t screen_number, float val) { 
  if((s_guess==8)&&(val<75)) {s_guess=int(val/3.96); s_guess=15;} // 14S or 15S
  if(last_volts != val) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    Fvolts2(screen_number, last_volts, 0);   // un-draw old
    Fvolts2(screen_number, val, 1);         // draw new
    last_volts=val;                        // Remember what we just drew, so we can un-draw it later
  }
} // Fvolts

void Fvolts2(uint8_t s,float val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  //else scrn[s].Set_Text_colour(0,192,192); // teal
  else scrn[s].Set_Text_colour(WHITE);

  //#define AMP_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  //#define AMP_SIZE 3  // 3 * 3 = 18px wide, 3 * 8 = 24px high
  scrn[s].Set_Text_Size( 7 );  
  //scrn[s].Print_Number_Int(val, 130/2 - (3 * 6 * AMP_SIZE)/2 , 1, 0, ' ',10); // "center" for 3 digits   ( num,  x, y, length, filler, base)

  dtostrf(val*10, 3, 0, msgString); // 4 is mininum width, 2 is precision
  // sprintf(msgString, "%2.1f", val); // arduino does not have %f
  //scrn[s].Print_Number_Int(val, 0,0);
  scrn[s].Print_String(msgString, 0,0);

  //scrn[s].Fill_Rect( 7*6*2-5, 7*8-11,  7*6*2, 7*8-3, WHITE); // fake the decimal point
  //                   X        Y      W  H
  scrn[s].Fill_Rect( 7*6*2-5, 7*8-11,  5, 5, WHITE); // fake the decimal point

  socBar(s,socv(val/s_guess));		// Show capacity remaining (these are REST numbers - not under-load ones)
} // Fvolts2



// Function to display the battery VOLTS
void volts(uint8_t screen_number, float val) { 
  if((s_guess==8)&&(val<75)) {s_guess=int(val/3.96); s_guess=15;} // 14S or 15S
  if(last_volts != val) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    volts2(screen_number, last_volts, 0);   // un-draw old
    volts2(screen_number, val, 1);         // draw new
    last_volts=val;                        // Remember what we just drew, so we can un-draw it later
  }
} // volts

void volts2(uint8_t s,float val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal

  //#define AMP_SIZE 4  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  //#define AMP_SIZE 3  // 3 * 3 = 18px wide, 3 * 8 = 24px high
  scrn[s].Set_Text_Size( 3 );  
  //scrn[s].Print_Number_Int(val, 130/2 - (3 * 6 * AMP_SIZE)/2 , 1, 0, ' ',10); // "center" for 3 digits   ( num,  x, y, length, filler, base)

  dtostrf(val, 4, 1, msgString); // 4 is mininum width, 2 is precision
  // sprintf(msgString, "%2.1f", val); // arduino does not have %f
  //scrn[s].Print_Number_Int(val, 0,0);
  scrn[s].Print_String(msgString, 0,0);

  if(draw) scrn[s].Set_Text_colour(WHITE);
  dtostrf(val/s_guess, 3, 1, &msgString[100]); // 4 is mininum width, 2 is precision

  sprintf(msgString, "%2dS. volts %s/cell", s_guess, &msgString[100]);
  scrn[s].Set_Text_Size( 1 );  
  scrn[s].Print_String(msgString, 0, 3*8);

  socBar(s,socv(val/s_guess));		// Show capacity remaining (these are REST numbers - not under-load ones)
  pc_c(s,socv(val/s_guess));		// Show capacity as a number also

} // volts2


// Function to display the current AMPS being drawn
void amps(uint8_t screen_number, float val) { 
  if(last_amps != val) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    amps2(screen_number, last_amps, 0);   // un-draw old
    amps2(screen_number, val, 1);         // draw new
    last_amps=val;                        // Remember what we just drew, so we can un-draw it later
  }
} // amps

void amps2(uint8_t s,float val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(0,192,192); // teal
  dtostrf(val, 6, 1, &msgString[100]); // -888.5
  //sprintf(msgString, "%5.1fA", val);
  sprintf(msgString, "%sA", &msgString[100]);
  scrn[s].Set_Text_Size( 1 );  
  scrn[s].Print_String(msgString, 4*6*3,0);
} // amps2



// Function to display watts being drawn (or negative - regen - added)
void watts(uint8_t screen_number, long val) { 
  if( last_watts != val ) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    watts2(screen_number, last_watts, 0);   // un-draw old
    watts2(screen_number, val, 1);         // draw new
    last_watts=val;                        // Remember what we just drew, so we can un-draw it later
  }
} // watts

void watts2(uint8_t s,long val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else if(val>=0) scrn[s].Set_Text_colour(0,192,192); // teal
  else  scrn[s].Set_Text_colour(GREEN); // make it green for regen :-)
  sprintf(msgString, "%6ldW", val);
  scrn[s].Set_Text_Size( 1 );  
  scrn[s].Print_String(msgString, 4*6*3,1*8);
} // watts2



// Function to display rpm being drawn (or negative - regen - added)
void rpm(uint8_t screen_number, int val) { 
  if( last_rpm != val ) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    rpm2(screen_number, last_rpm, 0);   // un-draw old
    rpm2(screen_number, val, 1);         // draw new
    last_rpm=val;                        // Remember what we just drew, so we can un-draw it later
  }
} //rpm 

void rpm2(uint8_t s,int val, uint8_t draw) {
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  scrn[s].Set_Text_colour(0,192,192); // teal
  sprintf(msgString, "%6dR", val);
  scrn[s].Set_Text_Size( 1 );  
  scrn[s].Print_String(msgString, 4*6*3,2*8);
} // rpm2



int socv(float val) { // Convert an Li-iON cell voltage to a charge %
  int pct;
  if(val<3.7) {
    val = 133.33*(val*val*val) - 1365.0*(val*val) + 4671.2*val - 5341.6; // See https://www.powerstream.com/lithium-ion-charge-voltage.htm
  } else {
    val = 175.33*val*val*val - 2304.0*val*val + 10164*val - 14939;
  }
  pct=val;
  if(pct>150)pct=150; // calc problem.
  return pct; // soc(screen_number,pct);
} // socv


// Function to display the remaining battery capacity in numbers (%) 
void soc(uint8_t screen_number, int val) { 
  if( last_soc != val ) { // Skip re-drawing anything that hasn't changed
    sel_screen(1 << screen_number);
    soc2(screen_number, last_soc, 0);   // un-draw old
    soc2(screen_number, val, 1);         // draw new
    last_soc=val;                        // Remember what we just drew, so we can un-draw it later
  }
} // soc

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
  if(spc>129)spc=130; // ignore 101%+
  //                              X  Y      W       H
  if(spc<130) scrn[s].Fill_Rect( 121,0,       129, 129-spc, BLACK); // Black top (overwrite prior green)
  if(spc>0){
    if(spc>49) scrn[s].Fill_Rect( 121,130-spc, 129,129,      GREEN); // Green bottom
    else if(spc>24) scrn[s].Fill_Rect( 121,130-spc, 129,129,      YELLOW);
    else scrn[s].Fill_Rect( 121,130-spc, 129,129,      RED);
  }

    // Draw_Line(x0, y0, x1, y1);
    // Draw_Fast_HLine(int16_t x, int16_t y, int16_t w);
  // mylcd.Set_Draw_color (YELLOW);, mylcd.Draw_Round_Rectangle (70,70,100,100,6);, mylcd.Draw_Fast_VLine (80,80,30);, mylcd.Draw_Fast_HLine (80,80,30);
  // mylcd.Draw_Circle (70,40,16);  , mylcd.Fill_Circle (73,46,6);, mylcd.Draw_Triangle (5,110, 15,120,50,16);, mylcd.Fill_Triangle (90,125, 20,120,50,99);
} // socBar

// degrees symbol: (246 I think)  ö


// Function to display pretty temperature data
void tempC(uint8_t s,uint32_t val, uint8_t draw, int x_offset, int y_offset, char *label, uint32_t orange, uint32_t red) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else { 
    if(val >= red) scrn[s].Set_Text_colour(RED);
    else if(val >= orange) scrn[s].Set_Text_colour(YELLOW);
    else scrn[s].Set_Text_colour(GREEN);
  }

  #define TEMP_SIZE 2  // 4 * 6 = 24px wide, 4 * 8 = 32px high
  scrn[s].Set_Text_Size( TEMP_SIZE );  
  
  sprintf(msgString, "%3lu", val);
 
  scrn[s].Print_String(msgString, x_offset,         y_offset);
  scrn[s].Set_Text_Size( 1 );  
  scrn[s].Print_String(degsym,    x_offset + 3*2*6, y_offset);
  scrn[s].Print_String(label,    x_offset + 2*6, y_offset+2*8);
  
} // tempC


// Function to display pretty temperature data
void FtempC(uint8_t s,uint8_t sz, uint32_t val, uint8_t draw, int x_offset, int y_offset, char *label, uint32_t orange, uint32_t red) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else { 
    if(val >= red) scrn[s].Set_Text_colour(RED);
    else if(val >= orange) scrn[s].Set_Text_colour(YELLOW);
    else scrn[s].Set_Text_colour(GREEN);
  }

  scrn[s].Set_Text_Size( sz );  
  
  sprintf(msgString, "%1s %3lu", label, val);
 
  scrn[s].Print_String(msgString, x_offset,         y_offset);
  scrn[s].Set_Text_Size( 1 );  
  scrn[s].Print_String(degsym,    x_offset + 5*sz*6, y_offset);
  // scrn[s].Print_String(label,    x_offset + sz*6, y_offset+sz*8);
  
} // FtempC


void temp_m(uint8_t screen_number, uint32_t val) { if( last_temp_m != val ) { tempC(screen_number, last_temp_m, 0, 0,       4*8, (char*)"",  80,  90); last_temp_m=val; tempC(screen_number, val, 1, 0,       4*8, (char*)" Motor",  80,  90); }}
void temp_e(uint8_t screen_number, uint32_t val) { if( last_temp_e != val ) { tempC(screen_number, last_temp_e, 0, 4*2*6+4, 4*8, (char*)"",  90, 100); last_temp_e=val; tempC(screen_number, val, 1, 4*2*6+4, 4*8, (char*)" ESC",    90, 100); }}
void temp_b(uint8_t screen_number, uint32_t val) { if( last_temp_b != val ) { tempC(screen_number, last_temp_b, 0, 0,       7*8, (char*)"",  45,  50); last_temp_b=val; tempC(screen_number, val, 1, 0,       7*8, (char*)"Battery", 45,  50); }}
void temp_f(uint8_t screen_number, uint32_t val) { if( last_temp_f != val ) { tempC(screen_number, last_temp_f, 0, 4*2*6+4, 7*8, (char*)"", 110, 120); last_temp_f=val; tempC(screen_number, val, 1, 4*2*6+4, 7*8, (char*)" FET",   110, 120); }}
//												    sz   val       drw  x       y    lbal      orange red
void Ftemp_m(uint8_t screen_number, uint32_t val) { if( last_temp_m != val ) { FtempC(screen_number, 3,last_temp_m, 0, 0,  7*8+0*3*9-2, (char*)"",  80,  90); last_temp_m=val; FtempC(screen_number, 3,val, 1, 0,  7*8+0*3*9-2, (char*)"M",  80,  90); }}
void Ftemp_b(uint8_t screen_number, uint32_t val) { if( last_temp_b != val ) { FtempC(screen_number, 3,last_temp_b, 0, 0,  7*8+1*3*9-2, (char*)"",  45,  50); last_temp_b=val; FtempC(screen_number, 3,val, 1, 0,  7*8+1*3*9-2, (char*)"B",  45,  50); }}
void Ftemp_e(uint8_t screen_number, uint32_t val) { if( last_temp_e != val ) { FtempC(screen_number, 3,last_temp_e, 0, 0,  7*8+2*3*9-2, (char*)"",  90, 100); last_temp_e=val; FtempC(screen_number, 3,val, 1, 0,  7*8+2*3*9-2, (char*)"E",  90, 100); }}

// Function to display Diagnostic bit string
void diag(uint8_t s,unsigned long int val, uint8_t draw, int x_offset, int y_offset, char *label, uint8_t colr) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else { 
    if(colr==1) scrn[s].Set_Text_colour(RED);
    else if(colr==2) scrn[s].Set_Text_colour(ORANGE);
    else if(colr==3) scrn[s].Set_Text_colour(YELLOW);
    else if(colr>3) scrn[s].Set_Text_colour(WHITE);
  }
  scrn[s].Set_Text_Size( 1 );
  sprintf(msgString, "%s%08lX", label, val);
  //  scrn[s].Print_Number_Int(val, 0, 16, 0, ' ',16);
  scrn[s].Print_String(msgString, x_offset,         y_offset);
} // diag

// Function to display Diagnostic bit string
void diags(uint8_t s,unsigned int val, uint8_t draw, int x_offset, int y_offset, char *label, uint8_t colr) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else { 
    if(colr==1) scrn[s].Set_Text_colour(RED);
    else if(colr==2) scrn[s].Set_Text_colour(ORANGE);
    else if(colr==3) scrn[s].Set_Text_colour(YELLOW);
    else if(colr>3) scrn[s].Set_Text_colour(WHITE);
  }
  scrn[s].Set_Text_Size( 1 );
  sprintf(msgString, "%s%04X", label, val);
  //  scrn[s].Print_Number_Int(val, 0, 16, 0, ' ',16);
  scrn[s].Print_String(msgString, x_offset,         y_offset);
} // diags

void volt_v(uint8_t s, float val, uint8_t draw, int x_offset, int y_offset, char *label, uint8_t nlen, uint8_t digs) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(WHITE);
  dtostrf(val, nlen, digs, &msgString[100]);
  sprintf(msgString, label, &msgString[100]);
  scrn[s].Set_Text_Size( 1 );
  scrn[s].Print_String(msgString, x_offset, y_offset);
} // volt_v

void percent(uint8_t s, int val, uint8_t draw, int x_offset, int y_offset, char *label) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Back_colour(BLACK);
  if(!draw) scrn[s].Set_Text_colour(BLACK);
  else scrn[s].Set_Text_colour(WHITE);
  sprintf(msgString, label, val);
  scrn[s].Set_Text_Size( 1 );
  scrn[s].Print_String(msgString, x_offset, y_offset);
} // percent



void diag_e(uint8_t screen_number,  unsigned long int val) { if( last_diag_e != val ) { diag(screen_number,  last_diag_e,  0, 0,    13*8+3, (char*)"E:",1);   last_diag_e=val;  diag(screen_number,  val, 1, 0,    13*8+3, (char*)"E:",1); }}
void diag_w(uint8_t screen_number,  unsigned long int val) { if( last_diag_w != val ) { diag(screen_number,  last_diag_w,  0, 0,    14*8+3, (char*)"W:",2);   last_diag_w=val;  diag(screen_number,  val, 1, 0,    14*8+3, (char*)"W:",2); }}
void diag_n(uint8_t screen_number,  unsigned long int val) { if( last_diag_n != val ) { diag(screen_number,  last_diag_n,  0, 0,    15*8+3, (char*)"N:",3);   last_diag_n=val;  diag(screen_number,  val, 1, 0,    15*8+3, (char*)"N:",3); }}
void diags_r(uint8_t screen_number,      unsigned int val) { if( last_diags_r != val ) { diags(screen_number, last_diags_r, 0, 11*6, 14*8+3, (char*)"rsv:",4); last_diags_r=val; diags(screen_number, val, 1, 11*6, 14*8+3, (char*)"rsv:",4); }}
void diags_i(uint8_t screen_number,      unsigned int val) { if( last_diags_i != val ) { diags(screen_number, last_diags_i, 0, 11*6, 15*8+3, (char*)"Int:",5); last_diags_i=val; diags(screen_number, val, 1, 11*6, 15*8+3, (char*)"Int:",5); }}

void volt_i(uint8_t screen_number, float val) { if( last_volt_i != val ) { volt_v(screen_number, last_volt_i, 0, 3*6, 1+10 * 8, (char*)"%s", 5, 1); last_volt_i=val;  volt_v(screen_number, val, 1, 0, 1+10 * 8, (char*)"vIN%s", 5, 1); }} 
void volt_e(uint8_t screen_number, float val) { if( last_volt_e != val ) { volt_v(screen_number, last_volt_e, 0, 3*6, 1+11 * 8, (char*)"%s", 7, 3); last_volt_e=val;  volt_v(screen_number, val, 1, 0, 1+11 * 8, (char*)"vEX%s", 5, 1); }} 
void volt_b(uint8_t screen_number, float val) { if( last_volt_b != val ) { volt_v(screen_number, last_volt_b, 0, 3*6, 1+12 * 8, (char*)"%s", 5, 1); last_volt_b=val;  volt_v(screen_number, val, 1, 0, 1+12 * 8, (char*)"BUS%s", 5, 1); }} 

void pc_t(uint8_t screen_number, int val) { if( last_pc_t != val ) { percent(screen_number, last_pc_t, 0, (11+3)*6, 1+11 * 8, (char*)"%d"); last_pc_t=val; percent(screen_number, val, 1, (11)*6, 1+11 * 8, (char*)"Th %d %%"); }}
void pc_m(uint8_t screen_number, int val) { if( last_pc_m != val ) { percent(screen_number, last_pc_m, 0, (11+3)*6, 1+12 * 8, (char*)"%d"); last_pc_m=val; percent(screen_number, val, 1, (11)*6, 1+12 * 8, (char*)"Mt %d %%"); }}
void pc_c(uint8_t screen_number, int val) { if( last_pc_c != val ) { percent(screen_number, last_pc_c, 0, (11+3)*6, 1+13 * 8, (char*)"%d"); last_pc_c=val; percent(screen_number, val, 1, (11)*6, 1+13 * 8, (char*)"Cp %d %%"); }}

void phase_a(uint8_t screen_number, int val) { if( last_phase_a != val ) { percent(screen_number, last_phase_a, 0, (11+4)*6, 1+10 * 8, (char*)"%s"); last_phase_a=val; percent(screen_number, val, 1, (11)*6, 1+10 * 8, (char*)"PhA %d"); }} 

void signal(uint8_t s,bool has_sig) {
  sel_screen(1 << s);
  scrn[s].Set_Text_Size(1);
  if(has_sig) {//	  1234567890123 = 13 = 78px wide
    scrn[s].Set_Text_Back_colour(BLACK);
    scrn[s].Set_Text_colour(RED);
    scrn[s].Print_String(" No Valid CAN Signal ", 2, 24);
  } else {
    scrn[s].Set_Text_Back_colour(RED);
    scrn[s].Set_Text_colour(WHITE);
    //scrn[s].Print_String(" No CAN Signal ", 20, 122); // 57 was middle of screen
    scrn[s].Print_String(" No CAN Signal ", 20, 24); // 57 was middle of screen
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
  if(flymode) {
    Fvolts(0,88.8);
    Ftemp_m(0,888);	// Motor
    Ftemp_b(0,888);	// Battery
    Ftemp_e(0,888);	// ESC
  } else {
    volts(0,88.8);
    amps(0,-88.8);
    watts(0,88888);
    rpm(0,8888);
    temp_m(0,888);	// Motor
    temp_e(0,888);	// ESC
    temp_b(0,888);	// Battery
    temp_f(0,888);	// FET

    diag_e(0,0x88888888);	// Error code
    diag_w(0,0x88888888);	// Warning code
    diag_n(0,0x88888888);	// Notice code
    diags_r(0,0x8888);	// reserved code
    diags_i(0,0x8888);	// ESC Init code

    volt_i(0,88.8);	// vIN	y=10
    volt_e(0,88.888);	// vEX	y=11
    volt_b(0,88.8);	// BUS	y=12

    phase_a(0,888);	// 		y=10 x=11*6
    pc_t(0,100);		// Throttle %	y=11
    pc_m(0,100);		// Motor PWM %	y=12
    pc_c(0,socv(3.8));		// Battrey Capacity % 	y=13
  }


  //check_engine(0,1);
  //pnd(0,1);
  //soc(0,88);
  socBar(0,88); // 88 percent
  //show_font(1);  
  //tempC(0,88,0,0); // Caller (us here) needs to cache the previous temp numbers
  //tempC(0,88,0,130/2);
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

    //pnd(s, s%3); // 0 (P) ,1 (N), ,2 (D)

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
    //check_engine(0,1);
  } 

  sel_screen(0); // de-select them
  delay(10);
  sel_screen(1);
  scrn[0].setRotation(1); // Orient all the screens so "UP is UP" // 0 puts top at 9oclock // 1 puts top at noon

  instrument_check(); // Show 888 on everything (for testing)
}








void good_can() {
  if(can_ok<3) { can_ok=3; sel_screen(1+2+4+8); scrn[0].Fill_Screen(BLACK); } // Clear all guage-test readings as soon as we get any real good data
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
    CAN0.readMsgBuf(&rxId, &len, rxBuf.b);              // Read data: len = data length, buf = data byte(s)

    if ((rxId>0)||(len>0)) { // we were getting spurious zeros too much...
      if(can_ok<2) { can_ok=2; signal(0,true); }

     if((rxId & 0x80000000) == 0x80000000) {            // HBCi used Extended IDs
       unsigned long hbcid=rxId & 0x1FFFFFFF;

       
       if	 (hbcid==0x14A30001) { good_can();
	 //uint32_t batvolti=rxBuf[1];batvolti*=256;batvolti+=rxBuf[0]; // was 1 and 0
	 //float batvolt=float(batvolti); batvolt=batvolt/10.0;	// 14s or 15s.  	100%=4.2v  0%=3.0v
	 float batvolt=float(rxBuf.i[0]); batvolt=batvolt/10.0;	// 14s or 15s.  	100%=4.2v  0%=3.0v
							// 45v .. 63v		0x251 = 59.3v
							// 42v .. 58.8v      peter: 3v=0
	 if(rxBuf.si[0]!=0) volt_i(0,batvolt);             
	 //uint32_t batampsu=rxBuf[3];batampsu*=256;batampsu+=rxBuf[2];	// fix this (signed)
	 //int32_t batampsi=(int32_t)batampsu;
	 float batamps=float(rxBuf.si[1]); batamps=batamps/10.0;
	 if(!flymode) amps(0,batamps);
	 //uint64_t rpmi=rxBuf[7];rpmi*=256;rpmi += rxBuf[6];rpmi*=256;rpmi += rxBuf[5];rpmi*=256;rpmi += rxBuf[4];
	 if(!flymode) rpm(0,rxBuf.l[1]);

       } else if (hbcid==0x14A30002) {  good_can(); // Extended ID: 0x14A30002  DLC: 8  Data: 0x1D 0x00 0x1D 0x00 0x1E 0x00 0x00 0x00

   //uint32_t motortemp=rxBuf[1]*=256+rxBuf[0]; // 90c max
	 //uint32_t motortemp=rxBuf[1]; motortemp*=256; motortemp=+rxBuf[0];  // 90c max
	 //uint32_t esctemp=rxBuf[3];   esctemp*=256;   esctemp+=rxBuf[2];		// 100c max
	 //uint32_t exttemp=rxBuf[7];   exttemp*=256;   exttemp+=rxBuf[6];		// 50c max
	 //temp_m(0,motortemp);
	 //temp_e(0,esctemp);
	 //temp_b(0,exttemp);
	 if(flymode) Ftemp_m(0,rxBuf.i[0]); else temp_m(0,rxBuf.i[0]);  // motortemp;
	 if(flymode) Ftemp_e(0,rxBuf.i[1]); else temp_e(0,rxBuf.i[1]);	//esctemp
	 if(flymode) Ftemp_b(0,rxBuf.i[3]); else temp_b(0,rxBuf.i[3]);	//exttemp

   //Serial.print(motortemp); Serial.print(" <= motor temp\n");
   
       } else if (hbcid==0x14A30003) { good_can();
	 //uint64_t pwmout=rxBuf[3];pwmout*=256; pwmout+=rxBuf[2];pwmout*=256; pwmout+=rxBuf[1];pwmout*=256; pwmout+=rxBuf[0];	// /1023 % ?
	 //uint64_t pwmin=rxBuf[7]; pwmin*=256;  pwmin+= rxBuf[6]; pwmin*=256; pwmin+= rxBuf[5];pwmin*=256;  pwmin+= rxBuf[4];	//
	 uint64_t pwmout=rxBuf.l[0];
	 uint64_t pwmin=rxBuf.l[1];
	 pwmout/=1023;
	 if(pwmout<101) { int p=pwmout; pc_t(0,p); }
	 pwmin/=1023;
	 if(pwmin<101) { int p=pwmin; pc_m(0,p); }
	 
       } else if (hbcid==0x14A30004) { good_can();
	 uint64_t error=rxBuf.l[0];   //rxBuf[3];error*=256; error+=rxBuf[2];error*=256; error+=rxBuf[1];error*=256; error+=rxBuf[0];
	 uint64_t warning=rxBuf.l[1]; //rxBuf[7];warning*=256;warning+=rxBuf[6];warning*=256;warning+=rxBuf[5];warning*=256;warning+=rxBuf[4];
	 if(!flymode) diag_e(0,error);
	 if(!flymode) diag_w(0,warning);
	 
       } else if (hbcid==0x14A30005) { good_can();
	 uint64_t notice=rxBuf.l[0]; // rxBuf[3];notice*=256;notice+=rxBuf[2];notice*=256;notice+=rxBuf[1];notice*=256;notice+=rxBuf[0];
	 uint32_t escstatus=rxBuf.i[3]; //rxBuf[7];escstatus*=256;escstatus+=rxBuf[6];
	 if(!flymode) diag_n(0,notice);
	 if(!flymode) diags_i(0,escstatus);
	 
       } else if (hbcid==0x14A30006) { good_can();
	 uint32_t batintvolti=rxBuf.i[0]; //rxBuf[1];batintvolti*=256;batintvolti+=rxBuf[0];
	 float batintvolt=float(batintvolti); batintvolt/=10.0;
 // Serial.print(batintvolti); Serial.print(" "); Serial.print(batintvolt); Serial.print(" <==v\n");
	 uint32_t extfeedvolti=rxBuf.i[1]; //rxBuf[3];extfeedvolti*=256;extfeedvolti+=rxBuf[2];
	 float extfeedvolt=float(extfeedvolti); extfeedvolt/=1000.0;
	 uint32_t phaseamps=rxBuf.i[3]; // rxBuf[7];phaseamps*=256;phaseamps+=rxBuf[6];
	 if(flymode) Fvolts(0,batintvolt);
	 if(!flymode) volts(0,batintvolt);
	 if(!flymode) volt_e(0,extfeedvolt);
	 if(!flymode) phase_a(0,phaseamps);
	 
       } else if (hbcid==0x14A30007) { good_can();
	 uint32_t mosfettemp=rxBuf.b[7];// ;mosfettemp*=256;mosfettemp+=rxBuf[6];
	 if(!flymode) temp_f(0,mosfettemp);
	 
       }

     }

     if(!flymode) {
       if((rxId & 0x80000000) == 0x80000000) {            // Determine if ID is standard (11 bits) or extended (29 bits)
         sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
         sprintf(buf,"EX:%.8lX L:%1d d:", (rxId & 0x1FFFFFFF), len);
       } else {
         sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
         sprintf(buf,"ID:%.3lX L:%d ", rxId, len);
       }
   
       Serial.print(msgString);
     
   
       if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
         //sprintf(msgString, " REMOTE REQUEST FRAME");
         Serial.print("RMT REQ");
         if(strlen(buf)<46) sprintf(&buf[strlen(buf)],"RMT"); // buf is 50 bytes
       } else {
         for(byte i = 0; i<len; i++){
           sprintf(msgString, " 0x%.2X", rxBuf.b[i]);				// 0        1         2         3         4         5
           Serial.print(msgString);						// 12345678901234567890123456789012345678901234567890
           if(strlen(buf)<46) sprintf(&buf[strlen(buf)],"%.2X ", rxBuf.b[i]);	// EX:12345678 L:123 d:12 34 56 78 90 12 34 56.
         }
       }
     }

#ifdef SHOW_DEBUG_ON_LCD
 
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

#endif

     if(!flymode) Serial.println();

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


/* Sample data capture:-



#       v3.2 C:\Users\cnd\Documents\Arduino\Arduino_Nano_SSD1283A_CAN_e_Paramotor_Display\Arduino_Nano_SSD1283A_CAN_e_Paramotor_Display.ino     Jan  3 2022 13:34:39
Entering Configuration Mode Successful!
Setting Baudrate Successful!
MCP2515 Initialized Successfully!
Extended ID: 0x14A30001  DLC: 8  Data: 0x51 0x02 0x00 0x00 0x00 0x00 0x00 0x00
Extended ID: 0x14A30002  DLC: 8  Data: 0x1C 0x00 0x1A 0x00 0x1E 0x00 0x00 0x00
Extended ID: 0x14A30003  DLC: 4  Data: 0x00 0x00 0x00 0x00
Extended ID: 0x14A30007  DLC: 8  Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x7C 0x00  * many

Extended ID: 0x14A30006  DLC: 8  Data: 0x51 0x02 0x00 0x00 0x00 0x00 0x00 0x00
Extended ID: 0x14A30006  DLC: 8  Data: 0x51 0x02 0x00 0x00 0x00 0x00 0x00 0x00
Extended ID: 0x14A30001  DLC: 8  Data: 0x50 0x02 0x04 0x00 0x15 0x03 0x00 0x00
Extended ID: 0x14A30001  DLC: 8  Data: 0x50 0x02 0x2E 0x00 0x16 0x03 0x00 0x00
Extended ID: 0x14A30001  DLC: 8  Data: 0x51 0x02 0xDA 0xFF 0xE6 0x06 0x00 0x00
Extended ID: 0x14A30002  DLC: 8  Data: 0x1D 0x00 0x1B 0x00 0x1E 0x00 0x00 0x00
Extended ID: 0x14A30001  DLC: 8  Data: 0x50 0x02 0x00 0x00 0x00 0x00 0x00 0x00
Extended ID: 0x14A30002  DLC: 8  Data: 0x1D 0x00 0x1B 0x00 0x1E 0x00 0x00 0x00
Extended ID: 0x14A30003  DLC: 4  Data: 0x00 0x00 0x00 0x00 



Extended ID: 0x14A30001  DLC: 8  Data: 0x51 0x02 0xDA 0xFF 0xE6 0x06 0x00 0x00
	01= battery terminal voltage	x0.1V		0x251 = 59.3
	23= current			x0.1A		0xFFDA =
	4567= RPM			x1RPM
Extended ID: 0x14A30002  DLC: 8  Data: 0x1C 0x00 0x1A 0x00 0x1E 0x00 0x00 0x00
	01 = motor temp			x1C
	23 = ESC temp			x1C
	67 = ext temp			x1C
Extended ID: 0x14A30003  DLC: 4  Data: 0x00 0x00 0x00 0x00
	01 = output PWM			x1/1023
	23 = input PWM			x1/1023
Extended ID: 0x14A30004  DLC: 8  Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
	0123 = ERROR  (bit patterns)
	4567 = WARNING
Extended ID: 0x14A30005  DLC: 8  Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
	0123 = NOTICE  (bit patterns)
	67 = ESC Init status
Extended ID: 0x14A30006  DLC: 8  Data: 0x51 0x02 0x00 0x00 0x00 0x00 0x00 0x00
	01= battery internal voltage	x0.1V
	23= external feeding voltage	x0.001V
	67= phase current		x1A
Extended ID: 0x14A30007  DLC: 8  Data: 0x00 0x00 0x00 0x00 0x00 0x00 0x7C 0x00  * many
	6= Internal BUS voltage		x0.1v
	7= MOS-FET temperature		x1C

BBatt

14s, 15s.
3v is 0%

Batt temp: 50c
Motor 90c
Controler temp 100c


FontSz  Cols	Rows
1	21	16
2	10	8
3	7	5
4	5	4
5	4	3
6	3	2
7	3	2
8	2	2
9	2	1


Volts	54.6
Amps	-999.1
Watts	99,999
RPM	9999
Motor	111.C
ESC	111.C
FET	111.C
Batt	111.C
Throttle -100%
Motor	-100%
Error	1234ABCD
Warn    1234ABCD
Notice  1234ABCD
INIT	12AB
vIN	88.8
vEX	88.001
PhaseA	888
?	12AB
vBUS	88.8










*/
