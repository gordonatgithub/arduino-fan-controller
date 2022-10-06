// IMPORTANT: ELEGOO_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Elegoo_TFTLCD.h FOR SETUP.
//Technical support:goodtft@163.com

#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library

//Include Wire Library
#include <Wire.h>

// use the TFT display
#define ENABLE_DISPLAY 1

// Use the fan controller IC
#define ENABLE_FAN_CONTROLLER_IC 1
#define ENABLE_AUTO_FAN_TEMP_CONTROL 0

// Config debug dump during loop()
#define ENABLE_FAN_DEBUG 0
#define ENABLE_SERIAL_SYNC 0

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Elegoo_TFTLCD tft;

// yea, globals
// int == 16 bit
struct fan_control
{
  int speed; // meausred fan RPM speed 
  float temp; // measured temperature from thermoresistor
  float temp_peak;// last measured peak temperature
  int percentage; // fan speed percent from max
  int prev_draw_percent; // last drawn fan speed percent
  int fan_fail_status; // Fan failure
  int over_temperature_status; // Over temperature output pin
};

#define MAX_FANS 2

// Fan characteristics for Noctua NF-A14 PWM
//#define MIN_FAN_RPM 300  // 300 RPM at 20% speed
//#define MAX_FAN_RPM 1600 // spec is 1500, measured is ~1605

// Fan characteristics for BL067 SilentWing3 PWM
#define MIN_FAN_RPM 200  // 200 RPM at 20% speed
#define MAX_FAN_RPM 1000 // spec is 1000

// User defined warnning temps (UI notification only, doesn't modify fan behavior)
#define TEMP_WARNING 30 // degrees C
#define TEMP_CRITCAL 40 // degrees C

#define UPDATE_MS 250 // update cycle in milliseconds

struct fan_control fan_ctrl[MAX_FANS];
int running = -1;

void setup(void)
{
  Serial.begin(9600);

#if ENABLE_SERIAL_SYNC  
  while (!Serial); //leonardo workaround to sync to debugger
#endif

#if ENABLE_DISPLAY
  // init TFT display
  initTFT();
#endif
  
  // init vars
  initFanStruct();
  
#if ENABLE_DISPLAY
  // Init display
  initFanScreen();
#endif

#if ENABLE_FAN_CONTROLLER_IC
  // init SMBus
  // range 10-400 kHz, default arduino is 100 kHz
  Wire.begin();
  
  // Init MAX6615 fan controller
  running = initFanCtrl();
#endif

  Serial.println(F("Init Done!"));
}

int debug_counter = 0;
void loop(void)
{
  unsigned long start, end;
  start = micros();

  if (running == 0)
  {
#if ENABLE_FAN_CONTROLLER_IC
    running = updateFanResults();
#endif

#if ENABLE_DISPLAY
    updateFanScreen();
#endif 
  }
  else
  {
#if ENABLE_DISPLAY
    drawErrorScreen();
#endif 
  }

  // serial debug 
  end = micros() - start;

#if ENABLE_FAN_DEBUG
  Serial.print("Loop: ");
  Serial.print(debug_counter);
  Serial.print(", ");
  Serial.print(end);
  Serial.print(" us\n");
  
  debug_counter++;
#endif

  // disable delay since TFT draw time is ~950 ms
#if (ENABLE_DISPLAY == 0)  
  delay(UPDATE_MS);
#endif  
}

void initTFT()
{
  Serial.println(F("TFT LCD init"));

#ifdef USE_Elegoo_SHIELD_PINOUT
  Serial.println(F("Using Elegoo 2.8\" TFT Arduino Shield Pinout"));
#else
  Serial.println(F("Using Elegoo 2.8\" TFT Breakout Board Pinout"));
#endif

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());

  tft.reset();

  uint16_t identifier = tft.readID();
   if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  }else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier==0x0101)
  {     
      identifier=0x9341;
       Serial.println(F("Found 0x9341 LCD driver"));
  }else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Elegoo 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_Elegoo_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Elegoo_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    identifier=0x9341;
  }

  tft.begin(identifier);  
}

void initFanStruct()
{
  memset(fan_ctrl, 0, MAX_FANS * sizeof(struct fan_control));
}

unsigned long initFanScreen()
{
  tft.fillScreen(BLACK);
  tft.setRotation(1);

  Serial.print("TFT rot size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
  
  return 0;
}

unsigned long updateFanScreen()
{
  tft.setCursor(0, 0);
  tft.setTextColor(YELLOW, BLACK);
  tft.setTextSize(2);

  // Update Fan A params
  drawFanInfo(0);
  tft.print("\n\n");
  // Update Fan B params
  drawFanInfo(1);
  
  // LARGE MAX temp
  drawFanMaxTemp();

  drawFanGraph();
}

void drawFanInfo(int index)
{
  char pstr[32];
  int temp_whole; // whole integer part
  int temp_dec; // decimal part

  // sanity check
  if (index >= MAX_FANS)
  {
    Serial.println("Invalid fan index!");
    return;
  }
  
  sprintf(pstr, "Fan %c", 'A' + index);
  tft.println(pstr);

  // print out max string length, otherwise parts of text will be left over
  if (fan_ctrl[index].fan_fail_status == 1)
  {
    tft.setTextColor(WHITE, RED);
    tft.println("TACH FAIL");
    tft.setTextColor(YELLOW, BLACK);
  }
  else if (fan_ctrl[index].fan_fail_status == 1)
  {
    tft.setTextColor(WHITE, RED);
    tft.println("OVER HEAT");
    tft.setTextColor(YELLOW, BLACK);
  }
  else if (fan_ctrl[index].percentage == 0)
  {
    tft.println("Off      ");
  }
  else
  {
    tft.println("Running  ");
  }
  
  sprintf(pstr, "RPM %4d  %3d%%\n", fan_ctrl[index].speed % 10000, fan_ctrl[index].percentage);
  tft.print(pstr);  
  temp_whole = (int)fan_ctrl[index].temp;
  temp_dec = (int)((fan_ctrl[index].temp - (float)temp_whole) * 100);
  sprintf(pstr, "Temp %3d.%02d", temp_whole % 1000, temp_dec);
  tft.print(pstr);
  //tft.print("\367""C\n"); // degree symbol
  tft.print(" C\n");
}

void drawFanMaxTemp()
{
  int i;
  int hi_temp = 0;
  float max_temp = 0.0f;
  char buf[4];
  
  for (i = 0; i < MAX_FANS; i++)
  {
    max_temp = max(fan_ctrl[i].temp, max_temp);  
  }
  
  hi_temp = round(max_temp); // round to nearest integer
  
  tft.setTextSize(5); // 50 pixels high
  tft.setCursor(0, tft.height() - 50); // draw on very bottom
  
  // change color if over limit
  if (hi_temp >= TEMP_CRITCAL)
  {
    tft.setTextColor(RED, BLACK);
  }
  else if (hi_temp >= TEMP_WARNING)
  {
    tft.setTextColor(YELLOW, BLACK);
  }
  else
  {
    tft.setTextColor(GREEN, BLACK);    
  }

  sprintf(buf, "%3d", hi_temp);
  tft.print(buf);
  tft.println(" C");
}

void drawFanGraph()
{
  // draw rectacle representing percentage of fan speed for each temperature
  int i;
  int x1, y1, w, h;
  int old_h, old_y1;
  int color;
  w = 36; // rectangle width

  for (i = 0; i < MAX_FANS; i++)
  {
    // get percentage of fanspeed
    int val;// = fan_ctrl[i].speed * 100 / MAX_FAN_RPM; // don't use .speed since update cycle is too long
    val = constrain(fan_ctrl[i].percentage, 0, 100);

    x1 = tft.width() - (3*w  + w/2) + (2*w*i); // assume 320 pixel wide screen // move next bar over

    // compare to old value, if nothing changes skip drawing
    if (val == fan_ctrl[i].prev_draw_percent)
    {
      continue;
    }

    // optimize drawing to reduce screen flicker
    // draw rectangle from bottom growing up
    h = val * tft.height() / 100;
    old_h = fan_ctrl[i].prev_draw_percent * tft.height() / 100;
    y1 = tft.height() - h; // upper left corner
    old_y1 = tft.height() - old_h; // upper left corner

    if (val > fan_ctrl[i].prev_draw_percent)
    {
      // usage percent went up, draw only delta of bar graph
      h = h - old_h;
      color = BLUE;
    }
    else
    {
      // usage percent went down, remove bar graph (ie, draw more black!)
      y1 = old_y1;
      h = old_h - h;
      color = BLACK;
    }

    tft.fillRect(x1, y1, w, h, color);

    fan_ctrl[i].prev_draw_percent = val; // remember current value

    /*
    Serial.print("Fan ");
    Serial.print(i);
    Serial.print(val);
    Serial.print("%, pos (");
    Serial.print(x1);
    Serial.print(", ");
    Serial.print(y1);
    Serial.print(") ");
    Serial.print(" (");
    Serial.print(w);
    Serial.print(", ");
    Serial.print(h);
    Serial.println(") ");
    /**/    
  }
  return;
}

void drawErrorScreen()
{
  // turn the screen RED
  tft.fillScreen(RED);
}

//////////////////////////////////////////////////////////////////////////////
// MAX 6615//6616 Fan controller
//////////////////////////////////////////////////////////////////////////////
// SMBus device address
#define FAN_CTRL_ADDR 24          // 0x18, ADD0 = ADD1 = 0

// Register address definition
#define FAN_REG_TEMP_HI_1 0          // Channel 1 temp MSBs
#define FAN_REG_TEMP_HI_2 1          // Channel 2 temp MSBs
#define FAN_REG_CFG 2
#define FAN_REG_OT_LIMIT_1 3      // Temperature channel 1 /OT limit
#define FAN_REG_OT_LIMIT_2 4
#define FAN_REG_OT_STATUS 5     // /OT status
  #define OT_FAN_1 0x80           // fan 1 Over Temperature
  #define OT_FAN_2 0x40           // fan 2 Over Temperature
#define FAN_REG_OT_MASK 6
#define FAN_REG_PWM_START_1 7     // PWM1 start duty cycle
#define FAN_REG_PWM_START_2 8
#define FAN_REG_PWM_MAX_1 9       // PWM1 max duty cycle
#define FAN_REG_PWM_MAX_2 10
#define FAN_REG_PWM_TARGET_1 11   // PWM1 target duty cycle
#define FAN_REG_PWM_TARGET_2 12
#define FAN_REG_PWM_VALUE_1 13    // PWM1 instantaneous duty cycle
#define FAN_REG_PWM_VALUE_2 14
#define FAN_REG_PWM_FAN_START_1 15  // Channel 1 fan-start temperature
#define FAN_REG_PWM_FAN_START_2 16
#define FAN_REG_PWM_FAN_CTRL 17   // Fan configuration
#define FAN_REG_PWM_CYCLE_RATE 18   // Duty-cycle rate of change
#define FAN_REG_PWM_CYCLE_STEP 19   // Duty-cycle step size
#define FAN_REG_PWM_FREQ_SELECT 20   // PWM frequency select
#define FAN_REG_GPIO_CFG 21       // GPIO function
#define FAN_REG_GPIO_VALUE 22     // GPIO value
#define FAN_REG_THERMISTOR_OFFSET 23    // Thermistor offset register
#define FAN_REG_TACH_VALUE_1 24   // Tach1 value register
#define FAN_REG_TACH_VALUE_2 25   // Tach2 value register
#define FAN_REG_TACH_LIMIT_1 26   // Tach1 limit register
#define FAN_REG_TACH_LIMIT_2 27   // Tach1 limit register
#define FAN_REG_FAN_STATUS 28     // Fan status byte
  #define FAIL_FAN_1 0x80         // fan 1 failure
  #define FAIL_FAN_2 0x40         // fan 2 failure  
#define FAN_REG_TEMP_LO_1 30     // Channel 1 temp LSBs
#define FAN_REG_TEMP_LO_2 31     // Channel 2 temp LSBs
#define FAN_REG_DEV_REV 253       // Read device revision
  #define DEV_REV_DEFAULT 1       // 0x1
#define FAN_REG_DEV_ID 254        // Read device ID
  #define DEV_ID_DEFAULT 104      // 0x68
#define FAN_REG_VENDOR_ID 255     // Read manufacturer ID
  #define VENDOR_ID_DEFAULT 77    // 0x4D

void dev_write(int addr, int val)
{
  Wire.beginTransmission(FAN_CTRL_ADDR);

  Wire.write(addr);
  Wire.write(val);

  Wire.endTransmission();
}

int dev_read(int addr)
{
  int result = -1;

  Wire.beginTransmission(FAN_CTRL_ADDR);
  Wire.write(addr);
  Wire.endTransmission();
  
  Wire.requestFrom(FAN_CTRL_ADDR, 1);
  result = Wire.read();

  return result;
}

int initFanCtrl()
{
  int val;
  // Sanity check
  val = dev_read(FAN_REG_DEV_REV);
  if (val != DEV_REV_DEFAULT)
  {
    Serial.print("Invalid revision:  ");
    Serial.println(val);
    return -1;
  }
  val = dev_read(FAN_REG_DEV_ID);
  if (val != DEV_ID_DEFAULT)
  {
    Serial.print("Invalid device ID:  ");
    Serial.println(val);
    return -1;
  }
  val = dev_read(FAN_REG_VENDOR_ID);
  if (val != VENDOR_ID_DEFAULT)
  {
    Serial.print("Invalid manufacturer ID:  ");
    Serial.println(val);
    return -1;
  }

  Serial.println("Found MAX6615 device");

  // Configure device
  dev_write(FAN_REG_CFG, 0x18); // keep default
  
  // Set Over Temmperature limit
  dev_write(FAN_REG_OT_LIMIT_1, 50); // 50 C
  dev_write(FAN_REG_OT_LIMIT_2, 50); // 50 C

  // start off temp 20C (20% fan speed), max temp 38C (100% fan speed)
  // approx 48/240 -> 240/240 over 18C, each degree is ~10.66
  
  // set start PWM duty cycle
  dev_write(FAN_REG_PWM_START_1, 48); // 48/240 = 20% (noctua NF-A14 fan works down at 10%, spec is 20%)
  dev_write(FAN_REG_PWM_START_2, 48); // 48/240 = 20%

  // Fan Start temperature
  dev_write(FAN_REG_PWM_FAN_START_1, 20); // 68F
  dev_write(FAN_REG_PWM_FAN_START_2, 20); // 68F
  
  // Configure PWM fan settings (see table 5)
  dev_write(FAN_REG_PWM_CYCLE_RATE, 0x24); // 5s to ramp from 33% -> 100%, default is 80s

  // see table 6.
  dev_write(FAN_REG_PWM_CYCLE_STEP, 0x55); // each step is 10/240 % of duty cycle per degree

  dev_write(FAN_REG_PWM_FREQ_SELECT, 0x20); // 35 kHz

  // offset thermoresistor by -2 deg, range -16 to 14 C, steps of 2 C
  val = -2;
  val = (((val >> 1) & 0xF) << 4) | ((val >> 1) & 0xF);
  dev_write(FAN_REG_THERMISTOR_OFFSET, val);
  
  dev_write(FAN_REG_TACH_LIMIT_1, 1000); // set to a largish number
  dev_write(FAN_REG_TACH_LIMIT_2, 1000);

#if ENABLE_AUTO_FAN_TEMP_CONTROL
  // use Automatic PWM Duty-Cycle Control
  dev_write(FAN_REG_PWM_FAN_CTRL, 0x24);
#else
  // use Manual control
  dev_write(FAN_REG_PWM_FAN_CTRL, 0x0);
#endif // ENABLE_AUTO_FAN_TEMP_CONTROL

  return 0;
}

int updateFanResults()
{
  // READ SMBus registers
  int val;
  int i;
  
  // Check for FAN_FAIL and over temperature status
  val = dev_read(FAN_REG_FAN_STATUS);
  if ((val & FAIL_FAN_1) != 0)
  {
    Serial.println("Fan 1 failed"); // occurs if RPM is over threshold
    fan_ctrl[0].fan_fail_status = 1;
  }
  if ((val & FAIL_FAN_2) != 0)
  {
    Serial.println("Fan 2 failed"); // occurs if RPM is over threshold    
    fan_ctrl[1].fan_fail_status = 1;
  }
  
  val = dev_read(FAN_REG_OT_STATUS);
  if ((val & OT_FAN_1) != 0)
  {
    Serial.println("Fan 1 over temperature"); // occurs if temp threshold exceeded
    fan_ctrl[0].over_temperature_status = 1;
  }
  if ((val & OT_FAN_2) != 0)
  {
    Serial.println("Fan 2 over temperature"); // occurs if temp threshold exceeded      
    fan_ctrl[1].over_temperature_status = 1;
  }

  for (i = 0; i < MAX_FANS; i++)
  {
    // Update global variables
    fan_ctrl[i].temp = (float)(dev_read(FAN_REG_TEMP_HI_1 + i)) + (float)(dev_read(FAN_REG_TEMP_LO_1 + i) >> 5) * 0.125f;
  
    val = dev_read(FAN_REG_PWM_VALUE_1 + i);
    fan_ctrl[i].percentage = val * 100 / 240;
  
    val = dev_read(FAN_REG_TACH_VALUE_1 + i);
    // max measured value was 35 when fan was at ~1600 RPM. 
    // min measured value was 162 when fan was at ~354 RPM
    // approx appears to be time in ms per period of fan pulse, convert to RPM by 1 / (val/1000) * 60
    // updates once every 67 seconds(!)
    fan_ctrl[i].speed = (int)((60000) / (unsigned int)val); 
  }
  
#if (ENABLE_AUTO_FAN_TEMP_CONTROL == 0)
  // in manual fan mode, recalculate target speed
  updateFanSpeed();
#endif // ENABLE_AUTO_FAN_TEMP_CONTROL

#if ENABLE_FAN_DEBUG
  // DEBUG
  Serial.print("Fan TEMP: ");
  Serial.print(fan_ctrl[0].temp);
  Serial.print("  ");
  Serial.println(fan_ctrl[1].temp);
  Serial.print("Fan %: ");
  Serial.print(fan_ctrl[0].percentage);
  Serial.print("  ");
  Serial.println(fan_ctrl[1].percentage);
  Serial.print("Fan RPM: ");
  Serial.print(fan_ctrl[0].speed);
  Serial.print("  ");
  Serial.println(fan_ctrl[1].speed);

  // for manual mode
  /*
  val = dev_read(FAN_REG_PWM_TARGET_1);
  Serial.print("Fan Target: ");
  Serial.println(val);  
  */
  
  // DEBUG
  /*
  testing++;
  if ((testing % 40) == 0)
  {
    //dev_write(FAN_REG_PWM_TARGET_1, testval << 1);
    dev_write(FAN_REG_PWM_TARGET_1, 48);
    testval++;
  }
  */
  
  Serial.print("Status: ");
  Serial.print(fan_ctrl[0].fan_fail_status);
  Serial.print("  ");
  Serial.println(fan_ctrl[1].fan_fail_status);
  Serial.print("OT: ");
  Serial.print(fan_ctrl[0].over_temperature_status);
  Serial.print("  ");
  Serial.println(fan_ctrl[1].over_temperature_status);
#endif

  return 0;
}

// manual mode, user defined behavior of fan speed.
// use this mode since it seems(?!) MAX6615 cannot set 0% duty cycle when temperature is 
// below the fan-start temperature, it always sets to start duty cycle.

// For simplicty, use only temperature as input to table lookup for speed (as percentage or PWM duty cycle)
// table is based on 0C to max required. MAX6615 will limit negative temperature to zero

// start off temp 20C (20% fan speed), max temp 38C (100% fan speed)
#define FAN_SPEED_MAP_MAX 50 // since this is case fan, don't need to go so high.
#define HYSTERESIS_COOLOFF 3.0f // recalc after 3C drop

int fan_speed_map[FAN_SPEED_MAP_MAX] =
{
  0, // 0C
  0, // 1C
  0, // 2C
  0, // 3C
  0, // 4C
  0, // 5C
  0, // 6C
  0, // 7C
  0, // 8C
  0, // 9C
  0, // 10C
  0, // 11C
  0, // 12C
  0, // 13C
  0, // 14C
  0, // 15C
  0, // 16C
  0, // 17C
  0, // 18C
  0, // 19C
  20, // 20C
  24, // 21C
  29, // 22C
  33, // 23C
  37, // 24C
  42, // 25C
  46, // 26C
  51, // 27C
  55, // 28C
  60, // 29C
  64, // 30C
  68, // 31C
  73, // 32C
  77, // 33C
  82, // 34C
  86, // 35C
  90, // 36C
  95, // 37C
  100, // 38C
  100, // 39C
  100, // 40C
  100, // 41C
  100, // 42C
  100, // 43C
  100, // 44C
  100, // 45C
  100, // 46C
  100, // 47C
  100, // 48C
  100, // 49C
};


void updateFanSpeed()
{
  int val;
  int temp;
  int i;
  int recalc = 0;
  static int startup_delay = 0;

  if (startup_delay < 5)
  {
    // skip the first ~5 sec since it seems temp and speed are not stable after power on
    startup_delay++;
    return;
  }
  for (i = 0; i < MAX_FANS; i++)
  {
    // check if we need to recalculate value, use hysteresis when cooling down to prevent bouncing
    if (fan_ctrl[i].temp > fan_ctrl[i].temp_peak)
    {
      // temp is going up, recalculatelcula
      recalc = 1;
    }
    else if (fan_ctrl[i].temp <= (fan_ctrl[i].temp_peak - HYSTERESIS_COOLOFF))
    {
      // temp has dropped below cooloff, recalculate
      recalc = 1;
    }
    else
    {
      // do nothing
      recalc = 0;
    }

    if (recalc != 0)
    {
      temp = round(fan_ctrl[i].temp);
      temp = constrain(temp, 0, FAN_SPEED_MAP_MAX); // ensure valid array index
      val = fan_speed_map[temp];

#if ENABLE_FAN_DEBUG
      // DEBUG
      Serial.print("Fan ");
      Serial.print(i);
      Serial.print(" Target Speed: ");
      Serial.print(val);
      Serial.print("%\n");
#endif
    
      val = val * 240 / 100; // convert to PWM x/240 value
  
      // write to register
      dev_write(FAN_REG_PWM_TARGET_1 + i, val);
  
      fan_ctrl[i].temp_peak = fan_ctrl[i].temp;
    }
  }
  
  return;
}
