#ifdef ARDUINO_AVR_MICRO
  // Required for Arduino Micro to work with rosserial
  #define USE_USBCON
#endif

#include <ros.h>
#include <neostate_msgs/StatusLEDArray.h>

#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN     6

Adafruit_NeoPixel strip(12, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

// Array indicating which leds should blink (if not blinking set to zero as off led cannot blink)
uint32_t blinking_led_colors[12] = { 0 };

// Time led stays on & off
const uint32_t on_time = 100;
const uint32_t off_time = 100;

/* ----- ROS ----- */
void messageCb( const neostate_msgs::StatusLEDArray &msg){
  // reset all led values (also blinking ones)
  strip.clear();
  memset(blinking_led_colors, 0, sizeof(blinking_led_colors));

  // loop through message and change led values one by one
  for (int i = 0; i < msg.LED_array_length && i < strip.numPixels(); i++){
    uint32_t new_color = strip.Color(msg.LED_array[i].red, msg.LED_array[i].green, msg.LED_array[i].blue);
    strip.setPixelColor(i, new_color);
    // define if led should be blinking
    if(msg.LED_array[i].blinking){
      blinking_led_colors[i] = new_color;
    } else {
      blinking_led_colors[i] = 0;
    }
  }
  // Show changes on physical leds
  strip.show();
}

ros::NodeHandle  nh;
ros::Subscriber<neostate_msgs::StatusLEDArray> sub("set_status_led_array", messageCb );

void setup()
{

  /* ----- ROS ----- */
  nh.initNode();
  nh.subscribe(sub);

  /* ----- Initialize NeoPixel strip ----- */
  /* Blinking red on all 8 leds */
  strip.begin();
  for (int i=0; i < strip.numPixels(); ++i){
    blinking_led_colors[i] = strip.Color(26, 0,0);
  }
  
}

void loop()
{ 
  /* Check if it is time to change status (on/off) */
  set_blinking_leds();
  
  nh.spinOnce();
}


void set_blinking_leds(){
  static uint32_t _ms = millis();
  static bool     _on = true;

  // check if enough time has passed
  if(millis() - _ms < (_on ? on_time : off_time))
    return; // not the time to switch yet
  
  _ms = millis();  // reset timer

  // Change value of every pixel which is set to blinking
  for(int i = 0; i < strip.numPixels(); i++){
      if(blinking_led_colors[i] != 0){
       if(!_on){
         strip.setPixelColor(i, blinking_led_colors[i]);
       } else {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
       }
     }
   }
   // Invert status indicater
   _on = !_on;
   
   strip.show();
} 
