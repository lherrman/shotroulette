#include <Adafruit_NeoPixel.h>
#include <NSEncoder.h>

#include <avr/power.h>

#define LEDPIN 3
#define NUMPIXELS 16

#define BTN_OUT_0 8
#define BTN_OUT_1 9
#define BTN_OUT_2 10
#define BTN_OUT_3 11
#define BTN_IN_0 4
#define BTN_IN_1 5
#define BTN_IN_2 6
#define BTN_IN_3 7

#define BTN_USER_LEFT 11
#define BTN_USER_RIGHT 10

#define ENCODER_S1_PIN 11
#define ENCODER_S2_PIN 10

NSEncoder enc(ENCODER_S1_PIN, ENCODER_S2_PIN, 2);

#define BTN_USER_SELECT A2

#define MAXPLAYERS 4

#define DEBUG 0

Adafruit_NeoPixel pixels =
    Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

bool buttons[16] = {0};  // buttons for detecting shotglasses
int user_inputs[3] = {0, 0, 0};  // current reqading from user input buttons
int user_inputs_FP[3] = {0, 0, 0};  
int last_user_inputs[3] = {0, 0, 0}; // used for debouncing button inputs

unsigned long user_inputs_last_debounce[3] = {};
unsigned long debounce_delay = 1; //debounce time

int last_diff = 0;

int current_state = -1;
int next_state = 0;
double led_colors[16][3] = {0.};
double pixel_buffer_rgb[NUMPIXELS][3] = {0.};
double pixel_buffer_hsv[NUMPIXELS][3] = {0.};

double background_keycolor[3] = {0., 0.1, 1.} ;

int t_anim = 0;  // animation frames since current state enter

double player_color[MAXPLAYERS][3] = {0.};
double player_pos[MAXPLAYERS] = {0.};  // players positions 0 - 360deg

unsigned long time;
unsigned long time_last_frame;
unsigned long time_state_enter;
int animation_delta_ms = 30;
int dt = 0;

void setup() {
  Serial.begin(9600);
  pixels.begin();
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  setPinModes();
}

void loop() {
  time = millis();
  buttons_update();
  statemachine();
  animation_update();
  
  int diff = 0;
  if((diff = enc.get_diffPosition()) != 0) //If value is updated
  {
    player_pos[0] -= diff * 22.5;
    last_diff = diff;
  }

}

void statemachine() {
  switch (next_state) {
    // STATE: bootup
    case 0:
      if (current_state != next_state) {  // state entry
        current_state = next_state;
        Serial.println("bootup state (0) entered");
        time_state_enter = 0;
        t_anim = 0;
        set_player_color(0, 1., 0, 0);
        set_player_color(1, 0, 1., 0);
        set_player_color(2, 1, 1., 0);
        set_player_color(3, 1., 0, 1.);
      }
      // cyclic code:

      // change state after given time (bootup animation) 
      if (time - time_state_enter > 1000* 3) {
        next_state = 2;
      }

      if (current_state != next_state) {  // state exit
      }
      break;

    // STATE filling
    case 1:
      if (current_state != next_state) {  // state entry
        current_state = next_state;
        Serial.println("filling state (1) entered");
        t_anim = 0;
      }
      // cyclic code:

      if (current_state != next_state) {  // state exit
      }
      break;

    // STATE ready
    case 2:
      if (current_state != next_state) {  // state entry
        current_state = next_state;
        Serial.println("ready state (2) entered");
        t_anim = 0;
      }
      // cyclic code:

      if (current_state != next_state) {  // state exit
      }
      break;
  }
  return;
}

void animation_update() {
  dt = time - time_last_frame;
  if (dt > animation_delta_ms) {
    t_anim += 1;
    switch (current_state) {
      case 0:
        animation_state_0();
        break;

      case 2:
        animation_state_2();
        break; 
    }
    
    update_pixel();
    
#if DEBUG == 1
    print_debug_info();
#endif
    
    time_last_frame = time;
  }
}



void animation_state_0() {
  int active_pos = t_anim % NUMPIXELS;
  int n_rotation = floor(t_anim / NUMPIXELS);

  for (int n = 0; n < NUMPIXELS; n++) {
    double* h_ptr = &pixel_buffer_hsv[n][0];
    double* s_ptr = &pixel_buffer_hsv[n][1];
    double* v_ptr = &pixel_buffer_hsv[n][2];
    *h_ptr = fmod((n * 360. / NUMPIXELS) + (t_anim * 2.), 360.);
    *s_ptr = 1.;
    *v_ptr = 1.;
  }

  hsv2rgb_buffer();

  for (int n = 0; n < NUMPIXELS; n++) {
    led_colors[n][0] = pixel_buffer_rgb[n][0];
    led_colors[n][1] = pixel_buffer_rgb[n][1];
    led_colors[n][2] = pixel_buffer_rgb[n][2];

    
  }
}

void animation_state_2() {
  
  // check for flank at user input wheel for adjusting position
  if (user_inputs_FP[0] == HIGH) {
   player_pos[0] += 22.5 ;
  }
  if (user_inputs_FP[1] == HIGH) {
   player_pos[0] -= 22.5 ;
    if (player_pos[0] < 0) {
        double pp = 360. + player_pos[0];
      player_pos[0] = pp;
    }
  }
  player_pos[0] = fmod(player_pos[0], 360.);  
  
  // Draw Background
  for (int n = 0; n < NUMPIXELS; n++) {
    led_colors[n][0] = background_keycolor[0];
    led_colors[n][1] = background_keycolor[1];
    led_colors[n][2] = background_keycolor[2]; 
  }
  // mapping angle 0-360 to int led position 0-16
  int pos_p1 = int((player_pos[0] / 360.) * 16);
  
  // set led at users position to users color
  led_colors[pos_p1][0] = player_color[0][0];
  led_colors[pos_p1][1] = player_color[0][1];
  led_colors[pos_p1][2] = player_color[0][2];


  
}

void update_pixel() {
  // update WS2812 leds
  for (int i = 0; i < NUMPIXELS; i++) {
    int r = int(led_colors[i][0] * 255);
    int g = int(led_colors[i][1] * 255);
    int b = int(led_colors[i][2] * 255);
    pixels.setPixelColor(i, pixels.Color(r, b, g));
  }
  pixels.show();
}


void set_player_color(int player_id, uint8_t r, uint8_t g, uint8_t b) {
  player_color[player_id][0] = r;
  player_color[player_id][1] = g;
  player_color[player_id][2] = b;
}

void print_debug_info() {
  Serial.print("Animation Step: ");
  Serial.print(t_anim);
  Serial.print("  State: ");
  Serial.println(current_state);
}

void buttons_update() {
   int n_user_buttons = 3;
  
  // reset flank varaibles
  for (int bt=0; bt < n_user_buttons; bt++){
    user_inputs_FP[bt] = 0;
  }
  
  // read user input buttons 0:left 1:right 2:select
  bool reading_inp[3];
  
  reading_inp[0] = digitalRead(BTN_USER_LEFT);
  reading_inp[1] = digitalRead(BTN_USER_RIGHT);
  reading_inp[2] = digitalRead(BTN_USER_SELECT);
  
  for (int bt=0; bt < n_user_buttons; bt++){
    // debouncing
    if (reading_inp[bt] != last_user_inputs[bt]) {
      user_inputs_last_debounce[bt] = time;
    }

    if ((time - user_inputs_last_debounce[bt]) > debounce_delay) {
      if (reading_inp[bt] != user_inputs[bt]) {
        user_inputs[bt] = reading_inp[bt]; 
      }
    }  
    
    last_user_inputs[bt] = reading_inp[bt];
    
    // set flank positiv variable
    if (user_inputs[bt] == 1 && last_user_inputs[bt] == 0) {
      user_inputs_FP[bt] = 1;
    }
  }
  
  
  // scan over 4x4 button matrix to detect shotglas
  bool* btns = &buttons[0];

  for (int n = 0; n < 4; n++) {
    digitalWrite(BTN_OUT_0 + n, HIGH);
    delay(1);
    *btns = digitalRead(BTN_IN_0);
    *(btns + 1) = digitalRead(BTN_IN_1);
    *(btns + 2) = digitalRead(BTN_IN_2);
    *(btns + 3) = digitalRead(BTN_IN_3);
    btns += 4;
    digitalWrite(BTN_OUT_0 + n, LOW);
  }
  return;
}



void setPinModes() {
  pinMode(BTN_OUT_0, OUTPUT);
  pinMode(BTN_OUT_1, OUTPUT);
  pinMode(BTN_OUT_2, OUTPUT);
  pinMode(BTN_OUT_3, OUTPUT);

  pinMode(BTN_IN_0, INPUT);
  pinMode(BTN_IN_1, INPUT);
  pinMode(BTN_IN_2, INPUT);
  pinMode(BTN_IN_3, INPUT);

  pinMode(BTN_USER_LEFT, INPUT);
  pinMode(BTN_USER_RIGHT, INPUT);
  pinMode(BTN_USER_SELECT, INPUT);
}



void rgb2hsv_buffer() {
  double min, max, delta;

  double *out_h, *out_s, *out_v;
  double *in_r, *in_g, *in_b;

  for (int n = 0; n < NUMPIXELS; n++) {
    out_h = &pixel_buffer_hsv[n][0];
    out_s = out_h + 1;
    out_v = out_h + 2;

    in_r = &pixel_buffer_rgb[n][0];
    in_g = in_r + 1;
    in_b = in_r + 2;

    min = *in_r < *in_g ? *in_r : *in_g;
    min = min < *in_b ? min : *in_b;

    max = *in_r > *in_g ? *in_r : *in_g;
    max = max > *in_b ? max : *in_b;

    *out_v = max;  // v
    delta = max - min;
    if (delta < 0.00001) {
      *out_s = 0;
      *out_h = 0;  // undefined, maybe nan?
      continue;
    }
    if (max > 0.0) {  // NOTE: if Max is == 0, this divide would cause a crash
      *out_s = (delta / max);  // s
    } else {
      // if max is 0, then r = g = b = 0
      // s = 0, h is undefined
      *out_s = 0.0;
      *out_h = NAN;  // its now undefined
      continue;
    }
    if (*in_r >= max)  // > is bogus, just keeps compilor happy
      *out_h = (*in_g - *in_b) / delta;  // between yellow & magenta
    else if (*in_g >= max)
      *out_h = 2.0 + (*in_b - *in_r) / delta;  // between cyan & yellow
    else
      *out_h = 4.0 + (*in_r - *in_g) / delta;  // between magenta & cyan

    *out_h *= 60.0;  // degrees

    if (*out_h < 0.0) *out_h += 360.0;

    continue;
  }
  return;
}

void hsv2rgb_buffer() {
  double hh, p, q, t, ff;
  long i;
  double *in_h, *in_s, *in_v;
  double *out_r, *out_g, *out_b;

  for (int n = 0; n < NUMPIXELS; n++) {
    in_h = &pixel_buffer_hsv[n][0];
    in_s = in_h + 1;
    in_v = in_h + 2;

    out_r = &pixel_buffer_rgb[n][0];
    out_g = out_r + 1;
    out_b = out_r + 2;

    if (*in_s <= 0.0) {  // < is bogus, just shuts up warnings
      *out_r = *in_v;
      *out_g = *in_v;
      *out_b = *in_v;
      continue;
    }
    hh = *in_h;
    if (hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = *in_v * (1.0 - *in_s);
    q = *in_v * (1.0 - (*in_s * ff));
    t = *in_v * (1.0 - (*in_s * (1.0 - ff)));

    switch (i) {
      case 0:
        *out_r = *in_v;
        *out_g = t;
        *out_b = p;
        break;
      case 1:
        *out_r = q;
        *out_g = *in_v;
        *out_b = p;
        break;
      case 2:
        *out_r = p;
        *out_g = *in_v;
        *out_b = t;
        break;

      case 3:
        *out_r = p;
        *out_g = q;
        *out_b = *in_v;
        break;
      case 4:
        *out_r = t;
        *out_g = p;
        *out_b = *in_v;
        break;
      case 5:
      default:
        *out_r = *in_v;
        *out_g = p;
        *out_b = q;
        break;
    }
    continue;
  }
  return;
}
