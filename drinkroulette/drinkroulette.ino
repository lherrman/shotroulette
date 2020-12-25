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

#define NUMUSERBUTTONS 1

#define ENCODER_S1_PIN 11
#define ENCODER_S2_PIN 10

NSEncoder enc(ENCODER_S1_PIN, ENCODER_S2_PIN, 2);

#define BTN_USER_SELECT A2

#define MAXPLAYERS 4

#define DEBUG 0

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

typedef struct {
  double r;
  double g;
  double b;
} rgb;

typedef struct {
  double h;
  double s;
  double v;
} hsv;

static hsv rgb2hsv(rgb in);
static rgb hsv2rgb(hsv in);

bool buttons[16] = { 0 }; // buttons for detecting shotglasses
int user_inputs[NUMUSERBUTTONS] = { 0 }; // current reading from user input buttons 0: select
int user_inputs_FP[NUMUSERBUTTONS] = { 0 };
int last_user_inputs[NUMUSERBUTTONS] = { 0 }; // used for debouncing button inputs

int rot_encoder_diff = 0;
unsigned long rot_encoder_last_time[2] = { 0, 0 }; // 0: left   1:right
unsigned long rot_encoder_last_interval[2] = { 1000, 1000 }; // 0: left   1:right

int spinning_direction = 0;

unsigned long user_inputs_last_debounce[NUMUSERBUTTONS] = {};
unsigned long debounce_delay = 1; //debounce time

int current_state = -1;
int next_state = 0;

//double led_colors[16][3] = {0.};
rgb pixel_output[NUMPIXELS] = { 0. };
rgb pixel_buffer_rgb_0[NUMPIXELS] = { 0. };
rgb pixel_buffer_rgb_1[NUMPIXELS] = { 0. };
hsv pixel_buffer_hsv[NUMPIXELS] = { 0. };

int t_anim = 0; // animation frames since current state enter
int t_bg = 0; // animation frames since bg reset

rgb player_color[MAXPLAYERS] = { 0. };
double player_pos[MAXPLAYERS] = { 0. }; // players positions 0 - 360deg

unsigned long time;
unsigned long time_last_frame;
unsigned long time_state_enter;
int animation_delta_ms = 10;
int dt = 0;

void setup()
{
  Serial.begin(9600);
  pixels.begin();

  // set +5v pin for inc sensor
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  setPinModes();
}

void loop()
{
  time = millis();
  buttons_update();
  statemachine();
  animation_update();

}

void statemachine()
{
  switch (next_state) {
    // STATE: bootup
    case 0:
      if (current_state != next_state) { // state entry
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
      if (time - time_state_enter > 1000 * 1) {
        next_state = 101;
      }

      if (current_state != next_state) { // state exit
      }
      break;

    // STATE filling
    case 1:
      if (current_state != next_state) { // state entry
        current_state = next_state;
        Serial.println("filling state (1) entered");
        t_anim = 0;

        next_state = 4;
      }
      // cyclic code:

      if (current_state != next_state) { // state exit
      }
      break;

    // STATE ready
    case 2:
      if (current_state != next_state) { // state entry
        current_state = next_state;
        Serial.println("ready state (2) entered");
        t_anim = 0;
        rot_encoder_last_interval[0] = 1000;
        rot_encoder_last_interval[1] = 1000;
      }
      // cyclic code:
      // update player_position
      player_pos[0] = wrap360((rot_encoder_diff * 22.5) + player_pos[0]);

      //check if rotary encoder was spinned fast
      if (rot_encoder_last_interval[0] < 25) {
        spinning_direction = -1;
        next_state = 3;
      }
      else if (rot_encoder_last_interval[1] < 25) {
        spinning_direction = 1;
        next_state = 3;
      }

      if (current_state != next_state) { // state exit
      }
      break;

    // STATE spinning
    case 3:
      if (current_state != next_state) { // state entry
        current_state = next_state;
        Serial.println("spinning state (3) entered");
        t_anim = 0;
      }
      // cyclic code:

      if (current_state != next_state) { // state exit
      }
      break;
    // STATE take_shot
    case 4:
      if (current_state != next_state) { // state entry
        current_state = next_state;
        Serial.println("take_shot state (4) entered");
        t_anim = 0;
      }
      // cyclic code:
      player_pos[0] = wrap360((rot_encoder_diff * 22.5) + player_pos[0]);

      if (rot_encoder_diff) {
        next_state = 2;
      }
      
      if (current_state != next_state) { // state exit
      }
      break;

    // STATE: Transition 1 state 0 to 1
    case 101:
      if (current_state != next_state) { // state entry
        current_state = next_state;
        t_anim = 0;
      }
      // cyclic code:

      if (current_state != next_state) { // state exit
      }
      break;
    // STATE: Transition 2 state 3 to 4
    case 104:
      if (current_state != next_state) { // state entry
        current_state = next_state;
        t_anim = 0;
      }
      // cyclic code:
      
      if (current_state != next_state) { // state exit
      }
      break;      
  }
  return;
}

void animation_update()
{
  dt = time - time_last_frame;
  if (dt > animation_delta_ms) {
    t_anim += 1;
    t_bg += 1;
    switch (current_state) {
      case 0:
        animation_state_0();
        break;
      case 2:
        animation_state_2();
        break;
      case 3:
        animation_state_3();
        break;
      case 4:
        animation_state_4();
        break;
      case 101:
        transition_state_101();
        break;
      case 104:
        transition_state_104();
       break;  
    }

    update_pixel();

#if DEBUG == 1
    print_debug_info();
#endif

    time_last_frame = time;
  }
}

void transition_state_101()
{
  get_default_background(220., 1., 1.);
  int t_trans = t_anim / 2;
  for (int n = 0; n < t_trans; n++) {
    pixel_output[n] = pixel_buffer_rgb_0[n];
  }
  if (t_trans >= NUMPIXELS) {
    next_state = 2;
  }
}

void transition_state_104()
{
  get_default_background(220., 1., 1.);

  int fade_frames = 50;
  
  for (int n = 0; n < NUMPIXELS; n++) {   
    pixel_output[n].r = pixel_output[n].r * 0.80;
    pixel_output[n].g = pixel_output[n].g * 0.80;
    pixel_output[n].b = pixel_output[n].b * 0.80;
  }
  Serial.println(t_anim);
  int pos_p1 = int((player_pos[0] / 360.) * 16);
  pixel_output[pos_p1] = player_color[0];
  
  if (t_anim >= 5) {
    next_state = 4;
  }
}

void animation_state_0()
{ // animation for bootup
  int active_pos = t_anim % NUMPIXELS;

  for (int n = 0; n < NUMPIXELS; n++) {
    hsv in = { 0., 1., 1. };
    in.h = wrap360((n * 360. / NUMPIXELS) + (t_anim * 2.));
    pixel_output[n] = hsv2rgb(in);
  }
}

void animation_state_2()
{ // animation for ready state
  get_default_background(220., 1., 1.);
  buffer_to_output();

  // mapping angle 0-360 to int led position 0-16
  int pos_p1 = int((player_pos[0] / 360.) * 16);
  pixel_output[pos_p1] = player_color[0];
}

void animation_state_3()
{ // animation for spinning state
  get_default_background(220., 1., 1.);
  buffer_to_output();
  double falloff = 1 - (t_anim / 400.);
  player_pos[0] = wrap360((-spinning_direction * 22.5 * falloff) + player_pos[0]);
  int pos_p1 = int((player_pos[0] / 360.) * 16);
  pixel_output[pos_p1] = player_color[0];

  if (t_anim > 400) {
    next_state = 104;
  }
}

void animation_state_4()
{ //animation for take_shot
  double winning_pos = player_pos[0];
  double opp_pos = wrap360(winning_pos + 180.);

  int winning_pos_int = int((winning_pos / 360.) * 16);
  int opp_pos_int = int((opp_pos / 360.) * 16);

  for (int n = 0; n < NUMPIXELS; n++) {
    hsv in0;
    hsv in1;
    
    in0.h = wrap360(in0.h + (12 * sin((n * (6.2 / NUMPIXELS)) + (0.03 * t_bg))));
    in0.s = 0.8;
    in0.v = 0.5 * (1 + sin((n * 0.4) + (- (t_anim + 10) * -0.09) + (PI / 2) ));

    in1.h =  wrap360(in0.h + (12 * sin((n * (6.2 / NUMPIXELS)) + (0.03 * t_bg))));
    in1.s = 0.8;
    in1.v = 0.5 * (1 + sin((-n * 0.4) + (-(t_anim + 10) * -0.09) + (PI / 2)));

    if (t_anim < 50) {
      Serial.println(t_anim);
      in0.v *= 0.5 * (1 + sin(((PI/100) * t_anim) - (PI/4) ));
      in1.v *= 0.5 * (1 + sin(((PI/100) * t_anim) - (PI/4) ));
    }
    pixel_buffer_rgb_0[n] = hsv2rgb(in0);
    pixel_buffer_rgb_1[n] = hsv2rgb(in1);
  }

  for (int n = 0; n < NUMPIXELS / 2; n++) {
    pixel_output[n] = pixel_buffer_rgb_0[n];
  }
  for (int n = NUMPIXELS / 2; n < NUMPIXELS; n++) {
    pixel_output[n] = pixel_buffer_rgb_1[n];
  }

  // rotate output buffer
  for (int n = 0; n < winning_pos_int; n++) {
    rgb temp0 = pixel_output[15];
    for (int i = 14; i >= 0; --i) {
      pixel_output[i + 1] = pixel_output[i];
    }
    pixel_output[0] = temp0;
  }

  double multiplier = 0.5 * (1 + sin((0.09 * t_anim) + (PI/4)));
  pixel_output[winning_pos_int].r = multiplier * player_color[0].r;
  pixel_output[winning_pos_int].g = multiplier * player_color[0].g;
  pixel_output[winning_pos_int].b = multiplier * player_color[0].b;
}

void get_default_background(double h, double s, double v)
{
  // Draw Background
  hsv bg_hsv = { h, s, v };
  for (int n = 0; n < NUMPIXELS; n++) {
    hsv in = bg_hsv;
    in.h = wrap360(in.h + (12 * sin((n * (6.2 / NUMPIXELS)) + (0.03 * t_bg))));
    pixel_buffer_rgb_0[n] = hsv2rgb(in);
  }
}

void update_pixel()
{
  // update WS2812 leds
  for (int i = 0; i < NUMPIXELS; i++) {
    int r = int(pixel_output[i].r * 255);
    int g = int(pixel_output[i].g * 255);
    int b = int(pixel_output[i].b * 255);
    pixels.setPixelColor(i, pixels.Color(r, b, g));
  }
  pixels.show();
}

void buffer_to_output()
{
  for (int n = 0; n < NUMPIXELS; n++) {
    pixel_output[n] = pixel_buffer_rgb_0[n];
  }
}

void set_player_color(int player_id, uint8_t r, uint8_t g, uint8_t b)
{
  player_color[player_id].r = r;
  player_color[player_id].g = g;
  player_color[player_id].b = b;
}

void print_debug_info()
{
  Serial.print("Animation Step: ");
  Serial.print(t_anim);
  Serial.print("  State: ");
  Serial.println(current_state);
}

void buttons_update()
{
  // reset flank varaibles
  for (int bt = 0; bt < NUMUSERBUTTONS; bt++) {
    user_inputs_FP[bt] = 0;
  }

  // reading rotary encoder differential
  if ((rot_encoder_diff = enc.get_diffPosition()) != 0) { //If value is updated

    if (rot_encoder_diff > 0) {
      rot_encoder_last_interval[0] = time - rot_encoder_last_time[0];
      rot_encoder_last_time[0] = time;
    }
    else {
      rot_encoder_last_interval[1] = time - rot_encoder_last_time[1];
      rot_encoder_last_time[1] = time;
    }
  }
  // read user input buttons 0: select
  bool reading_inp[NUMUSERBUTTONS];
  reading_inp[0] = digitalRead(BTN_USER_SELECT);

  for (int bt = 0; bt < NUMUSERBUTTONS; bt++) {
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

double wrap360(double in)
{

  if (in < 360. && in >= 0) {
    return in;
  }
  else if (in >= 360.) {
    double out = fmod(in, 360.);
    return out;
  }
  else {
    double out = fmod(-in, 360.);
    ;
    out = 360. - out;
    return out;
  }
}

int wrap16(int in)
{
  if (in < 16 && in >= 0) {
    return in;
  }
  else if (in >= 16.) {
    return in - 16.;
  }
  else {
    return in + 16.;
  }
}

void setPinModes()
{
  /*
    pinMode(BTN_OUT_0, OUTPUT);
    pinMode(BTN_OUT_1, OUTPUT);
    pinMode(BTN_OUT_2, OUTPUT);
    pinMode(BTN_OUT_3, OUTPUT);

    pinMode(BTN_IN_0, INPUT);
    pinMode(BTN_IN_1, INPUT);
    pinMode(BTN_IN_2, INPUT);
    pinMode(BTN_IN_3, INPUT);
  */
  pinMode(ENCODER_S1_PIN, INPUT);
  pinMode(ENCODER_S2_PIN, INPUT);
  pinMode(BTN_USER_SELECT, INPUT);
}

hsv rgb2hsv(rgb in)
{
  hsv out;
  double min, max, delta;

  min = in.r < in.g ? in.r : in.g;
  min = min < in.b ? min : in.b;

  max = in.r > in.g ? in.r : in.g;
  max = max > in.b ? max : in.b;

  out.v = max; // v
  delta = max - min;
  if (delta < 0.00001) {
    out.s = 0;
    out.h = 0; // undefined, maybe nan?
    return out;
  }
  if (max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
    out.s = (delta / max); // s
  }
  else {
    // if max is 0, then r = g = b = 0
    // s = 0, h is undefined
    out.s = 0.0;
    out.h = NAN; // its now undefined
    return out;
  }
  if (in.r >= max) // > is bogus, just keeps compilor happy
    out.h = (in.g - in.b) / delta; // between yellow & magenta
  else if (in.g >= max)
    out.h = 2.0 + (in.b - in.r) / delta; // between cyan & yellow
  else
    out.h = 4.0 + (in.r - in.g) / delta; // between magenta & cyan

  out.h *= 60.0; // degrees

  if (out.h < 0.0)
    out.h += 360.0;

  return out;
}

rgb hsv2rgb(hsv in)
{
  double hh, p, q, t, ff;
  long i;
  rgb out;

  if (in.s <= 0.0) { // < is bogus, just shuts up warnings
    out.r = in.v;
    out.g = in.v;
    out.b = in.v;
    return out;
  }
  hh = in.h;
  if (hh >= 360.0)
    hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = in.v * (1.0 - in.s);
  q = in.v * (1.0 - (in.s * ff));
  t = in.v * (1.0 - (in.s * (1.0 - ff)));

  switch (i) {
    case 0:
      out.r = in.v;
      out.g = t;
      out.b = p;
      break;
    case 1:
      out.r = q;
      out.g = in.v;
      out.b = p;
      break;
    case 2:
      out.r = p;
      out.g = in.v;
      out.b = t;
      break;

    case 3:
      out.r = p;
      out.g = q;
      out.b = in.v;
      break;
    case 4:
      out.r = t;
      out.g = p;
      out.b = in.v;
      break;
    case 5:
    default:
      out.r = in.v;
      out.g = p;
      out.b = q;
      break;
  }
  return out;
}
