// vim: foldmethod=marker
// includes {{{1
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/bootrom.h"
#include "ws2812.pio.h"

#include "tusb.h"
#include "usb_descriptors.h"

// configuration {{{1

// analog values go from 0 to 9. The change needed to detect a key press
#define SENSITIVITY 6
// time between mouse events when a key is pressed
#define MOUSE_PERIOD_MS 30u
// time between pressing a key and it being considered held (not tapped)
#define HOLD_DELAY_MS 333u
// time to wait for second key press to lock a layer
#define LOCK_DELAY_MS 200u
// ignore changes in digital key during this time to debounce it
#define DEBOUNCING_DELAY_MS 20u
// period to send kb status to other side
#define COMM_STATUS_DELAY_MS 20u

#define BAUD_RATE 500000

#define N_SEL_PINS 5
#define N_ANA_PINS 4
#define N_ANALOG_HWKKEYS (N_SEL_PINS * N_ANA_PINS)
#define N_DIGITAL_HWKKEYS 32
#define N_KEYS 36

uint8_t left_sel_pins[N_SEL_PINS] = { 14, 15, 3, 1, 0 };
uint8_t right_sel_pins[N_SEL_PINS] = { 0, 1, 3, 6, 7 };
uint8_t ana_pins[N_ANA_PINS] = { 26, 27, 28, 29 };

int8_t leftAnalogHwIdToSwId[N_ANALOG_HWKKEYS] = {
  17, 14,  9,  4, 16, 13,  8,  3, 15, 12,
   7,  2, -1, 11,  6,  1, -1, 10,  5,  0,
};
int8_t rightAnalogHwIdToSwId[N_ANALOG_HWKKEYS] = {
  -1, 32, 27, 22, -1, 31, 26, 21, 34, 30,
  25, 20, 35, 29, 24, 19, 33, 28, 23, 18,
};
int8_t rightDigitalHwIdToSwId[N_DIGITAL_HWKKEYS] = {
  -1, -1, 18, 20, 19, 25, 21, 26, 23, 24, 30, 29, 31, 28, 22, 27,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 32, 35, 34, 33, -1, -1,
};
int8_t leftDigitalHwIdToSwId[N_DIGITAL_HWKKEYS] = {
  -1, -1, 10, 13, 17, 16,  0,  6, 15, 14,  5, 11,  8, 12,  9,  7,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  3,  1,  2,  4, -1, -1,
};

// status {{{1
typedef enum { noSide, leftSide, rightSide } keyboardSide;
typedef enum { analog, digital } keyboardType;
struct {
  keyboardSide mySide;
  bool usbReady;
  bool usbActive;
  bool toggleUsb;
  keyboardSide otherSide;
  bool otherSideUsbReady;
  bool otherSideUsbActive;
  bool otherSideToggleUsb;
  uint32_t now;
  bool commOK;
  uint32_t lastActiveTimestamp;
} status;

void update_now() {
  status.now = time_us_32();
  if (status.now == 0) status.now = 1;
}

void setTimestamp(uint32_t *timestamp) {
  *timestamp = status.now;
}

static inline bool elapsed_µs(uint32_t *timestamp, uint32_t delay_µs) {
  if (*timestamp == 0) return false;
  if ((status.now - *timestamp) < delay_µs) return false;
  *timestamp = 0;
  return true;
}
static inline bool elapsed_ms(uint32_t *timestamp, uint32_t delay_ms) {
  return elapsed_µs(timestamp, delay_ms * 1000);
}

// timer {{{1
typedef struct {
  uint32_t timestamp;
  uint32_t delay;
  bool enabled;
} Timer;

static void timer_enable_µs(Timer *t, uint32_t delay_µs)
{
  t->timestamp = status.now;
  t->delay = delay_µs;
  t->enabled = true;
}

static void timer_enable_ms(Timer *t, uint32_t delay_ms)
{
  timer_enable_µs(t, delay_ms * 1000);
}

void timer_disable(Timer *t)
{
  t->enabled = false;
}

bool timer_is_enabled(Timer *t)
{
  return t->enabled;
}

bool timer_elapsed(Timer *t)
{
  return t->enabled && (status.now - t->timestamp) > t->delay;
}

// types {{{1
typedef enum {
  COLEMAK,
  ACC,
  QWERTY,
  QWE_ACC,
  NAV,
  RAT,
  SYM,
  NUM,
  FUN,
  NUM2,
  NO_LAYER,
} layer_id_t;

typedef enum {
  CTRL  = 0b00000001,
  SHFT  = 0b00000010,
  ALT   = 0b00000100,
  GUI   = 0b00001000,
  RCTRL = 0b00010000,
  RSHFT = 0b00100000,
  RALT  = 0b01000000,
  RGUI  = 0b10000000,
} modifier_t;

typedef enum {
  K_NONE    = 0x00, K_ERR1,    K_ERR2,    K_ERR3,
  K_A       = 0x04, K_B,       K_C,       K_D,
  K_E       = 0x08, K_F,       K_G,       K_H,
  K_I       = 0x0C, K_J,       K_K,       K_L,
  K_M       = 0x10, K_N,       K_O,       K_P,
  K_Q       = 0x14, K_R,       K_S,       K_T,
  K_U       = 0x18, K_V,       K_W,       K_X,
  K_Y       = 0x1C, K_Z,       K_1,       K_2,
  K_3       = 0x20, K_4,       K_5,       K_6,
  K_7       = 0x24, K_8,       K_9,       K_0,
  K_ENT     = 0x28, K_ESC,     K_BS,      K_TAB,
  K_SPC     = 0x2C, K_MINUS,   K_EQUAL,   K_LBRAKT,
  K_RBRAKT  = 0x30, K_BKSLASH, K_SHARP,   K_SMCOL,
  K_APOSTR  = 0x34, K_GRAVE,   K_COMMA,   K_DOT,
  K_SLASH   = 0x38, K_CAPS,    K_F1,      K_F2,
  K_F3      = 0x3C, K_F4,      K_F5,      K_F6,
  K_F7      = 0x40, K_F8,      K_F9,      K_F10,
  K_F11     = 0x44, K_F12,     K_PRTSC,   K_SCRLK,
  K_PAUSE   = 0x48, K_INSERT,  K_HOME,    K_PGUP,
  K_DEL     = 0x4C, K_END,     K_PGDN,    K_RIGHT,
  K_LEFT    = 0x50, K_DOWN,    K_UP,      K_NUMLK,
  // keypad
  K_EURO2   = 0x64, K_APP,     K_POWER,
  // keypad, F13-24
  K_STOP    = 0x78, K_REDO,    K_UNDO,    K_CUT,
  K_COPY    = 0x7C, K_PASTE,   K_FIND,    K_MUTE,
  K_VOLUP   = 0x80, K_VOLDOWN,
  // modifiers
  K_CTRL    = 0xE0, K_SHFT,    K_ALT,     K_GUI,
  K_RCTRL   = 0xE4, K_RSHFT,   K_RALT,    K_RGUI,
  // alias for compose key -- must configure this on OS
  K_COMPOSE = K_RGUI,
} keycode_t;

bool keycode_is_modifier(keycode_t keycode)
{
  return keycode >= K_CTRL && keycode <= K_RGUI;
}

modifier_t keycode_to_modifier(keycode_t keycode)
{
  return 1 << (keycode - 0xE0);
}

typedef enum {
  but_left     = 0b00001,
  but_right    = 0b00010,
  but_middle   = 0b00100,
  but_backward = 0b01000,
  but_forward  = 0b10000,
} button_t;

// ascii to mod-key {{{1
typedef struct {
  modifier_t mod;
  keycode_t key;
} mod_key;
const mod_key ascii_to_mod_key[] = {
  // table for US keyboard layout
  // must change if OS is configured for a different layout
#define S SHFT
  [0x00] = {0,0        }, {0,0       }, {0,0       }, {0,0       },
  [0x04] = {0,0        }, {0,0       }, {0,0       }, {0,0       },
  [0x08] = {0,K_BS     }, {0,K_TAB   }, {0,K_ENT   }, {0,0       },
  [0x0C] = {0,0        }, {0,0       }, {0,0       }, {0,0       },
  [0x10] = {0,0        }, {0,0       }, {0,0       }, {0,0       },
  [0x14] = {0,0        }, {0,0       }, {0,0       }, {0,0       },
  [0x18] = {0,0        }, {0,0       }, {0,0       }, {0,K_ESC   },
  [0x1C] = {0,0        }, {0,0       }, {0,0       }, {0,0       },
  [0x20] = {0,K_SPC    }, {S,K_1     }, {S,K_APOSTR}, {S,K_3     }, //  !"#
  [0x24] = {S,K_4      }, {S,K_5     }, {S,K_7     }, {0,K_APOSTR}, // $%&'
  [0x28] = {S,K_9      }, {S,K_0     }, {S,K_8     }, {S,K_EQUAL }, // ()*+
  [0x2C] = {0,K_COMMA  }, {0,K_MINUS }, {0,K_DOT   }, {0,K_SLASH }, // ,-./
  [0x30] = {0,K_0      }, {0,K_1     }, {0,K_2     }, {0,K_3     }, // 0123
  [0x34] = {0,K_4      }, {0,K_5     }, {0,K_6     }, {0,K_7     }, // 4567
  [0x38] = {0,K_8      }, {0,K_9     }, {S,K_SMCOL }, {0,K_SMCOL }, // 89:;
  [0x3C] = {S,K_COMMA  }, {0,K_EQUAL }, {S,K_DOT   }, {S,K_SLASH }, // <=>?
  [0x40] = {S,K_2      }, {S,K_A     }, {S,K_B     }, {S,K_C     }, // @ABC
  [0x44] = {S,K_D      }, {S,K_E     }, {S,K_F     }, {S,K_G     }, // DEFG
  [0x48] = {S,K_H      }, {S,K_I     }, {S,K_J     }, {S,K_K     }, // HIJK
  [0x4C] = {S,K_L      }, {S,K_M     }, {S,K_N     }, {S,K_O     }, // LMNO
  [0x50] = {S,K_P      }, {S,K_Q     }, {S,K_R     }, {S,K_S     }, // PQRS
  [0x54] = {S,K_T      }, {S,K_U     }, {S,K_V     }, {S,K_W     }, // TUVW
  [0x58] = {S,K_X      }, {S,K_Y     }, {S,K_Z     }, {0,K_LBRAKT}, // XYZ[
  [0x5C] = {0,K_BKSLASH}, {0,K_RBRAKT}, {S,K_6     }, {S,K_MINUS }, // \]^_
  [0x60] = {0,K_GRAVE  }, {0,K_A     }, {0,K_B     }, {0,K_C     }, // `abc
  [0x64] = {0,K_D      }, {0,K_E     }, {0,K_F     }, {0,K_G     }, // defg
  [0x68] = {0,K_H      }, {0,K_I     }, {0,K_J     }, {0,K_K     }, // hijk
  [0x6C] = {0,K_L      }, {0,K_M     }, {0,K_N     }, {0,K_O     }, // lmno
  [0x70] = {0,K_P      }, {0,K_Q     }, {0,K_R     }, {0,K_S     }, // pqrs
  [0x74] = {0,K_T      }, {0,K_U     }, {0,K_V     }, {0,K_W     }, // tuvw
  [0x78] = {0,K_X      }, {0,K_Y     }, {0,K_Z     }, {S,K_LBRAKT}, // xyz{
  [0x7C] = {S,K_BKSLASH}, {S,K_RBRAKT}, {S,K_GRAVE }, {0,K_DEL   }, // |}~
#undef S
};

// for a codepoint in unicode -- 0 to 0x10FFFF
typedef uint32_t unicode;

// interfaces {{{1
typedef struct usb USB;
typedef struct controller Controller;
typedef struct key Key;
typedef struct action Action;

void usb_init(USB *self);
void usb_task(USB *self);
void usb_setModifiers(USB *self, modifier_t new_modifiers);
void usb_pressKeycode(USB *self, keycode_t keycode);
void usb_releaseKeycode(USB *self, keycode_t keycode);
void usb_pressMouseButton(USB *self, button_t button);
void usb_releaseMouseButton(USB *self, button_t button);
void usb_moveMouse(USB *self, int8_t v, int8_t h, int8_t wv, int8_t wh);

void controller_init(Controller *self, USB *usb);
void controller_task(Controller *self);
void controller_changeBaseLayer(Controller *self, layer_id_t layer);
layer_id_t controller_baseLayer(Controller *self);
void controller_pressKeycode(Controller *self, keycode_t keycode);
void controller_releaseKeycode(Controller *self, keycode_t keycode);
void controller_pressString(Controller *self, char s[]);
char controller_pressAscii(Controller *self, char unshifted, char shifted);
void controller_releaseAscii(Controller *self, char c);
void controller_pressModifier(Controller *self, modifier_t modifier);
void controller_releaseModifier(Controller *self, modifier_t modifier);
void controller_changeLayer(Controller *self, layer_id_t layer);
void controller_lockLayer(Controller *self, layer_id_t layer);
void controller_pressMouseButton(Controller *self, button_t button);
void controller_releaseMouseButton(Controller *self, button_t button);
void controller_moveMouse(Controller *self, int v, int h, int wv, int wh);
void controller_setDelayedReleaseAction(Controller *self, Action action);
void controller_doCommand(Controller *self, int command);
void controller_keyPressed(Controller *self, Key *key);
void controller_keyReleased(Controller *self, Key *key);


Key *Key_keyWithId(uint8_t keyId);
void key_init(Key *self, Controller *controller, uint8_t keyId);
int8_t key_id(Key *self);
keyboardSide key_side(Key *self);
void key_setNewAnalogRaw(Key *self, uint16_t newRaw);
void key_setVal(Key *self, uint8_t newVal);
int8_t key_val(Key *self);
void key_processChanges(Key *self);
void key_setReleaseAction(Key *self, Action action);
Action *key_releaseAction(Key *self);
char *key_description(Key *self);
void key_setMinRawRange(Key *self, uint16_t range);

enum holdType { noHoldType, modHoldType, layerHoldType };
Action Action_noAction(void);
char *action_description(Action *a);
void action_actuate(Action *self, Key *key, Controller *controller);
bool action_isTypingAction(Action *self);
bool action_isMouseMovementAction(Action *self);
Action action_holdAction(Action *self);
Action action_tapAction(Action *self);
enum holdType action_holdType(Action *self);


// log {{{1

#define LOG_E 0b00000001
#define LOG_I 0b00000010
#define LOG_R 0b00000100
#define LOG_U 0b00001000
#define LOG_C 0b00010000
#define LOG_T 0b00100000
#define LOG_K 0b01000000
#define LOG_L 0b10000000

uint8_t log_level = LOG_L | LOG_R | LOG_C;

void log_set_level(uint8_t new_level)
{
  log_level = new_level;
}

#define log(level, ...) \
    if ((level) & (log_level)) { \
      printf(__VA_ARGS__); \
      putchar_raw('\n'); \
      fflush(stdout); \
      /*sleep_us(200);*/ \
      tud_task(); \
    }


// Action {{{1
// all things that can happen when a key is pressed or released
typedef enum {
  no_action,
  key_action,
  asc_action,
  str_action,
  mod_action,
  layer_action,
  base_layer_action,
  hold_layer_action,
  once_layer_action,
  lock_layer_action,
  key_or_mod_action,
  str_or_mod_action,
  key_or_layer_action,
  str_or_layer_action,
  mouse_move_action,
  mouse_button_action,
  command_action,

  rel_key_action,
  rel_asc_action,
  rel_mod_action,
  rel_layer_action,
  rel_once_layer_action,
  rel_button_action,
} action_type_t;

// names for those actions (for debug messages)
char *action_name[] = {
  [no_action]             = "no",
  // press actions
  [key_action]            = "key",
  [asc_action]            = "asc",
  [str_action]            = "str",
  [mod_action]            = "mod",
  [layer_action]          = "layer",
  [base_layer_action]     = "base_layer",
  [hold_layer_action]     = "hold_layer",
  [once_layer_action]     = "once_layer",
  [lock_layer_action]     = "lock_layer",
  [key_or_mod_action]     = "key_or_mod",
  [str_or_mod_action]     = "str_or_mod",
  [key_or_layer_action]   = "key_or_layer",
  [str_or_layer_action]   = "str_or_layer",
  [mouse_move_action]     = "mouse_move",
  [mouse_button_action]   = "mouse_button",
  [command_action]        = "command",
  // release actions
  [rel_key_action]        = "rel_key",
  [rel_asc_action]        = "rel_asc",
  [rel_mod_action]        = "rel_mod",
  [rel_layer_action]      = "rel_layer",
  [rel_once_layer_action] = "rel_once_layer",
  [rel_button_action]     = "rel_button",
};

// additional data for each action
typedef struct {
  keycode_t keycode;
} key_action_t;
typedef struct {
  char unshifted;
  char shifted;
} asc_action_t;
typedef struct {
  char pressed;
} rea_action_t;
typedef struct {
  char *str;
} str_action_t;
typedef struct {
  modifier_t modifier;
} mod_action_t;
typedef struct {
  layer_id_t layer_id;
} layer_action_t;
typedef struct {
  keycode_t keycode;
  modifier_t modifier;
} key_or_mod_action_t;
typedef struct {
  char *str;
  modifier_t modifier;
} str_or_mod_action_t;
typedef struct {
  keycode_t keycode;
  layer_id_t layer_id;
} key_or_layer_action_t;
typedef struct {
  char *str;
  layer_id_t layer_id;
} str_or_layer_action_t;
typedef struct {
  enum {
    mv_up,
    mv_down,
    mv_left,
    mv_right,
    wh_up,
    wh_down,
    wh_left,
    wh_right,
  } move;
} mouse_move_action_t;
typedef struct {
  button_t button;
} mouse_button_action_t;
typedef struct {
  enum { RESET, WORDLOCK, USB_SIDE } command;
} command_action_t;

struct action {
  action_type_t action_type;
  union {
    key_action_t key;
    asc_action_t asc;
    rea_action_t rea;
    str_action_t str;
    mod_action_t mod;
    layer_action_t layer;
    key_or_mod_action_t key_or_mod;
    str_or_mod_action_t str_or_mod;
    key_or_layer_action_t key_or_layer;
    str_or_layer_action_t str_or_layer;
    mouse_move_action_t mouse_move;
    mouse_button_action_t mouse_button;
    command_action_t command;
  };
};

// key actions
// do nothing
#define NO_ACTION  (Action){ no_action }
// send a keycode
#define KEY(k)     (Action){ key_action,          .key = k }
// send the keycode corresponding to ascii char (different if shifted)
#define ASC(u,s)   (Action){ asc_action,          .asc = { u, s } }
// send sequence of keycodes to type utf8 string
#define STR(s)     (Action){ str_action,          .str = { s } }
// send modifiers
#define MOD(m)     (Action){ mod_action,          .mod = m }
// tap=send keycode; hold=send modifiers
#define KOM(k,m)   (Action){ key_or_mod_action,   .key_or_mod = { k, m } }
// tap=send utf8 string; hold=send modifiers
#define SOM(s,m)   (Action){ str_or_mod_action,   .str_or_mod = { s, m } }
// tap=send keycode; hold=change layer
#define KOL(k,l)   (Action){ key_or_layer_action, .key_or_layer = { k, l } }
// tap=send utf8 string; hold=change layer
#define SOL(s,l)   (Action){ str_or_layer_action, .str_or_layer = { s, l } }
// change layer
#define LAY(l)     (Action){ layer_action,        .layer = { l } }
// change layer while held
#define LAH(l)     (Action){ hold_layer_action,   .layer = { l } }
// change layer for next key only
#define LA1(l)     (Action){ once_layer_action,   .layer = { l } }
// change layer and keep it changed
#define LCK(l)     (Action){ lock_layer_action,   .layer = { l } }
// change base layer
#define BAS(l)     (Action){ base_layer_action,   .layer = { l } }
// execute a command
#define COM(c)     (Action){ command_action,      .command = { c } }
// send a mouse movement
#define MOU(m)     (Action){ mouse_move_action,   .mouse_move = { m } }
// send a mouse button press
#define BUT(b)     (Action){ mouse_button_action, .mouse_button = { b } }
// auxiliary actions, associated to the release of a key
// release a keycode
#define REK(k)     (Action){ rel_key_action,      .key = k }
// release the keycode corresponding to the ascii char
#define REA(c)     (Action){ rel_asc_action,      .rea = c }
// release a modifier
#define REM(m)     (Action){ rel_mod_action,      .mod = m }
// release a layer (go back do base layer)
#define REL()      (Action){ rel_layer_action }
// release the "one key" layer (go back to the layer it was before)
#define REO()      (Action){ rel_once_layer_action }
// release mouse button
#define REB(b)     (Action){ rel_button_action,   .mouse_button = { b } }


char *action_description(Action *a)
{
  static char description[25];
  sprintf(description, "act%d:%.18s", a->action_type, action_name[a->action_type]);
  return description;
}

void no_actuate(Action *self, Key *key, Controller *controller) {
}
// actuate on key press
void key_actuate(Action *self, Key *key, Controller *controller) {
  controller_pressKeycode(controller, self->key.keycode);
  key_setReleaseAction(key, REK(self->key.keycode));
}
void asc_actuate(Action *self, Key *key, Controller *controller) {
  char pressed = controller_pressAscii(controller, self->asc.unshifted, self->asc.shifted);
  key_setReleaseAction(key, REA(pressed));
}
void str_actuate(Action *self, Key *key, Controller *controller) {
  controller_pressString(controller, self->str.str);
  key_setReleaseAction(key, NO_ACTION);
}
void mod_actuate(Action *self, Key *key, Controller *controller) {
  controller_pressModifier(controller, self->mod.modifier);
  key_setReleaseAction(key, REM(self->mod.modifier));
}
void layer_actuate(Action *self, Key *key, Controller *controller) {
  controller_changeLayer(controller, self->layer.layer_id);
  key_setReleaseAction(key, NO_ACTION);
}
void base_layer_actuate(Action *self, Key *key, Controller *controller) {
  controller_changeBaseLayer(controller, self->layer.layer_id);
  key_setReleaseAction(key, NO_ACTION);
}
void hold_layer_actuate(Action *self, Key *key, Controller *controller) {
  controller_changeLayer(controller, self->layer.layer_id);
  key_setReleaseAction(key, REL());
}
void once_layer_actuate(Action *self, Key *key, Controller *controller) {
  controller_changeLayer(controller, self->layer.layer_id);
  key_setReleaseAction(key, REO());
}
void lock_layer_actuate(Action *self, Key *key, Controller *controller) {
  controller_lockLayer(controller, self->layer.layer_id);
  key_setReleaseAction(key, NO_ACTION);
}
// mouse
void mouse_move_actuate(Action *self, Key *key, Controller *controller) {
  int val = key_val(key);
  if (val == 0) return;
  int h = 0, v = 0, wh = 0, wv = 0;
  int move[] = { 0, 85, 170, 260, 360, 480, 640, 880, 1280, 2000 };
  int wheel[] = { 0, 11, 22, 34, 48, 66, 92, 134, 208, 346 };
  switch (self->mouse_move.move) {
    case mv_up   : v  = -move[val]; break;
    case mv_down : v  = +move[val]; break;
    case mv_right: h  = +move[val]; break;
    case mv_left : h  = -move[val]; break;
    case wh_up   : wv = +wheel[val]; break;
    case wh_down : wv = -wheel[val]; break;
    case wh_right: wh = +wheel[val]; break;
    case wh_left : wh = -wheel[val]; break;
  }
  controller_moveMouse(controller, v, h, wv, wh);
}
void mouse_button_actuate(Action *self, Key *key, Controller *controller) {
  controller_pressMouseButton(controller, self->mouse_button.button);
  key_setReleaseAction(key, REB(self->mouse_button.button));
}
void command_actuate(Action *self, Key *key, Controller *controller) {
  controller_doCommand(controller, self->command.command);
  key_setReleaseAction(key, NO_ACTION);
}

// actuate on key release
void rel_key_actuate(Action *self, Key *key, Controller *controller) {
  controller_releaseKeycode(controller, self->key.keycode);
}
void rel_asc_actuate(Action *self, Key *key, Controller *controller) {
  controller_releaseAscii(controller, self->rea.pressed);
}
void rel_mod_actuate(Action *self, Key *key, Controller *controller) {
  controller_releaseModifier(controller, self->mod.modifier);
}
void rel_layer_actuate(Action *self, Key *key, Controller *controller) {
  controller_changeLayer(controller, controller_baseLayer(controller));
}
void rel_once_layer_actuate(Action *self, Key *key, Controller *controller) {
  controller_setDelayedReleaseAction(controller, REL());
}
void rel_button_actuate(Action *self, Key *key, Controller *controller) {
  controller_releaseMouseButton(controller, self->mouse_button.button);
}

Action Action_noAction(void)
{
  return NO_ACTION;
}

#define ACTION_CASE(action)                                                    \
  case action##_action:                                                        \
    action##_actuate(self, key, controller);                                   \
    break
void action_actuate(Action *self, Key *key, Controller *controller)
{
  log(LOG_T, "actuate %s %s", key_description(key), action_description(self));
  switch (self->action_type) {
    ACTION_CASE(no);
    ACTION_CASE(key);
    ACTION_CASE(asc);
    ACTION_CASE(str);
    ACTION_CASE(mod);
    ACTION_CASE(layer);
    ACTION_CASE(base_layer);
    ACTION_CASE(hold_layer);
    ACTION_CASE(once_layer);
    ACTION_CASE(lock_layer);
    ACTION_CASE(mouse_move);
    ACTION_CASE(mouse_button);
    ACTION_CASE(command);
    ACTION_CASE(rel_key);
    ACTION_CASE(rel_asc);
    ACTION_CASE(rel_mod);
    ACTION_CASE(rel_layer);
    ACTION_CASE(rel_once_layer);
    ACTION_CASE(rel_button);
    default: log(LOG_E, "Do not know how to actuate action type %d.", self->action_type);
  }
}

enum holdType action_holdType(Action *self)
{
  switch (self->action_type) {
    case key_or_mod_action:
    case str_or_mod_action:
      return modHoldType;
    case key_or_layer_action:
    case str_or_layer_action:
      return layerHoldType;
    default:
      return noHoldType;
  }
}

bool action_isTypingAction(Action *self)
{
  switch (self->action_type) {
    case key_action:
    case asc_action:
    //case str_action:
      return true;
    default:
      return false;
  }
}

bool action_isMouseMovementAction(Action *self)
{
  return self->action_type == mouse_move_action;
}

Action action_tapAction(Action *self)
{
  switch (self->action_type) {
    case key_or_mod_action:
      return KEY(self->key_or_mod.keycode);
    case str_or_mod_action:
      return STR(self->str_or_mod.str);
    case key_or_layer_action:
      return KEY(self->key_or_layer.keycode);
    case str_or_layer_action:
      return STR(self->str_or_layer.str);
    default:
      return *self;
  }
}

Action action_holdAction(Action *self)
{
  switch (self->action_type) {
    case key_or_mod_action:
      return MOD(self->key_or_mod.modifier);
    case str_or_mod_action:
      return MOD(self->str_or_mod.modifier);
    case key_or_layer_action:
      return LAH(self->key_or_layer.layer_id);
    case str_or_layer_action:
      return LAH(self->str_or_layer.layer_id);
    default:
      return *self;
  }
}


// layers {{{1
Action layer[][N_KEYS] = {
  [COLEMAK] = {
    KEY(K_Q       ), KEY(K_W       ), KEY(K_F       ), KEY(K_P       ), KEY(K_B       ),
    KOM(K_A,GUI   ), KOM(K_R,ALT   ), KOM(K_S,CTRL  ), KOM(K_T,SHFT  ), KEY(K_G       ),
    KEY(K_Z       ), KOM(K_X,RALT  ), KEY(K_C       ), KEY(K_D       ), KEY(K_V       ),
    KOL(K_ESC,RAT ), KOL(K_SPC,NAV ), KOL(K_TAB,NUM ),
    KEY(K_J       ), KEY(K_L       ), KEY(K_U       ), KEY(K_Y       ), LA1(ACC   ),
    KEY(K_M       ), KOM(K_N,SHFT  ), KOM(K_E,CTRL  ), KOM(K_I,ALT   ), KOM(K_O,GUI   ),
    KEY(K_K       ), KEY(K_H       ), KEY(K_COMMA   ), KOM(K_DOT,RALT), KEY(K_SLASH   ),
    KOL(K_ENT,ACC ), KOL(K_BS,SYM  ), KOL(K_DEL,FUN ),
  },
  [ACC] = {
    ASC('\'', '`' ), ASC('"', '~'  ), STR("«"       ), STR("»"       ), STR("ª"       ),
    STR("á"       ), STR("à"       ), KEY(K_S       ), KOM(K_T,SHFT  ), KEY(K_G       ),
    STR("â"       ), STR("ã"       ), STR("ç"       ), KEY(K_D       ), KEY(K_V       ),
    KOL(K_ESC,RAT ), KOL(K_SPC,NAV ), KOL(K_TAB,NUM ),
    STR("º"       ), STR("€"       ), STR("ú"       ), KEY(K_Y       ), KEY(K_COMPOSE ),
    KEY(K_M       ), SOM("ñ",SHFT  ), STR("é"       ), STR("í"       ), STR("ó"       ),
    KEY(K_K       ), KEY(K_H       ), STR("ê"       ), STR("õ"       ), STR("ô"       ),
    KOL(K_ENT,NUM2), KOL(K_BS,SYM  ), KOL(K_DEL,FUN ),
  },
  [QWERTY] = {
    KEY(K_Q       ), KEY(K_W       ), KEY(K_E       ), KEY(K_R       ), KEY(K_T       ),
    KOM(K_A,GUI   ), KOM(K_S,ALT   ), KOM(K_D,CTRL  ), KOM(K_F,SHFT  ), KEY(K_G       ),
    KEY(K_Z       ), KOM(K_X,RALT  ), KEY(K_C       ), KEY(K_V       ), KEY(K_B       ),
    KOL(K_ESC,RAT ), KOL(K_SPC,NAV ), KOL(K_TAB,NUM ),
    KEY(K_Y       ), KEY(K_U       ), KEY(K_I       ), KEY(K_O       ), KEY(K_P       ),
    KEY(K_H       ), KOM(K_J,SHFT  ), KOM(K_K,CTRL  ), KOM(K_L,ALT   ), KOM(K_SMCOL,GUI),
    KEY(K_N       ), KEY(K_M       ), KEY(K_COMMA   ), KOM(K_DOT,RALT), KEY(K_SLASH   ),
    KOL(K_ENT,NUM2), KOL(K_BS,SYM  ), KOL(K_DEL,FUN ),
  },
  [RAT] = {
    COM(RESET     ), NO_ACTION,       BAS(QWERTY    ), BAS(COLEMAK   ), NO_ACTION,
    MOD(GUI       ), MOD(ALT       ), MOD(CTRL      ), MOD(SHFT      ), NO_ACTION,
    NO_ACTION,       MOD(RALT      ), LCK(FUN       ), LCK(RAT     ), NO_ACTION,
    NO_ACTION,       NO_ACTION,       NO_ACTION,
    KEY(K_VOLUP   ), MOU(wh_left   ), MOU(mv_up     ), MOU(wh_right  ), MOU(wh_up     ),
    KEY(K_VOLDOWN ), MOU(mv_left   ), MOU(mv_down   ), MOU(mv_right  ), MOU(wh_down   ),
    KEY(K_MUTE    ), NO_ACTION,       NO_ACTION,       NO_ACTION,       NO_ACTION,
    BUT(but_right ), BUT(but_left  ), BUT(but_middle),
  },
  [NAV] = {
    COM(USB_SIDE  ), NO_ACTION,       BAS(QWERTY    ), BAS(COLEMAK   ), NO_ACTION,
    MOD(GUI       ), MOD(ALT       ), MOD(CTRL      ), MOD(SHFT      ), NO_ACTION,
    NO_ACTION,       MOD(RALT      ), LCK(SYM       ), LCK(NAV       ), NO_ACTION,
    NO_ACTION,       NO_ACTION,       NO_ACTION,
    KEY(K_INSERT  ), KEY(K_HOME    ), KEY(K_UP      ), KEY(K_END     ), KEY(K_PGUP    ),
    COM(WORDLOCK  ), KEY(K_LEFT    ), KEY(K_DOWN    ), KEY(K_RIGHT   ), KEY(K_PGDN    ),
    NO_ACTION,       NO_ACTION,       NO_ACTION,       NO_ACTION,       NO_ACTION,
    KEY(K_ENT     ), KEY(K_BS      ), KEY(K_DEL     ),
  },
  [NUM] = {
    NO_ACTION,       NO_ACTION,       BAS(QWERTY    ), BAS(COLEMAK   ), NO_ACTION,
    MOD(GUI       ), MOD(ALT       ), MOD(CTRL      ), MOD(SHFT      ), NO_ACTION,
    NO_ACTION,       MOD(RALT      ), LCK(NUM2      ), LCK(NUM       ), NO_ACTION,
    NO_ACTION,       NO_ACTION,       NO_ACTION,
    ASC('*', '|'  ), KEY(K_7       ), KEY(K_8       ), KEY(K_9       ), ASC('+', '='  ),
    ASC('/', '\\' ), KEY(K_4       ), KEY(K_5       ), KEY(K_6       ), KEY(K_0       ),
    ASC('-', '_'  ), KEY(K_1       ), KEY(K_2       ), KEY(K_3       ), ASC('.', ','  ),
    KEY(K_ENT     ), KEY(K_BS      ), KEY(K_DEL     ),
  },
  [SYM] = {
    ASC('\'','/'  ), ASC('"', '?'  ), ASC('[', '{'  ), ASC(']', '}'  ), ASC('-', '_'  ),
    ASC(';', ':'  ), ASC('*', '^'  ), ASC('(', '<'  ), ASC(')', '>'  ), ASC('=', '+'  ),
    ASC('`', '~'  ), ASC('!', '$'  ), ASC('@', '%'  ), ASC('#', '&'  ), ASC('\\','|'  ),
    KEY(K_ESC     ), KEY(K_SPC     ), KEY(K_TAB     ),
    NO_ACTION,       BAS(COLEMAK   ), BAS(QWERTY    ), NO_ACTION,       COM(USB_SIDE  ),
    NO_ACTION,       MOD(SHFT      ), MOD(CTRL      ), MOD(ALT       ), MOD(GUI       ),
    NO_ACTION,       LCK(SYM       ), LCK(NAV       ), MOD(RALT      ), NO_ACTION,
    NO_ACTION,       NO_ACTION,       NO_ACTION,
  },
  [FUN] = {
    KEY(K_F12     ), KEY(K_F7      ), KEY(K_F8      ), KEY(K_F9      ), KEY(K_PRTSC   ),
    KEY(K_F11     ), KEY(K_F4      ), KEY(K_F5      ), KEY(K_F6      ), KEY(K_SCRLK   ),
    KEY(K_F10     ), KEY(K_F1      ), KEY(K_F2      ), KEY(K_F3      ), KEY(K_PAUSE   ),
    KEY(K_APP     ), KEY(K_SPC     ), KEY(K_TAB     ),
    NO_ACTION,       BAS(COLEMAK   ), BAS(QWERTY    ), NO_ACTION,       NO_ACTION,
    NO_ACTION,       MOD(SHFT      ), MOD(CTRL      ), MOD(ALT       ), MOD(GUI       ),
    NO_ACTION,       LCK(FUN       ), LCK(RAT       ), MOD(RALT      ), NO_ACTION,
    NO_ACTION,       NO_ACTION,       NO_ACTION,
  },
  [NUM2] = {
    KEY(K_LBRAKT  ), KEY(K_7       ), KEY(K_8       ), KEY(K_9       ), KEY(K_RBRAKT  ),
    KEY(K_SMCOL   ), KEY(K_4       ), KEY(K_5       ), KEY(K_6       ), KEY(K_EQUAL   ),
    KEY(K_GRAVE   ), KEY(K_1       ), KEY(K_2       ), KEY(K_3       ), KEY(K_BKSLASH ),
    KEY(K_DOT     ), KEY(K_0       ), KEY(K_MINUS   ),
    NO_ACTION,       BAS(COLEMAK   ), BAS(QWERTY    ), NO_ACTION,       NO_ACTION,
    NO_ACTION,       MOD(SHFT      ), MOD(CTRL      ), MOD(ALT       ), MOD(GUI       ),
    NO_ACTION,       LCK(NUM2      ), LCK(NUM       ), MOD(RALT      ), NO_ACTION,
    NO_ACTION,       NO_ACTION,       NO_ACTION,
  },
};

bool layer_hasMouseMovementAction(layer_id_t layer_num)
{
  for (int k = 0; k < N_KEYS; k++) {
    if (action_isMouseMovementAction(&layer[layer_num][k])) return true;
  }
  return false;
}

// WS2812 rgb led {{{1

#define WS2812_PIN 16
#define IS_RGBW true

bool led_capsLock, led_wordLock, led_usbReady;

static inline void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t pixel_grbw =
    ((uint32_t) (r) << 16) |
    ((uint32_t) (g) << 24) |
    ((uint32_t) (b) << 8);
  pio_sm_put_blocking(pio0, 0, pixel_grbw);
}

void led_updateColor()
{
  uint8_t r = 0, g = 0, b = 0;
  if (!status.usbActive && !status.otherSideUsbActive) {
    r = 50;
  } else if (status.usbActive) {
    if (led_capsLock) {
      b = 10;
    } else if (led_wordLock) {
      b = 1;
    } else {
      g = 1;
    }
  }
  led_set_rgb(r, g, b);
}

void led_setCapsLock(bool val)
{
  led_capsLock = val;
  led_updateColor();
}

void led_setWordLock(bool val)
{
  led_wordLock = val;
  led_updateColor();
}

void setUsbSide(keyboardSide side)
{
  status.usbActive = (side == status.mySide);
  status.otherSideUsbActive = (side == status.otherSide);
}

void led_init()
{
  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
  led_set_rgb(5, 5, 5);
}

// Keycodeq {{{1

#define KCQ_N 200
typedef struct {
  struct kcq_data {
    enum command {
      none,
      keycodePress,
      keycodeRelease,
      modifierPress,
      modifierRelease,
    } command;
    union {
      keycode_t keycode;
      modifier_t modifier;
    };
  } data[KCQ_N];
  uint8_t first;
  uint8_t count;
  uint8_t max_count;
} Keycodeq;

void keycodeq_init(Keycodeq *self)
{
  self->first = 0;
  self->max_count = self->count = 0;
}

void keycodeq_insertData(Keycodeq *self, struct kcq_data data)
{
  if (self->count >= KCQ_N) {
    log(LOG_E, "keycode queue full!");
    return;
  }
  self->data[(self->first + self->count) % KCQ_N] = data;
  self->count++;
  if (self->count > self->max_count) {
    self->max_count = self->count;
    log(LOG_I, "max keycode queue count: %d", self->max_count);
  }
}

void keycodeq_insertKeycodePress(Keycodeq *self, keycode_t keycode)
{
  struct kcq_data data = { .command = keycodePress, .keycode = keycode };
  keycodeq_insertData(self, data);
}

void keycodeq_insertKeycodeRelease(Keycodeq *self, keycode_t keycode)
{
  struct kcq_data data = { .command = keycodeRelease, .keycode = keycode };
  keycodeq_insertData(self, data);
}

void keycodeq_insertModifierPress(Keycodeq *self, modifier_t modifier)
{
  struct kcq_data data = { .command = modifierPress, .modifier = modifier };
  keycodeq_insertData(self, data);
}

void keycodeq_insertModifierRelease(Keycodeq *self, modifier_t modifier)
{
  struct kcq_data data = { .command = modifierRelease, .modifier = modifier };
  keycodeq_insertData(self, data);
}

enum command keycodeq_head(Keycodeq *self)
{
  if (self->count == 0) return none;
  return self->data[self->first].command;
}

keycode_t keycodeq_removeKeycode(Keycodeq *self)
{
  if (self->count == 0) return 0;
  keycode_t keycode = self->data[self->first].keycode;
  self->first = (self->first + 1) % KCQ_N;
  self->count--;
  return keycode;
}
modifier_t keycodeq_removeModifier(Keycodeq *self)
{
  if (self->count == 0) return 0;
  modifier_t modifier = self->data[self->first].modifier;
  self->first = (self->first + 1) % KCQ_N;
  self->count--;
  return modifier;
}


// USB {{{1
// interfaces with tinyUSB

struct usb {
  Keycodeq keycodeq;
  uint8_t keycodes[6];
  uint8_t n_keycodes;
  uint8_t modifiers;
  uint8_t sent_modifiers;
  button_t buttons;
};

USB *USB_singleton;

void usb_init(USB *self)
{
  USB_singleton = self;
  keycodeq_init(&self->keycodeq);
  self->sent_modifiers = 0;
  self->modifiers = 0;
  self->n_keycodes = 0;
  memset(self->keycodes, 0, 6);
  self->buttons = 0;

  tusb_init();
}

void usb_pressModifier(USB *self, modifier_t modifier)
{
  self->modifiers |= modifier;
  keycodeq_insertModifierPress(&self->keycodeq, modifier);
}

void usb_releaseModifier(USB *self, modifier_t modifier)
{
  self->modifiers &= ~modifier;
  keycodeq_insertModifierRelease(&self->keycodeq, modifier);
}

void usb_setModifiers(USB *self, modifier_t new_modifiers)
{
  modifier_t release_modifiers = self->modifiers & ~new_modifiers;
  if (release_modifiers != 0) {
    usb_releaseModifier(self, release_modifiers);
  }
  modifier_t press_modifiers = ~self->modifiers & new_modifiers;
  if (press_modifiers != 0) {
    usb_pressModifier(self, press_modifiers);
  }
}


void usb_pressKeycode(USB *self, keycode_t keycode)
{
  if (keycode_is_modifier(keycode)) {
    usb_pressModifier(self, keycode_to_modifier(keycode));
  } else {
    keycodeq_insertKeycodePress(&self->keycodeq, keycode);
  }
}

void usb_releaseKeycode(USB *self, keycode_t keycode)
{
  if (keycode_is_modifier(keycode)) {
    usb_releaseModifier(self, keycode_to_modifier(keycode));
  } else {
    keycodeq_insertKeycodeRelease(&self->keycodeq, keycode);
  }
}

void usb_sendKeyboardReport(USB *self)
{
  if (status.usbActive) {
    log(LOG_R, "send report %02x %02x.%02x.%02x.%02x.%02x.%02x",
        self->sent_modifiers,
        self->keycodes[0], self->keycodes[1], self->keycodes[2],
        self->keycodes[3], self->keycodes[4], self->keycodes[5]);
    tud_hid_keyboard_report(REPORT_ID_KEYBOARD, self->sent_modifiers, self->keycodes);
  }
}

void usb_sendMouseReport(USB *self, uint8_t buttons, int8_t v, int8_t h, int8_t wv, int8_t wh)
{
  // buttons, x, y, scroll, pan
  if (status.usbActive) {
    log(LOG_U, "usb mouse: B%x ^%d >%d %d %d", buttons, v, h, wv, wh);
    tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, h, v, wv, wh);
  }
}

void usb_pressMouseButton(USB *self, button_t button)
{
  self->buttons |= button;
  usb_sendMouseReport(self, self->buttons, 0, 0, 0, 0);
}

void usb_releaseMouseButton(USB *self, button_t button)
{
  self->buttons &= ~button;
  usb_sendMouseReport(self, self->buttons, 0, 0, 0, 0);
}

void usb_moveMouse(USB *self, int8_t v, int8_t h, int8_t wv, int8_t wh)
{
  usb_sendMouseReport(self, self->buttons, v, h, wv, wh);
}

void usb__removeKeycodeAt(USB *self, int8_t i)
{
  if (self->n_keycodes < i + 1) return;
  self->n_keycodes--;
  memmove(&self->keycodes[i], &self->keycodes[i+1], self->n_keycodes - i);
  self->keycodes[self->n_keycodes] = 0;
}
void usb__removeKeycode(USB *self, keycode_t keycode)
{
  for (int i = 0; i < self->n_keycodes; i++) {
    if (self->keycodes[i] == keycode) {
      usb__removeKeycodeAt(self, i);
      i--;
    }
  }
}
void usb__insertKeycode(USB *self, keycode_t keycode)
{
  if (self->n_keycodes >= 6) {
    usb__removeKeycodeAt(self, 0);
  }
  self->keycodes[self->n_keycodes++] = keycode;
}

void usb__sendKeycodePresses(USB *self)
{
  while (keycodeq_head(&self->keycodeq) == keycodePress) {
    usb__insertKeycode(self, keycodeq_removeKeycode(&self->keycodeq));
  }
  usb_sendKeyboardReport(self);
}
void usb__sendModifierPresses(USB *self)
{
  while (keycodeq_head(&self->keycodeq) == modifierPress) {
    self->sent_modifiers |= keycodeq_removeModifier(&self->keycodeq);
  }
  usb_sendKeyboardReport(self);
}
void usb__sendKeycodeReleases(USB *self)
{
  while (keycodeq_head(&self->keycodeq) == keycodeRelease) {
    usb__removeKeycode(self, keycodeq_removeKeycode(&self->keycodeq));
    break;
  }
  usb_sendKeyboardReport(self);
}
void usb__sendModifierReleases(USB *self)
{
  enum command cmd = keycodeq_head(&self->keycodeq);
  while (keycodeq_head(&self->keycodeq) == modifierRelease) {
    self->sent_modifiers &= ~keycodeq_removeModifier(&self->keycodeq);
    break;
  }
  usb_sendKeyboardReport(self);
}

void usb_task(USB *self)
{
  tud_task();
  status.usbReady = tud_ready();
  if (!status.usbActive) return;
  if (keycodeq_head(&self->keycodeq) == none) return;
  if (tud_suspended()) tud_remote_wakeup();
  if (!tud_hid_ready()) return;
  switch (keycodeq_head(&self->keycodeq)) {
    case none:
      break;
    case keycodePress:
      usb__sendKeycodePresses(self);
      break;
    case modifierPress:
      usb__sendModifierPresses(self);
      break;
    case keycodeRelease:
      usb__sendKeycodeReleases(self);
      break;
    case modifierRelease:
      usb__sendModifierReleases(self);
      break;
  }
}


// comm {{{1

#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

static uart_inst_t *comm_uart_id = NULL;
static int comm_error_count = 0, comm_received_message_count = 0;
static Timer send_timer, recv_timer;

void comm_init(int id)
{
  uint tx_pin, rx_pin;
  if (id == 0) {
    comm_uart_id = uart0;
    tx_pin = UART0_TX_PIN;
    rx_pin = UART0_RX_PIN;
  } else {
    comm_uart_id = uart1;
    tx_pin = UART1_TX_PIN;
    rx_pin = UART1_RX_PIN;
  }
  uart_init(comm_uart_id, BAUD_RATE);
  uart_set_fifo_enabled(comm_uart_id, true);
  gpio_set_function(tx_pin, GPIO_FUNC_UART);
  gpio_set_function(rx_pin, GPIO_FUNC_UART);
}

uint8_t comm_getc()
{
  return uart_getc(comm_uart_id);
}

void comm_putc(uint8_t c)
{
  uart_putc_raw(comm_uart_id, c);
}

static void comm__encode_key_val(uint8_t keyId, uint8_t val, uint8_t buf[2])
{
  uint8_t x = val * 3 + keyId;
  uint8_t y = (x >> 3) ^ x;
  buf[0] = ((y & 0b0111) << 4) | val;
  buf[1] = ((y & 0b1000) << 3) | 0b10000000 | keyId;
}

void comm_sendMessage(uint8_t keyId, uint8_t val)
{
  uint8_t buf[2];
  comm__encode_key_val(keyId, val, buf);
  comm_putc(buf[0]);
  comm_putc(buf[1]);
}

bool comm_receiveMessage(uint8_t *keyIdp, uint8_t *valp)
{
  static uint8_t buf[2];
  static uint8_t count = 0;
  while (uart_is_readable(comm_uart_id)) {
    uint8_t c = comm_getc();
    uint8_t keyId, val;
    if (count == 0 && c > 127) {
      comm_error_count++;
      log(LOG_C, "Err comm0 sync: [%02hhx] %d/%d", c, comm_error_count, comm_received_message_count);
      continue;
    }
    buf[count++] = c;
    if (count == 2) {
      comm_received_message_count++;
      count = 0;
      val   = buf[0] & 0b10001111;
      keyId = buf[1] & 0b00111111;
      uint8_t b[2];
      comm__encode_key_val(keyId, val, b);
      if (b[0] != buf[0] || b[1] != buf[1]) {
        comm_error_count++;
        log(LOG_C, "Err comm1 checksum: [%02hhx %02hhx] %d/%d", buf[0], buf[1], comm_error_count, comm_received_message_count);
        continue;
      }
      *keyIdp = keyId;
      *valp = val;
      return true;
    }
  }
  return false;
}

void comm_sendStatus()
{
  uint8_t val = 0;
  if (status.mySide == rightSide) val |= 0b0001;
  if (status.usbReady)            val |= 0b0010;
  if (status.usbActive)           val |= 0b0100;
  if (status.toggleUsb)           val |= 0b1000;
  comm_sendMessage(62, val);
  timer_enable_ms(&send_timer, COMM_STATUS_DELAY_MS);
}

void comm_task()
{
  uint8_t msgVal;
  uint8_t msgId;
  while (comm_receiveMessage(&msgId, &msgVal)) {
    status.commOK = true;
    timer_enable_ms(&recv_timer, COMM_STATUS_DELAY_MS * 2);
    Key *key = Key_keyWithId(msgId);
    if (key != NULL) {
      if (msgVal > 9) {
        comm_error_count++;
        log(LOG_C, "Err comm2 invalid value: [%02hhx %02hhx] %d/%d", msgId, msgVal, comm_error_count, comm_received_message_count);
      } else {
        key_setVal(key, msgVal);
      }
    } else if (msgId == 62) {
      status.otherSide          = ((msgVal & 0b0001) == 0) ? leftSide : rightSide;
      status.otherSideUsbReady  = ((msgVal & 0b0010) != 0);
      status.otherSideUsbActive = ((msgVal & 0b0100) != 0);
      status.otherSideToggleUsb = ((msgVal & 0b1000) != 0);
    } else {
      comm_error_count++;
      log(LOG_C, "Err comm3 invalid id: [%02hhx %02hhx] %d/%d", msgId, msgVal, comm_error_count, comm_received_message_count);
    }
  }
  if (status.commOK && timer_elapsed(&recv_timer))
    status.commOK = false;
}



// Key {{{1
// stores info about a key

struct key {
  Controller *controller;
  // key id, O-17 for left (0=Q,1=W), 18-35 for right (18=Y,19=U)
  int8_t keyId;
  // current state, a 0 to 9 value and a pressed state
  int8_t val;
  bool pressed;
  bool valChanged;
  bool pressChanged;
  // what to do when key is released
  Action releaseAction;
  // key can be analog or digital
  union {
    struct {
      // last value read from sensor
      uint16_t rawAnalogValue;
      uint16_t minRawRange;
      // scaled values, for filtering
      uint32_t minRaw_S;
      uint32_t maxRaw_S;
      uint32_t filteredRaw_S;
      // minimum value since key was released and maximum value since key was pressed
      //   a key press/release is recognized relative to these values
      int8_t minVal;
      int8_t maxVal;
    } /*analog*/;
    struct {
      bool lastDigitalValue;
      bool rawDigitalValue;
      bool ignoreNewValues;
      Timer debounceTimer;
    } /*digital*/;
  };
  Key *next; // to implement lists of keys in Controller
};

Key keys[N_KEYS];

static void key__sendIfChanged(Key *self);

void Key_init(Controller *controller)
{
  for (uint8_t keyId = 0; keyId < N_KEYS; keyId++) {
    key_init(&keys[keyId], controller, keyId);
  }

}

Key *Key_keyWithId(uint8_t keyId)
{
  if (keyId > N_KEYS) return NULL;
  return &keys[keyId];
}

void Key_processKeyChanges()
{
  for (uint8_t keyId = 0; keyId < N_KEYS; keyId++) {
    key_processChanges(&keys[keyId]);
  }
}

void Key_sendChangedKeys(keyboardSide side)
{
  uint8_t firstKeyId, lastKeyId;
  if (side == leftSide) {
    firstKeyId = 0;
    lastKeyId = 17;
  } else {
    firstKeyId = 18;
    lastKeyId = 35;
  }
  for (uint8_t keyId = firstKeyId; keyId <= lastKeyId; keyId++) {
    Key *key = &keys[keyId];
    key__sendIfChanged(key);
  }
}

char *key_description(Key *self)
{
  static char description[4];
  sprintf(description, "k%d", self->keyId);
  return description;
}

void key_init(Key *self, Controller *controller, uint8_t keyId)
{
  memset(self, 0, sizeof(*self));
  self->controller = controller;
  self->keyId = keyId;
  self->filteredRaw_S = UINT32_MAX;
  self->val = 0;
  self->pressed = false;
  self->minVal = 0;
}

int8_t key_id(Key *self)
{
  return self->keyId;
}

keyboardSide key_side(Key *self)
{
  if (self->keyId >=  0 && self->keyId <= 17) return leftSide;
  if (self->keyId >= 18 && self->keyId <= 35) return rightSide;
  return noSide;
}

void key_setMinRawRange(Key *self, uint16_t range)
{
  self->minRawRange = range;
}

static void filter_SS(uint32_t *old_S, uint32_t new_S, uint8_t weight)
{
  *old_S = *old_S + (new_S >> weight) - (*old_S >> weight);
}
static void filter_SnS(uint32_t *old_S, uint16_t new, uint8_t scale, uint8_t weight)
{
  filter_SS(old_S, ((uint32_t)new) << scale, weight);
}
static void key__filterRawValue(Key *self)
{
  if (self->filteredRaw_S == UINT32_MAX) {
    // if it's the first value, initialize
    self->filteredRaw_S = self->rawAnalogValue << 13;
    self->maxRaw_S = self->filteredRaw_S;
    self->minRaw_S = self->filteredRaw_S;
    return;
  }
  filter_SnS(&self->filteredRaw_S, self->rawAnalogValue, 13, 2);
  // if value is outside of max or min limits, fast drift min or max to value
  if (self->filteredRaw_S < self->minRaw_S) {
    filter_SS(&self->minRaw_S, self->filteredRaw_S, 1);
  } else if (self->filteredRaw_S > self->maxRaw_S) {
    filter_SS(&self->maxRaw_S, self->filteredRaw_S, 1);
  } else {
    // if value is close to max or min, slowly drift min or max to value
    uint32_t dist = (self->maxRaw_S - self->minRaw_S) / 3;
    if ((self->filteredRaw_S - self->minRaw_S) < dist) {
      filter_SS(&self->minRaw_S, self->filteredRaw_S, 13);
    } else if ((self->maxRaw_S - self->filteredRaw_S) < dist) {
      filter_SS(&self->maxRaw_S, self->filteredRaw_S, 13);
    }
  }
}

void key_processChanges(Key *self)
{
  if (self->keyId == -1) return;
  if (self->pressChanged) {
    self->pressChanged = false;
    if (self->pressed) {
      controller_keyPressed(self->controller, self);
    } else {
      controller_keyReleased(self->controller, self);
    }
  }
}

void key__sendIfChanged(Key *self)
{
  if (self->keyId == -1) return;
  if (self->valChanged) {
    self->valChanged = false;
    comm_sendMessage(self->keyId, self->val);
  }
}

int8_t key_val(Key *self)
{
  return self->val;
}

void key_setReleaseAction(Key *self, Action action)
{
  self->releaseAction = action;
}

Action *key_releaseAction(Key *self)
{
  return &self->releaseAction;
}

void key_setVal(Key *self, uint8_t newVal)
{
  if (self->keyId == -1) return;
  if (newVal == self->val) return;
  self->val = newVal;
  self->valChanged = true;

  if (self->pressed) {
    self->maxVal = MAX(self->maxVal, newVal);
    if (self->maxVal - newVal >= SENSITIVITY) {
      self->minVal = newVal;
      self->pressed = false;
      self->pressChanged = true;
    }
  } else {
    self->minVal = MIN(self->minVal, newVal);
    if (newVal - self->minVal >= SENSITIVITY) {
      self->maxVal = newVal;
      self->pressed = true;
      self->pressChanged = true;
    }
  }
  //log(LOG_K, "newVal k%d %d->%d m%d M%d p%d",
  //    self->keyId, self->val, newVal, self->minVal, self->maxVal, self->pressed);
}

static int constrain(int val, int minimum, int maximum)
{
  if (val < minimum) return minimum;
  if (val > maximum) return maximum;
  return val;
}

void key_setNewAnalogRaw(Key *self, uint16_t newRaw)
{
  self->rawAnalogValue = newRaw;
  key__filterRawValue(self);
  if (self->keyId == -1) return;
  int minRaw = self->minRaw_S >> 13;
  int maxRaw = self->maxRaw_S >> 13;
  int rawRange = maxRaw - minRaw;
  if (rawRange < self->minRawRange) return;
  int old_val_90 = self->val * 10;
  int new_val_90 = constrain((newRaw - minRaw) * 100 / rawRange, 0, 90);
  if (abs(new_val_90 - old_val_90) > 6) {
    int8_t newVal = (new_val_90 + 5) / 10;
    key_setVal(self, newVal);
  }
}

void key_setNewDigitalRaw(Key *self, bool newRaw)
{
  self->rawDigitalValue = newRaw;
  if (self->ignoreNewValues && timer_elapsed(&self->debounceTimer))
    self->ignoreNewValues = false;
  if (self->ignoreNewValues) return;
  if (newRaw == self->lastDigitalValue) return;
  self->lastDigitalValue = newRaw;
  timer_enable_ms(&self->debounceTimer, DEBOUNCING_DELAY_MS);
  self->ignoreNewValues = true;
  key_setVal(self, newRaw ? 9 : 0);
}

// KeyList {{{1
// A key can be at most in one controller list.
// a keylist is implemented as a linked list, using the "next" field in Key
Key *keyList_removeFirstKey(Key **list)
{
  Key *key = *list;
  if (key != NULL) *list = key->next;
  return key;
}

Key *keyList_firstKey(Key **list)
{
  return *list;
}

bool keyList_empty(Key **list)
{
  return *list == NULL;
}

void keyList_insertKey(Key **list, Key *key)
{
  key->next = NULL;
  if (*list == NULL) {
    *list = key;
    return;
  }
  Key *previous = *list;
  while (previous->next != NULL) {
    previous = previous->next;
  }
  previous->next = key;
}

void keyList_removeKey(Key **list, Key *key)
{
  if (*list == NULL) return;
  if (*list == key) {
    *list = (*list)->next;
    return;
  }
  Key *previous = *list;
  while (previous->next != NULL) {
    if (previous->next == key) {
      previous->next = key->next;
      return;
    }
    previous = previous->next;
  }
}

bool keyList_containsKey(Key **list, Key *searchedKey)
{
  for (Key *key = *list; key != NULL; key = key->next) {
    if (key == searchedKey) return true;
  }
  return false;
}

void keyList_print(Key **list)
{
  printf("[");
  for (Key *key = *list; key != NULL; key = key->next) {
    printf("%s ", key_description(key));
  }
  printf("]\n");
}

// auxiliary functions for unicode {{{1
//   very basic support for á->Á and finding a codepoint in a utf8 str
unicode unicode_to_upper(unicode lower)
{
  unicode upper = lower;
  if (lower >= 'a' && lower <= 'z') upper = lower - 0x20;
  else if (lower >= 0xe0 && lower <= 0xfe && lower != 0xf7) upper = lower - 0x20;
  else if (lower == 0xff) upper = 0x178;
  else if (lower >= 0x100 && lower <= 0x137 && (lower&1) == 1) upper = lower - 1;
  else if (lower >= 0x139 && lower <= 0x148 && (lower&1) == 0) upper = lower - 1;
  else if (lower >= 0x14a && lower <= 0x177 && (lower&1) == 1) upper = lower - 1;
  else if (lower >= 0x179 && lower <= 0x17e && (lower&1) == 0) upper = lower - 1;
  // TODO: missing cases
  return upper;
}

int utf8_nbytes(char *s)
{
  // returns number of uft8 bytes in s for one unicode char
  // only analyses first byte:
  //   0xxxxxxx 1
  //   110xxxxx 2
  //   1110xxxx 3
  //   11110xxx 4
  if ((*s & 0b10000000) == 0b00000000) return 1;
  if ((*s & 0b11100000) == 0b11000000) return 2;
  if ((*s & 0b11110000) == 0b11100000) return 3;
  if ((*s & 0b11111000) == 0b11110000) return 4;
  return 0;
}

uint32_t unicode_from_utf8(char *p)
{
  switch (utf8_nbytes(p)) {
    case 1:
      return   p[0];
    case 2:
      return ((p[0] & 0b00011111) <<  6)
           |  (p[1] & 0b00111111);
    case 3:
      return ((p[0] & 0b00001111) << 12)
           | ((p[1] & 0b00111111) <<  6)
           |  (p[2] & 0b00111111);
    case 4:
      return ((p[0] & 0b00000111) << 18)
           | ((p[1] & 0b00111111) << 12)
           | ((p[2] & 0b00111111) <<  6)
           |  (p[3] & 0b00111111);
    default:
      return 0; // error!
  }
}



// Controller {{{1
// controls the processing of keypresses

struct controller {
  layer_id_t currentLayer;
  layer_id_t baseLayer;
  layer_id_t lockLayer;
  USB *usb;
  Key *waitingKeys;
  Key *keysBeingHeld;
  Timer waitingKeyTimer;
  enum holdType holdType;
  keyboardSide holdSide;
  Timer moveMouseTimer;
  int16_t mousePos_v;
  int16_t mousePos_h;
  int16_t mousePos_wv;
  int16_t mousePos_wh;
  Action delayedReleaseAction;
  modifier_t modifiers;
  bool wordLocked;
  bool capsLocked;
  layer_id_t changeToLayer;
  Timer changeLayerTimer;
} *controller_singleton;

static void controller__setCurrentLayer(Controller *self, layer_id_t layer_id)
{
  self->currentLayer = layer_id;
  if (layer_hasMouseMovementAction(layer_id)) {
    timer_enable_ms(&self->moveMouseTimer, MOUSE_PERIOD_MS);
  } else {
    timer_disable(&self->moveMouseTimer);
  }
}
void controller_init(Controller *self, USB *usb)
{
  controller_singleton = self;
  memset(self, 0, sizeof(*self));
  self->usb = usb;
  self->baseLayer = COLEMAK;
  controller__setCurrentLayer(self, COLEMAK);
  self->lockLayer = NO_LAYER;
  /*self->waitingKeys = KeyList_create();*/
  /*self->keysBeingHeld = KeyList_create();*/
  self->holdType = noHoldType;
  self->holdSide = noSide;
  timer_disable(&self->moveMouseTimer);
  self->delayedReleaseAction = Action_noAction();
  self->modifiers = 0;
  self->wordLocked = false;
  self->capsLocked = false;
}

// auxiliary functions for wordLock {{{2
bool uni_in_word(unicode uni)
{
  if (uni == '_') return true;
  if (uni >= '0' && uni <= '9') return true;
  if (uni >= 'a' && uni <= 'z') return true;
  if (uni >= 'A' && uni <= 'Z') return true;
  if (uni != unicode_to_upper(uni) /*|| uni != uni_to_lower(uni)*/) return true;
  return false;
}

bool keycode_in_word(keycode_t keycode, bool shifted)
{
  // NOTE: review this if keyboard layout configured in OS is not US
  if (keycode == K_MINUS && shifted) return true;
  if (keycode == K_0 && !shifted) return true;
  if (keycode >= K_1 && keycode <= K_9 && !shifted) return true;
  if (keycode >= K_A && keycode <= K_Z) return true;
  if (keycode == K_BS || keycode == K_DEL) return true;
  return false;
}

bool keycode_in_word_invert_shift(keycode_t keycode)
{
  if (keycode >= K_A && keycode <= K_Z) return true;
  return false;
}
// }}}

void Controller_setCapsLock(bool newVal)
{
  controller_singleton->capsLocked = newVal;
  led_setCapsLock(newVal);
}

static void controller__setWordLock(Controller *self, bool newVal)
{
  self->wordLocked = newVal;
  led_setWordLock(newVal);
}

static void controller__setModifiers(Controller *self, modifier_t new_modifiers)
{
  self->modifiers = new_modifiers;
  usb_setModifiers(self->usb, new_modifiers);
}

static void controller__addModifiers(Controller *self, modifier_t new_modifiers)
{
  controller__setModifiers(self, self->modifiers | new_modifiers);
}

static void controller__removeModifiers(Controller *self, modifier_t new_modifiers)
{
  controller__setModifiers(self, self->modifiers & ~new_modifiers);
}

static bool controller__isShifted(Controller *self)
{
  return (self->modifiers & (SHFT | RSHFT)) != 0;
}

static void controller__sendPressKeycode(Controller *self, keycode_t keycode)
{
  if (self->wordLocked && !keycode_in_word(keycode, controller__isShifted(self))) {
    controller__setWordLock(self, false);
  }
  if (self->wordLocked && keycode_in_word_invert_shift(keycode)) {
    usb_setModifiers(self->usb, self->modifiers ^ SHFT);
  } else {
    usb_setModifiers(self->usb, self->modifiers);
  }
  usb_pressKeycode(self->usb, keycode);
  usb_setModifiers(self->usb, self->modifiers);
}

static void controller__sendReleaseKeycode(Controller *self, keycode_t keycode)
{
  usb_setModifiers(self->usb, self->modifiers);
  usb_releaseKeycode(self->usb, keycode);
}

void controller_changeLayer(Controller *self, layer_id_t layer)
{
  log(LOG_T, "%s(%d)", __func__, layer);
  if (self->lockLayer == NO_LAYER) {
    controller__setCurrentLayer(self, layer);
  }
  log(LOG_T, "layer=%d base=%d lock=%d", self->currentLayer, self->baseLayer, self->lockLayer);
}

void controller_lockLayer(Controller *self, layer_id_t layer)
{
  log(LOG_T, "%s(%d)", __func__, layer);
  if (self->lockLayer == layer) {
    // unlock if already locked in same layer
    self->lockLayer = NO_LAYER;
    controller__setCurrentLayer(self, self->baseLayer);
  } else {
    if (self->changeToLayer != layer) {
      // mark and wait for second tap
      self->changeToLayer = layer;
      timer_enable_ms(&self->changeLayerTimer, LOCK_DELAY_MS);
    } else {
      // lock on second tap
      self->lockLayer = layer;
      controller__setCurrentLayer(self, layer);
      timer_disable(&self->changeLayerTimer);
    }
  }
}

void controller_changeBaseLayer(Controller *self, layer_id_t layer)
{
  log(LOG_T, "%s(%d)", __func__, layer);
  if (self->changeToLayer != layer) {
    // mark and wait for second tap
    self->changeToLayer = layer;
    timer_enable_ms(&self->changeLayerTimer, LOCK_DELAY_MS);
  } else {
    // change base layer on second tap
    self->baseLayer = layer;
    timer_disable(&self->changeLayerTimer);
  }
}

layer_id_t controller_baseLayer(Controller *self)
{
  return self->baseLayer;
}

static void controller__pressKey(Controller *self, Key *key)
{
  Action action = layer[self->currentLayer][key_id(key)];
  if (action_isMouseMovementAction(&action)) {
    log(LOG_T, "ignoring mouse movement key press");
    return;
  }
  log(LOG_T, "pressKey %s %s", key_description(key), action_description(&action));
  key_setReleaseAction(key, Action_noAction()); // just in case...
  if (key_side(key) == self->holdSide) {
    if (action_isTypingAction(&action)) {
      log(LOG_T, "ignoring typing key on same side of held key");
      return;
    }
    keyList_insertKey(&self->keysBeingHeld, key);
    action = action_holdAction(&action);
    log(LOG_T, "hold: %s", action_description(&action));
  } else {
    action = action_tapAction(&action);
    log(LOG_T, "tap: %s", action_description(&action));
  }
  action_actuate(&action, key, self);
}

static void controller__releaseKey(Controller *self, Key *key)
{
  Action *action = key_releaseAction(key);
  log(LOG_T, "releaseKey %s %s", key_description(key), action_description(action));
  if (self->holdSide != noSide) {
    keyList_removeKey(&self->keysBeingHeld, key);
    if (keyList_empty(&self->keysBeingHeld)) {
      self->holdSide = noSide;
    }
  }
  action_actuate(action, key, self);
  key_setReleaseAction(key, Action_noAction());
}

static void controller__resetWaitingKeyTimeout(Controller *self)
{
  if (!keyList_empty(&self->waitingKeys)) {
    timer_enable_ms(&self->waitingKeyTimer, HOLD_DELAY_MS);
  } else {
    timer_disable(&self->waitingKeyTimer);
  }
}

void controller_keyPressed(Controller *self, Key *key)
{
  log(LOG_T, "keyPressed: %s", key_description(key));
  if (keyList_empty(&self->waitingKeys)) {
    Action *action = &layer[self->currentLayer][key_id(key)];
    if (action_holdType(action) == noHoldType) {
      log(LOG_T, " press action: %s", action_description(action));
      controller__pressKey(self, key);
    } else {
      log(LOG_T, "key wait 1");
      keyList_insertKey(&self->waitingKeys, key);
      controller__resetWaitingKeyTimeout(self);
    }
  } else {
    log(LOG_T, " key wait 2");
    keyList_insertKey(&self->waitingKeys, key);
    controller__resetWaitingKeyTimeout(self);
  }
}

void controller_holdWaitingKeysUntilKey(Controller *self, Key *lastKey)
{
  if (keyList_empty(&self->waitingKeys)) return;
  self->holdSide = key_side(keyList_firstKey(&self->waitingKeys));
  while (!keyList_empty(&self->waitingKeys)) {
    Key *key = keyList_removeFirstKey(&self->waitingKeys);
    controller__pressKey(self, key);
    if (key == lastKey) {
      break;
    }
  }
}

void controller_tapWaitingKeysUntilKey(Controller *self, Key *lastKey)
{
  if (keyList_empty(&self->waitingKeys)) return;
  while (!keyList_empty(&self->waitingKeys)) {
    Key *key = keyList_removeFirstKey(&self->waitingKeys);
    controller__pressKey(self, key);
    if (key == lastKey) {
      break;
    }
  }
}

void controller_keyReleased(Controller *self, Key *key)
{
  Action delayedAction = self->delayedReleaseAction;
  self->delayedReleaseAction = Action_noAction();
  log(LOG_T, "keyReleased: %s", key_description(key));
  if (keyList_containsKey(&self->waitingKeys, key)) {
    log(LOG_T, " was waiting");
    Key *firstKey = keyList_firstKey(&self->waitingKeys);
    if (firstKey == key || key_side(firstKey) == key_side(key)) { // it's a tap
      log(LOG_T, " it's a tap (%s)", key_description(key));
      controller_tapWaitingKeysUntilKey(self, key);
    } else { // it's a hold
      log(LOG_T, " it's a hold (%s, first %s)", key_description(key), key_description(firstKey));
      controller_holdWaitingKeysUntilKey(self, key);
    }
    controller__resetWaitingKeyTimeout(self);
  }
  controller__releaseKey(self, key);
  log(LOG_T, " delayed action %s", action_description(&delayedAction));
  action_actuate(&delayedAction, key, self);
}

void controller_pressKeycode(Controller *self, keycode_t keycode)
{
  log(LOG_T, "%s(%d)", __func__, keycode);
  if (keycode_is_modifier(keycode)) {
    controller__addModifiers(self, keycode_to_modifier(keycode));
  } else {
    controller__sendPressKeycode(self, keycode);
  }
}

void controller_releaseKeycode(Controller *self, keycode_t keycode)
{
  log(LOG_T, "%s(%d)", __func__, keycode);
  if (keycode_is_modifier(keycode)) {
    controller__removeModifiers(self, keycode_to_modifier(keycode));
  } else {
    controller__sendReleaseKeycode(self, keycode);
  }
}


static void controller__sendUsbPressAsciiChar(Controller *self, uint8_t ch)
{
  mod_key mk = ascii_to_mod_key[ch];
  if (mk.key == 0) return;
  // remove SHIFT from current modifiers, add SHIFT from table
  // NOTE: must change if keyboard layout needs other modifier for some ascii chars
  modifier_t mod = self->modifiers & ~(SHFT | RSHFT);
  mod |= mk.mod;
  usb_setModifiers(self->usb, mod);
  usb_pressKeycode(self->usb, mk.key);
}
static void controller__sendUsbReleaseAsciiChar(Controller *self, uint8_t ch)
{
  mod_key mk = ascii_to_mod_key[ch];
  if (mk.key == 0) return;
  // remove SHIFT from current modifiers, add SHIFT from table
  // NOTE: must change if keyboard layout needs other modifier for some ascii chars
  modifier_t mod = self->modifiers & ~(SHFT | RSHFT);
  mod |= mk.mod;
  usb_setModifiers(self->usb, mod);
  usb_releaseKeycode(self->usb, mk.key);
}

static void controller__sendUsbHexNibble(Controller *self, uint8_t h)
{
  uint8_t ch = h + '0';
  if (ch > '9') {
    ch += 'a' - ('9' + 1);
  }
  controller__sendUsbPressAsciiChar(self, ch);
  controller__sendUsbReleaseAsciiChar(self, ch);
}

static void controller__sendUsbHex(Controller *self, uint32_t hex)
{
  bool sent = false;
  for (int n = 7; n >= 0; n--) {
    uint8_t nib = (hex >> n*4) & 0b1111;
    if (nib != 0 || sent || n == 0) {
      controller__sendUsbHexNibble(self, nib);
      sent = true;
    }
  }
}
char compose_table[][3] = {
  "  ", "!!", "|c", "-L", "ox", "=Y", "!^", "so", // A0  ¡¢£¤¥¦§
  "\" ","OC", "^_a","<<", "-,", "-- ","OR", "-^", // A8 ¨©ª«¬­®¯
  "oo", "+-", "^2", "^3", "''", "mu", "P!", "^.", // B0 °±²³´µ¶·
  ",,", "^1", "^_o",">>", "14", "12", "34", "??", // B8 ¸¹º»¼½¾¿
  "`A", "'A", "^A", "~A", "\"A","*A", "AE", ",C", // C0 ÀÁÂÃÄÅÆÇ
  "`E", "'E", "^E", "\"E","`I", "'I", "^I", "\"I",// C8 ÈÉÊËÌÍÎÏ
  "DH", "~N", "`O", "'O", "^O", "~O", "\"O","xx", // D0 ÐÑÒÓÔÕÖ×
  "/O", "`U", "'U", "^U", "\"U","'Y", "TH", "ss", // D8 ØÙÚÛÜÝÞß
  "`a", "'a", "^a", "~a", "\"a","*a", "ae", ",c", // E0 àáâãäåæç
  "`e", "'e", "^e", "\"e","`i", "'i", "^i", "\"i",// E8 èéêëìíîï
  "dh", "~n", "`o", "'o", "^o", "~o", "\"o",":-", // F0 ðñòóôõö÷
  "/o", "`u", "'u", "^u", "\"u","'y", "th", "\"y",// F8 øùúûüýþÿ
};
static bool controller__hasComposeKeycodesForUnicode(Controller *self, unicode uni)
{
  if (uni < 0xA0 || uni > 0xFF) return false;
  int i = uni - 0xA0;
  if (compose_table[i][0] == '\0') return false;
  return true;
}
static char *controller__composeKeycodesForUnicode(Controller *self, unicode uni)
{
  if (uni < 0xA0 || uni > 0xFF) return NULL;
  int i = uni - 0xA0;
  return compose_table[i];
}
static void controller__sendUsbUnicodeChar(Controller *self, unicode uni)
{
  if (uni < 128) {
    controller__sendUsbPressAsciiChar(self, uni);
    controller__sendUsbReleaseAsciiChar(self, uni);
  } else if (controller__hasComposeKeycodesForUnicode(self, uni)) {
    char *compose_chars = controller__composeKeycodesForUnicode(self, uni);
    usb_pressKeycode(self->usb, K_COMPOSE);
    usb_releaseKeycode(self->usb, K_COMPOSE);
    for (int i = 0; i < 3 && compose_chars[i] != 0; i++) {
      controller__sendUsbPressAsciiChar(self, compose_chars[i]);
      controller__sendUsbReleaseAsciiChar(self, compose_chars[i]);
    }
  } else {
    // send C-S-u + unicode in hex + enter — this usually works in linux
    controller__setModifiers(self, RCTRL | RSHFT);
    usb_pressKeycode(self->usb, K_U);
    usb_releaseKeycode(self->usb, K_U);
    controller__setModifiers(self, 0);
    controller__sendUsbHex(self, uni);
    controller__sendUsbPressAsciiChar(self, '\n');
    controller__sendUsbReleaseAsciiChar(self, '\n');
  }
}

static void controller__sendUtf8Str(Controller *self, char s[])
{
  modifier_t save_modifiers = self->modifiers;
  bool capsLocked = self->capsLocked;
  bool shifted = controller__isShifted(self);
  if (capsLocked) {
    usb_pressKeycode(self->usb, K_CAPS);
    usb_releaseKeycode(self->usb, K_CAPS);
  }
  log(LOG_T, "shift:%d(%02x) caps:%d", shifted, self->usb->sent_modifiers, capsLocked);
  while (true) {
    unicode uni = unicode_from_utf8(s);
    if (uni == 0) break;
    if (self->wordLocked && !uni_in_word(uni)) controller__setWordLock(self, false);
    if (shifted ^ capsLocked ^ self->wordLocked) uni = unicode_to_upper(uni);
    controller__sendUsbUnicodeChar(self, uni);
    s += utf8_nbytes(s);
  }
  if (capsLocked) {
    usb_pressKeycode(self->usb, K_CAPS);
    usb_releaseKeycode(self->usb, K_CAPS);
  }
  controller__setModifiers(self, save_modifiers);
}

static uint8_t controller__sendPressAsciiChar(Controller *self, uint8_t ch)
{
  if (self->wordLocked && !uni_in_word(ch)) controller__setWordLock(self, false);
  if (self->wordLocked) ch = unicode_to_upper(ch);
  controller__sendUsbPressAsciiChar(self, ch);
  usb_setModifiers(self->usb, self->modifiers);
  return ch;
}

static void controller__sendReleaseAsciiChar(Controller *self, uint8_t ch)
{
  controller__sendUsbReleaseAsciiChar(self, ch);
  usb_setModifiers(self->usb, self->modifiers);
}

char controller_pressAscii(Controller *self, char unshifted_char, char shifted_char)
{
  log(LOG_T, "%s(%c,%c)S=%d", __func__, unshifted_char, shifted_char, controller__isShifted(self));
  char pressed_char = controller__isShifted(self) ? shifted_char : unshifted_char;
  return controller__sendPressAsciiChar(self, pressed_char);
}

void controller_releaseAscii(Controller *self, char pressed)
{
  log(LOG_T, "%s(%c)", __func__, pressed);
  controller__sendReleaseAsciiChar(self, pressed);
}

void controller_pressString(Controller *self, char s[])
{
  log(LOG_T, "%s(%c)", __func__, s[0]);
  controller__sendUtf8Str(self, s);
}

void controller_pressModifier(Controller *self, modifier_t modifier)
{
  log(LOG_T, "%s(%d)", __func__, modifier);
  controller__addModifiers(self, modifier);
}
void controller_releaseModifier(Controller *self, modifier_t modifier)
{
  log(LOG_T, "%s(%d)", __func__, modifier);
  controller__removeModifiers(self, modifier);
}
void controller_setDelayedReleaseAction(Controller *self, Action action)
{
  log(LOG_T, "set delayed");
  log(LOG_T, "%s: %s", __func__, action_description(&action));
  self->delayedReleaseAction = action;
}

void controller_pressMouseButton(Controller *self, button_t button)
{
  usb_pressMouseButton(self->usb, button);
}
void controller_releaseMouseButton(Controller *self, button_t button)
{
  usb_releaseMouseButton(self->usb, button);
}
void controller_moveMouse(Controller *self, int v, int h, int wv, int wh)
{
  // accumulate mouse movements (in centimickeys)
  self->mousePos_v += v;
  self->mousePos_h += h;
  self->mousePos_wv += wv;
  self->mousePos_wh += wh;
}
static void controller__sendMouseMovement(Controller *self)
{
  // convert accumulated mouse movements in centimickeys to mickeys
  int v = self->mousePos_v / 100;
  int h = self->mousePos_h / 100;
  int wv = self->mousePos_wv / 100;
  int wh = self->mousePos_wh / 100;
  log(LOG_T, "controller move %d %d %d %d\n", self->mousePos_v, self->mousePos_h, self->mousePos_wv, self->mousePos_wh);
  if (v != 0 || h != 0 || wv != 0 || wh != 0) {
    // remove used movements from accumulation
    self->mousePos_v -= v * 100;
    self->mousePos_h -= h * 100;
    self->mousePos_wv -= wv * 100;
    self->mousePos_wh -= wh * 100;
    log(LOG_T, "controller move %d %d %d %d\n", v, h, wv, wh);
    usb_moveMouse(self->usb, v, h, wv, wh);
  }
}
static void controller__timedMoveMouse(Controller *self)
{
  // mouse move actions call controller_moveMouse, that accumulates the movements
  for (uint8_t k = 0; k < N_KEYS; k++) {
    Action *action = &layer[self->currentLayer][k];
    if (action_isMouseMovementAction(action)) {
      Key *key = Key_keyWithId(k);
      action_actuate(action, key, self);
    }
  }
  // send accumulated movements in one USB event
  controller__sendMouseMovement(self);
  timer_enable_ms(&self->moveMouseTimer, MOUSE_PERIOD_MS);
}

void controller_doCommand(Controller *self, int command)
{
  if (command == WORDLOCK) {
    controller__setWordLock(self, !self->wordLocked);
    return;
  } else if (command == RESET) {
    reset_usb_boot(0, 0);
    return;
  } else if (command == USB_SIDE) {
    printf("CMD: USB_SIDE\n");
    status.toggleUsb = true;
    return;
  }
  printf("%s(%d) not implemented\n", __func__, command);
  printf("Layers: current=%d base=%d\n", self->currentLayer, self->baseLayer);
  printf("waiting: "); keyList_print(&self->waitingKeys);
  printf("being held: "); keyList_print(&self->keysBeingHeld);
  self->baseLayer = COLEMAK;
  controller__setCurrentLayer(self, COLEMAK);
}

void controller_task(Controller *self)
{
  Key_processKeyChanges();
  if (timer_elapsed(&self->changeLayerTimer)) self->changeToLayer = NO_LAYER;
  if (timer_elapsed(&self->moveMouseTimer)) {
    controller__timedMoveMouse(self);
  }
  if (timer_elapsed(&self->waitingKeyTimer)) {
    log(LOG_T, "hold timeout");
    controller_holdWaitingKeysUntilKey(self, NULL);
  }
}

// LocalReader {{{1
// reads the keys physically connected to local microcontroller
typedef struct {
  uint8_t *sel_pins;
  keyboardSide side;
  keyboardType kb_type;
  int8_t hw_version;
  int8_t *hwIdToKeyId;
} LocalReader;

void localReader__initAnalogGPIO(LocalReader *self)
{
  adc_init();
  for (int i = 0; i < N_ANA_PINS; i++) {
    adc_gpio_init(ana_pins[i]);
  }
  for (int i = 0; i < N_SEL_PINS; i++) {
    uint pin = self->sel_pins[i];
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
  }
}

void localReader__initDigitalGPIO(LocalReader *self)
{
  for (int pin = 0; pin < N_DIGITAL_HWKKEYS; pin++) {
    if (self->hwIdToKeyId[pin] != -1) {
      gpio_init(pin);
      gpio_set_dir(pin, GPIO_IN);
      gpio_pull_up(pin);
    }
  }
}

static uint16_t readPin(uint sel, uint ana)
{
  adc_select_input(ana);
  gpio_init(sel);
  gpio_set_dir(sel, GPIO_OUT);
  gpio_put(sel, 1);
  uint16_t raw = adc_read();
  gpio_put(sel, 0);
  sleep_us(250);
  return raw;
}

int v1, v2;
static bool detect_resistor(uint8_t pin1, uint8_t pin2)
{
  // can only detect resistor connecting analog pins (26 to 29)
  if (pin1 < 26 || pin1 > 29 || pin2 < 26 || pin2 > 29) return false;
  uint8_t adc1 = pin1 - 26;
  adc_init();
  adc_gpio_init(pin1);
  adc_select_input(adc1);
  gpio_init(pin2);
  gpio_set_dir(pin2, GPIO_OUT);
  gpio_put(pin2, false);
  sleep_us(500);
  v1 = adc_read();
  gpio_put(pin2, true);
  sleep_us(500);
  v2 = adc_read();
  gpio_deinit(28);
  return abs(v1 - v2) > 500;
}
static bool detect_connection(uint8_t pin1, uint8_t pin2)
{
  bool v1, v2;
  gpio_init(pin1);
  gpio_set_dir(pin1, GPIO_IN);
  gpio_init(pin2);
  gpio_set_dir(pin2, GPIO_OUT);
  gpio_put(pin2, false);
  sleep_us(10);
  v1 = gpio_get(pin1);
  gpio_put(pin2, true);
  sleep_us(10);
  v2 = gpio_get(pin1);
  gpio_deinit(pin1);
  gpio_deinit(pin2);
  return !v1 && v2;
}

static uint8_t readKeyboardVersion()
{
  // hardware type 2 and 3 have digital choc keys
  // they can be recognized because they have a resistor connecting analog pins.
  // type 2 is a right half, and has a resistor connecting pins 28 and 29
  // type 3 is a left half, and has a resistor connecting pins 28 and 26
  if (detect_resistor(28, 29)) return 2;
  if (detect_resistor(28, 26)) return 3;

  // hardware type 0 and 1 have analog hall sensors
  // on type 1, the right keyboard half, pin 2 is connected to pin 1;
  // on type 0, the left half, pin 2 is connected to pin 3
  if (detect_connection(1, 2)) return 1;
  if (detect_connection(3, 2)) return 0;

  return 255;
}

void localReader_discoverTypeSideAndVersion(LocalReader *self)
{
  self->hw_version = readKeyboardVersion();
  switch (self->hw_version) {
    case 0:
      self->kb_type = analog;
      self->side = leftSide;
      self->sel_pins = left_sel_pins;
      self->hwIdToKeyId = leftAnalogHwIdToSwId;
      comm_init(1);
      break;
    case 1:
      self->kb_type = analog;
      self->side = rightSide;
      self->sel_pins = right_sel_pins;
      self->hwIdToKeyId = rightAnalogHwIdToSwId;
      comm_init(1);
      break;
    case 2:
      self->kb_type = digital;
      self->side = rightSide;
      self->sel_pins = NULL;
      self->hwIdToKeyId = rightDigitalHwIdToSwId;
      comm_init(0);
      break;
    case 3:
      self->kb_type = digital;
      self->side = leftSide;
      self->sel_pins = NULL;
      self->hwIdToKeyId = leftDigitalHwIdToSwId;
      comm_init(0);
      break;
    default:
      self->side = noSide;
  }
}

void localReader_init(LocalReader *self, Controller *controller)
{
  localReader_discoverTypeSideAndVersion(self);
  if (self->side == noSide) return;
  if (self->kb_type == analog) {
    localReader__initAnalogGPIO(self);
    for (int8_t hwId = 0; hwId < N_ANALOG_HWKKEYS; hwId++) {
      int8_t keyId = self->hwIdToKeyId[hwId];
      if (keyId != -1) {
        Key *key = Key_keyWithId(keyId);
        key_setMinRawRange(key, 80);
      }
    }
  } else {
    localReader__initDigitalGPIO(self);
  }
}

keyboardSide localReader_keyboardSide(LocalReader *self)
{
  return self->side;
}

void localReader_readDigitalKeys(LocalReader *self)
{
  uint32_t new = gpio_get_all();
  uint8_t bit = 0;
  uint32_t bitmask = 1;
  while (bit < N_DIGITAL_HWKKEYS) {
    Key *key = Key_keyWithId(self->hwIdToKeyId[bit]);
    if (key != NULL) {
      key_setNewDigitalRaw(key, (new & bitmask) == 0);
    }
    bit++;
    bitmask <<= 1;
  }
}

void localReader_readAnalogKeys(LocalReader *self)
{
  uint8_t keyId = 0;
  for (int sel=0; sel < N_SEL_PINS; sel++) {
    uint pin = self->sel_pins[sel];
    gpio_put(pin, 1);
    for (int ana=0; ana < N_ANA_PINS; ana++) {
      adc_select_input(ana);
      uint16_t raw = adc_read();
      Key *key = Key_keyWithId(self->hwIdToKeyId[keyId]);
      key_setNewAnalogRaw(key, raw);
      keyId++;
    }
    gpio_put(pin, 0);
    sleep_us(50);
  }
}
void localReader_readKeys(LocalReader *self)
{
  switch (self->kb_type) {
    case analog:
      return localReader_readAnalogKeys(self);
    case digital:
      return localReader_readDigitalKeys(self);
  }
}
// }}}
// USB callbacks {{{1
#if 0
//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void)itf;
	(void)rts;

	if (dtr) {
		tud_cdc_write_str("Connected\n");
	}
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  // skip if hid is not ready yet
  if ( !tud_hid_ready() ) return;

  switch(report_id)
  {
    case REPORT_ID_KEYBOARD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_keyboard_key = false;

      if ( btn )
      {
        uint8_t keycode[6] = { 0 };
        keycode[0] = HID_KEY_A;

        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
        has_keyboard_key = true;
      }else
      {
        // send empty key report if previously has key pressed
        if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
        has_keyboard_key = false;
      }
    }
    break;

    case REPORT_ID_MOUSE:
    {
      int8_t const delta = 5;

      // no button, right + down, no scroll, no pan
      tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta, delta, 0, 0);
    }
    break;

    case REPORT_ID_CONSUMER_CONTROL:
    {
      // use to avoid send multiple consecutive zero report
      static bool has_consumer_key = false;

      if ( btn )
      {
        // volume down
        uint16_t volume_down = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
        tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &volume_down, 2);
        has_consumer_key = true;
      }else
      {
        // send empty key report (release key) if previously has key pressed
        uint16_t empty_key = 0;
        if (has_consumer_key) tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &empty_key, 2);
        has_consumer_key = false;
      }
    }
    break;

    case REPORT_ID_GAMEPAD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_gamepad_key = false;

      hid_gamepad_report_t report =
      {
        .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
        .hat = 0, .buttons = 0
      };

      if ( btn )
      {
        report.hat = GAMEPAD_HAT_UP;
        report.buttons = GAMEPAD_BUTTON_A;
        tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));

        has_gamepad_key = true;
      }else
      {
        report.hat = GAMEPAD_HAT_CENTERED;
        report.buttons = 0;
        if (has_gamepad_key) tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
        has_gamepad_key = false;
      }
    }
    break;

    default: break;
  }
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  uint32_t const btn = board_button_read();

  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }else
  {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_KEYBOARD, btn);
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint8_t len)
{
  (void) instance;
  (void) len;

return;
  uint8_t next_report_id = report[0] + 1;

  if (next_report_id < REPORT_ID_COUNT)
  {
    send_hid_report(next_report_id, board_button_read());
  }
}

#endif
// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD)
    {
      printf("led report %d %b\n", bufsize, buffer[0]);
      // bufsize should be (at least) 1
      if ( bufsize < 1 ) return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
      {
        // Capslock On: disable blink, turn led on
        printf("capslock on\n");
        Controller_setCapsLock(true);
      }else
      {
        // Caplocks Off: back to normal blink
        printf("capslock off\n");
        Controller_setCapsLock(false);
      }
    }
  }
}

// main {{{1
void log_keys(keyboardSide side, int version)
{
  static Timer timer;
  static uint32_t ct = 0;

  if ((log_level & LOG_L) != 0) {
    if (!timer_is_enabled(&timer)) {
      timer_enable_ms(&timer, 1000);
    }
    uint8_t firstKeyId, lastKeyId;
    if (side == leftSide) {
      firstKeyId = 0;
      lastKeyId = 17;
    } else {
      firstKeyId = 18;
      lastKeyId = 35;
    }
    ct++;
    if (timer_elapsed(&timer)) {
      timer_enable_ms(&timer, 1000);
      printf("%s ", status.mySide == leftSide ? "LEFT" : "RIGHT");
      printf("U:%c%c%c%c ", status.usbReady ? 'R' : 'r', status.usbActive ? 'A' : 'a', status.otherSideUsbReady ? 'R' : 'r', status.otherSideUsbActive ? 'A' : 'a');
      printf("C:%c ", status.commOK ? 'Y' : 'n');
      printf("%uHz ", ct);
      printf("V%d ", version);
      printf("L%d ", controller_singleton->currentLayer);
      if (version == 2 || version ==  3) {
        printf("%d|%d\n", v1, v2);
        for (int i = firstKeyId; i <= lastKeyId; i++) {
          printf("%5u", keys[i].rawDigitalValue);
        }
      } else if (version == 0 || version == 1) {
        printf("\n");
        for (int i = firstKeyId; i <= lastKeyId; i++) {
          printf("%5u", keys[i].minRaw_S >> 13);
        }
        printf("\n");
        for (int i = firstKeyId; i <= lastKeyId; i++) {
          printf("%5u", keys[i].maxRaw_S >> 13);
        }
        printf("\n");
        for (int i = firstKeyId; i <= lastKeyId; i++) {
          printf("%5u", (keys[i].maxRaw_S - keys[i].minRaw_S) >> 13);
        }
        printf("\n");
        for (int i = firstKeyId; i <= lastKeyId; i++) {
          printf("%5u", keys[i].rawAnalogValue);
        }
      }
      printf("\n");
      for (int i = firstKeyId; i <= lastKeyId; i++) {
        printf("%5u", keys[i].val);
      }
      printf("\n");
      fflush(stdout);
      ct = 0;
    }
  }
}

void fatal(char *msg)
{
  while (true) {
    printf("FATAL ERROR: %s\n", msg);
    led_set_rgb(100, 0, 0);
    sleep_ms(500);
    led_set_rgb(0, 0, 0);
    sleep_ms(500);
  }
}

void hardware_init()
{
  stdio_init_all();
  stdio_set_translate_crlf(&stdio_usb, false);
  setTimestamp(&status.lastActiveTimestamp);
  status.toggleUsb = true;

  led_init();
}


void synchronizeAndDecideUsbSide()
{
  bool shouldSendStatus = timer_elapsed(&send_timer);
  if (status.usbActive && !status.usbReady) status.toggleUsb = true;
  if (status.usbActive && status.toggleUsb) status.usbActive = false;
  if (status.otherSideToggleUsb) {
    if (status.usbReady) status.usbActive = true;
    status.otherSideToggleUsb = false;
    shouldSendStatus = true;
  }
  if (status.otherSideUsbActive) status.usbActive = false;
  if (status.toggleUsb) shouldSendStatus = true;
  if (status.usbActive || status.otherSideUsbActive)
    setTimestamp(&status.lastActiveTimestamp);
  if (status.usbReady && !status.usbActive && !status.otherSideUsbActive) {
    if (status.commOK && status.mySide == leftSide && elapsed_ms(&status.lastActiveTimestamp, COMM_STATUS_DELAY_MS * 3))
      status.usbActive = true;
    if (elapsed_ms(&status.lastActiveTimestamp, COMM_STATUS_DELAY_MS * 6))
      status.usbActive = true;
    if (status.usbActive) shouldSendStatus = true;
  }
  if (shouldSendStatus) {
    comm_sendStatus();
  }
  status.toggleUsb = false;

  led_updateColor();
}


int main()
{
  USB usb;
  Controller controller;
  LocalReader localReader;

  update_now();
  hardware_init();
  usb_init(&usb);
  controller_init(&controller, &usb);
  Key_init(&controller);

  localReader_init(&localReader, &controller);
  status.mySide = localReader_keyboardSide(&localReader);
  if (status.mySide == noSide) fatal("Cannot determine keyboard side");
  status.otherSide = (status.mySide == leftSide) ? rightSide : leftSide;

  setUsbSide(noSide);

  while (true) {
    update_now();
    comm_task();
    localReader_readKeys(&localReader);
    if (status.usbActive) {
      controller_task(&controller);
    } else if (status.otherSideUsbActive) {
      Key_sendChangedKeys(status.mySide);
    }
    log_keys(status.mySide, localReader.hw_version);
    usb_task(&usb);
    synchronizeAndDecideUsbSide();
  }
}
#if 0
int xxmain() {

  stdio_set_translate_crlf(&stdio_usb, false);
  stdio_init_all();
  /*
  adc_init();
  gpio_init(29);
  gpio_set_dir(29, GPIO_OUT);
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  int v1, v2, v3;
  bool sel=true;
  for(;;) {
    gpio_put(29, sel);
    sleep_ms(200);
    sel = !sel;
    adc_select_input(0);
    v1 = adc_read();
    adc_select_input(1);
    v2 = adc_read();
    adc_select_input(2);
    v3 = adc_read();
    printf("%d %d %d\n", v1, v2, v3);
    printf("\n");
  }
  */
//  /*
  gpio_init_mask(0b00111100000000001111111111111100);
  gpio_pull_up(2);
  gpio_pull_up(3);
  gpio_pull_up(4);
  gpio_pull_up(5);
  gpio_pull_up(6);
  gpio_pull_up(7);
  gpio_pull_up(8);
  gpio_pull_up(9);
  gpio_pull_up(10);
  gpio_pull_up(11);
  gpio_pull_up(12);
  gpio_pull_up(13);
  gpio_pull_up(14);
  gpio_pull_up(15);
  gpio_pull_up(26);
  gpio_pull_up(27);
  gpio_pull_up(28);
  gpio_pull_up(29);
  gpio_set_dir_in_masked(0b00111100000000001111111111111100);
  for(;;) {
    uint32_t val = gpio_get_all();
    printf("%032b\n", val);
    sleep_ms(200);
  }
//  */
  /*
  gpio_init(pin);
  gpio_pull_up(pin);
  gpio_set_dir(pin, GPIO_IN);
  while (true) {
    while(gpio_get(pin));
    uint32_t t0 = time_us_32();
    for(int i=0; i<N; i++) {
      set(i, gpio_get(pin));
    }
    int n = 1;
    bool vv = false;
    int nn = 0;
    printf("%u\n", time_us_32()-t0);
    for(int i=0; i<N; i++) {
      if (get(i) == vv) {
        n++;
      } else {
        if (n < 50000) nn += n;
        else {
          printf("%d%d-%d ", !vv, vv, nn);
          nn = 0;
        }
        printf("%d-%d ", vv, n);
        n=1;
        vv=!vv;
      }
      //printf("%d", v[i]);
      //if(i%190 == 189) printf("\n");
    }
          printf("%d%d-%d ", !vv, vv, nn);
        printf("%d-%d ", vv, n);
    printf("\n");
    //sleep_ms(200);
  }
    */
}
#endif

