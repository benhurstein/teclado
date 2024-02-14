// vim: foldmethod=marker
// includes  {{{1
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

#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"

// constants+types+globals  {{{1
#define SENSITIVITY 6

#define UART_ID uart1
#define BAUD_RATE 500000
#define UART_TX_PIN 4
#define UART_RX_PIN 5

typedef enum { noSide, leftSide, rightSide } keyboardSide;
keyboardSide mySide = noSide;
keyboardSide otherSide = noSide;
keyboardSide usbSide = noSide;

typedef enum {
  COLEMAK,
  COL_ACC,
  QWERTY,
  QWE_ACC,
  NAV,
  MOUSE,
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
  K_NONE    = 0x00,  K_ERR1,    K_ERR2,    K_ERR3,
  K_A       = 0x04,  K_B,       K_C,       K_D,
  K_E       = 0x08,  K_F,       K_G,       K_H,
  K_I       = 0x0C,  K_J,       K_K,       K_L,
  K_M       = 0x10,  K_N,       K_O,       K_P,
  K_Q       = 0x14,  K_R,       K_S,       K_T,
  K_U       = 0x18,  K_V,       K_W,       K_X,
  K_Y       = 0x1C,  K_Z,       K_1,       K_2,
  K_3       = 0x20,  K_4,       K_5,       K_6,
  K_7       = 0x24,  K_8,       K_9,       K_0,
  K_ENT     = 0x28,  K_ESC,     K_BS,      K_TAB,
  K_SPC     = 0x2C,  K_MINUS,   K_EQUAL,   K_LBRAKT,
  K_RBRAKT  = 0x30,  K_BKSLASH, K_SHARP,   K_SMCOL,
  K_APOSTR  = 0x34,  K_GRAVE,   K_COMMA,   K_DOT,
  K_SLASH   = 0x38,  K_CAPS,    K_F1,      K_F2,
  K_F3      = 0x3C,  K_F4,      K_F5,      K_F6,
  K_F7      = 0x40,  K_F8,      K_F9,      K_F10,
  K_F11     = 0x44,  K_F12,     K_PRTSC,   K_SCRLK,
  K_PAUSE   = 0x48,  K_INSERT,  K_HOME,    K_PGUP,
  K_DEL     = 0x4C,  K_END,     K_PGDN,    K_RIGHT,
  K_LEFT    = 0x50,  K_DOWN,    K_UP,      K_NUMLK,
// keypad
  K_EURO2   = 0x64,  K_APP,     K_POWER,
// keypad, F13-24
  K_STOP    = 0x78,  K_REDO,    K_UNDO,    K_CUT,
  K_COPY    = 0x7C,  K_PASTE,   K_FIND,    K_MUTE,
  K_VOLUP   = 0x80,  K_VOLDOWN,
// etc
  K_CTRL    = 0xE0,  K_SHFT,    K_ALT,     K_GUI,
  K_RCTRL   = 0xE4,  K_RSHFT,   K_RALT,    K_RGUI,
  K_COMPOSE = K_RGUI,
} keycode_t;

typedef enum {
    but_left     = 0b00001,
    but_right    = 0b00010,
    but_middle   = 0b00100,
    but_backward = 0b01000,
    but_forward  = 0b10000,
} button_t;

// ascii to mod-key  {{{1
const struct mod_key { modifier_t mod; keycode_t key; } ascii_to_mod_key[] = {
  [0x00] = {0,    0         }, {0,    0       }, {0,    0       }, {0,    0       },
  [0x04] = {0,    0         }, {0,    0       }, {0,    0       }, {0,    0       },
  [0x08] = {0,    K_BS      }, {0,    K_TAB   }, {0,    K_ENT   }, {0,    0       },
  [0x0C] = {0,    0         }, {0,    0       }, {0,    0       }, {0,    0       },
  [0x10] = {0,    0         }, {0,    0       }, {0,    0       }, {0,    0       },
  [0x14] = {0,    0         }, {0,    0       }, {0,    0       }, {0,    0       },
  [0x18] = {0,    0         }, {0,    0       }, {0,    0       }, {0,    K_ESC   },
  [0x1C] = {0,    0         }, {0,    0       }, {0,    0       }, {0,    0       },
  [0x20] = {0,    K_SPC     }, {SHFT, K_1     }, {SHFT, K_APOSTR}, {SHFT, K_3     },// !"#
  [0x24] = {SHFT, K_4       }, {SHFT, K_5     }, {SHFT, K_7     }, {0,    K_APOSTR},//$%&'
  [0x28] = {SHFT, K_9       }, {SHFT, K_0     }, {SHFT, K_8     }, {SHFT, K_EQUAL },//()*+
  [0x2C] = {0,    K_COMMA   }, {0,    K_MINUS }, {0,    K_DOT   }, {0,    K_SLASH },//,-./
  [0x30] = {0,    K_0       }, {0,    K_1     }, {0,    K_2     }, {0,    K_3     },//0123
  [0x34] = {0,    K_4       }, {0,    K_5     }, {0,    K_6     }, {0,    K_7     },//4567
  [0x38] = {0,    K_8       }, {0,    K_9     }, {SHFT, K_SMCOL }, {0,    K_SMCOL },//89:;
  [0x3C] = {SHFT, K_COMMA   }, {0,    K_EQUAL }, {SHFT, K_DOT   }, {SHFT, K_SLASH },//<=>?
  [0x40] = {SHFT, K_2       }, {SHFT, K_A     }, {SHFT, K_B     }, {SHFT, K_C     },//@ABC
  [0x44] = {SHFT, K_D       }, {SHFT, K_E     }, {SHFT, K_F     }, {SHFT, K_G     },//DEFG
  [0x48] = {SHFT, K_H       }, {SHFT, K_I     }, {SHFT, K_J     }, {SHFT, K_K     },//HIJK
  [0x4C] = {SHFT, K_L       }, {SHFT, K_M     }, {SHFT, K_N     }, {SHFT, K_O     },//LMNO
  [0x50] = {SHFT, K_P       }, {SHFT, K_Q     }, {SHFT, K_R     }, {SHFT, K_S     },//PQRS
  [0x54] = {SHFT, K_T       }, {SHFT, K_U     }, {SHFT, K_V     }, {SHFT, K_W     },//TUVW
  [0x58] = {SHFT, K_X       }, {SHFT, K_Y     }, {SHFT, K_Z     }, {0,    K_LBRAKT},//XYZ[
  [0x5C] = {0,    K_BKSLASH }, {0,    K_RBRAKT}, {SHFT, K_6     }, {SHFT, K_MINUS },//\]^_
  [0x60] = {0,    K_GRAVE   }, {0,    K_A     }, {0,    K_B     }, {0,    K_C     },//`abc
  [0x64] = {0,    K_D       }, {0,    K_E     }, {0,    K_F     }, {0,    K_G     },//defg
  [0x68] = {0,    K_H       }, {0,    K_I     }, {0,    K_J     }, {0,    K_K     },//hijk
  [0x6C] = {0,    K_L       }, {0,    K_M     }, {0,    K_N     }, {0,    K_O     },//lmno
  [0x70] = {0,    K_P       }, {0,    K_Q     }, {0,    K_R     }, {0,    K_S     },//pqrs
  [0x74] = {0,    K_T       }, {0,    K_U     }, {0,    K_V     }, {0,    K_W     },//tuvw
  [0x78] = {0,    K_X       }, {0,    K_Y     }, {0,    K_Z     }, {SHFT, K_LBRAKT},//xyz{
  [0x7C] = {SHFT, K_BKSLASH }, {SHFT, K_RBRAKT}, {SHFT, K_GRAVE }, {0,    K_DEL   },//|}~
};


// definitions  {{{1
typedef struct controller Controller;
typedef struct key Key;
typedef struct action Action;
typedef struct usb USB;

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
void controller_setTimedAction(Controller *self, Key *key, Action action, int interval_ms);
void controller_keyPressed(Controller *self, Key *key);
void controller_keyReleased(Controller *self, Key *key);


// local sequence of calls:
//   newRaw -> calculateVal -> setVal -> valChanged
//      if master, can call controller_keyPressed or controller_keyReleased
//      if slave, sends val to master
// when master receives new val from slave, calls setVal
void key_init(Key *self, Controller *controller, uint8_t hwId, uint8_t swId);
int8_t key_id(Key *self);
keyboardSide key_side(Key *self);
void key_setNewRaw(Key *self, uint16_t newRaw);
void key_calculateVal(Key *self);
void key_setVal(Key *self, uint8_t newVal);
int8_t key_val(Key *self);
void key_setReleaseAction(Key *self, Action action);
Action *key_releaseAction(Key *self);
char *key_description(Key *self);
void key_setMinRawRange(Key *self, uint16_t range);

enum holdType { noHoldType, modHoldType, layerHoldType };
Action Action_noAction(void);
char *action_description(Action *a);
void action_actuate(Action *self, Key *key, Controller *controller);
bool action_isTypingAction(Action *self);
Action action_holdAction(Action *self);
Action action_tapAction(Action *self);
enum holdType action_holdType(Action *self);


// log  {{{1

#define LOG_E 0b00000001
#define LOG_I 0b00000010
#define LOG_R 0b00000100
#define LOG_U 0b00001000
#define LOG_C 0b00010000
#define LOG_T 0b00100000
#define LOG_K 0b01000000
#define LOG_L 0b10000000

uint8_t log_level = LOG_L;//LOG_E | LOG_T | LOG_R;

void log_set_level(uint8_t new_level)
{
  log_level = new_level;
}

#define log(level, ...) \
    if ((level & log_level)) { \
      printf(__VA_ARGS__); \
      putchar_raw('\n'); \
      fflush(stdout); \
      sleep_us(200); \
      tud_task(); \
    }

//

// Action  {{{1
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
  [no_action] = "no",
  [key_action] = "key",
  [asc_action] = "asc",
  [str_action] = "str",
  [mod_action] = "mod",
  [layer_action] = "layer",
  [base_layer_action] = "base_layer",
  [hold_layer_action] = "hold_layer",
  [once_layer_action] = "once_layer",
  [lock_layer_action] = "lock_layer",
  [key_or_mod_action] = "key_or_mod",
  [str_or_mod_action] = "str_or_mod",
  [key_or_layer_action] = "key_or_layer",
  [str_or_layer_action] = "str_or_layer",
  [mouse_move_action] = "mouse_move",
  [mouse_button_action] = "mouse_button",
  [command_action] = "command",

  [rel_key_action] = "rel_key",
  [rel_asc_action] = "rel_asc",
  [rel_mod_action] = "rel_mod",
  [rel_layer_action] = "rel_layer",
  [rel_once_layer_action] = "rel_once_layer",
  [rel_button_action] = "rel_button",
};

// extra data for each action
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
  enum { RESET, WORDLOCK } command;
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
void mouse_move_actuate(Action *self, Key *key, Controller *controller) {
  int val = key_val(key);
  if (key_val(key) != 0) {
    int h=0, v=0, wh=0, wv=0;
    int interval;
    switch (self->mouse_move.move) {
      case mv_up   : v  = -val; interval =  50 - 3*val; break;
      case mv_down : v  = +val; interval =  50 - 3*val; break;
      case mv_right: h  = +val; interval =  50 - 3*val; break;
      case mv_left : h  = -val; interval =  50 - 3*val; break;
      case wh_up   : wv =   +1; interval = 100; break;
      case wh_down : wv =   -1; interval = 100; break;
      case wh_right: wh =   +1; interval = 100; break;
      case wh_left : wh =   -1; interval = 100; break;
    }
    controller_moveMouse(controller, v, h, wv, wh);
    controller_setTimedAction(controller, key, *self, interval);
  }
  key_setReleaseAction(key, NO_ACTION);
}
void mouse_button_actuate(Action *self, Key *key, Controller *controller) {
  controller_pressMouseButton(controller, self->mouse_button.button);
  key_setReleaseAction(key, REB(self->mouse_button.button));
}
void command_actuate(Action *self, Key *key, Controller *controller) {
  controller_doCommand(controller, self->command.command);
  key_setReleaseAction(key, NO_ACTION);
}


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

#define ACTION_CASE(action) case action##_action: action##_actuate(self, key, controller); break
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
    case str_action:
      return true;
    default:
      return false;
  }
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


// layers  {{{1
Action layer[][36] = {
  [COLEMAK] = {
    KEY(K_Q       ), KEY(K_W       ), KEY(K_F       ), KEY(K_P       ), KEY(K_B       ),
    KOM(K_A,GUI   ), KOM(K_R,ALT   ), KOM(K_S,CTRL  ), KOM(K_T,SHFT  ), KEY(K_G       ),
    KEY(K_Z       ), KOM(K_X,RALT  ), KEY(K_C       ), KEY(K_D       ), KEY(K_V       ),
    KOL(K_ESC,MOUSE),KOL(K_SPC,NAV ), KOL(K_TAB,NUM ),
    KEY(K_J       ), KEY(K_L       ), KEY(K_U       ), KEY(K_Y       ), LA1(COL_ACC   ),
    KEY(K_M       ), KOM(K_N,SHFT  ), KOM(K_E,CTRL  ), KOM(K_I,ALT   ), KOM(K_O,GUI   ),
    KEY(K_K       ), KEY(K_H       ), KEY(K_COMMA   ), KOM(K_DOT,RALT), KEY(K_SLASH   ),
    KOL(K_ENT,COL_ACC), KOL(K_BS,SYM  ), KOL(K_DEL,FUN ),
  },
  [COL_ACC] = {
    STR("'"       ), ASC('"', '\'' ), KEY(K_F       ), STR("ª"       ), STR("BenhurUFSM"),
    STR("á"       ), STR("à"       ), KEY(K_S       ), KOM(K_T,SHFT  ), KEY(K_G       ),
    STR("â"       ), STR("ã"       ), STR("ç"       ), KEY(K_D       ), KEY(K_V       ),
    KOL(K_ESC,MOUSE),KOL(K_SPC,NAV ), KOL(K_TAB,NUM ),
    KEY(K_J       ), STR("º"       ), STR("ú"       ), KEY(K_APOSTR  ), KEY(K_COMPOSE ),
    KEY(K_M       ), SOM("ñ",SHFT  ), STR("é"       ), STR("í"       ), STR("ó"       ),
    KEY(K_K       ), KEY(K_H       ), STR("ê"       ), STR("õ"       ), STR("ô"       ),
    KOL(K_ENT,NUM2), KOL(K_BS,SYM  ), KOL(K_DEL,FUN ),
  },
  [QWERTY] = {
    KEY(K_Q       ), KEY(K_W       ), KEY(K_E       ), KEY(K_R       ), KEY(K_T       ),
    KOM(K_A,GUI   ), KOM(K_S,ALT   ), KOM(K_D,CTRL  ), KOM(K_F,SHFT  ), KEY(K_G       ),
    KEY(K_Z       ), KOM(K_X,RALT  ), KEY(K_C       ), KEY(K_V       ), KEY(K_B       ),
    KOL(K_ESC,MOUSE),KOL(K_SPC,NAV ), KOL(K_TAB,NUM ),
    KEY(K_Y       ), KEY(K_U       ), KEY(K_I       ), KEY(K_O       ), KEY(K_P       ),
    KEY(K_H       ), KOM(K_J,SHFT  ), KOM(K_K,CTRL  ), KOM(K_L,ALT   ), KOM(K_SMCOL,GUI),
    KEY(K_N       ), KEY(K_M       ), KEY(K_COMMA   ), KOM(K_DOT,RALT), KEY(K_SLASH   ),
    KOL(K_ENT,NUM2), KOL(K_BS,SYM  ), KOL(K_DEL,FUN ),
  },
  [MOUSE] = {
    COM(RESET     ), NO_ACTION,       BAS(QWERTY    ), BAS(COLEMAK   ), NO_ACTION,
    MOD(GUI       ), MOD(ALT       ), MOD(CTRL      ), MOD(SHFT      ), NO_ACTION,
    NO_ACTION,       MOD(RALT      ), LCK(FUN       ), LCK(MOUSE     ), NO_ACTION,
    NO_ACTION,       NO_ACTION,       NO_ACTION,
    KEY(K_VOLUP),    MOU(wh_left   ), MOU(mv_up     ), MOU(wh_right  ), MOU(wh_up     ),
    KEY(K_VOLDOWN),  MOU(mv_left   ), MOU(mv_down   ), MOU(mv_right  ), MOU(wh_down   ),
    KEY(K_MUTE    ), NO_ACTION,       NO_ACTION,       NO_ACTION,       NO_ACTION,
    BUT(but_right ), BUT(but_left  ), BUT(but_middle),
  },
  [NAV] = {
    NO_ACTION,       NO_ACTION,       BAS(QWERTY    ), BAS(COLEMAK   ), NO_ACTION,
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
    NO_ACTION,       BAS(COLEMAK   ), BAS(QWERTY    ), NO_ACTION,       NO_ACTION,
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
    NO_ACTION,       LCK(FUN       ), LCK(MOUSE     ), MOD(RALT      ), NO_ACTION,
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

// WS2812 rgb led  {{{1

#define WS2812_PIN 16
#define IS_RGBW true

static inline void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t pixel_grbw =
    ((uint32_t) (r) << 16) |
    ((uint32_t) (g) << 24) |
    ((uint32_t) (b) << 8);
  pio_sm_put_blocking(pio0, 0, pixel_grbw);
}

void led_init()
{
  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
  led_set_rgb(0, 0, 0);
}

// USB  {{{1
// interfaces with tinyUSB


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
    uint8_t keycode;
    uint8_t modifier;
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
  self->data[(self->first+self->count++)%KCQ_N] = data;
  if (self->count > self->max_count) {
    self->max_count = self->count;
    log(LOG_I, "max keycode queue count: %d", self->max_count);
  }
}

void keycodeq_insertKeycodePress(Keycodeq *self, keycode_t keycode)
{
  keycodeq_insertData(self, (struct kcq_data){keycodePress, keycode, 0});
}

void keycodeq_insertKeycodeRelease(Keycodeq *self, keycode_t keycode)
{
  keycodeq_insertData(self, (struct kcq_data){keycodeRelease, keycode, 0});
}

void keycodeq_insertModifierPress(Keycodeq *self, modifier_t modifier)
{
  keycodeq_insertData(self, (struct kcq_data){modifierPress, 0, modifier});
}

void keycodeq_insertModifierRelease(Keycodeq *self, modifier_t modifier)
{
  keycodeq_insertData(self, (struct kcq_data){modifierRelease, 0, modifier});
}

enum command keycodeq_head(Keycodeq *self)
{
  if (self->count == 0) return none;
  return self->data[self->first].command;
}

enum command keycodeq_remove(Keycodeq *self, keycode_t *keycode_p, modifier_t *modifier_p)
{
  if (self->count == 0) return none;
  enum command command = self->data[self->first].command;
  *keycode_p = self->data[self->first].keycode;
  *modifier_p = self->data[self->first].modifier;
  self->first = (self->first+1)%KCQ_N;
  self->count--;
  return command;
}
keycode_t keycodeq_removeKeycode(Keycodeq *self)
{
  if (self->count == 0) return 0;
  keycode_t keycode = self->data[self->first].keycode;
  self->first = (self->first+1)%KCQ_N;
  self->count--;
  return keycode;
}
modifier_t keycodeq_removeModifier(Keycodeq *self)
{
  if (self->count == 0) return 0;
  modifier_t modifier = self->data[self->first].modifier;
  self->first = (self->first+1)%KCQ_N;
  self->count--;
  return modifier;
}


struct usb {
  Keycodeq keycodeq;
  uint8_t keycodes[6];
  uint8_t n_keycodes;
  uint8_t modifiers;
  uint8_t sent_modifiers;
  button_t buttons;
  bool capsLocked;
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
  self->capsLocked = false;

  tusb_init();
}

void uart_send_usb_side()
{
  uart_putc_raw(UART_ID, 'M');
}

bool uart_receive_usb_side()
{
  if (!uart_is_readable(UART_ID)) return false;
  uint8_t c = uart_getc(UART_ID);
  if (c == 'M') return true;
  return false;
}

void usb_testConnection(USB *self)
{
  tud_task();
  if (usbSide != noSide) return;
  if (tud_connected()) {
    usbSide = mySide;
    uart_send_usb_side();
  } else if (uart_receive_usb_side()) {
    usbSide = otherSide;
  }
}

void USB_caps(bool locked)
{
  USB_singleton->capsLocked = locked;
}

bool keycode_is_modifier(keycode_t keycode)
{
  return keycode >= 0xE0 && keycode <= 0xE7;
}

modifier_t keycode_to_modifier(keycode_t keycode)
{
  return 1 << (keycode - 0xE0);
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

void usb_sendReport(USB *self)
{
  if (mySide == usbSide) {
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
  if (mySide == usbSide) {
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
  if (self->n_keycodes < i+1) return;
  self->n_keycodes--;
  memmove(&self->keycodes[i], &self->keycodes[i+1], self->n_keycodes - i);
  self->keycodes[self->n_keycodes] = 0;
}
void usb__removeKeycode(USB *self, keycode_t keycode)
{
  for (int i=0; i<self->n_keycodes; i++) {
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
  usb_sendReport(self);
}
void usb__sendModifierPresses(USB *self)
{
  while (keycodeq_head(&self->keycodeq) == modifierPress) {
    self->sent_modifiers |= keycodeq_removeModifier(&self->keycodeq);
  }
  usb_sendReport(self);
}
void usb__sendKeycodeReleases(USB *self)
{
  while (keycodeq_head(&self->keycodeq) == keycodeRelease) {
    usb__removeKeycode(self, keycodeq_removeKeycode(&self->keycodeq));
    break;
  }
  usb_sendReport(self);
}
void usb__sendModifierReleases(USB *self)
{
  enum command cmd = keycodeq_head(&self->keycodeq);
  while (keycodeq_head(&self->keycodeq) == modifierRelease) {
    self->sent_modifiers &= ~keycodeq_removeModifier(&self->keycodeq);
    break;
  }
  usb_sendReport(self);
}

bool usb_isCapsLocked(USB *self)
{
  return self->capsLocked;
}

void usb_task(USB *self)
{
  tud_task();
  //hid_task();
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


// UART  {{{1
void uart_send_key_val(uint8_t keyId, uint8_t val)
{
  uint8_t b0 = val+0xA0;
  uint8_t b1 = keyId+0x50;
  uint8_t b2 = b0 ^ b1;
  uart_putc_raw(UART_ID, 0);
  uart_putc_raw(UART_ID, b0);
  uart_putc_raw(UART_ID, b1);
  uart_putc_raw(UART_ID, b2);
}

bool uart_receive_key_val(uint8_t *keyIdp, uint8_t *valp)
{
  static uint8_t buffer[4];
  static uint8_t count;
  static int errct, recct;
  while (true) {
    if (!uart_is_readable(UART_ID)) break;
    uint8_t c = uart_getc(UART_ID);
    if (count == 0 && c != 0) {
      log(LOG_C, "Err comm0.%02x", c);
      continue;
    }
    buffer[count++] = c;
    if (count == 4) {
      recct++;
      count = 0;
      uint8_t b0 = buffer[1];
      uint8_t b1 = buffer[2];
      uint8_t b2 = buffer[3];
      if (b2 != (b0 ^ b1)) {
        errct++;
        log(LOG_C, "Err comm1: [%02x %02x %02x] %d/%d", b0, b1, b2, errct, recct);
        continue;
      }
      b0 -= 0xA0;
      b1 -= 0x50;
      if (b0 > 9 || b1 > 19) {
        errct++;
        log(LOG_C, "Err comm2: [%02x %02x %02x] %d/%d", b0, b1, b2, errct, recct);
        continue;
      }
      *keyIdp = b1;
      *valp = b0;
      return true;
    }
  }
  return false;
}


// Key  {{{1
// stores info about a key

struct key {
  Controller *controller;
  // low level id, depending on wiring, can be repeated on other side
  int8_t hwId;
  // higher level id, O-17 for left (0=Q,1=W), 18-35 for right (18=Y,19=U)
  int8_t swId;
  // last value read from sensor
  uint16_t newRaw;
  uint16_t minRawRange;
  // scaled values, for filtering
  uint32_t minRaw_S;
  uint32_t maxRaw_S;
  uint32_t filteredRaw_S;
  // current state, a 0 to 9 value and a pressed state
  int8_t val;
  bool pressed;
  // minimum value since key was released and maximum value since key was pressed
  //   a key press/release is recognized relative to these values
  int8_t minVal;
  int8_t maxVal;
  // what to do when key is actuated
  Action releaseAction;
};

char *key_description(Key *self)
{
  static char description[4];
  sprintf(description, "k%d", self->swId);
  return description;
}

void key_init(Key *self, Controller *controller, uint8_t hwId, uint8_t swId)
{
  self->controller = controller;
  self->hwId = hwId;
  self->swId = swId;
  self->filteredRaw_S = UINT32_MAX;
  self->val = 0;
  self->pressed = false;
  self->minVal = 0;
}

int8_t key_id(Key *self)
{
  return self->swId;
}

keyboardSide key_side(Key *self)
{
  if (self->swId >=  0 && self->swId <= 17) return leftSide;
  if (self->swId >= 18 && self->swId <= 35) return rightSide;
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
void key__filterRawValues(Key *self)
{
  if (self->filteredRaw_S == UINT32_MAX) {
    // if it's the first value, initialize
    self->filteredRaw_S = self->newRaw << 13;
    self->maxRaw_S = self->filteredRaw_S;
    self->minRaw_S = self->filteredRaw_S;
    return;
  }
  filter_SnS(&self->filteredRaw_S, self->newRaw, 13, 2);
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


void key__valChanged(Key *self)
{
  uint8_t newVal = self->val;
  if (self->swId == -1) return;
  if (key_side(self) == mySide && mySide != usbSide) {
    uart_send_key_val(self->hwId, self->val);
    return;
  }

  if (self->pressed) {
    self->maxVal = MAX(self->maxVal, newVal);
    if (self->maxVal - newVal >= SENSITIVITY) {
      self->pressed = false;
      controller_keyReleased(self->controller, self);
      self->minVal = newVal;
    }
  } else {
    self->minVal = MIN(self->minVal, newVal);
    if (newVal - self->minVal >= SENSITIVITY) {
      self->pressed = true;
      controller_keyPressed(self->controller, self);
      self->maxVal = newVal;
    }
  }
  log(LOG_K, "valChanged -> m%d M%d p%d", self->minVal, self->maxVal, self->pressed);
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
  if (newVal != self->val) {
    log(LOG_K, "newVal %d %d->%d", self->swId, self->val, newVal);
    self->val = newVal;
    key__valChanged(self);
  }
}

static int constrain(int val, int minimum, int maximum)
{
  if (val < minimum) return minimum;
  if (val > maximum) return maximum;
  return val;
}

void key_calculateVal(Key *self)
{
  key__filterRawValues(self);
  if (self->swId == -1) return;
  int minRaw = self->minRaw_S >> 13;
  int maxRaw = self->maxRaw_S >> 13;
  int rawRange = maxRaw - minRaw;
  int newRaw = self->newRaw;
  if (rawRange < self->minRawRange) return;
  int val_90 = constrain((newRaw - minRaw) * 100 / rawRange, 0, 90);
  if (abs(val_90 - self->val * 10) > 6) {
    key_setVal(self, (val_90 + 5) / 10);
  }
}

void key_setNewRaw(Key *self, uint16_t newRaw)
{
  self->newRaw = newRaw;
}


// KeyList  {{{1
typedef struct key_list_node KeyListNode;
typedef struct key_list KeyList;

struct key_list_node {
  Key *key;
  KeyListNode *next;
};

KeyListNode *KeyListNode_create(Key *key, KeyListNode *next)
{
  KeyListNode *node = malloc(sizeof(KeyListNode));
  if (node != NULL) {
    node->key = key;
    node->next = next;
  }
  return node;
}

struct key_list {
  KeyListNode *first;
  KeyListNode *last;
};

KeyList *KeyList_create(void)
{
  KeyList *self = malloc(sizeof(KeyList));
  if (self != NULL) {
    self->first = NULL;
    self->last = NULL;
  }
  return self;
}

void keyList_print(KeyList *self)
{
  printf("[");
  for (KeyListNode *node = self->first; node != NULL; node = node->next) {
    printf("%s ", key_description(node->key));
  }
  printf("]\n");
}

bool keyList_empty(KeyList *self)
{
  return self->first == NULL;
}

Key *keyList_firstKey(KeyList *self)
{
  if (keyList_empty(self)) return NULL;
  return self->first->key;
}
Key *keyList_removeFirstKey(KeyList *self)
{
  if (keyList_empty(self)) return NULL;
  KeyListNode *node = self->first;
  Key *key = node->key;
  self->first = node->next;
  if (self->first == NULL) self->last = NULL;
  free(node);
  return key;
}

void keyList_destroy(KeyList *self)
{
  while (!keyList_empty(self)) {
    keyList_removeFirstKey(self);
  }
  free(self);
}

void keyList_insertKey(KeyList *self, Key *key)
{
//  printf("before insert %p(%d): ", key, key->swId); keyList_print(self);
  KeyListNode *node = KeyListNode_create(key, NULL);
  assert(node != NULL); // BADABOOM!!!
  if (keyList_empty(self)) {
    self->first = self->last = node;
  } else {
    self->last->next = node;
    self->last = node;
  }
//  printf("after insert: "); keyList_print(self);
}

void keyList_removeKey(KeyList *self, Key *key)
{
  if (keyList_empty(self)) return;
  if (self->first->key == key) {
    keyList_removeFirstKey(self);
  } else {
    KeyListNode *prev = self->first;
    while (true) {
      KeyListNode *node = prev->next;
      if (node == NULL) break;
      if (node->key == key) {
        prev->next = node->next;
        if (prev->next == NULL) self->last = prev;
        free(node);
        break;
      }
      prev = node;
    }
  }
}

bool keyList_containsKey(KeyList *self, Key *key)
{
  for (KeyListNode *node = self->first; node != NULL; node = node->next) {
//    printf("testing %p(%d) and %p(%d)\n", key, key->swId, node->key, node->key->swId);
    if (node->key == key) return true;
  }
  return false;
}

// auxiliary functions for unicode  {{{1
//   very basic support for á->Á and finding a codepoint in a utf8 str
uint32_t to_majus(uint32_t minus)
{
  uint16_t majus = minus;
  if (minus >= 0x61 && minus <= 0x7e) majus = minus - 0x20;
  else if (minus >= 0xe0 && minus <= 0xfe && minus != 0xf7) majus = minus - 0x20;
  else if (minus == 0xff) majus = 0x178;
  else if (minus >= 0x100 && minus <= 0x137 && (minus&1) == 1) majus = minus - 1;
  else if (minus >= 0x139 && minus <= 0x148 && (minus&1) == 0) majus = minus - 1;
  else if (minus >= 0x14a && minus <= 0x177 && (minus&1) == 1) majus = minus - 1;
  else if (minus >= 0x179 && minus <= 0x17e && (minus&1) == 0) majus = minus - 1;
  // TODO: missing cases
  return majus;
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
           | ((p[2] & 0b00111111) << 6)
           |  (p[3] & 0b00111111);
    default:
      return 0; // error!
  }
}



// Controller  {{{1
// controls the processing of keypresses

struct controller {
  layer_id_t currentLayer;
  layer_id_t baseLayer;
  layer_id_t lockLayer;
  KeyList *keysOnHold;
  KeyList *keysBeingHeld;
  USB *usb;
  enum holdType holdType;
  keyboardSide holdSide;
  Action timedAction;
  uint32_t timedTimestamp;
  Key *timedKey;
  Action delayedReleaseAction;
  uint32_t holdTimeout;
  modifier_t modifiers;
  bool wordLocked;
};

void controller_init(Controller *self, USB *usb)
{
  self->usb = usb;
  self->currentLayer = self->baseLayer = COLEMAK;
  self->lockLayer = NO_LAYER;
  self->keysOnHold = KeyList_create();
  self->keysBeingHeld = KeyList_create();
  self->holdType = noHoldType;
  self->holdSide = noSide;
  self->timedTimestamp = 0;
  self->delayedReleaseAction = Action_noAction();
  self->modifiers = 0;
  self->wordLocked = false;
}

// auxiliary functions for wordLock  {{{2
bool uni_in_word(uint32_t uni)
{
  if (uni == '_') return true;
  if (uni >= '0' && uni <= '9') return true;
  if (uni >= 'a' && uni <= 'z') return true;
  if (uni >= 'A' && uni <= 'Z') return true;
  if (uni != to_majus(uni) /*|| uni != to_minus(uni)*/) return true;
  return false;
}

bool keycode_in_word(keycode_t keycode, bool shifted)
{
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

void static controller__setWordLocked(Controller *self, bool newVal)
{
  self->wordLocked = newVal;
  led_set_rgb(0, 0, self->wordLocked ? 20 : 0);
}

void controller__set_modifiers(Controller *self, modifier_t new_modifiers)
{
  self->modifiers = new_modifiers;
  usb_setModifiers(self->usb, new_modifiers);
}

void controller__add_modifiers(Controller *self, modifier_t new_modifiers)
{
  controller__set_modifiers(self, self->modifiers | new_modifiers);
}

void controller__remove_modifiers(Controller *self, modifier_t new_modifiers)
{
  controller__set_modifiers(self, self->modifiers & ~new_modifiers);
}

bool controller__is_shifted(Controller *self)
{
  return (self->modifiers & (SHFT|RSHFT)) != 0;
}

void controller__send_usb_press_keycode(Controller *self, keycode_t keycode)
{
  usb_pressKeycode(self->usb, keycode);
}

void controller__send_usb_release_keycode(Controller *self, keycode_t keycode)
{
  usb_releaseKeycode(self->usb, keycode);
}

void controller__send_press_keycode(Controller *self, keycode_t keycode)
{
  if (self->wordLocked && !keycode_in_word(keycode, controller__is_shifted(self))) {
    controller__setWordLocked(self, false);
  }
  if (self->wordLocked && keycode_in_word_invert_shift(keycode)) {
    usb_setModifiers(self->usb, self->modifiers ^ SHFT);
  } else {
    usb_setModifiers(self->usb, self->modifiers);
  }
  controller__send_usb_press_keycode(self, keycode);
  usb_setModifiers(self->usb, self->modifiers);
}

void controller__send_release_keycode(Controller *self, keycode_t keycode)
{
  usb_setModifiers(self->usb, self->modifiers);
  controller__send_usb_release_keycode(self, keycode);
}

void controller_changeLayer(Controller *self, layer_id_t layer)
{
  log(LOG_T, "%s(%d)", __func__, layer);
  if (self->lockLayer == NO_LAYER) {
    self->currentLayer = layer;
  }
  log(LOG_T, "layer=%d base=%d lock=%d", self->currentLayer, self->baseLayer, self->lockLayer);
}

void controller_lockLayer(Controller *self, layer_id_t layer)
{
  log(LOG_T, "%s(%d)", __func__, layer);
  if (self->lockLayer == layer) {
    self->lockLayer = NO_LAYER;
    self->currentLayer = self->baseLayer;
  } else {
    self->currentLayer = layer;
    self->lockLayer = layer;
  }
}

void controller_changeBaseLayer(Controller *self, layer_id_t layer)
{
  log(LOG_T, "%s(%d)", __func__, layer);
  self->baseLayer = layer;
}

layer_id_t controller_baseLayer(Controller *self)
{
  return self->baseLayer;
}

void controller__pressKey(Controller *self, Key *key)
{
  Action action = layer[self->currentLayer][key_id(key)];
  log(LOG_T, "pressKey %s %s", key_description(key), action_description(&action));
  key_setReleaseAction(key, Action_noAction()); // just in case...
  if (key_side(key) == self->holdSide) {
    if (action_isTypingAction(&action)) {
      log(LOG_T, "ignoring typing key on same side of held key");
      return;
    }
    keyList_insertKey(self->keysBeingHeld, key);
    action = action_holdAction(&action);
    log(LOG_T, "hold: %s", action_description(&action));
  } else {
    action = action_tapAction(&action);
    log(LOG_T, "tap: %s", action_description(&action));
  }
  action_actuate(&action, key, self);
}

void controller__releaseKey(Controller *self, Key *key)
{
  log(LOG_T, "releaseKey %s %s", key_description(key), action_description(key_releaseAction(key)));
  keyList_removeKey(self->keysBeingHeld, key);
  if (keyList_empty(self->keysBeingHeld)) {
    self->holdSide = noSide;
  }
  action_actuate(key_releaseAction(key), key, self);
  key_setReleaseAction(key, Action_noAction());
}

void controller__resetHoldTimeout(Controller *self)
{
  if (!keyList_empty(self->keysOnHold)) {
    self->holdTimeout = board_millis() + 333;
    if (self->holdTimeout == 0) self->holdTimeout++;
  } else {
    self->holdTimeout = 0;
  }
}

void controller_keyPressed(Controller *self, Key *key)
{
  log(LOG_T, "keyPressed: %s", key_description(key));
  if (!keyList_empty(self->keysOnHold)) {
    log(LOG_T, " on hold2");
    keyList_insertKey(self->keysOnHold, key);
    controller__resetHoldTimeout(self);
  } else {
    Action *action = &layer[self->currentLayer][key_id(key)];
    if (action_holdType(action) == noHoldType) {
      log(LOG_T, " press action: %s", action_description(action));
      controller__pressKey(self, key);
    } else {
      log(LOG_T, "on hold1");
      keyList_insertKey(self->keysOnHold, key);
      controller__resetHoldTimeout(self);
    }
  }
}

void controller_holdKeysOnHold(Controller *self)
{
  self->holdSide = key_side(keyList_firstKey(self->keysOnHold));
  while (!keyList_empty(self->keysOnHold)) {
    Key *key = keyList_removeFirstKey(self->keysOnHold);
    controller__pressKey(self, key);
  }
}

void controller_holdKeysOnHoldUntilKey(Controller *self, Key *lastKey)
{
  self->holdSide = key_side(keyList_firstKey(self->keysOnHold));
  while (!keyList_empty(self->keysOnHold)) {
    Key *key = keyList_removeFirstKey(self->keysOnHold);
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
  if (keyList_containsKey(self->keysOnHold, key)) {
    log(LOG_T, " was on hold");
    Key *firstKey = keyList_firstKey(self->keysOnHold);
    if (firstKey == key) { // it's a tap
      log(LOG_T, " it's a tap (%s)", key_description(key));
      keyList_removeFirstKey(self->keysOnHold);
      controller__pressKey(self, key);
      controller__releaseKey(self, key);
      controller__resetHoldTimeout(self);
    } else { // it's a hold
      log(LOG_T, " it's a hold (%s, first %s)", key_description(key), key_description(firstKey));
      controller_holdKeysOnHoldUntilKey(self, key);
      controller__releaseKey(self, key);
      controller__resetHoldTimeout(self);
    }
  } else {
    log(LOG_T, " was not held");
    controller__releaseKey(self, key);
  }
  log(LOG_T, " delayed action %s", action_description(&delayedAction));
  action_actuate(&delayedAction, key, self);
}

void controller_pressKeycode(Controller *self, keycode_t keycode)
{
  log(LOG_T, "%s(%d)", __func__, keycode);
  controller__send_press_keycode(self, keycode);
}

void controller_releaseKeycode(Controller *self, keycode_t keycode)
{
  log(LOG_T, "%s(%d)", __func__, keycode);
  controller__send_release_keycode(self, keycode);
}


void controller__send_usb_press_ascii_char(Controller *self, uint8_t ch)
{
  struct mod_key mk = ascii_to_mod_key[ch];
  if (mk.key != 0) {
    usb_setModifiers(self->usb, mk.mod|(self->modifiers & ~(SHFT|RSHFT)));
    controller__send_usb_press_keycode(self, mk.key);
  }
}
void controller__send_usb_release_ascii_char(Controller *self, uint8_t ch)
{
  struct mod_key mk = ascii_to_mod_key[ch];
  if (mk.key != 0) {
    controller__send_usb_release_keycode(self, mk.key);
  }
}

void controller__send_usb_hex_nibble(Controller *self, uint8_t h)
{
  keycode_t k;
  if      (h == 0) k = K_0;
  else if (h < 10) k = K_1 + h - 1;
  else             k = K_A + h - 10;
  controller__send_usb_press_keycode(self, k);
  controller__send_usb_release_keycode(self, k);
}

void controller__send_usb_hex(Controller *self, uint32_t hex)
{
  bool sent = false;
  for (int n = 7; n >= 0; n--) {
    uint8_t nib = (hex >> n*4) & 0b1111;
    if (nib != 0 || sent || n == 0) {
      controller__send_usb_hex_nibble(self, nib);
      sent = true;
    }
  }
}
void controller__send_usb_unicode_char(Controller *self, uint32_t uni)
{
  if (uni < 128) {
    controller__send_usb_press_ascii_char(self, uni);
    controller__send_usb_release_ascii_char(self, uni);
  } else {
    // send C-S-u unicode in hex — this works in linux
    usb_setModifiers(self->usb, RSHFT|RCTRL);
    controller__send_usb_press_keycode(self, K_U);
    controller__send_usb_release_keycode(self, K_U);
    usb_setModifiers(self->usb, 0);
    controller__send_usb_hex(self, uni);
    controller__send_usb_press_keycode(self, K_ENT);
    controller__send_usb_release_keycode(self, K_ENT);
  }
}

void controller__send_utf8_str(Controller *self, char s[])
{
  bool capsLocked = usb_isCapsLocked(self->usb);
  bool shifted = controller__is_shifted(self);
  if (capsLocked) {
    controller__send_usb_press_keycode(self, K_CAPS);
    controller__send_usb_release_keycode(self, K_CAPS);
  }
  log(LOG_T, "shift:%d(%02x) caps:%d", shifted, self->usb->sent_modifiers, capsLocked);
  while (true) {
    uint32_t u = unicode_from_utf8(s);
    if (u == 0) break;
    if (self->wordLocked && !uni_in_word(u)) controller__setWordLocked(self, false);
    if (shifted ^ capsLocked ^ self->wordLocked) u = to_majus(u);
    controller__send_usb_unicode_char(self, u);
    s += utf8_nbytes(s);
  }
  if (capsLocked) {
    controller__send_usb_press_keycode(self, K_CAPS);
    controller__send_usb_release_keycode(self, K_CAPS);
  }
  usb_setModifiers(self->usb, self->modifiers);
}

uint8_t controller__send_press_ascii_char(Controller *self, uint8_t ch)
{
  if (self->wordLocked && !uni_in_word(ch)) controller__setWordLocked(self, false);
  if (self->wordLocked) ch = to_majus(ch);
  controller__send_usb_press_ascii_char(self, ch);
  usb_setModifiers(self->usb, self->modifiers);
  return ch;
}

void controller__send_release_ascii_char(Controller *self, uint8_t ch)
{
  controller__send_usb_release_ascii_char(self, ch);
  usb_setModifiers(self->usb, self->modifiers);
}

char controller_pressAscii(Controller *self, char unshifted_char, char shifted_char)
{
  log(LOG_T, "%s(%c,%c)S=%d", __func__, unshifted_char, shifted_char, controller__is_shifted(self));
  char pressed_char = controller__is_shifted(self) ? shifted_char : unshifted_char;
  return controller__send_press_ascii_char(self, pressed_char);
}

void controller_releaseAscii(Controller *self, char pressed)
{
  log(LOG_T, "%s(%c)", __func__, pressed);
  controller__send_release_ascii_char(self, pressed);
}

void controller_pressString(Controller *self, char s[])
{
  log(LOG_T, "%s(%c)", __func__, s[0]);
  controller__send_utf8_str(self, s);
}

void controller_pressModifier(Controller *self, modifier_t modifier)
{
  log(LOG_T, "%s(%d)", __func__, modifier);
  controller__add_modifiers(self, modifier);
}
void controller_releaseModifier(Controller *self, modifier_t modifier)
{
  log(LOG_T, "%s(%d)", __func__, modifier);
  controller__remove_modifiers(self, modifier);
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
  //log(LOG_T, "controller move %d %d %d %d\n", v, h, wv, wh);
  usb_moveMouse(self->usb, v, h, wv, wh);
}
void controller_setDelayedReleaseAction(Controller *self, Action action)
{
  log(LOG_T, "set delayed");
  log(LOG_T, "%s: %s", __func__, action_description(&action));
  self->delayedReleaseAction = action;
}
void controller_doCommand(Controller *self, int command)
{
  if (command == WORDLOCK) {
    controller__setWordLocked(self, !self->wordLocked);
    return;
  } else if (command == RESET) {
    reset_usb_boot(0, 0);
    return;
  }
  printf("%s(%d) not implemented\n", __func__, command);
  printf("Layers: current=%d base=%d\n", self->currentLayer, self->baseLayer);
  printf("on hold: "); keyList_print(self->keysOnHold);
  printf("being held: "); keyList_print(self->keysBeingHeld);
  self->currentLayer = self->baseLayer = COLEMAK;
}
void controller_setTimedAction(Controller *self, Key *key, Action action, int interval_ms)
{
  // TODO there should be multiple simultaneous timeds, one per key
  self->timedAction = action;
  self->timedTimestamp = board_millis() + interval_ms;
  if (self->timedTimestamp == 0) self->timedTimestamp++;
  self->timedKey = key;
}

void controller_task(Controller *self)
{
  if (self->timedTimestamp != 0 && board_millis() >= self->timedTimestamp) {
    self->timedTimestamp = 0;
    action_actuate(&(self->timedAction), self->timedKey, self);
  }
  if (self->holdTimeout != 0 && board_millis() >= self->holdTimeout) {
    self->holdTimeout = 0;
    log(LOG_T, "hold timeout");
    controller_holdKeysOnHold(self);
  }
}

// LocalReader  {{{1
// reads the keys physically connected to local microcontroller
typedef struct {
  #define N_SEL 5
  #define N_ANA 4
  #define N_HWKEY (N_SEL*N_ANA)
  uint8_t sel_pin[N_SEL];
  Key keys[N_HWKEY];
  keyboardSide side;
} LocalReader;

void init_keys(Key keys[N_HWKEY], keyboardSide side, Controller *controller)
{
  int8_t leftHwIdToSwId[N_HWKEY] = {
    17, 14,  9,  4, 16, 13,  8,  3, 15, 12,
     7,  2, -1, 11,  6,  1, -1, 10,  5,  0,
  };
  int8_t rightHwIdToSwId[N_HWKEY] = {
    -1, 32, 27, 22, -1, 31, 26, 21, 34, 30,
    25, 20, 35, 29, 24, 19, 33, 28, 23, 18,
  };
  int8_t *hwIdToSwId = side == leftSide ? leftHwIdToSwId : rightHwIdToSwId;
  for (int8_t hwId = 0; hwId < N_HWKEY; hwId++) {
    int8_t swId = hwIdToSwId[hwId];
    key_init(&keys[hwId], controller, hwId, swId);
    key_setMinRawRange(&keys[hwId], 50);
  }
}

void localReader__initADC(LocalReader *self)
{
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_gpio_init(28);
  adc_gpio_init(29);
}

void localReader__initGPIO(LocalReader *self)
{
  for (int i=0; i < N_SEL; i++) {
    uint pin = self->sel_pin[i];
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
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

static keyboardSide readKeyboardSide()
{
  // on the right keyboard half, pin 2 is connected to pin 1;
  // on the left half, it is connected to pin 3
  gpio_init(2);
  gpio_set_dir(2, GPIO_IN);
  gpio_init(1);
  gpio_init(3);
  gpio_set_dir(1, GPIO_OUT);
  gpio_set_dir(3, GPIO_OUT);
  gpio_put(1, 0);
  gpio_put(3, 1);
  sleep_us(10);
  keyboardSide side = gpio_get(2) ? leftSide : rightSide;
  gpio_deinit(2);
  gpio_deinit(1);
  gpio_deinit(3);
  return side;
}

void localReader_initSide(LocalReader *self)
{
  self->side = readKeyboardSide();
  if (self->side == leftSide) {
    self->sel_pin[0] = 14;
    self->sel_pin[1] = 15;
    self->sel_pin[2] = 3;
    self->sel_pin[3] = 1;
    self->sel_pin[4] = 0;
  } else {
    self->sel_pin[0] = 0;
    self->sel_pin[1] = 1;
    self->sel_pin[2] = 3;
    self->sel_pin[3] = 6;
    self->sel_pin[4] = 7;
  }
  localReader__initGPIO(self);
}

void localReader_init(LocalReader *self, Controller *controller)
{
  localReader__initADC(self);
  localReader_initSide(self);
  init_keys(self->keys, self->side, controller);
}

keyboardSide localReader_keyboardSide(LocalReader *self)
{
  return self->side;
}

Key *localReader_keys(LocalReader *self)
{
  return self->keys;
}

void localReader_readKeys(LocalReader *self)
{
  uint8_t keyId = 0;
  Key *key = &self->keys[0];
  for (int sel=0; sel < N_SEL; sel++) {
    uint pin = self->sel_pin[sel];
    gpio_put(pin, 1);
    for (int ana=0; ana < N_ANA; ana++) {
      adc_select_input(ana);
      uint16_t raw = adc_read();
      key_setNewRaw(&self->keys[keyId], raw);
      keyId++;
    }
    gpio_put(pin, 0);
    sleep_us(80);
  }
  for (keyId=0; keyId < N_HWKEY; keyId++) {
    key_calculateVal(&self->keys[keyId]);
  }
}


// RemoteReader  {{{1
typedef struct {
  Key keys[N_HWKEY];
} RemoteReader;

void remoteReader_init(RemoteReader *self, Controller *controller, keyboardSide side)
{
  init_keys(self->keys, side, controller);
}

void remoteReader_readKeys(RemoteReader *self)
{
  uint8_t val;
  uint8_t keyId;
  while (uart_receive_key_val(&keyId, &val)) {
    Key *key = &self->keys[keyId];
    log(LOG_C, "rec %d %d", keyId, val);
    key_setVal(key, val);
  }
}


// main  {{{1
void log_keys(Key *keys)
{
  static uint32_t t0 = 0;
  static uint16_t ct = 0;

  if ((log_level & LOG_L) != 0) {
    ct++;
    if (ct == 1000) {
      uint32_t t1 = time_us_32();
      printf("%s(%d) ", mySide == leftSide ? "LEFT" : "RIGHT", mySide == usbSide);
      printf("%uHz\n", ct * 1000000u / (t1 - t0));
      for (int i = 0; i < N_HWKEY; i++) {
        printf("%5u", keys[i].minRaw_S >> 13);
      }
      putchar_raw('\n');
      for (int i = 0; i < N_HWKEY; i++) {
        printf("%5u", keys[i].maxRaw_S >> 13);
      }
      printf("\n");
      for (int i = 0; i < N_HWKEY; i++) {
        printf("%5u", (keys[i].maxRaw_S - keys[i].minRaw_S) >> 13);
      }
      printf("\n");
      for (int i = 0; i < N_HWKEY; i++) {
        printf("%5u", keys[i].newRaw);
      }
      printf("\n");
      fflush(stdout);
      t0 = time_us_32();
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
  board_init();
  stdio_init_all();
  stdio_set_translate_crlf(&stdio_usb, false);

  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  led_init();
}

int main()
{
  USB usb;
  Controller controller;
  LocalReader localReader;
  RemoteReader remoteReader;

  hardware_init();
  usb_init(&usb);
  controller_init(&controller, &usb);

  localReader_init(&localReader, &controller);
  mySide = localReader_keyboardSide(&localReader);
  if (mySide == noSide) fatal("Cannot determine keyboard side");
  otherSide = mySide == leftSide ? rightSide : leftSide;
  remoteReader_init(&remoteReader, &controller, otherSide);

  uint32_t t0 = time_us_32();
  while (usbSide == noSide && time_us_32() - t0 < 1000000) {
    usb_testConnection(&usb);
  }
  if (usbSide == noSide) fatal("Did not detect USB connection");

  while (true) {
    localReader_readKeys(&localReader);
    if (mySide == usbSide) {
      remoteReader_readKeys(&remoteReader);
      controller_task(&controller);
      usb_task(&usb);
    }

    log_keys(localReader_keys(&localReader));
  }
  return 0;
}

// USB callbacks  {{{1
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
        board_led_write(true);
        USB_caps(true);
      }else
      {
        // Caplocks Off: back to normal blink
        printf("capslock off\n");
        USB_caps(false);
        board_led_write(false);
      }
    }
  }
}

