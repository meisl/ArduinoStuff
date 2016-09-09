#ifndef Command_h
#define Command_h

#include "Arduino.h"

#define CMDFLAG_NONE        0
#define CMDFLAG_TOGGLE      _BV(0)
#define CMDFLAG_WRAPAROUND  _BV(1)
#define CMDFLAG_HASLIMITS   _BV(2)
#define CMDFLAG_CALLBACK    _BV(3)

struct cmd_t {
  const byte id;
  const byte flags;
  const char chr;
  const char* name;
  union {
    volatile byte *_byte;
    volatile bool *_bool;
    volatile int  *_int;
    void (*_func)();
  } valuePtr;
  int  vmin, vmax;

   cmd_t(
    const byte _id, 
    const char _chr, 
    const char* _name,
    volatile bool* _valuePtr
  ) : id(_id), flags(CMDFLAG_TOGGLE | CMDFLAG_HASLIMITS | CMDFLAG_WRAPAROUND), chr(_chr), name(_name), vmin(0), vmax(1) {
    valuePtr._bool = _valuePtr;
  }

  template< typename T > cmd_t(
    const byte _id, 
    const char _chr, 
    const char* _name,
    T* _valuePtr,
    const int _min,
    const int _max,
    const byte _flags
  ) : id(_id), flags(_flags | CMDFLAG_HASLIMITS), chr(_chr), name(_name), vmin(_min), vmax(_max)
  {
    if (sizeof(T) == 1) {
      valuePtr._byte = _valuePtr;  
    } else if (sizeof(T) == 2) {
      valuePtr._int = (int*)_valuePtr;  
    }
  }

  cmd_t(
    const byte _id, 
    const char _chr, 
    const char* _name,
    void (*_func)()
  ) : id(_id), flags(CMDFLAG_CALLBACK), chr(_chr), name(_name)
  {
    valuePtr._func = _func;
  }

  void operator()() {
    if (flags & CMDFLAG_CALLBACK) {
      return valuePtr._func();
    }
  };

  cmd_t *toggle() {
    if (flags & CMDFLAG_TOGGLE) {
      inc(1);  
    }
    return this;
  };

  cmd_t *report() {
    Serial.print(name);
    Serial.print(": ");
    if (flags & CMDFLAG_TOGGLE) {
      Serial.println(*valuePtr._bool ? "on" : "off");
    } else {
      Serial.println(*valuePtr._int);  
    }
    return this;
  };
  template< typename T > T set(T v) {
    if (flags & CMDFLAG_HASLIMITS) {
      if (flags & CMDFLAG_WRAPAROUND) {
        T delta = (T)vmax - (T)vmin + 1;
        while (v > (T)vmax) {
          v -= delta;
        }
        while (v < (T)vmin) {
          v += delta;
        }
      } else {
        if (v > (T)vmax) {
          v = (T)vmax;
        } else if (v < (T)vmin) {
          v = (T)vmin;
        }
      }
    }
    if (sizeof(T) == 1) {
      *valuePtr._byte = v;
    } else if (sizeof(T) == 2) {
      *valuePtr._int = v;
    }
    return v;
  };
  template< typename T > T inc(T delta) {
    if (sizeof(T) == 1) {
      return set(delta + *valuePtr._byte);
    } else if (sizeof(T) == 2) {
      return set(delta + *valuePtr._int);
    }
    return 0;
  };
};

#endif // Command_h

