#ifndef Command_h
#define Command_h

#include "Arduino.h"

#define CMDFLAG_NONE        0
#define CMDFLAG_TOGGLE      _BV(0)
#define CMDFLAG_WRAPAROUND  _BV(1)
#define CMDFLAG_HASLIMITS   _BV(2)
#define CMDFLAG_CALLBACK    _BV(3)

typedef void (*CommandCallback)();

class CommandBase {
  public:
    CommandBase(const byte _id, const char _chr, const char* _name, const byte _flags) 
    : id(_id), chr(_chr), name(_name), flags(_flags)
    {}

  protected:
    const byte  id;
    const char  chr;
    const char* name;
    const byte  flags;
};

class CallbackCommand : public CommandBase {
  public:
    CallbackCommand(const byte _id, const char _chr, const char* _name, CommandCallback _callback)
    : CommandBase(_id, _chr, _name, CMDFLAG_CALLBACK), callback(_callback) {
    }

  void operator()() {
    callback();
  };
  
  protected:
    CommandCallback callback;
  
};

template <typename T> class ValueCommand : public CommandBase {
  public:
    ValueCommand(
      const byte _id, 
      const char _chr, 
      const char* _name, 
      volatile T* _valuePtr, 
      const byte _flags
    ) : CommandBase(_id, _chr, _name, _flags), valuePtr(_valuePtr) {
    }

    ValueCommand(
      const byte _id,
      const char _chr,
      const char* _name,
      volatile T* _valuePtr,
      T limitA,
      T limitB,
      const byte _flags
    ) : CommandBase(_id, _chr, _name, _flags | CMDFLAG_HASLIMITS), valuePtr(_valuePtr) {
      vmin = min(limitA, limitB);
      vmax = max(limitA, limitB);
    }
    
    void set(T v) {
      if (this->flags & CMDFLAG_HASLIMITS) {
        if (this->flags & CMDFLAG_WRAPAROUND) {
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
      *valuePtr = v;  
    }
  
    void inc(T delta) {
      set(delta + *valuePtr);
    };
  
    void printValue() {
      Serial.print(*valuePtr);
    }

    ValueCommand* report() {
      Serial.print(name);
      Serial.print(": ");
      printValue();
      Serial.println();
      return this;
    };

    protected:
      volatile T* valuePtr;
      T vmin, vmax;
};


class BoolCommand : public ValueCommand<bool> {
  public:
    BoolCommand(const byte _id, const char _chr, const char* _name, volatile bool* _valuePtr)
    : ValueCommand(_id, _chr, _name, _valuePtr, false, true, CMDFLAG_TOGGLE | CMDFLAG_WRAPAROUND) {
    }

    void printValue() {
      Serial.print(*valuePtr ? "on" : "off");  
    }
    
    BoolCommand *toggle() {
      *valuePtr = !*valuePtr;
      return this;
    };
};

#endif // Command_h
