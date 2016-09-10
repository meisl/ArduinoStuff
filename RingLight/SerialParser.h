#ifndef SerialParser_h
#define SerialParser_h

#include "Arduino.h"  // could use <angle brackets> instead of "double quotes"
#include "Command.h"  // MUST use "double quotes" for this

#define STATE_LINE_START    0
#define STATE_LINE_END      1
#define STATE_OPTIONAL_ARG  2
#define STATE_NEXT_DIGIT    3
#define STATE_SKIP_TIL_EOL  4
#define STATE_ERROR         5


class ListBase {
  protected:
    virtual void _print() {
    }

  public:
    ListBase() {
    }
    
    virtual ListBase* tail() {
      return this;
    }
    
    virtual uint16_t length() {
      return 0;  
    }
    
    void print() { // TODO: make more obvious
      Serial.print("(");
      ListBase *a = this, *b = a->tail();
      if (a != b) {
        while (1) {
          a->_print();
          a = b;
          b = b->tail();
          if (a == b) {
            break;
          } else {
            Serial.print(", ");
          }
        }
      }
      Serial.print(")");
    }
    
    void println() {
      print();
      Serial.println();
    }
};

ListBase Empty;
ListBase *NIL = &Empty;

template <typename T> class List : public ListBase {

  protected:
    void _print() {
      Serial.print(this->head());
    }
    
    T *valuePtr;
    ListBase *next;
  
  public:
    List(T *_valuePtr, List<T> *tail = (List<T>*)NIL) : valuePtr(_valuePtr), next(tail) {
    }

    List(T _value, List<T> *tail = (List<T>*)NIL) : next(tail) {
      valuePtr = (T*)malloc(sizeof(T));
      *valuePtr = _value;
    }

    uint16_t length() {
      return 1 + next->length();
    }

    T head() {
      return *valuePtr;
    }
      
    List<T> *tail() {
      return (List<T>*)next;
    }    
      
    List<T> *append(T value) {
      return new List(value, this);
    }

};


template <typename T> List<T> *cons(T x, List<T> *xs) {
  return new List<T>(x, xs);  
}

ListBase foo = ListBase();


#endif // SerialParser_h
