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

template <typename T> class List;

class ListBase {
  protected:
    virtual void _print() {
    }

  public:
    virtual bool isEmpty() {
      return true;  
    }
    
    virtual ListBase* tail() {
      return this;
    }
    
    virtual uint16_t length() {
      return 0;  
    }
      
    template <typename T> List<T> *append(T value) {
      return new List<T>(value);
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

ListBase *NIL = new ListBase();



template <typename T> List<T> *cons(T x, List<T> *xs) {
  return new List<T>(x, xs);  
}

template <typename T> class List : public ListBase {

  protected:
    void _print() {
      Serial.print(this->head());
    }
    
    T *valuePtr;
    ListBase *next;
  
  public:
    List(T *_valuePtr, List<T> *tail = NULL) : valuePtr(_valuePtr) {
      next = (tail == NULL) ? NIL : tail;
    }

    List(T _value, List<T> *tail = NULL) {
      valuePtr = (T*)malloc(sizeof(T));
      *valuePtr = _value;
      next = (tail == NULL) ? NIL : tail;
    }

    bool isEmpty() {
      return false;
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
      
    virtual List<T> *append(T value) {
      List<T> *t = tail();
      if (t == NIL) {
        return new List<T>(head(), new List<T>(value)); // cannot dispatch to NIL->append
      } else {
        return new List<T>(head(), t->append(value));
      }
    }

};


#endif // SerialParser_h
