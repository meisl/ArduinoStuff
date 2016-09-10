#ifndef List_h
#define List_h

#include "Arduino.h"  // could use <angle brackets> instead of "double quotes"
#include "Command.h"  // MUST use "double quotes" for this

template <typename T> class List;

template <typename T> List<T> *cons(T x, List<T> *xs = NULL) {
  T *xPtr = (T*)malloc(sizeof(T));
  *xPtr = x;
  return new List<T>(xPtr, xs);  
}

class ListBase {
  public:
    virtual void freeAll() {
      // do nothing
    }
  
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
      return cons(value);
    }
};

static const ListBase *NIL = new ListBase();

void print(ListBase *a) {
  Serial.print("()");
}

void println(ListBase *xs) {
  print(xs);
  Serial.println();
}

template <typename T> void print(List<T> *a) {
  Serial.print("(");
  if (a != NIL) {
    while (1) {
      Serial.print(a->head());
      List<T> *b = a->tail();
      if (b == NIL) {
        break;
      }
      Serial.print(", ");
      a = b;
    }
  }
  Serial.print(")");  
}

template <typename T> void println(List<T> *xs) {
  print(xs);
  Serial.println();
}

template <typename T> class List : public ListBase {
  protected:
    T *valuePtr;
    ListBase *next;
  
  public:

    List(T *_valuePtr, List<T> *tail = NULL) : valuePtr(_valuePtr) {
      next = (tail == NULL) ? NIL : tail;
    }

/*
    List(T _value, List<T> *tail = NULL) {
      valuePtr = (T*)malloc(sizeof(T));
      *valuePtr = _value;
      next = (tail == NULL) ? NIL : tail;
    }
*/
    virtual void freeAll() {
      tail()->freeAll();
      free(this);
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
        return new List<T>(valuePtr, cons(value)); // cannot dispatch to NIL->append
      } else {
        return new List<T>(valuePtr, t->append(value));
      }
    }

};


#endif // List_h
