#ifndef Parser_h
#define Parser_h

#include "Arduino.h"  // could use <angle brackets> instead of "double quotes"
#include "Command.h"  // MUST use "double quotes" for this
#include "List.h"     // MUST use "double quotes" for this

#define STATE_LINE_START    0
#define STATE_LINE_END      1
#define STATE_OPTIONAL_ARG  2
#define STATE_NEXT_DIGIT    3
#define STATE_SKIP_TIL_EOL  4
#define STATE_ERROR         5

typedef int (*nextCharFunc_t)();

class Parser {
  private:
    byte state;
    List<CommandBase> *cmds;
    nextCharFunc_t nextChar;
  
  public:
    Parser(nextCharFunc_t _nextChar) 
    : state(STATE_LINE_START), cmds((List<CommandBase>*)NIL), nextChar(_nextChar) {
    }

    Parser *addCmd(CommandBase *c) {
      
      cmds = new List<CommandBase>(c, cmds);  //  cons(c , cmds); //
      /*bool foo = false;
      c = new BoolCommand(0, 'x', "xxx", &foo);
      cmds = new List<CommandBase>(c);
      */
      return this;  
    }

};



#endif // Parser_h
