#if FEATURE_RULES
//********************************************************************************************
//Rules processing
//********************************************************************************************
String rulesProcessing(String fileName, String& event)
{
  String log = F("EVT: ");
  log += event;
  telnetLog(log);

  fs::File f = SPIFFS.open(fileName, "r+");

  static byte nestingLevel = 0;
  int data = 0;

  nestingLevel++;
  if (nestingLevel > RULES_MAX_NESTING_LEVEL)
  {
    log = F("EVENT: Error: Nesting level exceeded!");
    nestingLevel--;
    return (log);
  }

  String line = "";
  boolean match = false;
  boolean codeBlock = false;
  boolean isCommand = false;
  boolean conditional = false;
  boolean condition = false;
  boolean ifBranche = false;

  byte buf[RULES_BUFFER_SIZE];
  int len = 0;
  while (f.available())
  {
    len = f.read((byte*)buf, RULES_BUFFER_SIZE);
    for (int x = 0; x < len; x++) {
      data = buf[x];

      if (data != 10)
        line += char(data);

      if (data == 10)    // if line complete, parse this rule
      {
        line.replace("\r", "");
        if (line.substring(0, 2) != "//" && line.length() > 0)
        {
          isCommand = true;

          int comment = line.indexOf("//");
          if (comment > 0)
            line = line.substring(0, comment);

          if (match || !codeBlock) {
            // only parse [xxx#yyy] if we have a matching ruleblock or need to eval the "on" (no codeBlock)
            // This to avoid waisting CPU time...
            line = parseTemplate(line, line.length());
          }
          line.trim();

          String lineOrg = line; // store original line for future use
          line.toLowerCase(); // convert all to lower case to make checks easier


          String eventTrigger = "";
          String action = "";

          if (!codeBlock)  // do not check "on" rules if a block of actions is to be processed
          {
            if (line.startsWith("on "))
            {
              line = line.substring(3);
              int split = line.indexOf(" do");
              if (split != -1)
              {
                eventTrigger = line.substring(0, split);
                action = lineOrg.substring(split + 7);
                action.trim();
              }
              if (eventTrigger == "*") // wildcard, always process
                match = true;
              else
                match = ruleMatch(event, eventTrigger);
              if (action.length() > 0) // single on/do/action line, no block
              {
                isCommand = true;
                codeBlock = false;
              }
              else
              {
                isCommand = false;
                codeBlock = true;
              }
            }
          }
          else
          {
            action = lineOrg;
          }

          String lcAction = action;
          lcAction.toLowerCase();
          if (lcAction == "endon") // Check if action block has ended, then we will wait for a new "on" rule
          {
            isCommand = false;
            codeBlock = false;
            match = false;
          }

          if (match) // rule matched for one action or a block of actions
          {
            int split = lcAction.indexOf("if "); // check for optional "if" condition
            if (split != -1)
            {
              conditional = true;
              String check = lcAction.substring(split + 3);
              condition = conditionMatchExtended(check);
              ifBranche = true;
              isCommand = false;
            }

            if (lcAction == "else") // in case of an "else" block of actions, set ifBranche to false
            {
              ifBranche = false;
              isCommand = false;
            }

            if (lcAction == "endif") // conditional block ends here
            {
              conditional = false;
              isCommand = false;
            }

            // process the action if it's a command and unconditional, or conditional and the condition matches the if or else block.
            if (isCommand && ((!conditional) || (conditional && (condition == ifBranche))))
            {
              action.replace(F("%event%"), event); // substitute %event% with literal event string
              int equalsPos = event.indexOf("=");
              if (equalsPos > 0){
                String tmpString = event.substring(equalsPos + 1);
                action.replace(F("%eventvalue%"), tmpString); // substitute %eventvalue% in actions with the actual value from the event
              }

              String log = F("ACT: ");
              log += action;
              telnetLog(log);

              delay(0);
              ExecuteCommand(action.c_str());
              delay(0);
            }
          }
        }

        line = "";
      }
    }
  }

  nestingLevel--;
  return (String());
}


/********************************************************************************************\
  Check if an event matches to a given rule
  \*********************************************************************************************/
boolean ruleMatch(String& event, String& rule)
{
  boolean match = false;
  String tmpEvent = event;
  String tmpRule = rule;

  int pos = rule.indexOf('*');
  if (pos != -1) // a * sign in rule, so use a'wildcard' match on message
    {
      tmpEvent = event.substring(0, pos - 1);
      tmpRule = rule.substring(0, pos - 1);
      return tmpEvent.equalsIgnoreCase(tmpRule);
    }
  /*
    todo if needed

    if (event.startsWith("Clock#Time")) // clock events need different handling...
    {
      int pos1 = event.indexOf("=");
      int pos2 = rule.indexOf("=");
      if (pos1 > 0 && pos2 > 0)
      {
        tmpEvent = event.substring(0, pos1);
        tmpRule  = rule.substring(0, pos2);
        if (tmpRule.equalsIgnoreCase(tmpEvent)) // if this is a clock rule
        {
          tmpEvent = event.substring(pos1 + 1);
          tmpRule  = rule.substring(pos2 + 1);
          unsigned long clockEvent = string2TimeLong(tmpEvent);
          unsigned long clockSet = string2TimeLong(tmpRule);
          if (matchClockEvent(clockEvent, clockSet))
            return true;
          else
            return false;
        }
      }
    }
  */

  // parse event into verb and value
  float value = 0;
  pos = event.indexOf("=");
  if (pos)
  {
    tmpEvent = event.substring(pos + 1);
    value = tmpEvent.toFloat();
    tmpEvent = event.substring(0, pos);
  }

  // parse rule
  int comparePos = 0;
  char compare = ' ';
  comparePos = rule.indexOf(">");
  if (comparePos > 0)
  {
    compare = '>';
  }
  else
  {
    comparePos = rule.indexOf("<");
    if (comparePos > 0)
    {
      compare = '<';
    }
    else
    {
      comparePos = rule.indexOf("=");
      if (comparePos > 0)
      {
        compare = '=';
      }
    }
  }

  float ruleValue = 0;

  if (comparePos > 0)
  {
    tmpRule = rule.substring(comparePos + 1);
    ruleValue = tmpRule.toFloat();
    tmpRule = rule.substring(0, comparePos);
  }

  switch (compare)
  {
    case '>':
      if (tmpRule.equalsIgnoreCase(tmpEvent) && value > ruleValue)
        match = true;
      break;

    case '<':
      if (tmpRule.equalsIgnoreCase(tmpEvent) && value < ruleValue)
        match = true;
      break;

    case '=':
      if (tmpRule.equalsIgnoreCase(tmpEvent) && value == ruleValue)
        match = true;
      break;

    case ' ':
      if (tmpRule.equalsIgnoreCase(tmpEvent))
        match = true;
      break;
  }
  return match;
}


/********************************************************************************************\
  Check expression
  \*********************************************************************************************/

boolean conditionMatchExtended(String& check) {
  int condAnd = -1;
  int condOr = -1;
  boolean rightcond = false;
  boolean leftcond = conditionMatch(check); // initial check

  do {
    condAnd = check.indexOf(F(" and "));
    condOr  = check.indexOf(F(" or "));

    if (condAnd > 0 || condOr > 0) { // we got AND/OR
      if (condAnd > 0 && ((condOr < 0 && condOr < condAnd) || (condOr > 0 && condOr > condAnd))) { //AND is first
        check = check.substring(condAnd + 5);
        rightcond = conditionMatch(check);
        leftcond = (leftcond && rightcond);
      } else { //OR is first
        check = check.substring(condOr + 4);
        rightcond = conditionMatch(check);
        leftcond = (leftcond || rightcond);
      }
    }
  } while (condAnd > 0 || condOr > 0);
  return leftcond;
}

boolean conditionMatch(const String& check)
{
  boolean match = false;

  char compare    = ' ';

  int posStart = check.length();
  int posEnd = posStart;
  int comparePos  = 0;

  if ((comparePos = check.indexOf("!=")) > 0 && comparePos < posStart) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '!' + '=';
  }
  if ((comparePos = check.indexOf("<>")) > 0 && comparePos < posStart) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '!' + '=';
  }
  if ((comparePos = check.indexOf(">=")) > 0 && comparePos < posStart) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '>' + '=';
  }
  if ((comparePos = check.indexOf("<=")) > 0 && comparePos < posStart) {
    posStart = comparePos;
    posEnd = posStart + 2;
    compare = '<' + '=';
  }
  if ((comparePos = check.indexOf("<")) > 0 && comparePos < posStart) {
    posStart = comparePos;
    posEnd = posStart + 1;
    compare = '<';
  }
  if ((comparePos = check.indexOf(">")) > 0 && comparePos < posStart) {
    posStart = comparePos;
    posEnd = posStart + 1;
    compare = '>';
  }
  if ((comparePos = check.indexOf("=")) > 0 && comparePos < posStart) {
    posStart = comparePos;
    posEnd = posStart + 1;
    compare = '=';
  }

  float Value1 = 0;
  float Value2 = 0;

  if (compare > ' ')
  {
    String tmpCheck1 = check.substring(0, posStart);
    String tmpCheck2 = check.substring(posEnd);
    // todo if (!isFloat(tmpCheck1) || !isFloat(tmpCheck2)) {
    //    Value1 = timeStringToSeconds(tmpCheck1);
    //    Value2 = timeStringToSeconds(tmpCheck2);
    //} else {
    Value1 = tmpCheck1.toFloat();
    Value2 = tmpCheck2.toFloat();
    //}
  }
  else
    return false;

  switch (compare)
  {
    case '>'+'=':
      if (Value1 >= Value2)
        match = true;
      break;

    case '<'+'=':
      if (Value1 <= Value2)
        match = true;
      break;

    case '!'+'=':
      if (Value1 != Value2)
        match = true;
      break;

    case '>':
      if (Value1 > Value2)
        match = true;
      break;

    case '<':
      if (Value1 < Value2)
        match = true;
      break;

    case '=':
      if (Value1 == Value2)
        match = true;
      break;
  }
  return match;
}

/********************************************************************************************\
  Parse string template
  \*********************************************************************************************/
String parseTemplate(String &tmpString, byte lineSize)
{
  String newString = tmpString;

  // check named uservars
  for (byte x = 0; x < USER_VAR_MAX; x++) {
    String varname = "%" + nUserVar[x].Name + "%";
    String svalue = toString(nUserVar[x].Value, nUserVar[x].Decimals);
    newString.replace(varname, svalue);
  }

  // check named uservar strings
  for (byte x = 0; x < USER_STRING_VAR_MAX; x++) {
    String varname = "%" + sUserVar[x].Name + "%";
    String svalue = String(sUserVar[x].Value);
    newString.replace(varname, svalue);
  }

  newString.replace(F("%systime%"), getTimeString(':'));
  newString.replace(F("%sysname%"), Settings.Name);
  return newString;
}

String getTimeString(char delimiter)
{
  String reply;
  if (hour() < 10)
    reply += F("0");
  reply += String(hour());
  if (delimiter != '\0')
    reply += delimiter;
  if (minute() < 10)
    reply += F("0");
  reply += minute();
  //if (delimiter != '\0')
  //  reply += delimiter;
  //if (second() < 10)
  //  reply += F("0");
  //reply += second();
  return reply;
}

/********************************************************************************************\
  Check rule timers
  \*********************************************************************************************/
void rulesTimers()
{
  for (byte x = 0; x < RULES_TIMER_MAX; x++)
  {
    if (RulesTimer[x].Value != 0L) // timer active?
    {
      if (timeOutReached(RulesTimer[x].Value)) // timer finished?
      {
        RulesTimer[x].Value = 0L; // turn off this timer
        String event = F("Timer#");
        event += RulesTimer[x].Name;
        rulesProcessing(FILE_RULES, event);
      }
    }
  }
}

// Return the time difference as a signed value, taking into account the timers may overflow.
// Returned timediff is between -24.9 days and +24.9 days.
// Returned value is positive when "next" is after "prev"
long timeDiff(unsigned long prev, unsigned long next)
{
  long signed_diff = 0;
  // To cast a value to a signed long, the difference may not exceed half the ULONG_MAX
  const unsigned long half_max_unsigned_long = 2147483647u; // = 2^31 -1
  if (next >= prev) {
    const unsigned long diff = next - prev;
    if (diff <= half_max_unsigned_long) {
      // Normal situation, just return the difference.
      // Difference is a positive value.
      signed_diff = static_cast<long>(diff);
    } else {
      // prev has overflow, return a negative difference value
      signed_diff = static_cast<long>((ULONG_MAX - next) + prev + 1u);
      signed_diff = -1 * signed_diff;
    }
  } else {
    // next < prev
    const unsigned long diff = prev - next;
    if (diff <= half_max_unsigned_long) {
      // Normal situation, return a negative difference value
      signed_diff = static_cast<long>(diff);
      signed_diff = -1 * signed_diff;
    } else {
      // next has overflow, return a positive difference value
      signed_diff = static_cast<long>((ULONG_MAX - prev) + next + 1u);
    }
  }
  return signed_diff;
}

// Compute the number of milliSeconds passed since timestamp given.
// N.B. value can be negative if the timestamp has not yet been reached.
long timePassedSince(unsigned long timestamp) {
  return timeDiff(timestamp, millis());
}

// Check if a certain timeout has been reached.
boolean timeOutReached(unsigned long timer)
{
  const long passed = timePassedSince(timer);
  return passed >= 0;
}

/********************************************************************************************\
  Calculate function for simple expressions
  \*********************************************************************************************/
#define CALCULATE_OK                            0
#define CALCULATE_ERROR_STACK_OVERFLOW          1
#define CALCULATE_ERROR_BAD_OPERATOR            2
#define CALCULATE_ERROR_PARENTHESES_MISMATCHED  3
#define CALCULATE_ERROR_UNKNOWN_TOKEN           4
#define STACK_SIZE 10 // was 50
#define TOKEN_MAX 20

float globalstack[STACK_SIZE];
float *sp = globalstack - 1;
float *sp_max = &globalstack[STACK_SIZE - 1];

#define is_operator(c)  (c == '+' || c == '-' || c == '*' || c == '/' || c == '^')

int push(float value)
{
  if (sp != sp_max) // Full
  {
    *(++sp) = value;
    return 0;
  }
  else
    return CALCULATE_ERROR_STACK_OVERFLOW;
}

float pop()
{
  if (sp != (globalstack - 1)) // empty
    return *(sp--);
  else
    return 0.0;
}

float apply_operator(char op, float first, float second)
{
  switch (op)
  {
    case '+':
      return first + second;
    case '-':
      return first - second;
    case '*':
      return first * second;
    case '/':
      return first / second;
    case '^':
      return pow(first, second);
    default:
      return 0;
  }
}

char *next_token(char *linep)
{
  while (isspace(*(linep++)));
  while (*linep && !isspace(*(linep++)));
  return linep;
}

int RPNCalculate(char* token)
{
  if (token[0] == 0)
    return 0; // geen moeite doen voor een lege string

  if (is_operator(token[0]) && token[1] == 0)
  {
    float second = pop();
    float first = pop();

    if (push(apply_operator(token[0], first, second)))
      return CALCULATE_ERROR_STACK_OVERFLOW;
  }
  else // Als er nog een is, dan deze ophalen
    if (push(atof(token))) // is het een waarde, dan op de stack plaatsen
      return CALCULATE_ERROR_STACK_OVERFLOW;

  return 0;
}

// operators
// precedence   operators         associativity
// 3            !                 right to left
// 2            * / %             left to right
// 1            + - ^             left to right
int op_preced(const char c)
{
  switch (c)
  {
    case '^':
      return 3;
    case '*':
    case '/':
      return 2;
    case '+':
    case '-':
      return 1;
  }
  return 0;
}

bool op_left_assoc(const char c)
{
  switch (c)
  {
    case '^':
    case '*':
    case '/':
    case '+':
    case '-':
      return true;     // left to right
      //case '!': return false;    // right to left
  }
  return false;
}

unsigned int op_arg_count(const char c)
{
  switch (c)
  {
    case '^':
    case '*':
    case '/':
    case '+':
    case '-':
      return 2;
      //case '!': return 1;
  }
  return 0;
}


int Calculate(const char *input, float* result)
{
  const char *strpos = input, *strend = input + strlen(input);
  char token[25];
  char c, oc, *TokenPos = token;
  char stack[32];       // operator stack
  unsigned int sl = 0;  // stack length
  char     sc;          // used for record stack element
  int error = 0;

  //*sp=0; // bug, it stops calculating after 50 times
  sp = globalstack - 1;
  oc = c = 0;
  while (strpos < strend)
  {
    // read one token from the input stream
    oc = c;
    c = *strpos;
    if (c != ' ')
    {
      // If the token is a number (identifier), then add it to the token queue.
      if ((c >= '0' && c <= '9') || c == '.' || (c == '-' && is_operator(oc)))
      {
        *TokenPos = c;
        ++TokenPos;
      }

      // If the token is an operator, op1, then:
      else if (is_operator(c))
      {
        *(TokenPos) = 0;
        error = RPNCalculate(token);
        TokenPos = token;
        if (error)return error;
        while (sl > 0)
        {
          sc = stack[sl - 1];
          // While there is an operator token, op2, at the top of the stack
          // op1 is left-associative and its precedence is less than or equal to that of op2,
          // or op1 has precedence less than that of op2,
          // The differing operator priority decides pop / push
          // If 2 operators have equal priority then associativity decides.
          if (is_operator(sc) && ((op_left_assoc(c) && (op_preced(c) <= op_preced(sc))) || (op_preced(c) < op_preced(sc))))
          {
            // Pop op2 off the stack, onto the token queue;
            *TokenPos = sc;
            ++TokenPos;
            *(TokenPos) = 0;
            error = RPNCalculate(token);
            TokenPos = token;
            if (error)return error;
            sl--;
          }
          else
            break;
        }
        // push op1 onto the stack.
        stack[sl] = c;
        ++sl;
      }
      // If the token is a left parenthesis, then push it onto the stack.
      else if (c == '(')
      {
        stack[sl] = c;
        ++sl;
      }
      // If the token is a right parenthesis:
      else if (c == ')')
      {
        bool pe = false;
        // Until the token at the top of the stack is a left parenthesis,
        // pop operators off the stack onto the token queue
        while (sl > 0)
        {
          *(TokenPos) = 0;
          error = RPNCalculate(token);
          TokenPos = token;
          if (error)return error;
          sc = stack[sl - 1];
          if (sc == '(')
          {
            pe = true;
            break;
          }
          else
          {
            *TokenPos = sc;
            ++TokenPos;
            sl--;
          }
        }
        // If the stack runs out without finding a left parenthesis, then there are mismatched parentheses.
        if (!pe)
          return CALCULATE_ERROR_PARENTHESES_MISMATCHED;

        // Pop the left parenthesis from the stack, but not onto the token queue.
        sl--;

        // If the token at the top of the stack is a function token, pop it onto the token queue.
        if (sl > 0)
          sc = stack[sl - 1];

      }
      else
        return CALCULATE_ERROR_UNKNOWN_TOKEN;
    }
    ++strpos;
  }
  // When there are no more tokens to read:
  // While there are still operator tokens in the stack:
  while (sl > 0)
  {
    sc = stack[sl - 1];
    if (sc == '(' || sc == ')')
      return CALCULATE_ERROR_PARENTHESES_MISMATCHED;

    *(TokenPos) = 0;
    error = RPNCalculate(token);
    TokenPos = token;
    if (error)return error;
    *TokenPos = sc;
    ++TokenPos;
    --sl;
  }

  *(TokenPos) = 0;
  error = RPNCalculate(token);
  TokenPos = token;
  if (error)
  {
    *result = 0;
    return error;
  }
  *result = *sp;
  return CALCULATE_OK;
}
#endif

