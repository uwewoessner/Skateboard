#ifndef _DEBOUNCE_BUTTON_H
#define _DEBOUNCE_BUTTON_H
#include <functional>
#include <list>

class debounceButton
{
public:
  debounceButton(int port);
  void init(bool useInterrupts);
  bool wasKlicked();
  bool wasDoubleKlicked();
  bool isPressed();
  static void handleInterrupt();
  static void update();
  static unsigned long debounceTime;
  static unsigned long klickTime;
  static unsigned long doubleKlickTime;
  static std::function<int(int)> readFunction;
  bool buttonState=false;
  bool useInterrupt=false;
  bool wasKlickedState=false;

private:
  void updateState();
  static std::list<debounceButton *>buttons;
  int port;
  bool lastPinState;
  bool pinState;
  unsigned long lastPinChangeTime;
  unsigned long changeTime;
  unsigned long previousChangeTime;
  unsigned long lastKlickOrDoubleKlick;
  bool doubleClick;
  static unsigned long currentTime;
};

#endif