#include "debounceButton.h"
#include <Arduino.h>

unsigned long debounceButton::currentTime = 0;

unsigned long debounceButton::debounceTime = 20; // the debounce time, increase if the output flickers
unsigned long debounceButton::klickTime = 180; // the timelimit after which two klicks are counted as a doubleklick
unsigned long debounceButton::doubleKlickTime = 200; // the timelimit after which two klicks are counted as a doubleklick

std::list<debounceButton *>debounceButton::buttons;

std::function<int(int)> debounceButton::readFunction  = [](int port) { return digitalRead(port); };
debounceButton::debounceButton(int p)
{
  port = p;
  previousChangeTime = lastPinChangeTime=changeTime=millis();
  lastKlickOrDoubleKlick = 0;
  lastPinState=false;
  doubleClick = false;
}
void debounceButton::init(bool ui)
{
  useInterrupt = ui;
  pinMode(port,INPUT_PULLUP);
  if(useInterrupt)
  {
    attachInterrupt(digitalPinToInterrupt(port), debounceButton::handleInterrupt, CHANGE);
  }
  buttons.push_back(this);
}

void debounceButton::update()
{
  currentTime = millis();
  for(const auto &button :buttons)
  {
    // an interrupt might have been missed if(!button->useInterrupt)
      button->updateState();
  }
}
void debounceButton::handleInterrupt()
{
  currentTime = millis();
  for(const auto &button :buttons)
      button->updateState();
}
void debounceButton::updateState()
{
  pinState = !readFunction(port);
  if ((currentTime - lastPinChangeTime) > debounceTime)
  {
    if (buttonState != pinState)
    {
      if (pinState == false && (currentTime - changeTime) < klickTime)
      {
        if ((currentTime - lastKlickOrDoubleKlick) < doubleKlickTime)
        {
          doubleClick = true;
        }
        lastKlickOrDoubleKlick = currentTime;
      }
      previousChangeTime = changeTime;
      changeTime = currentTime;
      buttonState = pinState;
    }
  }
  if(pinState != lastPinState)
  {
    lastPinChangeTime = currentTime;
  }

  if(lastKlickOrDoubleKlick != 0 && (currentTime - lastKlickOrDoubleKlick) >= doubleKlickTime)
  {
    lastKlickOrDoubleKlick =0;
    wasKlickedState = true;
  }
  lastPinState = pinState;
}


bool debounceButton::isPressed()
{
  if(port < 0)
  {
    return false;
  }
  if(buttonState && (currentTime - changeTime) >= doubleKlickTime)
  {
    lastKlickOrDoubleKlick = 0;
    return true;
  }
  return false;
}
bool debounceButton::wasKlicked()
{
  if(port < 0)
  {
    return false;
  }
  if(wasKlickedState)
  {
    wasKlickedState=false;
    return true;
  }
  return false;
}
bool debounceButton::wasDoubleKlicked()
{
  if(port < 0)
  {
    return false;
  }
  if (doubleClick)
  {
    doubleClick=false;
    lastKlickOrDoubleKlick =0;
    return true;
  }
  return false;
}
