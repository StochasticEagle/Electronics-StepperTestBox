#pragma once

#include <Arduino.h>

class Button {
public:
    typedef void (*Callback)();

    // Constructor: specify pin, active low or high logic, debounce interval, long press duration
    Button(uint8_t pin, bool activeLow = true, unsigned long debounceMs = 50, unsigned long longPressMs = 1000)
      : onShortPressPress(nullptr), 
        onShortPressRelease(nullptr),
        onLongPressPress(nullptr), 
        onLongPressRelease(nullptr),
        _pin(pin), 
        _activeLow(activeLow), 
        _debounceDelay(debounceMs), 
        _longPressDelay(longPressMs),
        _lastReading(false), 
        _lastDebounceTime(0), 
        _pressStartTime(0), 
        _longPressTriggered(false),
        _pressed(false)
    {
        pinMode(_pin, INPUT_PULLUP);
    }

    // Call this periodically in loop() to update button state and detect events
    void update()
    {
        bool reading = digitalRead(_pin);
        if (_activeLow) reading = !reading;

        unsigned long now = millis();

        // Debounce handling
        if (reading != _lastReading) {
            _lastDebounceTime = now;
        }

        _lastReading = reading;

        if ((now - _lastDebounceTime) > _debounceDelay) {
            // State stable enough to consider
            if (reading != _pressed) {
                _pressed = reading;
                if (_pressed) {
                    // Button just pressed
                    _pressStartTime = now;
                    _longPressTriggered = false;
                    if (onShortPressPress) onShortPressPress();
                } else {
                    // Button just released
                    // If long press not triggered, is short press release
                    if (!_longPressTriggered) {
                        if (onShortPressRelease) onShortPressRelease();
                    } else {
                        if (onLongPressRelease) onLongPressRelease();
                    }
                }
            } else if (_pressed && !_longPressTriggered) {
                // Button held down, check for long press
                if ((now - _pressStartTime) >= _longPressDelay) {
                    _longPressTriggered = true;
                    if (onLongPressPress) onLongPressPress();
                }
            }
        }
    }

    // Callbacks you can assign to your functions

    // Called once when the button is pressed
    Callback onShortPressPress;

    // Called once when the button is released (if press was short)
    Callback onShortPressRelease;

    // Called once when long press threshold reached during hold (while still pressed)
    Callback onLongPressPress;

    // Called once when the button is released after a long press
    Callback onLongPressRelease;

private:
    uint8_t _pin;
    bool _activeLow;
    unsigned long _debounceDelay;
    unsigned long _longPressDelay;
    bool _lastReading = false;
    unsigned long _lastDebounceTime;
    unsigned long _pressStartTime;
    bool _longPressTriggered;
    bool _pressed;
};
