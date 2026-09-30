#pragma once

#include <Arduino.h>

class Button {
public:
    using Callback = void (*)(Button &);

    Button(int pin = -1, int index = -1, bool activeLow = true, bool usePullup = true);

    void begin();
    void update(uint32_t nowMs = millis());

    Button &setCallbacks(Callback onPress, Callback onRelease = nullptr, Callback onHold = nullptr);
    Button &setHoldConfig(bool enabled, uint32_t holdTimeMs);
    Button &setDebounceMs(uint32_t debounceMs);

    bool isPressed() const;
    bool isEnabled() const;
    int getPin() const;

    int index_;

private:
    bool readPhysicalPressed() const;
    bool debounceComplete(uint32_t nowMs) const;
    void handleStableStateChange(uint32_t nowMs);
    void handleHold(uint32_t nowMs);

    
    int pin_;
    bool activeLow_;
    bool usePullup_;
    bool enabled_;

    bool rawState_;
    bool stableState_;
    uint32_t lastRawChangeMs_;
    uint32_t pressedSinceMs_;
    bool holdTriggered_;

    uint32_t debounceMs_;
    bool holdEnabled_;
    uint32_t holdTimeMs_;

    Callback onPress_;
    Callback onRelease_;
    Callback onHold_;
};

class ButtonMap {
public:
    static constexpr size_t kMaxButtons = 16;

    ButtonMap();

    Button &add(int pin, bool activeLow = true, bool usePullup = true);
    void begin();
    void update(uint32_t nowMs = millis());

    size_t size() const;
    bool isPressed(size_t index) const;
    bool isEnabled(size_t index) const;
    int getPin(size_t index) const;
    void setCallbacks(size_t index, Button::Callback onPress, Button::Callback onRelease = nullptr, Button::Callback onHold = nullptr);
    void setHoldConfig(size_t index, bool enabled, uint32_t holdTimeMs);
    void setDebounceMs(size_t index, uint32_t debounceMs);
    bool anyPressed() const;

private:
    Button *getButton(size_t index);
    const Button *getButton(size_t index) const;

    Button buttons_[kMaxButtons];
    size_t count_;
};
