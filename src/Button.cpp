#include "Button.h"

Button::Button(int pin, int index, bool activeLow, bool usePullup)
    : pin_(pin),
      index_(index),
      activeLow_(activeLow),
      usePullup_(usePullup),
      enabled_(pin >= 0),
      rawState_(false),
      stableState_(false),
      lastRawChangeMs_(0),
      pressedSinceMs_(0),
      holdTriggered_(false),
      debounceMs_(20),
      holdEnabled_(false),
      holdTimeMs_(600),
      onPress_(nullptr),
      onRelease_(nullptr),
      onHold_(nullptr) {}

void Button::begin() {
    if (!enabled_) {
        return;
    }

    pinMode(pin_, usePullup_ ? INPUT_PULLUP : INPUT);
    rawState_ = readPhysicalPressed();
    stableState_ = rawState_;
    lastRawChangeMs_ = millis();
    pressedSinceMs_ = stableState_ ? lastRawChangeMs_ : 0;
    holdTriggered_ = false;
}

void Button::update(uint32_t nowMs) {
    if (!enabled_) {
        return;
    }

    const bool currentRaw = readPhysicalPressed();
    if (currentRaw != rawState_) {
        rawState_ = currentRaw;
        lastRawChangeMs_ = nowMs;
    }

    if (!debounceComplete(nowMs)) {
        return;
    }

    handleStableStateChange(nowMs);
    handleHold(nowMs);
}

Button &Button::setCallbacks(Callback onPress, Callback onRelease, Callback onHold) {
    onPress_ = onPress;
    onRelease_ = onRelease;
    onHold_ = onHold;
    return *this;
}

Button &Button::setHoldConfig(bool enabled, uint32_t holdTimeMs) {
    holdEnabled_ = enabled;
    holdTimeMs_ = holdTimeMs;
    return *this;
}

Button &Button::setDebounceMs(uint32_t debounceMs) {
    debounceMs_ = debounceMs;
    return *this;
}

bool Button::isPressed() const {
    if (!enabled_) {
        return false;
    }
    return stableState_;
}

bool Button::isEnabled() const {
    return enabled_;
}

int Button::getPin() const {
    return pin_;
}

bool Button::readPhysicalPressed() const {
    const int value = digitalRead(pin_);
    return activeLow_ ? (value == LOW) : (value == HIGH);
}

bool Button::debounceComplete(uint32_t nowMs) const {
    return (nowMs - lastRawChangeMs_) >= debounceMs_;
}

void Button::handleStableStateChange(uint32_t nowMs) {
    if (stableState_ == rawState_) {
        return;
    }

    stableState_ = rawState_;
    if (stableState_) {
        pressedSinceMs_ = nowMs;
        holdTriggered_ = false;
        if (onPress_ != nullptr) {
            onPress_(*this);
        }
        return;
    }

    if (!holdTriggered_ && onRelease_ != nullptr) {
        onRelease_(*this);
    }

    holdTriggered_ = false;
}

void Button::handleHold(uint32_t nowMs) {
    if (!stableState_ || !holdEnabled_ || holdTriggered_) {
        return;
    }

    if ((nowMs - pressedSinceMs_) < holdTimeMs_) {
        return;
    }

    holdTriggered_ = true;
    if (onHold_ != nullptr) {
        onHold_(*this);
    }
}

ButtonMap::ButtonMap() : buttons_{}, count_(0) {}

Button &ButtonMap::add(int pin, bool activeLow, bool usePullup) {
    static Button overflowButton;

    if (count_ >= kMaxButtons) {
        return overflowButton;
    }

    buttons_[count_] = Button(pin, count_, activeLow, usePullup);

    return buttons_[count_++];
}

void ButtonMap::begin() {
    for (size_t i = 0; i < count_; ++i) {
        buttons_[i].begin();
    }
}

void ButtonMap::update(uint32_t nowMs) {
    for (size_t i = 0; i < count_; ++i) {
        buttons_[i].update(nowMs);
    }
}

size_t ButtonMap::size() const {
    return count_;
}

bool ButtonMap::isPressed(size_t index) const {
    const Button *button = getButton(index);
    return button != nullptr && button->isPressed();
}

bool ButtonMap::isEnabled(size_t index) const {
    const Button *button = getButton(index);
    return button != nullptr && button->isEnabled();
}

int ButtonMap::getPin(size_t index) const {
    const Button *button = getButton(index);
    return button != nullptr ? button->getPin() : -1;
}

void ButtonMap::setCallbacks(size_t index, Button::Callback onPress, Button::Callback onRelease, Button::Callback onHold) {
    Button *button = getButton(index);
    if (button != nullptr) {
        button->setCallbacks(onPress, onRelease, onHold);
    }
}

void ButtonMap::setHoldConfig(size_t index, bool enabled, uint32_t holdTimeMs) {
    Button *button = getButton(index);
    if (button != nullptr) {
        button->setHoldConfig(enabled, holdTimeMs);
    }
}

void ButtonMap::setDebounceMs(size_t index, uint32_t debounceMs) {
    Button *button = getButton(index);
    if (button != nullptr) {
        button->setDebounceMs(debounceMs);
    }
}

Button *ButtonMap::getButton(size_t index) {
    if (index >= count_) {
        return nullptr;
    }
    return &buttons_[index];
}

const Button *ButtonMap::getButton(size_t index) const {
    if (index >= count_) {
        return nullptr;
    }
    return &buttons_[index];
}

bool ButtonMap::anyPressed() const {
    for (size_t i = 0; i < count_; ++i) {
        if (buttons_[i].isPressed()) {
            return true;
        }
    }
    return false;
}
