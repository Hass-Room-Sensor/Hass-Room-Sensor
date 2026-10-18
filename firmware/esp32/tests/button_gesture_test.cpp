#include "app/ButtonGesture.hpp"

#include <cassert>
#include <chrono>

using app::ButtonGesture;
using namespace std::chrono_literals;

/** Exercises the button gesture behavior. */
int main() {
    const auto t0 = ButtonGesture::Clock::time_point{};

    // The task samples once per GPIO edge, then again at the recognizer's deadline.
    ButtonGesture scheduled;
    assert(!scheduled.next_update_delay(t0));
    assert(scheduled.update(true, t0) == ButtonGesture::Event::NONE);
    assert(scheduled.is_processing());
    assert(scheduled.next_update_delay(t0) == 50ms);
    const auto debounce = scheduled.next_update_delay(t0);
    assert(debounce);
    assert(scheduled.update(true, t0 + *debounce) == ButtonGesture::Event::PRESSED);
    assert(scheduled.next_update_delay(t0 + 50ms) == 10s);
    assert(scheduled.update(false, t0 + 1s) == ButtonGesture::Event::NONE);
    assert(scheduled.next_update_delay(t0 + 1s) == 50ms);
    assert(scheduled.update(false, t0 + 1050ms) == ButtonGesture::Event::SHORT_PRESS);
    assert(!scheduled.is_processing());
    assert(!scheduled.next_update_delay(t0 + 1050ms));

    ButtonGesture scheduledHold;
    scheduledHold.update(true, t0);
    const auto pressDelay = scheduledHold.next_update_delay(t0);
    assert(pressDelay);
    scheduledHold.update(true, t0 + *pressDelay);
    const auto holdDelay = scheduledHold.next_update_delay(t0 + 50ms);
    assert(holdDelay);
    assert(scheduledHold.update(true, t0 + 50ms + *holdDelay) == ButtonGesture::Event::FACTORY_RESET);
    assert(!scheduledHold.next_update_delay(t0 + 10050ms));

    // A delayed task must still recognize a completed hold if release arrives before its timeout sample.
    ButtonGesture delayedHold;
    delayedHold.update(true, t0);
    delayedHold.update(true, t0 + 50ms);
    delayedHold.update(false, t0 + 11s);
    assert(delayedHold.update(false, t0 + 11050ms) == ButtonGesture::Event::FACTORY_RESET);

    ButtonGesture shortPress;
    assert(shortPress.update(true, t0) == ButtonGesture::Event::NONE);
    assert(shortPress.update(false, t0 + 20ms) == ButtonGesture::Event::NONE); // bounce
    assert(shortPress.update(true, t0 + 25ms) == ButtonGesture::Event::NONE);
    assert(shortPress.update(true, t0 + 50ms) == ButtonGesture::Event::NONE);
    assert(shortPress.update(true, t0 + 75ms) == ButtonGesture::Event::PRESSED);
    assert(shortPress.update(false, t0 + 300ms) == ButtonGesture::Event::NONE);
    assert(shortPress.update(false, t0 + 350ms) == ButtonGesture::Event::SHORT_PRESS);
    assert(shortPress.update(false, t0 + 400ms) == ButtonGesture::Event::NONE);

    ButtonGesture longPress;
    longPress.update(true, t0);
    assert(longPress.update(true, t0 + 50ms) == ButtonGesture::Event::PRESSED);
    assert(longPress.update(true, t0 + 10049ms) == ButtonGesture::Event::NONE);
    assert(longPress.update(true, t0 + 10050ms) == ButtonGesture::Event::FACTORY_RESET);
    assert(longPress.update(true, t0 + 20s) == ButtonGesture::Event::NONE);
    longPress.update(false, t0 + 21s);
    assert(longPress.update(false, t0 + 21s + 50ms) == ButtonGesture::Event::NONE);

    ButtonGesture heldAtStartup{true};
    assert(heldAtStartup.update(true, t0 + 10s) == ButtonGesture::Event::NONE);
    heldAtStartup.update(false, t0 + 11s);
    assert(heldAtStartup.update(false, t0 + 11s + 50ms) == ButtonGesture::Event::NONE);
    heldAtStartup.update(true, t0 + 12s);
    assert(heldAtStartup.update(true, t0 + 12s + 50ms) == ButtonGesture::Event::PRESSED);
    heldAtStartup.update(false, t0 + 12s + 100ms);
    assert(heldAtStartup.update(false, t0 + 12s + 150ms) == ButtonGesture::Event::SHORT_PRESS);

    ButtonGesture interrupted;
    interrupted.update(true, t0);
    assert(interrupted.update(true, t0 + 50ms) == ButtonGesture::Event::PRESSED);
    interrupted.update(false, t0 + 9s);
    assert(interrupted.update(false, t0 + 9s + 50ms) == ButtonGesture::Event::SHORT_PRESS);
    assert(interrupted.update(false, t0 + 15s) == ButtonGesture::Event::NONE);
}
