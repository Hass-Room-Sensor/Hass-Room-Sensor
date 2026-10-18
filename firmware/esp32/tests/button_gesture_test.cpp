#include "app/ButtonGesture.hpp"

#include <cassert>
#include <chrono>

using app::ButtonGesture;
using namespace std::chrono_literals;

int main() {
    const auto t0 = ButtonGesture::Clock::time_point{};

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
