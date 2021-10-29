#pragma once
#include <cstdint>

namespace slope::app {

enum class Key : int {
    Unknown         = -1,
    Space           = 32,
    Apostrophe      = 39,
    Comma           = 44,
    Minus           = 45,
    Period          = 46,
    Slash           = 47,
    Num_0           = 48,
    Num_1           = 49,
    Num_2           = 50,
    Num_3           = 51,
    Num_4           = 52,
    Num_5           = 53,
    Num_6           = 54,
    Num_7           = 55,
    Num_8           = 56,
    Num_9           = 57,
    Semicolon       = 59,
    Equal           = 61,
    A               = 65,
    B               = 66,
    C               = 67,
    D               = 68,
    E               = 69,
    F               = 70,
    G               = 71,
    H               = 72,
    I               = 73,
    J               = 74,
    K               = 75,
    L               = 76,
    M               = 77,
    N               = 78,
    O               = 79,
    P               = 80,
    Q               = 81,
    R               = 82,
    S               = 83,
    T               = 84,
    U               = 85,
    V               = 86,
    W               = 87,
    X               = 88,
    Y               = 89,
    Z               = 90,
    LeftBracket     = 91,
    Backslash       = 92,
    RightBracket    = 93,
    GraveAccent     = 96,
    World_1         = 161,
    World_2         = 162,
    Escape          = 256,
    Enter           = 257,
    Tab             = 258,
    Backspace       = 259,
    Insert          = 260,
    Delete          = 261,
    Right           = 262,
    Left            = 263,
    Down            = 264,
    Up              = 265,
    PageUp          = 266,
    PageDown        = 267,
    Home            = 268,
    End             = 269,
    CapsLock        = 280,
    ScrollLock      = 281,
    NumLock         = 282,
    PrintScreen     = 283,
    Pause           = 284,
    F1              = 290,
    F2              = 291,
    F3              = 292,
    F4              = 293,
    F5              = 294,
    F6              = 295,
    F7              = 296,
    F8              = 297,
    F9              = 298,
    F10             = 299,
    F11             = 300,
    F12             = 301,
    F13             = 302,
    F14             = 303,
    F15             = 304,
    F16             = 305,
    F17             = 306,
    F18             = 307,
    F19             = 308,
    F20             = 309,
    F21             = 310,
    F22             = 311,
    F23             = 312,
    F24             = 313,
    F25             = 314,
    KP_0            = 320,
    KP_1            = 321,
    KP_2            = 322,
    KP_3            = 323,
    KP_4            = 324,
    KP_5            = 325,
    KP_6            = 326,
    KP_7            = 327,
    KP_8            = 328,
    KP_9            = 329,
    KP_Decimal      = 330,
    KP_Divide       = 331,
    KP_Multiply     = 332,
    KP_Subtract     = 333,
    KP_Add          = 334,
    KP_Enter        = 335,
    KP_Equal        = 336,
    LeftShift       = 340,
    LeftControl     = 341,
    LeftAlt         = 342,
    LeftSuper       = 343,
    RightShift      = 344,
    RightControl    = 345,
    RightAlt        = 346,
    RightSuper      = 347,
    Menu            = 348
};

enum class MouseButton : int {
    B_1 = 0,
    B_2 = 1,
    B_3 = 2,
    B_4 = 3,
    B_5 = 4,
    B_6 = 5,
    B_7 = 6,
    B_8 = 7
};

enum class KeyAction : int {
    Release = 0,
    Press   = 1,
    Repeat  = 2
};

struct KeyMod {
    using Raw = int;

    static constexpr Raw Shift      = 0x01;
    static constexpr Raw Control    = 0x02;
    static constexpr Raw Alt        = 0x04;
    static constexpr Raw Super      = 0x08;
    static constexpr Raw CapsLock   = 0x10;
    static constexpr Raw NumLock    = 0x20;
};

} // slope::app