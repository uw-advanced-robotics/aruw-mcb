/*
 * Copyright (c) 2021, Niklas Hauser
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "data.hpp"
#include "static.hpp"

namespace modm::platform
{

/// @ingroup modm_platform_gpio
/// @{
using GpioA0 = GpioStatic<detail::DataA0>;
using GpioOutputA0 = GpioA0;
using GpioInputA0  = GpioA0;

using GpioA1 = GpioStatic<detail::DataA1>;
using GpioOutputA1 = GpioA1;
using GpioInputA1  = GpioA1;

using GpioA2 = GpioStatic<detail::DataA2>;
using GpioOutputA2 = GpioA2;
using GpioInputA2  = GpioA2;

using GpioA3 = GpioStatic<detail::DataA3>;
using GpioOutputA3 = GpioA3;
using GpioInputA3  = GpioA3;

using GpioA4 = GpioStatic<detail::DataA4>;
using GpioOutputA4 = GpioA4;
using GpioInputA4  = GpioA4;

using GpioA5 = GpioStatic<detail::DataA5>;
using GpioOutputA5 = GpioA5;
using GpioInputA5  = GpioA5;

using GpioA6 = GpioStatic<detail::DataA6>;
using GpioOutputA6 = GpioA6;
using GpioInputA6  = GpioA6;

using GpioA7 = GpioStatic<detail::DataA7>;
using GpioOutputA7 = GpioA7;
using GpioInputA7  = GpioA7;

using GpioA8 = GpioStatic<detail::DataA8>;
using GpioOutputA8 = GpioA8;
using GpioInputA8  = GpioA8;

using GpioA9 = GpioStatic<detail::DataA9>;
using GpioOutputA9 = GpioA9;
using GpioInputA9  = GpioA9;

using GpioA10 = GpioStatic<detail::DataA10>;
using GpioOutputA10 = GpioA10;
using GpioInputA10  = GpioA10;

using GpioA11 = GpioStatic<detail::DataA11>;
using GpioOutputA11 = GpioA11;
using GpioInputA11  = GpioA11;

using GpioA12 = GpioStatic<detail::DataA12>;
using GpioOutputA12 = GpioA12;
using GpioInputA12  = GpioA12;

using GpioA13 = GpioStatic<detail::DataA13>;
using GpioOutputA13 = GpioA13;
using GpioInputA13  = GpioA13;

using GpioA14 = GpioStatic<detail::DataA14>;
using GpioOutputA14 = GpioA14;
using GpioInputA14  = GpioA14;

using GpioA15 = GpioStatic<detail::DataA15>;
using GpioOutputA15 = GpioA15;
using GpioInputA15  = GpioA15;

using GpioB0 = GpioStatic<detail::DataB0>;
using GpioOutputB0 = GpioB0;
using GpioInputB0  = GpioB0;

using GpioB1 = GpioStatic<detail::DataB1>;
using GpioOutputB1 = GpioB1;
using GpioInputB1  = GpioB1;

using GpioB2 = GpioStatic<detail::DataB2>;
using GpioOutputB2 = GpioB2;
using GpioInputB2  = GpioB2;

using GpioB3 = GpioStatic<detail::DataB3>;
using GpioOutputB3 = GpioB3;
using GpioInputB3  = GpioB3;

using GpioB4 = GpioStatic<detail::DataB4>;
using GpioOutputB4 = GpioB4;
using GpioInputB4  = GpioB4;

using GpioB5 = GpioStatic<detail::DataB5>;
using GpioOutputB5 = GpioB5;
using GpioInputB5  = GpioB5;

using GpioB6 = GpioStatic<detail::DataB6>;
using GpioOutputB6 = GpioB6;
using GpioInputB6  = GpioB6;

using GpioB7 = GpioStatic<detail::DataB7>;
using GpioOutputB7 = GpioB7;
using GpioInputB7  = GpioB7;

using GpioB8 = GpioStatic<detail::DataB8>;
using GpioOutputB8 = GpioB8;
using GpioInputB8  = GpioB8;

using GpioB9 = GpioStatic<detail::DataB9>;
using GpioOutputB9 = GpioB9;
using GpioInputB9  = GpioB9;

using GpioB10 = GpioStatic<detail::DataB10>;
using GpioOutputB10 = GpioB10;
using GpioInputB10  = GpioB10;

using GpioB11 = GpioStatic<detail::DataB11>;
using GpioOutputB11 = GpioB11;
using GpioInputB11  = GpioB11;

using GpioB12 = GpioStatic<detail::DataB12>;
using GpioOutputB12 = GpioB12;
using GpioInputB12  = GpioB12;

using GpioB13 = GpioStatic<detail::DataB13>;
using GpioOutputB13 = GpioB13;
using GpioInputB13  = GpioB13;

using GpioB14 = GpioStatic<detail::DataB14>;
using GpioOutputB14 = GpioB14;
using GpioInputB14  = GpioB14;

using GpioB15 = GpioStatic<detail::DataB15>;
using GpioOutputB15 = GpioB15;
using GpioInputB15  = GpioB15;

using GpioC0 = GpioStatic<detail::DataC0>;
using GpioOutputC0 = GpioC0;
using GpioInputC0  = GpioC0;

using GpioC1 = GpioStatic<detail::DataC1>;
using GpioOutputC1 = GpioC1;
using GpioInputC1  = GpioC1;

using GpioC2 = GpioStatic<detail::DataC2>;
using GpioOutputC2 = GpioC2;
using GpioInputC2  = GpioC2;

using GpioC3 = GpioStatic<detail::DataC3>;
using GpioOutputC3 = GpioC3;
using GpioInputC3  = GpioC3;

using GpioC4 = GpioStatic<detail::DataC4>;
using GpioOutputC4 = GpioC4;
using GpioInputC4  = GpioC4;

using GpioC5 = GpioStatic<detail::DataC5>;
using GpioOutputC5 = GpioC5;
using GpioInputC5  = GpioC5;

using GpioC6 = GpioStatic<detail::DataC6>;
using GpioOutputC6 = GpioC6;
using GpioInputC6  = GpioC6;

using GpioC7 = GpioStatic<detail::DataC7>;
using GpioOutputC7 = GpioC7;
using GpioInputC7  = GpioC7;

using GpioC8 = GpioStatic<detail::DataC8>;
using GpioOutputC8 = GpioC8;
using GpioInputC8  = GpioC8;

using GpioC9 = GpioStatic<detail::DataC9>;
using GpioOutputC9 = GpioC9;
using GpioInputC9  = GpioC9;

using GpioC10 = GpioStatic<detail::DataC10>;
using GpioOutputC10 = GpioC10;
using GpioInputC10  = GpioC10;

using GpioC11 = GpioStatic<detail::DataC11>;
using GpioOutputC11 = GpioC11;
using GpioInputC11  = GpioC11;

using GpioC12 = GpioStatic<detail::DataC12>;
using GpioOutputC12 = GpioC12;
using GpioInputC12  = GpioC12;

using GpioC13 = GpioStatic<detail::DataC13>;
using GpioOutputC13 = GpioC13;
using GpioInputC13  = GpioC13;

using GpioC14 = GpioStatic<detail::DataC14>;
using GpioOutputC14 = GpioC14;
using GpioInputC14  = GpioC14;

using GpioC15 = GpioStatic<detail::DataC15>;
using GpioOutputC15 = GpioC15;
using GpioInputC15  = GpioC15;

using GpioD0 = GpioStatic<detail::DataD0>;
using GpioOutputD0 = GpioD0;
using GpioInputD0  = GpioD0;

using GpioD1 = GpioStatic<detail::DataD1>;
using GpioOutputD1 = GpioD1;
using GpioInputD1  = GpioD1;

using GpioD2 = GpioStatic<detail::DataD2>;
using GpioOutputD2 = GpioD2;
using GpioInputD2  = GpioD2;

using GpioD3 = GpioStatic<detail::DataD3>;
using GpioOutputD3 = GpioD3;
using GpioInputD3  = GpioD3;

using GpioD4 = GpioStatic<detail::DataD4>;
using GpioOutputD4 = GpioD4;
using GpioInputD4  = GpioD4;

using GpioD5 = GpioStatic<detail::DataD5>;
using GpioOutputD5 = GpioD5;
using GpioInputD5  = GpioD5;

using GpioD6 = GpioStatic<detail::DataD6>;
using GpioOutputD6 = GpioD6;
using GpioInputD6  = GpioD6;

using GpioD7 = GpioStatic<detail::DataD7>;
using GpioOutputD7 = GpioD7;
using GpioInputD7  = GpioD7;

using GpioD8 = GpioStatic<detail::DataD8>;
using GpioOutputD8 = GpioD8;
using GpioInputD8  = GpioD8;

using GpioD9 = GpioStatic<detail::DataD9>;
using GpioOutputD9 = GpioD9;
using GpioInputD9  = GpioD9;

using GpioD10 = GpioStatic<detail::DataD10>;
using GpioOutputD10 = GpioD10;
using GpioInputD10  = GpioD10;

using GpioD11 = GpioStatic<detail::DataD11>;
using GpioOutputD11 = GpioD11;
using GpioInputD11  = GpioD11;

using GpioD12 = GpioStatic<detail::DataD12>;
using GpioOutputD12 = GpioD12;
using GpioInputD12  = GpioD12;

using GpioD13 = GpioStatic<detail::DataD13>;
using GpioOutputD13 = GpioD13;
using GpioInputD13  = GpioD13;

using GpioD14 = GpioStatic<detail::DataD14>;
using GpioOutputD14 = GpioD14;
using GpioInputD14  = GpioD14;

using GpioD15 = GpioStatic<detail::DataD15>;
using GpioOutputD15 = GpioD15;
using GpioInputD15  = GpioD15;

using GpioE0 = GpioStatic<detail::DataE0>;
using GpioOutputE0 = GpioE0;
using GpioInputE0  = GpioE0;

using GpioE1 = GpioStatic<detail::DataE1>;
using GpioOutputE1 = GpioE1;
using GpioInputE1  = GpioE1;

using GpioE2 = GpioStatic<detail::DataE2>;
using GpioOutputE2 = GpioE2;
using GpioInputE2  = GpioE2;

using GpioE3 = GpioStatic<detail::DataE3>;
using GpioOutputE3 = GpioE3;
using GpioInputE3  = GpioE3;

using GpioE4 = GpioStatic<detail::DataE4>;
using GpioOutputE4 = GpioE4;
using GpioInputE4  = GpioE4;

using GpioE5 = GpioStatic<detail::DataE5>;
using GpioOutputE5 = GpioE5;
using GpioInputE5  = GpioE5;

using GpioE6 = GpioStatic<detail::DataE6>;
using GpioOutputE6 = GpioE6;
using GpioInputE6  = GpioE6;

using GpioE7 = GpioStatic<detail::DataE7>;
using GpioOutputE7 = GpioE7;
using GpioInputE7  = GpioE7;

using GpioE8 = GpioStatic<detail::DataE8>;
using GpioOutputE8 = GpioE8;
using GpioInputE8  = GpioE8;

using GpioE9 = GpioStatic<detail::DataE9>;
using GpioOutputE9 = GpioE9;
using GpioInputE9  = GpioE9;

using GpioE10 = GpioStatic<detail::DataE10>;
using GpioOutputE10 = GpioE10;
using GpioInputE10  = GpioE10;

using GpioE11 = GpioStatic<detail::DataE11>;
using GpioOutputE11 = GpioE11;
using GpioInputE11  = GpioE11;

using GpioE12 = GpioStatic<detail::DataE12>;
using GpioOutputE12 = GpioE12;
using GpioInputE12  = GpioE12;

using GpioE13 = GpioStatic<detail::DataE13>;
using GpioOutputE13 = GpioE13;
using GpioInputE13  = GpioE13;

using GpioE14 = GpioStatic<detail::DataE14>;
using GpioOutputE14 = GpioE14;
using GpioInputE14  = GpioE14;

using GpioE15 = GpioStatic<detail::DataE15>;
using GpioOutputE15 = GpioE15;
using GpioInputE15  = GpioE15;

using GpioF0 = GpioStatic<detail::DataF0>;
using GpioOutputF0 = GpioF0;
using GpioInputF0  = GpioF0;

using GpioF1 = GpioStatic<detail::DataF1>;
using GpioOutputF1 = GpioF1;
using GpioInputF1  = GpioF1;

using GpioF2 = GpioStatic<detail::DataF2>;
using GpioOutputF2 = GpioF2;
using GpioInputF2  = GpioF2;

using GpioF3 = GpioStatic<detail::DataF3>;
using GpioOutputF3 = GpioF3;
using GpioInputF3  = GpioF3;

using GpioF4 = GpioStatic<detail::DataF4>;
using GpioOutputF4 = GpioF4;
using GpioInputF4  = GpioF4;

using GpioF5 = GpioStatic<detail::DataF5>;
using GpioOutputF5 = GpioF5;
using GpioInputF5  = GpioF5;

using GpioF6 = GpioStatic<detail::DataF6>;
using GpioOutputF6 = GpioF6;
using GpioInputF6  = GpioF6;

using GpioF7 = GpioStatic<detail::DataF7>;
using GpioOutputF7 = GpioF7;
using GpioInputF7  = GpioF7;

using GpioF8 = GpioStatic<detail::DataF8>;
using GpioOutputF8 = GpioF8;
using GpioInputF8  = GpioF8;

using GpioF9 = GpioStatic<detail::DataF9>;
using GpioOutputF9 = GpioF9;
using GpioInputF9  = GpioF9;

using GpioF10 = GpioStatic<detail::DataF10>;
using GpioOutputF10 = GpioF10;
using GpioInputF10  = GpioF10;

using GpioF11 = GpioStatic<detail::DataF11>;
using GpioOutputF11 = GpioF11;
using GpioInputF11  = GpioF11;

using GpioF12 = GpioStatic<detail::DataF12>;
using GpioOutputF12 = GpioF12;
using GpioInputF12  = GpioF12;

using GpioF13 = GpioStatic<detail::DataF13>;
using GpioOutputF13 = GpioF13;
using GpioInputF13  = GpioF13;

using GpioF14 = GpioStatic<detail::DataF14>;
using GpioOutputF14 = GpioF14;
using GpioInputF14  = GpioF14;

using GpioF15 = GpioStatic<detail::DataF15>;
using GpioOutputF15 = GpioF15;
using GpioInputF15  = GpioF15;

using GpioG0 = GpioStatic<detail::DataG0>;
using GpioOutputG0 = GpioG0;
using GpioInputG0  = GpioG0;

using GpioG1 = GpioStatic<detail::DataG1>;
using GpioOutputG1 = GpioG1;
using GpioInputG1  = GpioG1;

using GpioG2 = GpioStatic<detail::DataG2>;
using GpioOutputG2 = GpioG2;
using GpioInputG2  = GpioG2;

using GpioG3 = GpioStatic<detail::DataG3>;
using GpioOutputG3 = GpioG3;
using GpioInputG3  = GpioG3;

using GpioG4 = GpioStatic<detail::DataG4>;
using GpioOutputG4 = GpioG4;
using GpioInputG4  = GpioG4;

using GpioG5 = GpioStatic<detail::DataG5>;
using GpioOutputG5 = GpioG5;
using GpioInputG5  = GpioG5;

using GpioG6 = GpioStatic<detail::DataG6>;
using GpioOutputG6 = GpioG6;
using GpioInputG6  = GpioG6;

using GpioG7 = GpioStatic<detail::DataG7>;
using GpioOutputG7 = GpioG7;
using GpioInputG7  = GpioG7;

using GpioG8 = GpioStatic<detail::DataG8>;
using GpioOutputG8 = GpioG8;
using GpioInputG8  = GpioG8;

using GpioG9 = GpioStatic<detail::DataG9>;
using GpioOutputG9 = GpioG9;
using GpioInputG9  = GpioG9;

using GpioG10 = GpioStatic<detail::DataG10>;
using GpioOutputG10 = GpioG10;
using GpioInputG10  = GpioG10;

using GpioG11 = GpioStatic<detail::DataG11>;
using GpioOutputG11 = GpioG11;
using GpioInputG11  = GpioG11;

using GpioG12 = GpioStatic<detail::DataG12>;
using GpioOutputG12 = GpioG12;
using GpioInputG12  = GpioG12;

using GpioG13 = GpioStatic<detail::DataG13>;
using GpioOutputG13 = GpioG13;
using GpioInputG13  = GpioG13;

using GpioG14 = GpioStatic<detail::DataG14>;
using GpioOutputG14 = GpioG14;
using GpioInputG14  = GpioG14;

using GpioG15 = GpioStatic<detail::DataG15>;
using GpioOutputG15 = GpioG15;
using GpioInputG15  = GpioG15;

using GpioH0 = GpioStatic<detail::DataH0>;
using GpioOutputH0 = GpioH0;
using GpioInputH0  = GpioH0;

using GpioH1 = GpioStatic<detail::DataH1>;
using GpioOutputH1 = GpioH1;
using GpioInputH1  = GpioH1;

using GpioH2 = GpioStatic<detail::DataH2>;
using GpioOutputH2 = GpioH2;
using GpioInputH2  = GpioH2;

using GpioH3 = GpioStatic<detail::DataH3>;
using GpioOutputH3 = GpioH3;
using GpioInputH3  = GpioH3;

using GpioH4 = GpioStatic<detail::DataH4>;
using GpioOutputH4 = GpioH4;
using GpioInputH4  = GpioH4;

using GpioH5 = GpioStatic<detail::DataH5>;
using GpioOutputH5 = GpioH5;
using GpioInputH5  = GpioH5;

using GpioH6 = GpioStatic<detail::DataH6>;
using GpioOutputH6 = GpioH6;
using GpioInputH6  = GpioH6;

using GpioH7 = GpioStatic<detail::DataH7>;
using GpioOutputH7 = GpioH7;
using GpioInputH7  = GpioH7;

using GpioH8 = GpioStatic<detail::DataH8>;
using GpioOutputH8 = GpioH8;
using GpioInputH8  = GpioH8;

using GpioH9 = GpioStatic<detail::DataH9>;
using GpioOutputH9 = GpioH9;
using GpioInputH9  = GpioH9;

using GpioH10 = GpioStatic<detail::DataH10>;
using GpioOutputH10 = GpioH10;
using GpioInputH10  = GpioH10;

using GpioH11 = GpioStatic<detail::DataH11>;
using GpioOutputH11 = GpioH11;
using GpioInputH11  = GpioH11;

using GpioH12 = GpioStatic<detail::DataH12>;
using GpioOutputH12 = GpioH12;
using GpioInputH12  = GpioH12;

using GpioH13 = GpioStatic<detail::DataH13>;
using GpioOutputH13 = GpioH13;
using GpioInputH13  = GpioH13;

using GpioH14 = GpioStatic<detail::DataH14>;
using GpioOutputH14 = GpioH14;
using GpioInputH14  = GpioH14;

using GpioH15 = GpioStatic<detail::DataH15>;
using GpioOutputH15 = GpioH15;
using GpioInputH15  = GpioH15;

using GpioI0 = GpioStatic<detail::DataI0>;
using GpioOutputI0 = GpioI0;
using GpioInputI0  = GpioI0;

using GpioI1 = GpioStatic<detail::DataI1>;
using GpioOutputI1 = GpioI1;
using GpioInputI1  = GpioI1;

using GpioI2 = GpioStatic<detail::DataI2>;
using GpioOutputI2 = GpioI2;
using GpioInputI2  = GpioI2;

using GpioI3 = GpioStatic<detail::DataI3>;
using GpioOutputI3 = GpioI3;
using GpioInputI3  = GpioI3;

using GpioI4 = GpioStatic<detail::DataI4>;
using GpioOutputI4 = GpioI4;
using GpioInputI4  = GpioI4;

using GpioI5 = GpioStatic<detail::DataI5>;
using GpioOutputI5 = GpioI5;
using GpioInputI5  = GpioI5;

using GpioI6 = GpioStatic<detail::DataI6>;
using GpioOutputI6 = GpioI6;
using GpioInputI6  = GpioI6;

using GpioI7 = GpioStatic<detail::DataI7>;
using GpioOutputI7 = GpioI7;
using GpioInputI7  = GpioI7;

using GpioI8 = GpioStatic<detail::DataI8>;
using GpioOutputI8 = GpioI8;
using GpioInputI8  = GpioI8;

using GpioI9 = GpioStatic<detail::DataI9>;
using GpioOutputI9 = GpioI9;
using GpioInputI9  = GpioI9;

using GpioI10 = GpioStatic<detail::DataI10>;
using GpioOutputI10 = GpioI10;
using GpioInputI10  = GpioI10;

using GpioI11 = GpioStatic<detail::DataI11>;
using GpioOutputI11 = GpioI11;
using GpioInputI11  = GpioI11;

/// @}

} // namespace modm::platform

