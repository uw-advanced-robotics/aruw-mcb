/*
 * Copyright (c) 2024, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

/*
We use __has_include guards here to support the case when fibers are not in use
but we still need to provide the interface to the rest of the library.
Then only the functionality that is enabled is supported and the fiber yield()
function returns immediately, thus blocks in-place.
*/

// includes the functions related to time
#if __has_include(<modm/architecture/interface/clock.hpp>)
#	include "fiber/functions.hpp"
#endif
// polyfill implementation of an empty yield
#if __has_include("fiber/no_yield.hpp")
#	include "fiber/no_yield.hpp"
#endif
// pulls in the fiber and scheduler implementation
#if __has_include("fiber/task.hpp")
#	include "fiber/task.hpp"
#endif
