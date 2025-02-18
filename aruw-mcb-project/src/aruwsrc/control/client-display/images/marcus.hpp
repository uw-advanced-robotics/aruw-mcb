/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MARCUS_HPP_
#define MARCUS_HPP_

#include <cstdint>
#include <tuple>

namespace aruwsrc::control::client_display::images
{

using LineTuple = std::tuple<int16_t, int16_t, int16_t, int16_t>;

// clang-format off
static constexpr LineTuple MARCUS_LINES[]{
    LineTuple(305, 744, 374, 809),
    LineTuple(376, 807, 425, 749),
    LineTuple(224, 749, 287, 733),
    LineTuple(208, 749, 154, 731),
    LineTuple(146, 733, 158, 743),
    LineTuple(158, 743, 157, 777),
    LineTuple(141, 797, 141, 797),
    LineTuple(138, 829, 158, 777),
    LineTuple(516, 731, 575, 733),
    LineTuple(562, 776, 562, 780),
    LineTuple(574, 738, 574, 738),
    LineTuple(558, 781, 577, 740),
    LineTuple(568, 765, 582, 798),
    LineTuple(582, 798, 633, 828),
    LineTuple(638, 829, 643, 849),
    LineTuple(643, 849, 670, 850),
    LineTuple(674, 850, 684, 863),
    LineTuple(683, 867, 693, 937),
    LineTuple(693, 937, 653, 981),
    LineTuple(653, 981, 490, 1005),
    LineTuple(307, 1010, 402, 997),
    LineTuple(485, 997, 483, 996),
    LineTuple(407, 998, 407, 996),
    LineTuple(476, 994, 476, 994),
    LineTuple(410, 997, 473, 996),
    LineTuple(477, 994, 509, 1001),
    LineTuple(308, 1013, 118, 994),
    LineTuple(118, 994, 18, 924),
    LineTuple(18, 924, 23, 867),
    LineTuple(23, 867, 38, 869),
    LineTuple(39, 865, 126, 873),
    LineTuple(143, 867, 151, 858),
    LineTuple(129, 873, 142, 869),
    LineTuple(149, 857, 575, 854),
    LineTuple(597, 861, 597, 861),
    LineTuple(573, 855, 596, 861),
    LineTuple(649, 859, 653, 859),
    LineTuple(600, 861, 672, 850),
    LineTuple(147, 868, 164, 928),
    LineTuple(164, 928, 137, 918),
    LineTuple(137, 918, 131, 888),
    LineTuple(127, 888, 137, 873),
    LineTuple(130, 882, 95, 890),
    LineTuple(518, 927, 578, 872),
    LineTuple(607, 901, 673, 925),
    LineTuple(140, 993, 175, 974),
    LineTuple(163, 995, 191, 989),
    LineTuple(50, 865, 63, 859),
    LineTuple(63, 859, 84, 842),
    LineTuple(85, 840, 117, 828),
    LineTuple(187, 821, 122, 826),
    LineTuple(484, 1015, 491, 1005),
    LineTuple(486, 1013, 449, 1006),
    LineTuple(488, 1013, 514, 1027),
    LineTuple(518, 1027, 533, 1024),
    LineTuple(530, 1024, 544, 1041),
    LineTuple(547, 1054, 474, 1076),
    LineTuple(472, 1075, 404, 1076),
    LineTuple(336, 1071, 332, 1071),
    LineTuple(400, 1076, 334, 1069),
    LineTuple(313, 1059, 334, 1069),
    LineTuple(315, 1059, 320, 1038),
    LineTuple(320, 1036, 334, 1033),
    LineTuple(344, 1019, 344, 1019),
    LineTuple(334, 1033, 346, 1017),
    LineTuple(311, 1013, 343, 1017),
    LineTuple(348, 1022, 371, 1012),
    LineTuple(387, 1028, 417, 1020),
    LineTuple(459, 1025, 417, 1020),
    LineTuple(394, 1028, 457, 1027),
    LineTuple(345, 1028, 357, 1035),
    LineTuple(405, 1037, 361, 1034),
    LineTuple(441, 1032, 415, 1035),
    LineTuple(350, 1072, 399, 1054),
    LineTuple(326, 1054, 342, 1059),
    LineTuple(315, 1054, 333, 1042),
    LineTuple(359, 1058, 336, 1044),
    LineTuple(363, 1059, 373, 1052),
    LineTuple(375, 1068, 401, 1063),
    LineTuple(394, 1072, 422, 1065),
    LineTuple(419, 1073, 464, 1061),
    LineTuple(452, 1072, 476, 1067),
    LineTuple(378, 1052, 511, 1047),
    LineTuple(513, 1045, 520, 1033),
    LineTuple(538, 1044, 518, 1033),
    LineTuple(538, 1044, 538, 1035),
    LineTuple(361, 1044, 412, 1044),
    LineTuple(454, 1040, 499, 1038),
    LineTuple(499, 1045, 461, 1045),
    LineTuple(457, 1040, 497, 1044),
    LineTuple(499, 1037, 496, 1044),
    LineTuple(413, 1042, 408, 1049),
    LineTuple(410, 1049, 364, 1047),
    LineTuple(535, 990, 608, 968),
    LineTuple(598, 978, 636, 976),
    LineTuple(628, 966, 650, 975),
    LineTuple(178, 803, 162, 772),
    LineTuple(164, 805, 160, 780),
    LineTuple(580, 800, 580, 812),
    LineTuple(579, 812, 537, 812),
    LineTuple(570, 822, 537, 812),
    LineTuple(570, 822, 566, 835),
    LineTuple(561, 824, 559, 828),
    LineTuple(544, 828, 563, 828),
    LineTuple(544, 826, 547, 819),
    LineTuple(551, 819, 563, 826),
    LineTuple(540, 833, 579, 845),
    LineTuple(579, 845, 593, 856),
    LineTuple(619, 852, 594, 852),
    LineTuple(621, 852, 645, 849),
    LineTuple(612, 824, 591, 815),
    LineTuple(626, 828, 593, 829),
    LineTuple(593, 829, 556, 815),
    LineTuple(642, 850, 626, 849),
    LineTuple(626, 845, 629, 840),
    LineTuple(303, 850, 182, 852),
    LineTuple(247, 840, 203, 847),
    LineTuple(241, 833, 171, 850),
    LineTuple(229, 835, 91, 856),
    LineTuple(212, 829, 140, 845),
    LineTuple(210, 822, 101, 847),
    LineTuple(94, 843, 136, 831),
    LineTuple(212, 828, 212, 822),
    LineTuple(250, 840, 241, 833),
    LineTuple(185, 822, 117, 842)
};
// clang-format on

static constexpr int NUM_LINES_MARCUS = sizeof(MARCUS_LINES) / sizeof(LineTuple);

};  // namespace aruwsrc::control::client_display::images

#endif  // MARCUS_HPP_
