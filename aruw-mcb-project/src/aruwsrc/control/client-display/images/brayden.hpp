
/*
 * Husky line-art logo for the ARUW client display.
 */

#ifndef BRAYDEN_HPP_
#define BRAYDEN_HPP_

#include "image.hpp"

namespace aruwsrc::control::client_display::images
{
// clang-format off
static constexpr LineTuple BRAYDEN_LINES[]{
    // Outer ears
    LineTuple(100, 160, 145, 45),
    LineTuple(145, 45, 220, 115),
    LineTuple(280, 115, 355, 45),
    LineTuple(355, 45, 400, 160),

    // Inner ears
    LineTuple(125, 135, 150, 80),
    LineTuple(150, 80, 190, 125),
    LineTuple(310, 125, 350, 80),
    LineTuple(350, 80, 375, 135),

    // Head outline and cheeks
    LineTuple(100, 160, 75, 220),
    LineTuple(75, 220, 90, 295),
    LineTuple(90, 295, 130, 355),
    LineTuple(130, 355, 180, 390),
    LineTuple(180, 390, 250, 410),
    LineTuple(250, 410, 320, 390),
    LineTuple(320, 390, 370, 355),
    LineTuple(370, 355, 410, 295),
    LineTuple(410, 295, 425, 220),
    LineTuple(425, 220, 400, 160),

    // Forehead markings
    LineTuple(145, 145, 185, 165),
    LineTuple(185, 165, 220, 205),
    LineTuple(280, 205, 315, 165),
    LineTuple(315, 165, 355, 145),
    LineTuple(220, 115, 220, 205),
    LineTuple(280, 115, 280, 205),
    LineTuple(220, 205, 250, 225),
    LineTuple(250, 225, 280, 205),

    // Left eye
    LineTuple(115, 215, 155, 195),
    LineTuple(155, 195, 195, 215),
    LineTuple(195, 215, 165, 235),
    LineTuple(165, 235, 115, 215),
    LineTuple(130, 215, 160, 215),
    LineTuple(160, 215, 180, 215),

    // Right eye
    LineTuple(305, 215, 345, 195),
    LineTuple(345, 195, 385, 215),
    LineTuple(385, 215, 335, 235),
    LineTuple(335, 235, 305, 215),
    LineTuple(320, 215, 350, 215),
    LineTuple(350, 215, 370, 215),

    // Bridge of the nose
    LineTuple(195, 215, 210, 260),
    LineTuple(210, 260, 250, 280),
    LineTuple(305, 215, 290, 260),
    LineTuple(290, 260, 250, 280),

    // Muzzle
    LineTuple(210, 260, 180, 290),
    LineTuple(180, 290, 195, 320),
    LineTuple(195, 320, 230, 335),
    LineTuple(230, 335, 250, 325),
    LineTuple(250, 325, 270, 335),
    LineTuple(270, 335, 305, 320),
    LineTuple(305, 320, 320, 290),
    LineTuple(320, 290, 290, 260),

    // Nose
    LineTuple(220, 275, 235, 265),
    LineTuple(235, 265, 265, 265),
    LineTuple(265, 265, 280, 275),
    LineTuple(280, 275, 270, 290),
    LineTuple(270, 290, 230, 290),
    LineTuple(230, 290, 220, 275),

    // Mouth
    LineTuple(250, 290, 250, 310),
    LineTuple(250, 310, 225, 320),
    LineTuple(250, 310, 275, 320),

    // Cheek fur details
    LineTuple(95, 260, 130, 280),
    LineTuple(130, 280, 155, 275),
    LineTuple(105, 285, 140, 305),
    LineTuple(140, 305, 170, 300),
    LineTuple(405, 260, 370, 280),
    LineTuple(370, 280, 345, 275),
    LineTuple(395, 285, 360, 305),
    LineTuple(360, 305, 330, 300),

    // Lower jaw
    LineTuple(195, 320, 210, 350),
    LineTuple(210, 350, 250, 365),
    LineTuple(250, 365, 290, 350),
    LineTuple(290, 350, 305, 320)
};
// clang-format on

static constexpr int NUM_LINES_BRAYDEN =
    sizeof(BRAYDEN_LINES) / sizeof(LineTuple);

Image brayden(
    {.size = NUM_LINES_BRAYDEN,
     .lines = BRAYDEN_LINES,
     .IMAGE_X_OFFSET = 0,
     .IMAGE_Y_OFFSET = 0,
     .IMAGE_SCALE = 1.0});

}  // namespace aruwsrc::control::client_display::images

#endif  // HUSKY_HPP_
