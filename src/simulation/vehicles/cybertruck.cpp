#include "simulation/vehicles/vehicle.h"
#include "simulation/vehicles/cybertruck.h"
#include <vector>

// See vehicle.h for parameter list
Vehicle CYBERTRUCK(46, 14, 2.5f, 1.0f, 10.0f, 3.0f, 2.0f, 0.1f);

void draw_cybertruck(Renderer *ren, Particle *cpart, float rotation) {
    draw_px(CYBERTRUCK_PIXELS, ren, cpart, rotation);
}

std::vector<VehiclePixel> CYBERTRUCK_PIXELS({
    VehiclePixel(-5, -7, 236, 241, 244),
    VehiclePixel(-4, -7, 236, 241, 244),
    VehiclePixel(-3, -7, 109, 113, 114),
    VehiclePixel(-2, -7, 0, 2, 3),
    VehiclePixel(-1, -7, 3, 5, 4),
    VehiclePixel(-9, -6, 0, 0, 0),
    VehiclePixel(-8, -6, 255, 255, 255),
    VehiclePixel(-7, -6, 17, 17, 19),
    VehiclePixel(-6, -6, 131, 136, 142),
    VehiclePixel(-5, -6, 115, 119, 128),
    VehiclePixel(-4, -6, 112, 119, 125),
    VehiclePixel(-3, -6, 102, 109, 115),
    VehiclePixel(-2, -6, 105, 112, 118),
    VehiclePixel(-1, -6, 89, 94, 100),
    VehiclePixel(0, -6, 94, 98, 101),
    VehiclePixel(1, -6, 65, 69, 72),
    VehiclePixel(2, -6, 10, 15, 19),
    VehiclePixel(3, -6, 244, 249, 253),
    VehiclePixel(4, -6, 222, 230, 233),
    VehiclePixel(5, -6, 249, 254, 255),
    VehiclePixel(6, -6, 133, 138, 142),
    VehiclePixel(-12, -5, 84, 84, 84),
    VehiclePixel(-11, -5, 254, 252, 253),
    VehiclePixel(-10, -5, 17, 18, 22),
    VehiclePixel(-9, -5, 69, 72, 77),
    VehiclePixel(-8, -5, 58, 63, 67),
    VehiclePixel(-7, -5, 65, 70, 74),
    VehiclePixel(-6, -5, 59, 63, 72),
    VehiclePixel(-5, -5, 58, 63, 69),
    VehiclePixel(-4, -5, 54, 59, 65),
    VehiclePixel(-3, -5, 26, 29, 36),
    VehiclePixel(-2, -5, 29, 32, 39),
    VehiclePixel(-1, -5, 53, 61, 64),
    VehiclePixel(0, -5, 52, 55, 62),
    VehiclePixel(1, -5, 54, 57, 64),
    VehiclePixel(2, -5, 47, 50, 57),
    VehiclePixel(3, -5, 54, 57, 64),
    VehiclePixel(4, -5, 33, 36, 43),
    VehiclePixel(5, -5, 52, 53, 58),
    VehiclePixel(6, -5, 204, 214, 223),
    VehiclePixel(7, -5, 204, 213, 220),
    VehiclePixel(8, -5, 205, 214, 221),
    VehiclePixel(9, -5, 206, 217, 223),
    VehiclePixel(10, -5, 209, 220, 226),
    VehiclePixel(11, -5, 208, 219, 225),
    VehiclePixel(12, -5, 214, 222, 225),
    VehiclePixel(13, -5, 132, 140, 143),
    VehiclePixel(-15, -4, 0, 0, 2),
    VehiclePixel(-14, -4, 255, 255, 255),
    VehiclePixel(-13, -4, 123, 124, 128),
    VehiclePixel(-12, -4, 13, 14, 18),
    VehiclePixel(-11, -4, 11, 15, 18),
    VehiclePixel(-10, -4, 14, 15, 20),
    VehiclePixel(-9, -4, 20, 19, 25),
    VehiclePixel(-8, -4, 8, 12, 15),
    VehiclePixel(-7, -4, 5, 9, 12),
    VehiclePixel(-6, -4, 6, 7, 11),
    VehiclePixel(-5, -4, 12, 11, 16),
    VehiclePixel(-4, -4, 16, 15, 20),
    VehiclePixel(-3, -4, 0, 0, 0),
    VehiclePixel(-2, -4, 7, 7, 5),
    VehiclePixel(-1, -4, 12, 10, 15),
    VehiclePixel(0, -4, 7, 7, 9),
    VehiclePixel(1, -4, 10, 10, 12),
    VehiclePixel(2, -4, 7, 11, 14),
    VehiclePixel(3, -4, 3, 7, 10),
    VehiclePixel(4, -4, 0, 0, 5),
    VehiclePixel(5, -4, 0, 3, 10),
    VehiclePixel(6, -4, 197, 206, 213),
    VehiclePixel(7, -4, 192, 203, 207),
    VehiclePixel(8, -4, 194, 205, 209),
    VehiclePixel(9, -4, 199, 208, 217),
    VehiclePixel(10, -4, 206, 217, 223),
    VehiclePixel(11, -4, 206, 215, 222),
    VehiclePixel(12, -4, 200, 209, 214),
    VehiclePixel(13, -4, 202, 211, 216),
    VehiclePixel(14, -4, 199, 208, 215),
    VehiclePixel(15, -4, 195, 204, 209),
    VehiclePixel(16, -4, 192, 201, 206),
    VehiclePixel(17, -4, 197, 206, 213),
    VehiclePixel(18, -4, 205, 212, 218),
    VehiclePixel(19, -4, 175, 180, 183),
    VehiclePixel(20, -4, 0, 0, 0),
    VehiclePixel(-18, -3, 0, 0, 5),
    VehiclePixel(-17, -3, 247, 251, 252),
    VehiclePixel(-16, -3, 139, 139, 141),
    VehiclePixel(-15, -3, 2, 3, 5),
    VehiclePixel(-14, -3, 0, 0, 2),
    VehiclePixel(-13, -3, 0, 0, 5),
    VehiclePixel(-12, -3, 0, 0, 4),
    VehiclePixel(-11, -3, 0, 0, 5),
    VehiclePixel(-10, -3, 88, 89, 94),
    VehiclePixel(-9, -3, 150, 155, 161),
    VehiclePixel(-8, -3, 202, 213, 215),
    VehiclePixel(-7, -3, 242, 253, 255),
    VehiclePixel(-6, -3, 230, 243, 252),
    VehiclePixel(-5, -3, 216, 229, 238),
    VehiclePixel(-4, -3, 215, 228, 237),
    VehiclePixel(-3, -3, 207, 218, 224),
    VehiclePixel(-2, -3, 205, 216, 222),
    VehiclePixel(-1, -3, 207, 215, 226),
    VehiclePixel(0, -3, 198, 207, 216),
    VehiclePixel(1, -3, 200, 209, 216),
    VehiclePixel(2, -3, 200, 209, 218),
    VehiclePixel(3, -3, 200, 209, 218),
    VehiclePixel(4, -3, 209, 219, 221),
    VehiclePixel(5, -3, 202, 213, 219),
    VehiclePixel(6, -3, 169, 179, 189),
    VehiclePixel(7, -3, 136, 146, 156),
    VehiclePixel(8, -3, 83, 93, 103),
    VehiclePixel(9, -3, 63, 75, 89),
    VehiclePixel(10, -3, 70, 77, 96),
    VehiclePixel(11, -3, 74, 81, 97),
    VehiclePixel(12, -3, 69, 76, 86),
    VehiclePixel(13, -3, 69, 76, 86),
    VehiclePixel(14, -3, 71, 75, 84),
    VehiclePixel(15, -3, 72, 76, 85),
    VehiclePixel(16, -3, 74, 78, 87),
    VehiclePixel(17, -3, 75, 79, 88),
    VehiclePixel(18, -3, 78, 82, 91),
    VehiclePixel(19, -3, 87, 85, 96),
    VehiclePixel(20, -3, 90, 87, 94),
    VehiclePixel(21, -3, 100, 89, 95),
    VehiclePixel(22, -3, 170, 0, 3),
    VehiclePixel(-21, -2, 4, 4, 4),
    VehiclePixel(-20, -2, 254, 255, 255),
    VehiclePixel(-19, -2, 239, 247, 250),
    VehiclePixel(-18, -2, 235, 244, 249),
    VehiclePixel(-17, -2, 228, 238, 240),
    VehiclePixel(-16, -2, 231, 240, 245),
    VehiclePixel(-15, -2, 246, 254, 255),
    VehiclePixel(-14, -2, 251, 254, 255),
    VehiclePixel(-13, -2, 249, 255, 255),
    VehiclePixel(-12, -2, 213, 220, 228),
    VehiclePixel(-11, -2, 137, 144, 154),
    VehiclePixel(-10, -2, 80, 86, 98),
    VehiclePixel(-9, -2, 67, 77, 89),
    VehiclePixel(-8, -2, 80, 89, 104),
    VehiclePixel(-7, -2, 70, 78, 97),
    VehiclePixel(-6, -2, 73, 85, 101),
    VehiclePixel(-5, -2, 76, 87, 105),
    VehiclePixel(-4, -2, 59, 70, 88),
    VehiclePixel(-3, -2, 71, 82, 100),
    VehiclePixel(-2, -2, 81, 92, 110),
    VehiclePixel(-1, -2, 77, 88, 106),
    VehiclePixel(0, -2, 72, 84, 100),
    VehiclePixel(1, -2, 74, 86, 102),
    VehiclePixel(2, -2, 73, 85, 99),
    VehiclePixel(3, -2, 77, 89, 103),
    VehiclePixel(4, -2, 81, 90, 105),
    VehiclePixel(5, -2, 80, 90, 102),
    VehiclePixel(6, -2, 86, 95, 110),
    VehiclePixel(7, -2, 82, 91, 106),
    VehiclePixel(8, -2, 80, 89, 104),
    VehiclePixel(9, -2, 76, 88, 102),
    VehiclePixel(10, -2, 75, 87, 103),
    VehiclePixel(11, -2, 76, 84, 97),
    VehiclePixel(12, -2, 79, 83, 94),
    VehiclePixel(13, -2, 82, 86, 97),
    VehiclePixel(14, -2, 80, 84, 95),
    VehiclePixel(15, -2, 80, 84, 95),
    VehiclePixel(16, -2, 80, 84, 93),
    VehiclePixel(17, -2, 85, 89, 101),
    VehiclePixel(18, -2, 87, 91, 103),
    VehiclePixel(19, -2, 84, 88, 99),
    VehiclePixel(20, -2, 84, 90, 102),
    VehiclePixel(21, -2, 92, 94, 107),
    VehiclePixel(22, -2, 31, 1, 1),
    VehiclePixel(-23, -1, 254, 255, 255),
    VehiclePixel(-22, -1, 253, 254, 255),
    VehiclePixel(-21, -1, 201, 204, 209),
    VehiclePixel(-20, -1, 142, 151, 160),
    VehiclePixel(-19, -1, 143, 153, 163),
    VehiclePixel(-18, -1, 149, 161, 173),
    VehiclePixel(-17, -1, 174, 187, 196),
    VehiclePixel(-16, -1, 187, 199, 211),
    VehiclePixel(-15, -1, 193, 201, 214),
    VehiclePixel(-14, -1, 92, 104, 118),
    VehiclePixel(-13, -1, 85, 93, 106),
    VehiclePixel(-12, -1, 95, 103, 116),
    VehiclePixel(-11, -1, 94, 104, 114),
    VehiclePixel(-10, -1, 90, 103, 112),
    VehiclePixel(-9, -1, 92, 102, 114),
    VehiclePixel(-8, -1, 93, 103, 115),
    VehiclePixel(-7, -1, 88, 98, 108),
    VehiclePixel(-6, -1, 89, 97, 110),
    VehiclePixel(-5, -1, 90, 100, 112),
    VehiclePixel(-4, -1, 88, 98, 110),
    VehiclePixel(-3, -1, 92, 100, 113),
    VehiclePixel(-2, -1, 95, 103, 116),
    VehiclePixel(-1, -1, 94, 102, 113),
    VehiclePixel(0, -1, 93, 101, 114),
    VehiclePixel(1, -1, 91, 99, 112),
    VehiclePixel(2, -1, 90, 98, 109),
    VehiclePixel(3, -1, 92, 100, 111),
    VehiclePixel(4, -1, 93, 101, 112),
    VehiclePixel(5, -1, 87, 97, 107),
    VehiclePixel(6, -1, 90, 100, 112),
    VehiclePixel(7, -1, 84, 93, 108),
    VehiclePixel(8, -1, 81, 90, 105),
    VehiclePixel(9, -1, 80, 89, 104),
    VehiclePixel(10, -1, 47, 58, 78),
    VehiclePixel(11, -1, 81, 93, 115),
    VehiclePixel(12, -1, 80, 92, 114),
    VehiclePixel(13, -1, 76, 88, 110),
    VehiclePixel(14, -1, 75, 87, 111),
    VehiclePixel(15, -1, 80, 88, 109),
    VehiclePixel(16, -1, 90, 98, 109),
    VehiclePixel(17, -1, 91, 101, 111),
    VehiclePixel(18, -1, 88, 92, 104),
    VehiclePixel(19, -1, 91, 95, 107),
    VehiclePixel(20, -1, 86, 92, 104),
    VehiclePixel(21, -1, 90, 96, 108),
    VehiclePixel(-23, 0, 130, 138, 151),
    VehiclePixel(-22, 0, 153, 161, 172),
    VehiclePixel(-21, 0, 155, 162, 170),
    VehiclePixel(-20, 0, 0, 0, 0),
    VehiclePixel(-19, 0, 1, 0, 0),
    VehiclePixel(-18, 0, 1, 1, 0),
    VehiclePixel(-17, 0, 0, 0, 0),
    VehiclePixel(-16, 0, 0, 0, 0),
    VehiclePixel(-15, 0, 0, 1, 0),
    VehiclePixel(-14, 0, 46, 51, 71),
    VehiclePixel(-13, 0, 101, 110, 119),
    VehiclePixel(-12, 0, 109, 118, 127),
    VehiclePixel(-11, 0, 109, 118, 127),
    VehiclePixel(-10, 0, 107, 116, 125),
    VehiclePixel(-9, 0, 110, 119, 128),
    VehiclePixel(-8, 0, 111, 120, 129),
    VehiclePixel(-7, 0, 112, 121, 130),
    VehiclePixel(-6, 0, 111, 115, 126),
    VehiclePixel(-5, 0, 107, 114, 124),
    VehiclePixel(-4, 0, 111, 118, 128),
    VehiclePixel(-3, 0, 111, 118, 128),
    VehiclePixel(-2, 0, 114, 121, 131),
    VehiclePixel(-1, 0, 114, 121, 129),
    VehiclePixel(0, 0, 115, 119, 128),
    VehiclePixel(1, 0, 114, 118, 127),
    VehiclePixel(2, 0, 113, 117, 126),
    VehiclePixel(3, 0, 117, 121, 130),
    VehiclePixel(4, 0, 116, 120, 129),
    VehiclePixel(5, 0, 116, 120, 129),
    VehiclePixel(6, 0, 115, 119, 128),
    VehiclePixel(7, 0, 106, 112, 124),
    VehiclePixel(8, 0, 98, 106, 119),
    VehiclePixel(9, 0, 89, 96, 115),
    VehiclePixel(10, 0, 0, 0, 5),
    VehiclePixel(11, 0, 0, 1, 0),
    VehiclePixel(12, 0, 1, 1, 1),
    VehiclePixel(13, 0, 0, 0, 0),
    VehiclePixel(14, 0, 0, 0, 0),
    VehiclePixel(15, 0, 0, 0, 0),
    VehiclePixel(16, 0, 23, 26, 35),
    VehiclePixel(17, 0, 100, 107, 117),
    VehiclePixel(18, 0, 102, 106, 117),
    VehiclePixel(19, 0, 104, 108, 119),
    VehiclePixel(20, 0, 102, 108, 120),
    VehiclePixel(21, 0, 107, 111, 122),
    VehiclePixel(-23, 1, 69, 71, 83),
    VehiclePixel(-22, 1, 121, 113, 128),
    VehiclePixel(-21, 1, 25, 28, 43),
    VehiclePixel(-20, 1, 2, 0, 1),
    VehiclePixel(-19, 1, 0, 2, 14),
    VehiclePixel(-18, 1, 1, 6, 25),
    VehiclePixel(-17, 1, 9, 15, 31),
    VehiclePixel(-16, 1, 16, 16, 28),
    VehiclePixel(-15, 1, 0, 0, 0),
    VehiclePixel(-14, 1, 4, 4, 16),
    VehiclePixel(-13, 1, 108, 112, 123),
    VehiclePixel(-12, 1, 120, 127, 133),
    VehiclePixel(-11, 1, 126, 131, 137),
    VehiclePixel(-10, 1, 126, 131, 137),
    VehiclePixel(-9, 1, 126, 131, 137),
    VehiclePixel(-8, 1, 128, 132, 141),
    VehiclePixel(-7, 1, 128, 133, 139),
    VehiclePixel(-6, 1, 130, 135, 141),
    VehiclePixel(-5, 1, 128, 131, 140),
    VehiclePixel(-4, 1, 129, 132, 141),
    VehiclePixel(-3, 1, 127, 130, 135),
    VehiclePixel(-2, 1, 133, 136, 141),
    VehiclePixel(-1, 1, 134, 139, 143),
    VehiclePixel(0, 1, 132, 135, 140),
    VehiclePixel(1, 1, 134, 135, 139),
    VehiclePixel(2, 1, 132, 135, 140),
    VehiclePixel(3, 1, 130, 133, 140),
    VehiclePixel(4, 1, 129, 128, 136),
    VehiclePixel(5, 1, 128, 126, 137),
    VehiclePixel(6, 1, 125, 125, 135),
    VehiclePixel(7, 1, 122, 125, 132),
    VehiclePixel(8, 1, 109, 115, 129),
    VehiclePixel(9, 1, 34, 37, 56),
    VehiclePixel(10, 1, 3, 2, 0),
    VehiclePixel(11, 1, 3, 4, 9),
    VehiclePixel(12, 1, 4, 8, 20),
    VehiclePixel(13, 1, 13, 16, 31),
    VehiclePixel(14, 1, 11, 11, 23),
    VehiclePixel(15, 1, 5, 4, 9),
    VehiclePixel(16, 1, 0, 0, 0),
    VehiclePixel(17, 1, 35, 35, 47),
    VehiclePixel(18, 1, 120, 123, 132),
    VehiclePixel(19, 1, 124, 127, 134),
    VehiclePixel(20, 1, 122, 127, 133),
    VehiclePixel(21, 1, 127, 126, 132),
    VehiclePixel(-23, 2, 218, 175, 156),
    VehiclePixel(-22, 2, 144, 105, 100),
    VehiclePixel(-21, 2, 0, 0, 2),
    VehiclePixel(-20, 2, 10, 12, 27),
    VehiclePixel(-19, 2, 10, 12, 25),
    VehiclePixel(-18, 2, 0, 0, 9),
    VehiclePixel(-17, 2, 61, 61, 69),
    VehiclePixel(-16, 2, 26, 28, 43),
    VehiclePixel(-15, 2, 13, 15, 28),
    VehiclePixel(-14, 2, 0, 0, 0),
    VehiclePixel(-13, 2, 6, 8, 21),
    VehiclePixel(-12, 2, 126, 133, 139),
    VehiclePixel(-11, 2, 132, 137, 143),
    VehiclePixel(-10, 2, 133, 138, 144),
    VehiclePixel(-9, 2, 135, 140, 146),
    VehiclePixel(-8, 2, 136, 140, 149),
    VehiclePixel(-7, 2, 135, 140, 144),
    VehiclePixel(-6, 2, 136, 141, 147),
    VehiclePixel(-5, 2, 132, 137, 143),
    VehiclePixel(-4, 2, 127, 132, 138),
    VehiclePixel(-3, 2, 124, 129, 135),
    VehiclePixel(-2, 2, 127, 132, 138),
    VehiclePixel(-1, 2, 135, 138, 143),
    VehiclePixel(0, 2, 135, 138, 143),
    VehiclePixel(1, 2, 135, 138, 143),
    VehiclePixel(2, 2, 132, 137, 141),
    VehiclePixel(3, 2, 130, 134, 143),
    VehiclePixel(4, 2, 132, 132, 140),
    VehiclePixel(5, 2, 125, 125, 133),
    VehiclePixel(6, 2, 129, 129, 137),
    VehiclePixel(7, 2, 124, 129, 132),
    VehiclePixel(8, 2, 34, 36, 59),
    VehiclePixel(9, 2, 2, 1, 0),
    VehiclePixel(10, 2, 4, 7, 16),
    VehiclePixel(11, 2, 1, 3, 15),
    VehiclePixel(12, 2, 10, 10, 18),
    VehiclePixel(13, 2, 0, 1, 3),
    VehiclePixel(14, 2, 27, 27, 35),
    VehiclePixel(15, 2, 21, 23, 36),
    VehiclePixel(16, 2, 0, 0, 2),
    VehiclePixel(17, 2, 0, 0, 2),
    VehiclePixel(18, 2, 6, 1, 7),
    VehiclePixel(19, 2, 13, 7, 11),
    VehiclePixel(20, 2, 33, 28, 34),
    VehiclePixel(21, 2, 82, 73, 76),
    VehiclePixel(-23, 3, 1, 0, 0),
    VehiclePixel(-22, 3, 1, 0, 0),
    VehiclePixel(-21, 3, 0, 0, 2),
    VehiclePixel(-20, 3, 10, 14, 26),
    VehiclePixel(-19, 3, 1, 2, 7),
    VehiclePixel(-18, 3, 0, 0, 2),
    VehiclePixel(-17, 3, 0, 0, 0),
    VehiclePixel(-16, 3, 0, 0, 0),
    VehiclePixel(-15, 3, 7, 7, 19),
    VehiclePixel(-14, 3, 0, 0, 4),
    VehiclePixel(-13, 3, 1, 1, 1),
    VehiclePixel(-12, 3, 68, 75, 85),
    VehiclePixel(-11, 3, 71, 81, 91),
    VehiclePixel(-10, 3, 65, 75, 87),
    VehiclePixel(-9, 3, 58, 68, 80),
    VehiclePixel(-8, 3, 53, 65, 79),
    VehiclePixel(-7, 3, 51, 63, 77),
    VehiclePixel(-6, 3, 46, 58, 72),
    VehiclePixel(-5, 3, 46, 60, 73),
    VehiclePixel(-4, 3, 45, 59, 72),
    VehiclePixel(-3, 3, 53, 65, 79),
    VehiclePixel(-2, 3, 52, 64, 78),
    VehiclePixel(-1, 3, 52, 66, 75),
    VehiclePixel(0, 3, 52, 65, 74),
    VehiclePixel(1, 3, 53, 66, 75),
    VehiclePixel(2, 3, 50, 64, 73),
    VehiclePixel(3, 3, 49, 63, 72),
    VehiclePixel(4, 3, 49, 62, 78),
    VehiclePixel(5, 3, 49, 62, 78),
    VehiclePixel(6, 3, 46, 60, 73),
    VehiclePixel(7, 3, 43, 54, 72),
    VehiclePixel(8, 3, 1, 0, 2),
    VehiclePixel(9, 3, 0, 0, 0),
    VehiclePixel(10, 3, 6, 9, 18),
    VehiclePixel(11, 3, 10, 11, 15),
    VehiclePixel(12, 3, 0, 0, 2),
    VehiclePixel(13, 3, 6, 7, 12),
    VehiclePixel(14, 3, 0, 1, 5),
    VehiclePixel(15, 3, 15, 15, 23),
    VehiclePixel(16, 3, 5, 5, 13),
    VehiclePixel(17, 3, 7, 1, 1),
    VehiclePixel(18, 3, 26, 20, 24),
    VehiclePixel(19, 3, 42, 36, 40),
    VehiclePixel(20, 3, 70, 59, 67),
    VehiclePixel(-21, 4, 8, 11, 18),
    VehiclePixel(-20, 4, 2, 0, 6),
    VehiclePixel(-19, 4, 3, 4, 9),
    VehiclePixel(-18, 4, 0, 0, 2),
    VehiclePixel(-17, 4, 0, 2, 5),
    VehiclePixel(-16, 4, 0, 0, 9),
    VehiclePixel(-15, 4, 0, 0, 4),
    VehiclePixel(-14, 4, 0, 0, 4),
    VehiclePixel(-13, 4, 0, 1, 0),
    VehiclePixel(-12, 4, 0, 0, 2),
    VehiclePixel(-11, 4, 0, 0, 2),
    VehiclePixel(-10, 4, 0, 0, 2),
    VehiclePixel(-9, 4, 0, 0, 2),
    VehiclePixel(-8, 4, 0, 2, 1),
    VehiclePixel(-7, 4, 0, 1, 0),
    VehiclePixel(-6, 4, 0, 0, 4),
    VehiclePixel(-5, 4, 0, 0, 4),
    VehiclePixel(-4, 4, 0, 0, 2),
    VehiclePixel(-3, 4, 1, 1, 3),
    VehiclePixel(-2, 4, 0, 0, 2),
    VehiclePixel(-1, 4, 0, 1, 3),
    VehiclePixel(0, 4, 0, 0, 2),
    VehiclePixel(1, 4, 0, 0, 2),
    VehiclePixel(2, 4, 6, 6, 6),
    VehiclePixel(3, 4, 6, 6, 6),
    VehiclePixel(4, 4, 2, 3, 7),
    VehiclePixel(5, 4, 0, 0, 5),
    VehiclePixel(6, 4, 2, 1, 7),
    VehiclePixel(7, 4, 0, 3, 12),
    VehiclePixel(8, 4, 1, 2, 6),
    VehiclePixel(9, 4, 0, 0, 0),
    VehiclePixel(10, 4, 7, 7, 15),
    VehiclePixel(11, 4, 3, 6, 11),
    VehiclePixel(12, 4, 6, 5, 13),
    VehiclePixel(13, 4, 0, 0, 7),
    VehiclePixel(14, 4, 6, 7, 11),
    VehiclePixel(15, 4, 18, 17, 22),
    VehiclePixel(16, 4, 27, 26, 32),
    VehiclePixel(17, 4, 29, 23, 23),
    VehiclePixel(18, 4, 78, 72, 76),
    VehiclePixel(-20, 5, 0, 0, 7),
    VehiclePixel(-19, 5, 0, 2, 11),
    VehiclePixel(-18, 5, 2, 3, 8),
    VehiclePixel(-17, 5, 0, 0, 5),
    VehiclePixel(-16, 5, 18, 22, 34),
    VehiclePixel(-15, 5, 1, 0, 8),
    VehiclePixel(-14, 5, 15, 15, 15),
    VehiclePixel(-13, 5, 150, 153, 158),
    VehiclePixel(9, 5, 0, 0, 0),
    VehiclePixel(10, 5, 12, 12, 20),
    VehiclePixel(11, 5, 1, 1, 9),
    VehiclePixel(12, 5, 8, 8, 20),
    VehiclePixel(13, 5, 13, 13, 21),
    VehiclePixel(14, 5, 41, 41, 49),
    VehiclePixel(15, 5, 18, 15, 22),
    VehiclePixel(16, 5, 64, 62, 67),
    VehiclePixel(-20, 6, 3, 2, 7),
    VehiclePixel(-19, 6, 0, 0, 7),
    VehiclePixel(-18, 6, 11, 14, 29),
    VehiclePixel(-17, 6, 28, 32, 44),
    VehiclePixel(-16, 6, 2, 2, 4),
    VehiclePixel(-15, 6, 0, 0, 4),
    VehiclePixel(-14, 6, 16, 15, 20),
    VehiclePixel(10, 6, 1, 2, 4),
    VehiclePixel(11, 6, 13, 14, 16),
    VehiclePixel(12, 6, 26, 29, 36),
    VehiclePixel(13, 6, 41, 44, 53),
    VehiclePixel(14, 6, 48, 47, 53),
    VehiclePixel(15, 6, 40, 39, 44),
    VehiclePixel(-19, 7, 1, 0, 5),
    VehiclePixel(-18, 7, 1, 0, 5),
    VehiclePixel(-17, 7, 1, 0, 5),
    VehiclePixel(-16, 7, 0, 0, 2),
    VehiclePixel(11, 7, 54, 54, 54),
    VehiclePixel(12, 7, 61, 62, 67),
    VehiclePixel(13, 7, 70, 71, 76),
    VehiclePixel(14, 7, 81, 80, 85)
});