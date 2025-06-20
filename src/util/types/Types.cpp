#include "Types.h"



float deltaAngle(float a, float b) {
    return fmod((b - a + 540.0f), 360.0f) - 180.0f;
}

// v2i
v2i v2i::up() { return v2i(0, 1); }
v2i v2i::down() { return v2i(0, -1); }
v2i v2i::left() { return v2i(-1, 0); }
v2i v2i::right() { return v2i(1, 0); }

v2f v2i::toV2f() const {
    return v2f((float)x, (float)y);
}

int v2i::lengthSq() const {
    return x * x + y * y;
}

float v2i::length() const {
    return sqrtf((float)lengthSq());
}

v2i v2i::explode() const {
    int xAbs = abs(x), yAbs = abs(y);
    return v2i(x * (xAbs > yAbs), y * (yAbs > xAbs));
}

v2i v2i::operator+(v2i b) const {
    return v2i(x + b.x, y + b.y);
}

v2i v2i::operator-(v2i b) const {
    return v2i(x - b.x, y - b.y);
}

v2i v2i::operator*(int b) const {
    return v2i(x * b, y * b);
}

v2i v2i::operator/(int b) const {
    return v2i(x / b, y / b);
}

v2f v2i::operator/(float b) const {
    return v2f(x / b, y / b);
}

bool v2i::operator==(v2i b) const {
    return x == b.x && y == b.y;
}

// v2f
v2f v2f::fromAngle(int angle) { 
    float rads = (float)angle * DEG2RAD;
    return v2f(sinf(rads), -cosf(rads));
}

v2f v2f::round() const {
    return v2f(roundf(x), roundf(y));
}

float v2f::lengthSq() const {
    return x * x + y * y;
}

float v2f::length() const {
    return sqrtf(lengthSq());
}

v2i v2f::roundToV2i() const {
    return v2i((int)roundf(x), (int)roundf(y));
}

float v2f::dot(v2f rhs) const {
    return x * rhs.x + y * rhs.y;
}

float v2f::det(v2f rhs) const {
    return x * rhs.y - y * rhs.x;
}   

float v2f::signedAngle(v2f rhs) const {
    return atan2f(det(rhs), dot(rhs));
}

v2f v2f::explode() const {
    float xAbs = fabsf(x), yAbs = fabsf(y);
    return v2f(x * (xAbs > yAbs), y * (yAbs > xAbs));
}

v2f v2f::normalize() const {
    float len = length();
    if (len == 0) return v2f(0, 0);
    return v2f(x / len, y / len);
}

v2f v2f::operator+(const v2f & b) const {
    return v2f(x + b.x, y + b.y);
}

v2f v2f::operator-(const v2f & b) const {
    return v2f(x - b.x, y - b.y);
}

v2f v2f::operator*(float b) const {
    return v2f(x * b, y * b);
}

v2f v2f::operator/(float b) const {
    return v2f(x / b, y / b);
}

v3f v2f::toFlatV3F() const {
    return v3f(x, 0, y);
}

// v3f
v3f v3f::operator/(float b) const {
    return v3f(x / b, y / b, z / b);
}

// MapCell
bool MapCell::is_discovered() const { return (value & (1 << 0)) != 0; }
bool MapCell::is_wall_north() const { return (value & (1 << 1)) != 0; }
bool MapCell::is_wall_east() const { return (value & (1 << 2)) != 0; }
bool MapCell::is_wall_south() const { return (value & (1 << 3)) != 0; }
bool MapCell::is_wall_west() const { return (value & (1 << 4)) != 0; }
bool MapCell::is_wall_highlight() const { return (value & (1 << 5)) != 0; }
bool MapCell::is_wall_in_dir(v2i dir) const {
    int xByte = ((int)powf(((1 - (dir.x + 1) / 2) + 1) & 3, 2) << 2) * (dir.x != 0);
    int yByte = ((int)powf(((1 - (dir.y + 1) / 2) + 1) & 3, 2) << 1) * (dir.y != 0);
    int byte = xByte + yByte;
    return (value & byte) == byte;
}

void MapCell::set_discovered(bool v) {
    value = (value & ~(1 << 0)) | (v << 0);
}

void MapCell::set_wall_north(bool v) {
    value = (value & ~(1 << 1)) | (v << 1);
}

void MapCell::set_wall_east(bool v) {
    value = (value & ~(1 << 2)) | (v << 2);
}

void MapCell::set_wall_south(bool v) {
    value = (value & ~(1 << 3)) | (v << 3);
}

void MapCell::set_wall_west(bool v) {
    value = (value & ~(1 << 4)) | (v << 4);
}

void MapCell::set_wall_highlight(bool v) {
    value = (value & ~(1 << 5)) | (v << 5);
}

// Map
bool Map::is_in_bounds(v2i pos) const {
    return pos.x >= 0 && pos.x < size.x && pos.y >= 0 && pos.y < size.y;
}

MapCell* Map::get_cell(v2i pos) const {
    if (!is_in_bounds(pos)) return nullptr;
    return &cells[pos.y * size.x + pos.x];
}
