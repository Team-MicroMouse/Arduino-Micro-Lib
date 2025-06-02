#ifndef MATH_H
#define MATH_H

#include <stdint.h>
#include <math.h>

const float PI = 3.14159265358979323846f;
const float DEG2RAD = PI / 180.0f;
const float RAD2DEG = 180.0f / PI;
const int CELL_SIZE = 180;
const float CELL_SIZE_F = (float)(CELL_SIZE);

struct v2f;
struct v3f;

struct v2i { 
    uint8_t x, y;

    v2i() : x(0), y(0) {}
    v2i(int x, int y) : x(x), y(y) {}

    static v2i up();
    static v2i down();
    static v2i left();
    static v2i right();

    v2f toV2f() const;

    int lengthSq() const;
    float length() const;
    v2i explode() const;

    v2i operator+(v2i b) const;
    v2i operator-(v2i b) const;
    v2i operator*(int b) const;
    v2i operator/(int b) const;
    v2f operator/(float b) const;
    bool operator==(v2i b) const;
};

struct v2f { 
    float x, y;

    v2f() : x(0), y(0) {}
    v2f(float x, float y) : x(x), y(y) {}

    static v2f fromAngle(int angle);

    v3f toFlatV3F() const;
    v2f round() const; 
    float lengthSq() const;
    float length() const;
    v2i roundToV2i() const;
    v2f explode() const;
    v2f normalize() const; 

    v2f operator+(const v2f & b) const;
    v2f operator-(const v2f & b) const;
    v2f operator*(float b) const;
    v2f operator/(float b) const;
    bool operator==(const v2f & b) const { return x == b.x && y == b.y; }
};

struct v3i { int x, y, z; v3i() : x(0), y(0), z(0) {} v3i(int x, int y, int z): x(x), y(y), z(z) {} };
struct v3f {
    float x, y, z;
    v3f() : x(0), y(0), z(0) {}
    v3f(float x, float y, float z) : x(x), y(y), z(z) {}
    v3f operator/(float b) const;
};

struct Guid { uint64_t a, b; };

struct RobotPosition {
    v2i position; 
    int angle; 
};

struct MapCell {
    int value;
    MapCell() : value(0) {}

    bool is_discovered() const; 
    bool is_wall_north() const;
    bool is_wall_east() const;
    bool is_wall_south() const;
    bool is_wall_west() const;
    bool is_wall_highlight() const; 
    bool is_wall_in_dir(v2i dir) const;

    void set_discovered(bool value);
    void set_wall_north(bool value);
    void set_wall_east(bool value);
    void set_wall_south(bool value);
    void set_wall_west(bool value);
    void set_wall_highlight(bool value);
};

struct Map { 
    MapCell* cells;
    v2i size;

    bool is_in_bounds(v2i pos) const;
    MapCell* get_cell(v2i pos) const;
};

struct v2iHasher { 
    size_t operator()(const v2i& v) const;
};

#endif