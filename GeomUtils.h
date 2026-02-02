#pragma once
#include <cfloat>
#include <vector>

const double EPS = 0.000001;
const double PI = 3.1415926;

bool Less(double a, double b);
bool LessEqual(double a, double b);
bool Greater(double a, double b);
bool GreaterEqual(double a, double b);

struct Vec2
{
	double m_x;
	double m_y;

	Vec2(){}
	Vec2(double x, double y) :m_x(x), m_y(y) { }
};

struct Arc
{
	Vec2 m_center;
	double m_rad;
	//ÄæÊ±Õë
	double m_start;
	double m_end;

	Arc(){}
	Arc(const Vec2& center, double rad, double start, double end) :m_center(center), m_rad(rad), m_start(start), m_end(end) { }
	Vec2 GetStartPoint() const;
	Vec2 GetEndPoint() const;
};

struct Rect
{
	Vec2 m_min;
	Vec2 m_max;

	Rect(){}
	Rect(const Vec2& min, const Vec2& max) :m_min(min), m_max(max) { }
};

//angle [0,2PI]
bool IsAngleInRange(double angle, double start, double end);

class BoundingBoxCalcer
{
public:
	void PushPoint(const Vec2& Point);
	Rect GetBBox();

private:
	Vec2 m_min{ DBL_MAX, DBL_MAX };
	Vec2 m_max{ -DBL_MAX, -DBL_MAX };
};

Rect GetBBox(const Arc& arc);

Vec2 RotateBy(const Vec2& point, const Vec2& center, double angle);

enum class ERectsRelation
{
	Outside,
	Inside,
	Intersection
};
ERectsRelation CalcRectsRelation(const Rect& rect1, const Rect& rect2);

struct ArcRect
{
	//ÓÒ¡¢(ÉÏ)¡¢×ó¡¢ÏÂ
	std::vector<Arc> m_edges;

	ArcRect(){}
	ArcRect(double cir_rad, const Vec2& swing_center, double swing_rad, double swing_start, double swing_end);
};

struct Circle
{
	Vec2 m_center;
	double m_rad;

	Circle(){}
	Circle(const Vec2& center, double rad) :m_center(center), m_rad(rad) { }
};

double PointDistance(const Vec2& p1, const Vec2& p2);

std::vector<Vec2> CircleIntersect(const Circle& cir1, const Circle& cir2);