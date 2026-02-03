#pragma once
#include <cfloat>
#include <vector>
#include <map>
#include <set>
#include <array>

const double EPS = 0.0001;
const double PI = 3.1415926;

bool Less(double a, double b);
bool LessEqual(double a, double b);
bool Greater(double a, double b);
bool GreaterEqual(double a, double b);
bool Equal(double a, double b);

struct Vec2
{
	double m_x;
	double m_y;

	Vec2(){}
	Vec2(double x, double y) :m_x(x), m_y(y) { }

	Vec2 operator+(const Vec2& other) const;
	Vec2 operator-(const Vec2& other) const;
	Vec2 operator*(double d) const;

	double Dot(const Vec2& other) const;
	double Cross(const Vec2& other) const;
	Vec2 Unit() const;
};

struct Vec2Cmp
{
	bool operator()(const Vec2& v1, const Vec2& v2)const
	{
		if (Equal(v1.m_y, v2.m_y))
			return Less(v1.m_x, v2.m_x);
		else
			return Less(v1.m_y, v2.m_y);
	}
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
	void PushPoint(const Vec2& point);
	void PushRect(const Rect& rect);
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

class UndirectedGraph
{
public:
	void AddEdge(int node1_id, int node2_id, int edge_id);

public:
	struct Edge
	{
		int m_to_node = -1;
		int m_edge_id = -1;
	};
	struct Node
	{
		int m_node_id = -1;
		std::vector<Edge> m_edges;
	};

	std::map<int, Node> m_nodes;
};

class ArcRectOutlineCalc
{
public:
	std::vector<Arc> CalcOutline(const std::vector<ArcRect>& rects);

private:
	void InitRectData(const std::vector<ArcRect>& rects);
	void CreateGraph();
	std::vector<Arc> TraversGraph();

	std::array<std::vector<Vec2>, 2> ArcIntersectionPoint(const Arc& arc1, const Arc& arc2);

	int IsPointOnArc(const Arc& arc, const Vec2& point);

	int GetPointID(const Vec2& point);

	Arc GetTravGraphStart();

	Vec2 GetArcTangent(const Arc& arc, const Vec2& start);

public:
	struct ArcRectData
	{
		ArcRect m_rect;
		Rect m_bbox;
		std::vector<Rect> m_arc_bbox;
	};
	std::vector<ArcRectData> m_rects_data;

	std::map < std::pair<int, int>, std::set<Vec2, Vec2Cmp>> m_arc_intersection;

	std::vector<Arc> m_all_arcs;

	int m_next_point_id = 0;
	std::map<Vec2, int, Vec2Cmp> m_pointid_map;
	std::vector<Vec2> m_all_points;

	UndirectedGraph m_graph;
};