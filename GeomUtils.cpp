#include "GeomUtils.h"
#include <algorithm>
#include <cmath>



bool Less(double a, double b)
{
    return a < b - EPS;
}

bool LessEqual(double a, double b)
{
    return a < b + EPS;
}

bool Greater(double a, double b)
{
    return a > b + EPS;
}

bool GreaterEqual(double a, double b)
{
    return a > b - EPS;
}

bool IsAngleInRange(double angle, double start, double end)
{
    
    if (Greater(start, end))
    {
        double diff = 2 * PI - start;
        start = 0;
        end += diff;
        if (Greater(end, 2 * PI))
            end -= 2 * PI;

        angle += diff;
        if (Greater(angle, 2 * PI))
            angle -= 2 * PI;
    }

    return Less(angle, end) && Greater(angle, start);
}

Rect GetBBox(const Arc& arc)
{
    BoundingBoxCalcer calc;
    calc.PushPoint(arc.GetStartPoint());
    calc.PushPoint(arc.GetEndPoint());

    if (IsAngleInRange(0, arc.m_start, arc.m_end))
    {
        Vec2 p(arc.m_center.m_x + arc.m_rad, arc.m_center.m_y);
        calc.PushPoint(p);
    }
    if (IsAngleInRange(PI * 0.5, arc.m_start, arc.m_end))
    {
        Vec2 p(arc.m_center.m_x, arc.m_center.m_y + arc.m_rad);
        calc.PushPoint(p);
    }
    if (IsAngleInRange(PI, arc.m_start, arc.m_end))
    {
        Vec2 p(arc.m_center.m_x - arc.m_rad, arc.m_center.m_y);
        calc.PushPoint(p);
    }
    if (IsAngleInRange(PI * 1.5, arc.m_start, arc.m_end))
    {
        Vec2 p(arc.m_center.m_x, arc.m_center.m_y - arc.m_rad);
        calc.PushPoint(p);
    }

    return calc.GetBBox();
}

Vec2 RotateBy(const Vec2& point, const Vec2& center, double angle)
{
    double x = (point.m_x - center.m_x) * std::cos(angle) - (point.m_y - center.m_y) * std::sin(angle) + center.m_x;
    double y = (point.m_y - center.m_y) * std::cos(angle) + (point.m_x - center.m_x) * std::sin(angle) + center.m_y;
    return Vec2(x, y);
}

ERectsRelation CalcRectsRelation(const Rect& rect1, const Rect& rect2)
{
    if (LessEqual(rect1.m_max.m_x, rect2.m_min.m_x) || LessEqual(rect1.m_max.m_y, rect2.m_min.m_y) ||
        GreaterEqual(rect1.m_min.m_x, rect2.m_max.m_x) || GreaterEqual(rect1.m_min.m_y, rect2.m_max.m_y))
        return ERectsRelation::Outside;

    if (LessEqual(rect1.m_max.m_x, rect2.m_max.m_x) || LessEqual(rect1.m_max.m_y, rect2.m_max.m_y) ||
        GreaterEqual(rect1.m_min.m_x, rect2.m_min.m_x) || GreaterEqual(rect1.m_min.m_y, rect2.m_min.m_y))
        return ERectsRelation::Inside;
    
    return ERectsRelation::Intersection;
}

double PointDistance(const Vec2& p1, const Vec2& p2)
{
    return std::sqrt((p1.m_x - p2.m_x) * (p1.m_x - p2.m_x) + (p1.m_y - p2.m_y) * (p1.m_y - p2.m_y));
}

std::vector<Vec2> CircleIntersect(const Circle& cir1, const Circle& cir2)
{
    double dis = PointDistance(cir1.m_center, cir2.m_center);
    if (GreaterEqual(dis, cir1.m_rad + cir2.m_rad) || LessEqual(dis, std::abs(cir1.m_rad - cir2.m_rad)))
        return {};

    double a = ((cir1.m_rad * cir1.m_rad) - (cir2.m_rad * cir2.m_rad) + dis * dis) / (2 * dis);
    double h = std::sqrt(cir1.m_rad * cir1.m_rad - a * a);

    Vec2 p2(cir1.m_center.m_x + a * (cir2.m_center.m_x - cir1.m_center.m_x) / dis,
        cir1.m_center.m_y + a * (cir2.m_center.m_y - cir1.m_center.m_y) / dis);

    Vec2 p3(p2.m_x + h * (cir2.m_center.m_y - cir1.m_center.m_y) / dis,
        p2.m_y - h * (cir2.m_center.m_x - cir1.m_center.m_x) / dis);

    Vec2 p4(p2.m_x - h * (cir2.m_center.m_y - cir1.m_center.m_y) / dis,
        p2.m_y + h * (cir2.m_center.m_x - cir1.m_center.m_x) / dis);
    return { p3,p4 };
}

void BoundingBoxCalcer::PushPoint(const Vec2& Point)
{
    m_min.m_x = std::min(m_min.m_x, Point.m_x);
    m_min.m_y = std::min(m_min.m_y, Point.m_y);
    m_max.m_x = std::max(m_max.m_x, Point.m_x);
    m_max.m_y = std::max(m_max.m_y, Point.m_y);
}

Rect BoundingBoxCalcer::GetBBox()
{
    return Rect(m_min, m_max);
}

Vec2 Arc::GetStartPoint() const
{
    Vec2 p(m_center.m_x + m_rad, m_center.m_y);
    return RotateBy(p, m_center, m_start);
}

Vec2 Arc::GetEndPoint() const
{
    Vec2 p(m_center.m_x + m_rad, m_center.m_y);
    return RotateBy(p, m_center, m_end);
}

ArcRect::ArcRect(double cir_rad, const Vec2& swing_center, double swing_rad, double swing_start, double swing_end)
{
    if (Greater(swing_rad, cir_rad))
    {
        m_edges.resize(4);
        {
            double sangle = swing_start - PI;
            double eangle = sangle + PI;
            Vec2 pos = RotateBy(Vec2(swing_center.m_x + swing_rad, 0), swing_center, swing_start);
            Arc arc(pos, cir_rad, sangle, eangle);
            m_edges[2] = arc;
        }
        {
            double eangle = swing_end - PI;
            double sangle = sangle - PI;
            Vec2 pos = RotateBy(Vec2(swing_center.m_x + swing_rad, 0), swing_center, swing_start);
            Arc arc(pos, cir_rad, sangle, eangle);
            m_edges[0] = arc;
        }
        {
            Arc arc(swing_center, swing_rad - cir_rad, swing_start, swing_end);
            m_edges[1] = arc;
        }
        {
            Arc arc(swing_center, swing_rad + cir_rad, swing_start, swing_end);
            m_edges[3] = arc;
        }
    }
    else
    {
        m_edges.resize(3);
        Arc bot(swing_center, swing_rad + cir_rad, swing_start, swing_end);
        m_edges[2] = bot;


    }
}
