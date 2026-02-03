#include "GeomUtils.h"
#include <algorithm>
#include <cmath>
#include <cassert>



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

bool Equal(double a, double b)
{
    return a > b - EPS && a < b + EPS;
}

bool IsAngleInRange(double angle, double start, double end)
{
    if (angle < 0)
        angle += 2 * PI;
    if (start < 0)
        start += 2 * PI;
    if (end < 0)
        end += 2 * PI;

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

void BoundingBoxCalcer::PushPoint(const Vec2& point)
{
    m_min.m_x = std::min(m_min.m_x, point.m_x);
    m_min.m_y = std::min(m_min.m_y, point.m_y);
    m_max.m_x = std::max(m_max.m_x, point.m_x);
    m_max.m_y = std::max(m_max.m_y, point.m_y);
}

void BoundingBoxCalcer::PushRect(const Rect& rect)
{
    PushPoint(rect.m_min);
    PushPoint(rect.m_max);
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
    if (swing_start < 0)
        swing_start += 2 * PI;
    if (swing_end < 0)
        swing_end += 2 * PI;
    if (Greater(swing_rad, cir_rad))
    {
        m_edges.resize(4);
        {
            double sangle = swing_start - PI;
            double eangle = sangle + PI;
            Vec2 pos = RotateBy(swing_center + Vec2(swing_rad, 0), swing_center, swing_start);
            Arc arc(pos, cir_rad, sangle, eangle);
            m_edges[2] = arc;
        }
        {
            double eangle = swing_end - PI;
            double sangle = eangle - PI;
            Vec2 pos = RotateBy(swing_center + Vec2(swing_rad, 0), swing_center, swing_end);
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

        Vec2 s_cirpos = RotateBy(Vec2(swing_center.m_x + swing_rad, swing_center.m_y), swing_center, swing_start);
        Vec2 e_cirpos = RotateBy(Vec2(swing_center.m_x + swing_rad, swing_center.m_y), swing_center, swing_end);
        Circle s_cir(s_cirpos, cir_rad);
        Circle e_cir(e_cirpos, cir_rad);

        auto res = CircleIntersect(s_cir, e_cir);
        if (res.size() != 2)
            assert(false);

        double across = swing_end - swing_start;
        if (across < 0)
            across += 2 * PI;
        Vec2 mid = RotateBy(s_cirpos, swing_center, across * 0.5);
        Vec2 top_pos;
        if ((res[0] - swing_center).Dot(mid) < 0)
            top_pos = res[0];
        else if ((res[1] - swing_center).Dot(mid) < 0)
            top_pos = res[1];
        else
            assert(false);

        {
            double sangle = std::atan2(top_pos.m_y - s_cirpos.m_y, top_pos.m_x - s_cirpos.m_x);
            double eangle = swing_start;
            m_edges[1] = Arc(s_cirpos, cir_rad, sangle, eangle);
        }

        {
            double eangle = std::atan2(top_pos.m_y - e_cirpos.m_y, top_pos.m_x - e_cirpos.m_x);
            double sangle = swing_end;
            m_edges[0] = Arc(e_cirpos, cir_rad, sangle, eangle);
        }
    }
}

Vec2 Vec2::operator+(const Vec2& other) const
{
    return Vec2(m_x + other.m_x, m_y + other.m_y);
}

Vec2 Vec2::operator-(const Vec2& other) const
{
    return Vec2(m_x - other.m_x, m_y - other.m_y);
}

Vec2 Vec2::operator*(double d) const
{
    return Vec2(m_x * d, m_y * d);
}

double Vec2::Dot(const Vec2& other) const
{
    return m_x * other.m_x + m_y * other.m_y;
}

double Vec2::Cross(const Vec2& other) const
{
    return m_x * other.m_y - m_y * other.m_x;
}

Vec2 Vec2::Unit() const
{
    double model = std::sqrt(m_x * m_x + m_y * m_y);
    if (Equal(model, 0))
        return *this;
    return Vec2(m_x / model, m_y / model);
}

void UndirectedGraph::AddEdge(int node1_id, int node2_id, int edge_id)
{
    Node& node1 = m_nodes[node1_id];
    node1.m_node_id = node1_id;
    Edge edge1;
    edge1.m_edge_id = edge_id;
    edge1.m_to_node = node2_id;
    node1.m_edges.push_back(edge1);

    Node& node2 = m_nodes[node2_id];
    node2.m_node_id = node2_id;
    Edge edge2;
    edge2.m_edge_id = edge_id;
    edge2.m_to_node = node1_id;
    node2.m_edges.push_back(edge2);
}

std::vector<Arc> ArcRectOutlineCalc::CalcOutline(const std::vector<ArcRect>& rects)
{
    InitRectData(rects);
    CreateGraph();
    std::vector<Arc> ret = TraversGraph();
    return ret;
}

void ArcRectOutlineCalc::InitRectData(const std::vector<ArcRect>& rects)
{
    m_rects_data.resize(rects.size());

    for (size_t i = 0; i < rects.size(); ++i)
    {
        auto& rect = rects[i];
        ArcRectData data;
        data.m_rect = rect;

        data.m_arc_bbox.resize(rect.m_edges.size());
        BoundingBoxCalcer calc;
        for (size_t j = 0; j < rect.m_edges.size(); ++j)
        {
            data.m_arc_bbox[j] = GetBBox(rect.m_edges[j]);
            calc.PushRect(data.m_arc_bbox[j]);
        }
        data.m_bbox = calc.GetBBox();
        m_rects_data[i] = data;
    }
}

void ArcRectOutlineCalc::CreateGraph()
{
    for (int rect1_idx = 0; rect1_idx < m_rects_data.size(); ++rect1_idx)
    {
        ArcRectData& rect1 = m_rects_data[rect1_idx];
        for (int arc1 = 0; arc1 < rect1.m_rect.m_edges.size(); ++arc1)
        {
            std::pair<int, int> arc1_idx = { rect1_idx,arc1 };
            if (m_arc_intersection.count(arc1_idx) == 0)
                m_arc_intersection[arc1_idx] = {};
        }

        for (int rect2_idx = rect1_idx + 1; rect2_idx < m_rects_data.size(); ++rect2_idx)
        {
            ArcRectData& rect2 = m_rects_data[rect2_idx];

            if (CalcRectsRelation(rect1.m_bbox, rect2.m_bbox) == ERectsRelation::Outside)
                continue;

            for (int arc1 = 0; arc1 < rect1.m_rect.m_edges.size(); ++arc1)
            {
                std::pair<int, int> arc1_idx = { rect1_idx,arc1 };
                for (int arc2 = 0; arc2 < rect2.m_rect.m_edges.size(); ++arc2)
                {
                    std::pair<int, int> arc2_idx = { rect2_idx,arc2 };
                    if (CalcRectsRelation(rect1.m_arc_bbox[arc1], rect2.m_arc_bbox[arc2]) == ERectsRelation::Outside)
                        continue;

                    Arc& a1 = rect1.m_rect.m_edges[arc1];
                    Arc& a2 = rect2.m_rect.m_edges[arc2];

                    auto res = ArcIntersectionPoint(a1, a2);
                    for (auto& p : res[0])
                        m_arc_intersection[arc1_idx].insert(p);
                    for (auto& p : res[1])
                        m_arc_intersection[arc2_idx].insert(p);
                }
            }
        }
    }

    for (auto& pair : m_arc_intersection)
    {
        auto& arc_idx = pair.first;
        Arc& arc = m_rects_data[arc_idx.first].m_rect.m_edges[arc_idx.second];

        if (pair.second.empty())
            m_all_arcs.push_back(arc);
        else
        {
            Vec2 center = arc.m_center;
            std::vector<double> angles;

            double sangle = arc.m_start;
            if (sangle < 0)
                sangle += 2 * PI;
            double eangle = arc.m_end;
            if (eangle < 0)
                eangle += 2 * PI;
            double diff = 0;

            if (Greater(sangle, eangle))
            {
                diff = 2 * PI - sangle;
                sangle = 0;
                eangle += diff;
            }
            for (auto& p : pair.second)
            {
                double angle = std::atan2(p.m_y - center.m_y, p.m_x - center.m_x);
                angle += diff;
                if (angle < 0)
                    angle += 2 * PI;
               
                if (Greater(angle, sangle) && Less(angle, eangle))
                    angles.push_back(angle);
                else
                    assert(false);
            }

            angles.push_back(sangle);
            angles.push_back(eangle);

            std::sort(angles.begin(), angles.end());
            for (int i = 0; i < angles.size(); ++i)
                angles[i] -= diff;

            auto itr = angles.begin() + 1;
            while (itr != angles.end())
            {
                if (Equal(*itr, *(itr - 1)))
                    itr = angles.erase(itr);
                else
                    ++itr;
            }

            for (int i = 0; i < angles.size() - 1; ++i)
            {
                m_all_arcs.push_back(Arc(arc.m_center, arc.m_rad, angles[i], angles[i + 1]));
            }
        }
    }

    for (int i = 0; i < m_all_arcs.size(); ++i)
    {
        auto& arc = m_all_arcs[i];
        int node1 = GetPointID(arc.GetStartPoint());
        int node2 = GetPointID(arc.GetEndPoint());
        m_graph.AddEdge(node1, node2, i);
    }
}

std::vector<Arc> ArcRectOutlineCalc::TraversGraph()
{
    std::vector<Arc> ret;
    Arc start_arc = GetTravGraphStart();
    Vec2 start_pos = start_arc.GetEndPoint();

    int start_node_id = GetPointID(start_pos);
    if (start_node_id == m_next_point_id - 1)
        assert(false);
    ret.push_back(start_arc);

    Vec2 cur_tan = GetArcTangent(start_arc, start_pos) * -1;
    int cur_node_id = start_node_id;
    Vec2 cur_pos = start_pos;
    double cur_rad = start_arc.m_rad;
    int prev_node_id = GetPointID(start_arc.GetStartPoint());
    int cur_trav_num = 0;

    const int end_node_id = prev_node_id;
    const int max_trav_num = 80;
    do
    {
        UndirectedGraph::Node& node = m_graph.m_nodes[cur_node_id];
        if (node.m_node_id < 0)
            assert(false);

        int next_node_id = -1;
        int next_edge_id = -1;
        double next_cross_val = 0;
        double next_rad = 0;
        bool next_cir_on_right = false;

        for (auto& edge : node.m_edges)
        {
            if (edge.m_edge_id == prev_node_id)
                continue;

            Arc& arc = m_all_arcs[edge.m_edge_id];
            Vec2 tan = GetArcTangent(arc, cur_pos);
            double val = cur_tan.Cross(tan);
            Vec2 cir_vec = arc.m_center - cur_pos;
            double cir_val = cur_tan.Cross(cir_vec);
            bool isright = false;
            if (Less(cir_val, 0))
                isright = true;

            if (Equal(tan.m_x, -cur_tan.m_x) && Equal(tan.m_y, -cur_tan.m_y))
            {
                continue;
            }

            bool selected = false;
            if (next_node_id == -1)
                selected = true;
            else
            {
                if (Less(val, next_cross_val))
                    selected = true;
                else if (Equal(val, next_cross_val))
                {
                    if (next_cir_on_right == isright)
                    {
                        if (isright)
                        {
                            if (Less(arc.m_rad, next_rad))
                                selected = true;
                        }
                        else
                        {
                            if (Greater(arc.m_rad, next_rad))
                                selected = true;
                        }
                    }
                    else if (isright)
                        selected = true;
                }
            }

            if (selected)
            {
                next_node_id = edge.m_to_node;
                next_cross_val = val;
                next_edge_id = edge.m_edge_id;
                next_rad = arc.m_rad;
                next_cir_on_right = isright;
            }

        }

        if (next_node_id == -1)
            assert(false);

        Arc arc = m_all_arcs[next_edge_id];
        prev_node_id = cur_node_id;
        cur_node_id = next_node_id;
        cur_pos = m_all_points[cur_node_id];
        cur_tan = GetArcTangent(arc, cur_pos) * -1;
        cur_rad = arc.m_rad;

        ret.push_back(arc);
        cur_trav_num++;

    } while (cur_node_id != end_node_id && cur_trav_num < max_trav_num);
    
    return ret;
}

std::array<std::vector<Vec2>, 2> ArcRectOutlineCalc::ArcIntersectionPoint(const Arc& arc1, const Arc& arc2)
{
    std::vector<Vec2> points;

    double dis = PointDistance(arc1.m_center, arc2.m_center);
    if (Equal(dis, 0) && Equal(arc1.m_rad - arc2.m_rad, 0))
    {
        double sa1 = arc1.m_start;
        if (sa1 < 0)
            sa1 += 2 * PI;
        double ea1 = arc1.m_end;
        if (ea1 < 0)
            ea1 += 2 * PI;

        double sa2 = arc2.m_start;
        if (sa2 < 0)
            sa2 += 2 * PI;
        double ea2 = arc2.m_end;
        if (ea2 < 0)
            ea2 += 2 * PI;

        if (IsAngleInRange(sa1, sa2, ea2))
            points.push_back(arc1.GetStartPoint());
        if (IsAngleInRange(ea1, sa2, ea2))
            points.push_back(arc1.GetEndPoint());
        if (IsAngleInRange(sa2, sa1, ea1))
            points.push_back(arc2.GetStartPoint());
        if (IsAngleInRange(ea2, sa1, ea1))
            points.push_back(arc2.GetEndPoint());
    }
    else if (Equal(dis, arc1.m_rad + arc2.m_rad))
    {

    }
    else if (Equal(dis, std::abs(arc1.m_rad + arc2.m_rad)))
    {

    }
    else
        points = CircleIntersect(Circle(arc1.m_center, arc1.m_rad), Circle(arc2.m_center, arc2.m_rad));

    if (points.empty())
        return {};

    std::array<std::vector<Vec2>, 2> ret;
    for (auto& p : points)
    {
        int res1 = IsPointOnArc(arc1, p);
        int res2 = IsPointOnArc(arc2, p);
        if (res1 < 0 || res2 < 0)
            continue;

        if (res1 == 0)
            ret[0].push_back(p);
        if (res2 == 0)
            ret[1].push_back(p);
    }

    return ret;
}

int ArcRectOutlineCalc::IsPointOnArc(const Arc& arc, const Vec2& point)
{
    double angle = std::atan2(point.m_y - arc.m_center.m_y, point.m_x - arc.m_center.m_x);
    if (angle < 0)
        angle += 2 * PI;
    double sangle = arc.m_start;
    if (sangle < 0)
        sangle += 2 * PI;
    double eangle = arc.m_end;
    if (eangle < 0)
        eangle += 2 * PI;

    if (Greater(sangle, eangle))
    {
        double diff = 2 * PI - sangle;
        sangle = 0;
        eangle += diff;
        if (eangle > 2 * PI)
            eangle -= 2 * PI;
        angle += diff;
        if (angle > 2 * PI)
            angle -= 2 * PI;
    }

    if (Greater(angle, sangle) && Less(angle, eangle))
        return 0;
    else if (Equal(angle, sangle))
        return 1;
    else if (Equal(angle, eangle))
        return 2;
    return -1;
}

int ArcRectOutlineCalc::GetPointID(const Vec2& point)
{
    if (m_pointid_map.count(point) > 0)
        return m_pointid_map[point];

    int id = m_next_point_id++;
    m_pointid_map[point] = id;
    m_all_points.push_back(point);
    return id;
}

Arc ArcRectOutlineCalc::GetTravGraphStart()
{
    int edge_id = -1;
    double min_pos = DBL_MAX;
    for (int i = 0; i < m_all_arcs.size(); ++i)
    {
        auto& arc = m_all_arcs[i];
        auto bbox = GetBBox(arc);
        if (edge_id == -1)
        {
            edge_id = i;
            min_pos = bbox.m_min.m_x;
        }
        else
        {
            if (Less(bbox.m_min.m_x, min_pos))
            {
                edge_id = i;
                min_pos = bbox.m_min.m_x;
            }
        }
    }

    if (edge_id == -1)
        assert(false);

    return m_all_arcs[edge_id];
}

Vec2 ArcRectOutlineCalc::GetArcTangent(const Arc& arc, const Vec2& start)
{
    double angle = PI * 0.5;
    Vec2 epoint = arc.GetEndPoint();
    if (Equal(epoint.m_x, start.m_x) && Equal(epoint.m_y, start.m_y))
        angle *= -1;
    Vec2 ret = start - arc.m_center;
    ret = RotateBy(ret, { 0,0 }, angle);
    return ret.Unit();
}
