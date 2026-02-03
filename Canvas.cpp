#include "Canvas.h"
#include "GeomUtils.h"
#include <QPainter>

Canvas::Canvas(QWidget *parent)
	: QWidget(parent)
{}

Canvas::~Canvas()
{}

void DrawCir(QPainter* painter, const Circle& c)
{
	painter->drawEllipse(c.m_center.m_x - c.m_rad, c.m_center.m_y - c.m_rad, c.m_rad * 2, c.m_rad * 2);
}

void DrawRect(QPainter* painter, const Rect& rec)
{
	painter->drawRect(rec.m_min.m_x, rec.m_min.m_y, rec.m_max.m_x - rec.m_min.m_x, rec.m_max.m_y - rec.m_min.m_y);
}

void DrawArc(QPainter* painter, const Arc& a)
{
	double across = a.m_end - a.m_start;
	if (a.m_start > a.m_end)
		across += 2 * PI;
	painter->drawArc(a.m_center.m_x - a.m_rad, (a.m_center.m_y - a.m_rad), a.m_rad * 2, a.m_rad * 2, -a.m_start * 16 * 180 / PI, -across * 16 * 180 / PI);
}

void Canvas::paintEvent(QPaintEvent * event)
{
	QWidget::paintEvent(event);
	QPainter painter(this);
	painter.translate(0, height());
	painter.scale(1, -1);

	bool origin = true;
	std::vector<ArcRect> recs;
	ArcRect rec1(50, { 200,200 }, 80, -PI * 0.5, PI * 0.2);
	recs.push_back(rec1);
	if (origin)
	{
		for (auto& a : rec1.m_edges)
			DrawArc(&painter, a);
	}

	ArcRect rec2(80, { 200,350 }, 180, -PI * 0.5, -PI * 0.2);
	recs.push_back(rec2);
	if (origin)
	{
		for (auto& a : rec2.m_edges)
			DrawArc(&painter, a);
	}

	ArcRectOutlineCalc calc;
	auto res = calc.CalcOutline(recs);

	if (!origin)
	{
		for (auto& a : res)
			DrawArc(&painter, a);
	}

}
