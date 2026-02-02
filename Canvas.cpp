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
	painter->drawArc(a.m_center.m_x - a.m_rad, (a.m_center.m_y - a.m_rad), a.m_rad * 2, a.m_rad * 2, -a.m_start * 16 * 180 / PI, -(a.m_end - a.m_start) * 16 * 180 / PI);
}

void Canvas::paintEvent(QPaintEvent * event)
{
	QWidget::paintEvent(event);
	QPainter painter(this);
	painter.translate(0, height());
	painter.scale(1, -1);
	double h = height();

	double s = 150;
	double e = 290;
	Vec2 c(500, 150);
	double r = 100;
	Arc arc(c, r, s*PI/180, e*PI/180);
	DrawArc(&painter, arc);

	
	Rect rec = GetBBox(arc);
	DrawRect(&painter, rec);


}
