#pragma once

#include <QWidget>

class Canvas  : public QWidget
{
	Q_OBJECT

public:
	Canvas(QWidget *parent = nullptr);
	~Canvas();

	virtual void paintEvent(QPaintEvent* event) override;
};
