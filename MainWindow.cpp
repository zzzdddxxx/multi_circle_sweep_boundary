#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    m_canvas = new Canvas(this);
    setCentralWidget(m_canvas);
}

MainWindow::~MainWindow()
{}
