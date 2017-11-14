#include <QPen>
#include <QColor>
#include <QPoint>
#include <QImage>
#include <QWidget>
#include "QSketch.h"
#include <QPaintEvent>
#include <QBasicTimer>

class QCanvas : public QWidget
{
	Q_OBJECT
	public:
	bool isUpdated;
	QCanvas(QWidget* widget=0);
	bool loadImage(QString fileName);
	bool saveImage(QString fileName, const char* fileFormat);

	public slots:
	void clear();
	bool isModified();
	void resizeImage();
	void resetViewInfo();
	void analyzeSketch();
	void optimizeSketch();
	void setCameraUpward();

	private:
	QPen pen;
	QPen plane;
	QPoint point;
	QPen marker;
	QImage image;
	void getPens();
	void getColors();
	QColor bgColor;
	QSketch sketch;
	QWidget* widget;
	void clearImage();
	QPen progressBar;
	void drawSketch();
	bool isMousePressed;
	QColor color(int index);
	QVector<QColor> colors;
	QBasicTimer basicTimer;
	bool isCameraUpward=false;
	void resizeWindow(int margin);
	QPoint mouseMove, mousePos;
	void paintEvent(QPaintEvent* event);
	void timerEvent(QTimerEvent* event);
	void wheelEvent(QWheelEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void mousePressEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
};