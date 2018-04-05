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
	void visualize();
	bool isModified();
	void resizeImage();
	void resetViewInfo();
	void inflateSketch();
	void analyzeSketch();
	void normalizeSketch();
	void setCameraUpward();

	private:
	QPen pen;
	QPoint point;
	QPen printer;
	QPen marker;
	QImage image;
	void getPens();
	void getColors();
	QColor bgColor;
	QSketch sketch;
	QWidget* widget;
	void clearImage();
	void drawSketch();
	QPen progressBar;
	QPen groundPlane;
	bool isMousePressed;
	QColor color(int index);
	void critical(QString op);
	QVector<QColor> colors;
	QBasicTimer basicTimer;
	QBrush brush(int index);
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
