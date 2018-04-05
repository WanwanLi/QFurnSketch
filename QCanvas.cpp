#include "QMesh.h"
#include <QPainter>
#include "QCanvas.h"
#include <QMessageBox>
#include "QOpenGLWindow.h"
#define RGB32 QImage::Format_RGB32

QCanvas::QCanvas(QWidget* widget) : QWidget(widget)
{
	this->getPens();
	this->setFocus();
	this->getColors();
	this->clearImage();
	this->widget=widget;
	this->isUpdated=true;
	this->resize(widget->size());
	this->basicTimer.start(15, this);
	this->sketch.viewer.resize(size());
	this->bgColor=qRgb(255, 255, 255);
	this->image=QImage(size(), RGB32);
}
void QCanvas::getColors()
{
	this->colors<<Qt::green<<Qt::blue<<Qt::red;
	this->colors<<Qt::magenta<<Qt::yellow<<Qt::cyan<<Qt::gray; 
	this->colors<<Qt::black<<Qt::darkBlue<<Qt::darkGreen<<Qt::darkRed;
	this->colors<<Qt::darkMagenta<<Qt::darkYellow<<Qt::darkCyan<<Qt::darkGray;
}
void QCanvas::getPens()
{
	this->pen=QPen(Qt::blue, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	this->marker=QPen(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	this->printer=QPen(Qt::black, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	this->progressBar=QPen(Qt::green, 10, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin);
	this->groundPlane=QPen(Qt::cyan, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
}
bool QCanvas::loadImage(QString fileName)
{
	if(image.load(fileName))
	{
		this->resize(image.size());
		this->resizeWindow(20);
		this->isUpdated=true;
		return true;
	}
	else if(sketch.load(fileName))
	{
		this->resize(sketch.size());
		this->image=QImage(size(), RGB32);
		this->image.fill(bgColor);
		this->resizeWindow(20);
		this->isUpdated=true;
		return true;
	}
	else return false;
}
bool QCanvas::saveImage(QString fileName, const char* fileFormat)
{
	if(tr(fileFormat)=="fsk")return sketch.save(fileName);
	if(!image.save(fileName, fileFormat))return false;
	return true;
}
void QCanvas::resizeWindow(int margin)
{
	this->widget->resize(QSize(image.width(), image.height()+margin));
}
void QCanvas::visualize()
{
	QSketch sketch(QSketch::sketchModelFile); if(!sketch.isValid)return;
	QVector<vec3*> quads=sketch.getPoint4D();
	QMesh mesh(sketch.path, sketch.point4D, quads);
	QOpenGLWindow* window=new QOpenGLWindow(colors, mesh.quads, mesh.coords);
	window->resize(400, 300); window->show();
}
void QCanvas::resizeImage()
{
	QImage image(size(), RGB32);
	QPainter painter(&image);
	painter.drawImage
	(
		QPoint(0, 0), image.scaled
		(
			size(), Qt::IgnoreAspectRatio,
			Qt::SmoothTransformation
		)
	);
	this->image=image;
	this->isUpdated=true;
	this->sketch.viewer.resize(size());
}
void QCanvas::clear()
{
	this->clearImage();
	this->isUpdated=true;
}
void QCanvas::critical(QString op)
{
	QMessageBox::critical
	(
		this, "IIlegal State Exception",
		"Can not "+op+" Sketch."
	);
}
void QCanvas::analyzeSketch()
{
	if(sketch.analyze())
	this->isUpdated=true;
	else critical("Analyze");
}
void QCanvas::inflateSketch()
{
	if(sketch.inflate())
	this->isUpdated=true;
	else critical("Inflate");
}
void QCanvas::normalizeSketch()
{
	if(sketch.normalize())
	this->isUpdated=true;
	else critical("Normalize");
}
void QCanvas::clearImage()
{
	this->image.fill(bgColor);
	this->sketch.clear();
	this->isUpdated=true;
}
void QCanvas::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	QRect rect=event->rect();
	painter.drawImage(rect, image, rect);
}
void QCanvas::mouseMoveEvent(QMouseEvent* event)
{
	if(!sketch.isOptimizing())return;
	if(isMousePressed)
	{
		this->mouseMove=event->pos()-mousePos;
		this->mousePos=event->pos();
		this->sketch.viewer.rotate(-mouseMove);
		this->isUpdated=true;
	}
}
void QCanvas::mousePressEvent(QMouseEvent* event)
{
	if(!sketch.isOptimizing())return;
	if(event->button()==Qt::LeftButton) 
	{
		this->isUpdated=true;
		this->isMousePressed=true;
		this->mousePos=event->pos();
	}
}
void QCanvas::mouseReleaseEvent(QMouseEvent* event)
{
        	this->isMousePressed=false;
}
void QCanvas::wheelEvent(QWheelEvent* event)
{
	if(!sketch.isOptimizing())return;
	qreal delta=-event->delta()/120.0;
	this->sketch.viewer.translate(delta);
	this->isUpdated=true;
}
void QCanvas::timerEvent(QTimerEvent* event)
{
	if(sketch.isOnUpdated())
	this->isUpdated=true;
	if(!isUpdated)return;
	this->image.fill(bgColor);
	this->sketch.update();
	this->drawSketch();
	this->update();
	this->isUpdated=false;
}
void QCanvas::drawSketch()
{
	QPainter painter(&image);
	for(int i=0; i<sketch.length(); i++)
	{
		pen.setColor(color(i));
		painter.setPen(pen);
		painter.drawPath(sketch[i]);
		if(sketch.is(sketch.NORMALIZED))
		painter.fillPath(sketch[i], brush(i));
	}
	painter.setPen(marker);
	sketch.drawMarkers(painter);
	sketch.drawRegularity3D(painter);
	painter.setPen(printer);
	sketch.drawStatusText(painter);
	painter.setPen(progressBar);
	sketch.drawProgressBar(painter);
	painter.setPen(groundPlane);
	sketch.drawGroundPlane(painter);
	sketch.drawAxis(painter);
}
QColor QCanvas::color(int index)
{
	return this->colors[index%(colors.size()-1)];
}
QBrush QCanvas::brush(int index)
{
	QColor color=this->colors[index%(colors.size()-1)];
	qreal red=qMin(color.redF()+0.25, 1.0);
	qreal green=qMin(color.greenF()+0.25, 1.0);
	qreal blue=qMin(color.blueF()+0.25, 1.0);
	return QBrush(QColor::fromRgbF(red, green, blue, 0.25f));
}
bool QCanvas::isModified()
{
	return false;
}
void QCanvas::setCameraUpward()
{
	this->isCameraUpward=!isCameraUpward;
}
void QCanvas::resetViewInfo()
{
}
