#include <QSize>
#include <QThread>
#include "QViewer.h"
#include <QVector3D>
#include "QAnalyzer.h"
#include <QPainterPath>

class QOptimizer;
class QSketch : public QObject
{
	Q_OBJECT

	private slots:
	void finished();
	void setValue(int value);

	public:
	veci path;
	QSketch();
	void clear();
	int length();
	QSize size();
	bool isValid;
	bool inflate();
	bool analyze();
	void antialias();
	bool normalize();
	int iterations=0;
	QViewer viewer;
	bool is(int state);
	bool isOptimizing();
	bool isOnUpdated();
	QAnalyzer analyzer;
	void paint(), update();
	int state=INITIALIZED;
	vec horizontal, forward;
	int getOptimizingState();
	QSketch(QString fileName);
	vec open(QString fileName);
	bool load(QString fileName);
	bool save(QString fileName);
	QVector<vec3*> getPoint4D();
	QThread *inflator, *normalizer;
	bool displayRegularity3D=false;
	bool displayGroundPlane=false;
	QPainterPath& operator[](int i);
	static enum {MOVE, LINE, CUBIC};
	void drawAxis(QPainter& painter);
	veci point2D, point3D; vec point4D;
	void drawMarkers(QPainter& painter);
	QOptimizer *optimizer2D, *optimizer3D;
	void drawStatusText(QPainter& painter);
	void drawProgressBar(QPainter& painter);
	void drawGroundPlane(QPainter& painter);
	void drawRegularity3D(QPainter& painter);
	QSketch(veci path, veci point3D, QViewer viewer);
	enum{INITIALIZED, ANALYZED, INFLATED, NORMALIZED};
	static QString sketch2DFile, sketch3DFile, sketchModelFile;

	private:
	vec4 groundPlane;
	bool isUpdated=false;
	vec3 xAxis, yAxis, zAxis;
	QOptimizer* optimizer();
	vec3 getPoint3D(int index);
	void updatePainterPaths();
	bool load(QStringList& list);
	void conncetJointToPlane();
	vec3 planeCenter=vec3(0, -0.25, 0);
	QVector<QPainterPath> painterPaths;
	int planeGrids=20; qreal planeSize=20;
	bool load(QStringList& list, bool isPoint3D);
	void setPoint3D(int startIndex, vec2 point);
	void drawLine(QPainter& painter, vec3 v0, vec3 v1);
	bool load(QTextStream& textStream, bool isPoint3D);
	void moveTo(int x, int y, int index), lineTo(int x, int y, int index);
	void cubicTo(int x1, int y1, int x2, int y2, int x3, int y3, int index);
	void drawRegularity3D(QPainter& painter, vec points, QString tag);
	void drawAxis(QPainter& painter, vec3 axis, qreal length, QPoint center);
	QOptimizer* newQOptimizer(QThread* thread, QString sketchFile, int iterations);
};
