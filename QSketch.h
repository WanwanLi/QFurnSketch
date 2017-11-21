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
	void setValue(int value);

	public:
	veci path;
	QSketch();
	bool load();
	void clear();
	int length();
	QSize size();
	bool analyze();
	bool optimize();
	int iterations=0;
	QViewer viewer;
	bool is(int state);
	int state=INITIAL;
	QThread* thread;
	QAnalyzer analyzer;
	void paint(), update();
	QOptimizer* optimizer;
	bool save(QString fileName);
	bool load(QString fileName);
	QPainterPath& operator[](int i);
	static enum{MOVE, LINE, CUBIC};
	bool displayGroundPlane=false;
	veci point2D, point3D; vec point4D;
	QString analysisFile="QSketch.sky";
	void drawMarkers(QPainter& painter);
	enum{INITIAL, ANALYZED, OPTIMIZED};
	void drawProgressBar(QPainter& painter);
	void drawGroundPlane(QPainter& painter);

	private:
	vec4 groundPlane;
	void updatePainterPaths();
	bool load(QStringList& list);
	vec3 planeCenter=vec3(0, -0.25, 0);
	int planeGrids=20;qreal planeSize=20;
	QVector<QPainterPath> painterPaths;
	bool load(QStringList& list, bool isPoint3D);
	void drawLine(QPainter& painter, vec3 v0, vec3 v1);
	bool load(QTextStream& textStream, bool isPoint3D);
	void moveTo(int x, int y, int index), lineTo(int x, int y, int index);
	void cubicTo(int x1, int y1, int x2, int y2, int x3, int y3, int index);
};
