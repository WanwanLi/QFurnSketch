#include "QDefine.h"
#include <QPainter>

class QSketch;
class QAnalyzer
{
	public:
	static enum 
	{
		VERTICAL,
		PARALLEL, 
		DISTANCE,
		COPLANAR,
		HORIZONTAL,
		SAME_POINTS,
		PERPENDICULAR,
		CONTACT_POINTS, 
		PARALLEL_PLANES,
		SYMMETRIC, IDENTICAL, 
	};
	int planesSize;
	QSketch* sketch;
	QVector<bool> isJoint;
	static int count(int value);
	int toValue(QString string);
	void  run(), update(), clear();
	void load(QSketch* sketch);
	void save(QString fileName);
	static QString toString(int value);
	QString fileName="QAnalysis.sky";
	QVector<vec2> avgLineDirections;
	void drawMarkers(QPainter& painter);
	void drawRegularity(QPainter& painter);
	QVector<QVector<vec4>> parallelLines;
	void operator<<(QStringList& stringList);
	QVector<veci> sketchCurves, sketchPaths;
	veci regularity, markerLines, markerPoints, startPoints;

	private:
	vec2 yAxis=vec2(0, 1);
	vec2 getPoint(int index);
	bool hasJointMarker=true;
	void initializeSketchCurves();
	void completeSketchCurves();
	vec2 firstPoint(int curveIndex);
	vec2 lastPoint(int curveIndex);
	bool equals(vec2 p1, vec2 p2);
	qreal cross2(vec2 p1, vec2 p2);
	vec2 getLineDirection(vec4 line);
	bool hasCurveIndexMarker=true;
	veci contactPoints, sketchPlanes;
	qreal error=0.05, minDistance=5.0;
	bool isParallel(int index, vec4 line2);
	void getContactPoints(int curveIndex);
	bool isJointType(const vec& positions);
	void updateAvgLineDirections(int index);
	void getCurvesRegularity(int curveIndex);
	bool hasCloseCurveCtrlPointsMarker=true;
	int indexOf(int curveIndex, int pointIndex);
	int getIndex(int curveIndex, int pointIndex);
	vec2 addLineDirection(vec2 dir1, vec2 dir2);
	veci sketchTypes, sketchSizes, sketchVector;
	vec2 getPoint(int curveIndex, int pointIndex);
	bool isParallel(const vec2& x, const vec2& y);
	void addParallelLines(vec4 line1, vec4 line2);
	bool isJointType(int curveIndex, int& planeIndex);
	void completeOpenCurveWithJoint(int curveIndex);
	vec2 intersect(vec2 p1, vec2 p2, vec2 p3, vec2 p4);
	int nextContactPoint(int curveIndex, int pointIndex);
	bool equals(const veci& curve, int index1, int index2);
	void getClosePoints(int curveIndex1, int curveIndex2);
	enum{LINE_SEGMENT, CLOSE_CURVE, OPEN_CURVE};
	void getCurvesRegularity(int curveIndex1, int curveIndex2);
};
