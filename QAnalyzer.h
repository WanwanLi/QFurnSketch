#include "QViewer.h"
#include <QPainter>

class QSketch;
class QAnalyzer
{
	public:
	static enum 
	{
		AXIS,
		JOINT,
		LOOP,
		HOLE,
		VERTICAL,
		PARALLEL, 
		DISTANCE,
		FORWARD,
		COPLANAR,
		HORIZONTAL,
		SAME_POINTS,
		LONGITUDINAL,
		PERPENDICULAR,
		GROUND_POINTS,
		CONTACT_POINTS, 
		SYMMETRIC, IDENTICAL, 
		PERPENDICULAR_PLANES
	};
	void initAxis();
	int planesSize;
	QSketch* sketch;
	veci sketchVector, sketchChords;
	QVector<vec3> axis;
	QVector<bool> isJoint;
	int toAxis(QString axis);
	QVector<vec4> planes;
	static int count(int value);
	QVector<vecb> jointGraph;
	int toValue(QString string);
	void  run(), clear();
	void updateRegularity();
	void load(QSketch* sketch);
	void save(QString fileName);
	static QString toString(int value);
	QVector<vec2> avgLineDirections;
	void drawMarkers(QPainter& painter);
	void drawRegularity(QPainter& painter);
	QVector<QVector<vec4>> parallelLines;
	void operator<<(QStringList& stringList);
	QVector<veci> sketchCurves, sketchPaths;
	static void save(QTextStream& textStream, veci regularity);
	veci regularity, joints, holes, samePoints, markerLines, markerPoints, startPoints;
	void load(veci path, veci point3D, veci sketchVector, veci regularity, QVector<vec4> planes, QVector<vec3> axis, QViewer viewer);

	private:
	bool updateJoints();
	vec2 yAxis=vec2(0, 1);
	vec2 getPoint(int index);
	bool hasJointMarker=true;
	void initializeJointGraph();
	QVector<vec> sketchLengths;
	void copySketchCurves();
	void simplifySketchCurves();
	void initializeSketchCurves();
	void finalizeSketchCurves();
	vec2 firstPoint(int curveIndex);
	vec2 lastPoint(int curveIndex);
	bool equals(vec2 p1, vec2 p2);
	qreal cross2(vec2 p1, vec2 p2);
	vec2 getLineDirection(vec4 line);
	bool hasCurveIndexMarker=true;
	veci contactPoints, sketchPlanes;
	bool isParallel(int index, vec4 line2);
	bool isJointType(const vec& positions);
	void completeOpenCurve(int curveIndex);
	void updateAvgLineDirections(int index);
	void getCurvesRegularity(int curveIndex);
	bool hasCloseCurveCtrlPointsMarker=true;
	int indexOf(int curveIndex, int pointIndex);
	int getIndex(int curveIndex, int pointIndex);
	bool isSketchCurve=false;
	veci sketchTypes, sketchSizes, sketchCurveSizes;
	vec2 addLineDirection(vec2 dir1, vec2 dir2);
	vec2 getPoint(int curveIndex, int pointIndex);
	void setLastPoint(int curveIndex, vec2 point);
	void setFirstPoint(int curveIndex, vec2 point);
	bool isParallel(const vec2& x, const vec2& y);
	void addParallelLines(vec4 line1, vec4 line2);
	bool isJointType(int curveIndex, int& planeIndex);
	vec2 intersect(vec2 p1, vec2 p2, vec2 p3, vec2 p4);
	bool equals(const veci& curve, int index1, int index2);
	qreal error=0.01, minLength=0.05, minDistance=10.0;
	void getClosePoints(int curveIndex1, int curveIndex2);
	void setPoint(int curveIndex, int pointIndex, vec2 point);
	void getCurvesRegularity(int curveIndex1, int curveIndex2);
	void addJointRegularity(int curveIndex, int jointIndex, int startIndex);
	enum{LINE_SEGMENT, HOLE_CURVE, CLOSE_CURVE, OPEN_CURVE, ENCLOSED_CURVE};


	void updateHoles();
	void addHoleRegularity(int curveIndex, int planeIndex);
	bool hasHoleMarker=true;
	void addHoleMarker(int jointIndex);
	bool isHoleType(int curveIndex, int& planeIndex);
	int max(veci array);
	void getSketchVector();
	void encloseSketchCurves(bool isLoop);
	void getPlanesRegularity();
	void addJointMarker(int jointIndex);
	void addJoint(int& jointIndex, vec2 move, vec2 line);
	qreal intersectWithCloseCurve(vec2& leftPoint, vec2& rightPoint, int curveIndex, vec2 begin, vec2 end);
	bool isOnOutsideOrEqual(vec2 left, vec2 right, vec2 begin, vec2 end);
	void addJointForCloseCurve(int curveIndex, bool isEnclosedCurve);
	void getSketchLengths(int curveIndex),  getCurvatures(int curveIndex);
	bool isShort(int curveIndex, int pointIndex);
	bool isJointInCloseCurve(int curveIndex, vec2 begin, vec2 end, vec2& left, vec2& right, qreal& distance);
	bool intersectWithEdge(qreal& distance, vec2& leftPoint, vec2& rightPoint, QVector<vec2> points, bool isJoint);
	qreal devide(vec2 vector1, vec2 vector2);
};
