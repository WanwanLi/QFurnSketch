#include "QViewer.h"
#include "QStanMath.h"
#include <stan/math.hpp>
using namespace stan::math;
#define Vector2v Matrix<var, 2, 1>
#define Vector3v Matrix<var, 3, 1>
#define Vector4v Matrix<var, 4, 1>
#define VectorXv Matrix<var, Dynamic, 1>
#define MatrixXv Matrix<var, Dynamic, Dynamic>

class QEnergy
{
	public:
	QEnergy(){}
	veci path;
	int planeSize;
	QVector<Vector3v> axis;
	QVector<veci> axisIndices;
	void save(QString fileName);
	void initViewInfo();
	veci samePoints;
	veci regularity;
	void analyze();
	veci point3D;
	veci getPoint2D();
	vec pointDistances;
	QVector<vec4> getPlanes();
	QViewer viewer;
	VectorXd sketch;
	var viewDistance=12;
	var accuracy=5000;
	bool isAddingSamePoints;
	vec4 ground=vec4(0, 1, 0, 0);
	vec horizontal, forward;
	QVector<Vector3v> groundPoints;
	Vector3v upward(const Vector4v& ground);
	Vector2v pointAt(int index);
	var QEnergy::weight(int type);
	vec toQVector(VectorXd vector);
	veci toQVector(VectorXi vector);
	vec toQVector(VectorXv vector);
	var decay(var distance, var speed);
	var scaleAt(const VectorXv& variables, int index);
	VectorXd variables;
	bool isPlaneOnly=true;
	bool isAddingAxis=false;
	Vector4v groundPlane();
	void getAxis(const VectorXv& variables);
	VectorXd toVectorXd(vec vector);
	vec2 sketchPixel(int x, int y, QViewer viewer);
	var stdDevScalesEnergy(const VectorXv& variables, int start, int end);
	void addPointDistances(const VectorXv& variables, int start, int end);
	var depthEnergy(const VectorXv& planes);
	var totalEnergy(const VectorXv& variables);
	void copySameGradients(VectorXd& grad);
	Vector3v toVector3v(QVector3D vector3D);
	QVector2D toQVector2D(Vector2v vector2v);
	QVector3D toQVector3D(Vector3v vector3v);
	QVector4D toQVector4D(Vector4v vector4v);
	Vector4v toVector4v(vec4 vector4D);
	QVector<QVector3D> toVector3D(QVector<Vector3v> vector3v);
	QVector<Vector3v> toVector3v(QVector<QVector3D> vector3D);
	var horizontalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane);
	var forwardEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane);
	Vector3v getLineDirection(const VectorXv& variables, int startIndex, int endIndex, int planeIndex);
	MatrixXv getSketchPoints(const VectorXd& variables);
	var accuracyEnergy(const VectorXv& variables, int start, int end);
	Vector4v getGroundPlane(const VectorXd& variables);
	var foreshorteningEnergy(const VectorXv& variables);
	Vector2v pointAt(const VectorXv& variable, int index);
	QVector<double> viewInfoVector=QVector<double>(12);
	var cosAngle(const VectorXv& variables, int i0, int i1, int i2);
	VectorXv toVectorXv(const VectorXd& vectorXd);
	Vector2v toVector2v(const VectorXd& vectorXd, int startIndex);
	Vector4v toVector4v(const VectorXd& vectorXd, int startIndex);
	Vector3v sketchPoint(const VectorXv& variables, int sketchIndex);
	Vector3v sketchPoint(const Vector2d& point, const Vector4v& plane);
	//Vector3v sketchPoint(const Vector2v& point, const Vector4v& plane);
	var stdDevAnglesEnergy(const VectorXv& variables, int start, int end);
	var parallelEnergy(const Vector4v& plane1, const Vector4v& plane2);
	void addAxis(int axisIndex, int startIndex, int endIndex, int planeIndex);
	Vector3v addLineDirection(Vector3v dir1, Vector3v dir2);
	var collinearEnergy(const MatrixXv& curve, const Vector4v& plane, int start, int end);
	var verticalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane);
	var totalEnergy(const VectorXi& sketch, const VectorXi& joint, const VectorXv& plane, int planeSize);
	QEnergy(veci path, veci sketch, int planeSize, veci regularity, QVector<vec4> planes, QVector<vec3> axis, QViewer viewer);
	var distanceEnergy(const Vector2v& point1, const Vector4v& plane1, const Vector2v& point2, const Vector4v& plane2);
	var coplanarEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& srcPlane, const Vector4v& destPlane);
	var perpendicularEnergy(const Vector2v& leftPoint, const Vector2v& midPoint, const Vector2v& rightPoint, const Vector4v& plane);
	var parallelEnergy(const Vector2v& startPoint1, const Vector2v& endPoint1, const Vector4v& plane1, const Vector2v& startPoint2, const Vector2v& endPoint2, const Vector4v& plane2);
};
