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
	enum 
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
		SYMMETRIC, IDENTICAL
	};
	veci path;
	int planeSize;
	int sketchWidth;
	int sketchHeight;
	void initViewInfo();
	veci samePoints;
	VectorXi regularity;
	VectorXi sketchVector;
	var viewDistance=12;
	vec4 ground=vec4(0, 1, 0, 0);
	Vector3v upward(const Vector4v& ground);
	Vector2v pointAt(int index);
	var QEnergy::weight(int type);
	vec toQVector(VectorXd vector);
	Vector3v up=Vector3v(0, 1, 0);
	veci toQVector(VectorXi vector);
	vec toQVector(VectorXv vector);
	Vector3v right=Vector3v(1, 0, 0);
	var decay(var distance, var speed);
	void setupViewer(QViewer viewer);
	Vector3v forward=Vector3v(0, 0, -1);
	VectorXd planeVector, variableVector;
	var depthEnergy(const VectorXv& planes);
	var totalEnergy(const VectorXv& variables);
	void copySameGradients(VectorXd& grad);
	Vector3v toVector3v(QVector3D vector3D);
	QVector2D toQVector2D(Vector2v vector2v);
	QVector3D toQVector3D(Vector3v vector3v);
	QVector4D toQVector4D(Vector4v vector4v);
	MatrixXv sketchCurves(VectorXd& variables);
	MatrixXd sketchPlanes(VectorXd& variables);
	var accuracyEnergy(const VectorXv& variables);
	Vector4v getGroundPlane(const VectorXd& variables);
	MatrixXv getSketchPoints(const VectorXd& variable);
	var foreshorteningEnergy(const VectorXv& variables);
	Vector2v pointAt(const VectorXv& variable, int index);
	QVector<double> viewInfoVector=QVector<double>(12);
	var cosAngle(const VectorXv& variables, int i0, int i1, int i2);
	Vector2v toVector2v(const VectorXd& vectorXd, int startIndex);
	Vector4v toVector4v(const VectorXd& vectorXd, int startIndex);
	var  aspectRatio=1.0, screenScale=100.0, focalLength=0.1;
	Vector3v sketchPoint(const VectorXv& variables, int sketchIndex);
	Vector3v sketchPoint(const Vector2i& point, const Vector4v& plane);
	Vector3v sketchPoint(const Vector2v& point, const Vector4v& plane);
	var stdDevAnglesEnergy(const VectorXv& variables, int start, int end);
	var parallelEnergy(const Vector4v& plane1, const Vector4v& plane2);
	QEnergy(veci path, veci sketch, int planesSize, veci regularity, QViewer viewer);
	var collinearEnergy(const MatrixXv& curve, const Vector4v& plane, int start, int end);
	var totalEnergy(const VectorXi& sketch, const VectorXi& joint, const VectorXv& plane, int planeSize);
	var verticalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane, const Vector4v& ground);
	var distanceEnergy(const Vector2v& point1, const Vector4v& plane1, const Vector2v& point2, const Vector4v& plane2);
	var coplanarEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& srcPlane, const Vector4v& destPlane);
	var perpendicularEnergy(const Vector2v& leftPoint, const Vector2v& midPoint, const Vector2v& rightPoint, const Vector4v& plane);
	var parallelEnergy(const Vector2v& startPoint1, const Vector2v& endPoint1, const Vector4v& plane1, const Vector2v& startPoint2, const Vector2v& endPoint2, const Vector4v& plane2);
};
