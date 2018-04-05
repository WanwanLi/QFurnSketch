#include <QVector>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <Eigen/Dense>
using namespace Eigen;
#define Vector2t Matrix<T, 2, 1>
#define Vector3t Matrix<T, 3, 1>
#define Vector4t Matrix<T, 4, 1>
#define VectorXt Matrix<T, Dynamic, 1>
#define MatrixXt Matrix<T, Dynamic, Dynamic>
#define Template(T) template<typename T>

Template(T) class QStanMath
{
	public:
	static T abs(T x){return x>0?x:-x;}
	static Vector3t xAxis, yAxis, zAxis;
	static T min(T x, T y){return x<y?x:y;}
	static T max(T x, T y){return x>y?x:y;}
	static Vector4t createPlane(T A, T B, T C, T D);
	static Vector2t toVector2t(QVector2D vector2d);
	static Vector3t toVector3t(QVector3D vector3d);
	static Vector4t toVector4t(QVector4D vector4d);
	static QVector2D toVector2D(Vector2d vector2d);
	static QVector3D toVector3D(Vector3d vector3d);
	static QVector4D toVector4D(Vector4d vector4d);
	static Vector3t getNormal(const Vector4t& plane);
	static Vector3t centerPoint(const Vector4t& plane);
	static Vector2t canvasPoint(const Vector3t& point);
	static QVector<T> toVector(const VectorXt& vectorXt);
	static QVector<T> toVector(const MatrixXt& matrixXt);
	static Vector4t normalizedPlane(const Vector4t& plane);
	static Vector3t* getQuad(Vector4t plane, Vector3t space);
	static Vector4t createPlane(const QVector<Vector3t>& points);
	static Vector4t perturbedPlane(const Vector4t& plane, T noise);
	static MatrixXt subCurve(const MatrixXt& curve, int start, int end);
	static T distanceSquareBetween(MatrixXt& curve1, MatrixXt& curve2);
	static T distanceToPlane(const Vector3t& point, const Vector4t& plane);
	static Vector3t sketchPoint(const Vector2t& point, const Vector4t& plane);
	static Vector4t createPlane(const Vector3t& point, const Vector3t& normal);
	static T distanceBetween(const VectorXt& vector1, const VectorXt& vector2);
	static Vector3t projectedPoint(const Vector3t& point, const Vector4t& plane);
	static MatrixXt projectedCurve(const MatrixXt& curve, const Vector4t& plane);
	static Vector3t* intersectPlane(Vector4t plane, Vector3t axis, Vector3t* points);
	static T distanceBetweenDirections(const Vector3t& dir1, const Vector3t& dir2);
	static Vector3t antialiasPoint(Vector3t point, Vector3t yAxis, Vector4t groundPlane);
	static Vector4t translatedPlane(const Vector4t& plane, const Vector3t& translation);
	static Vector3t intersectPlane(const Vector3t& origin, const Vector3t& direction, const Vector4t& plane);
	static T cosAngle(const Vector3t& begin1, const Vector3t& end1, const Vector3t& begin2, const Vector3t& end2);
	static T integrateDistanceSquare(const Vector3t& P0, const Vector3t& P1, const Vector3t& Q0, const Vector3t& Q1);
};
Template(T) Vector3t QStanMath<T>::xAxis=Vector3t(1, 0, 0);
Template(T) Vector3t QStanMath<T>::yAxis=Vector3t(0, 1, 0);
Template(T) Vector3t QStanMath<T>::zAxis=Vector3t(0, 0, 1);
Template(T) Vector3t QStanMath<T>::getNormal(const Vector4t& plane)
{
	Vector3t normal=plane.head(3);
	return normal/normal.norm();
}
Template(T) T QStanMath<T>::distanceBetweenDirections(const Vector3t& dir1, const Vector3t& dir2)
{
	T dot=dir1.normalized().dot(dir2.normalized()); return 1-dot*dot;
}
Template(T) T QStanMath<T>::distanceBetween(const VectorXt& vector1, const VectorXt& vector2)
{
	return (vector2-vector1).squaredNorm();
}
Template(T) T QStanMath<T>::distanceToPlane(const Vector3t& point, const Vector4t& plane)
{
	Vector4t plane1=normalizedPlane(plane); return abs(plane1.head(3).dot(point)+plane1(3));
}
Template(T) T QStanMath<T>::cosAngle(const Vector3t& begin1, const Vector3t& end1, const Vector3t& begin2, const Vector3t& end2)
{
	Vector3t dir1=(end1-begin1).normalized();
	Vector3t dir2=(end2-begin2).normalized();
	return dir1.dot(dir2);
}
Template(T) Vector4t QStanMath<T>::createPlane(T A, T B, T C, T D)
{
	Vector4t plane(A, B, C, D);
	return plane/plane.head(3).norm();
}
Template(T) Vector4t QStanMath<T>::createPlane(const Vector3t& point, const Vector3t& normal)
{
	return Vector4t(normal(0), normal(1), normal(2), -point.dot(normal));
}
Template(T) Vector4t QStanMath<T>::createPlane(const QVector<Vector3t>& points)
{
	return createPlane(points[0], ((points[0]-points[1]).cross(points[1]-points[2])).normalized());
}
Template(T) Vector4t QStanMath<T>::translatedPlane(const Vector4t& plane, const Vector3t& translation)
{
	return createPlane(centerPoint(plane)+translation, getNormal(plane));
}
Template(T) Vector4t QStanMath<T>::normalizedPlane(const Vector4t& plane)
{			
	return plane/plane.head(3).norm();
}
Template(T) Vector4t QStanMath<T>::perturbedPlane(const Vector4t& plane, T noise)
{
	return normalizedPlane(plane+Vector4t::Random()*noise);
}
Template(T) Vector3t QStanMath<T>::centerPoint(const Vector4t& plane)
{
	T A=plane(0), B=plane(1), C=plane(2), D=plane(3), E=1e-5; Vector3t P(0, 0, 0);
	if(abs(A)>E)P(0)=-D/A; else if(abs(B)>E)P(1)=-D/B; else P(2)=-D/C; return P;
}
Template(T) Vector3t QStanMath<T>::intersectPlane(const Vector3t& origin, const Vector3t& direction, const Vector4t& plane)
{
	Vector3t normal=plane.head(3);
	if(normal.dot(direction)==0)return origin;
	Vector3t center=centerPoint(plane);
	Vector3t connection=center-origin;
	T distance=normal.dot(connection)/normal.dot(direction);
	return origin+distance*direction;
}
Template(T) Vector3t QStanMath<T>::projectedPoint(const Vector3t& point, const Vector4t& plane)
{
	return intersectPlane(point, -plane.head(3), plane);
}
Template(T) QVector<T> QStanMath<T>::toVector(const MatrixXt& matrixXt)
{
	QVector<T> vector;
	for(int i=0; i<matrixXt.cols(); i++)
	{
		for(int j=0; j<matrixXt.rows(); j++)
		{
			vector<<matrixXt(j, i);
		}
	}
	return vector;
}
Template(T) QVector<T> QStanMath<T>::toVector(const VectorXt& vectorXt)
{
	QVector<T> vector;
	for(int i=0; i<vectorXt.size(); i++)vector<<vectorXt(i);
	return vector;
}
Template(T) MatrixXt QStanMath<T>::subCurve(const MatrixXt& curve, int start, int end)
{
	MatrixXt subcurve(3, end-start+1);
	for(int i=0; i<subcurve.cols(); i++)
	{
		subcurve.col(i)=curve.col(start+i);
	}
	return subcurve;
}
Template(T) MatrixXt QStanMath<T>::projectedCurve(const MatrixXt& curve, const Vector4t& plane)
{
	MatrixXt projectCurve(3, curve.cols());	
	for(int i=0; i<curve.cols(); i++)
	{
		projectCurve.col(i)=projectedPoint(curve.col(i), plane);
	}
	return projectCurve;
}
Template(T) T QStanMath<T>::integrateDistanceSquare(const Vector3t& P0, const Vector3t& P1, const Vector3t& Q0, const Vector3t& Q1)
{
	Vector3t u=P1-P0, v=Q1-Q0, C=P0-Q0, k=u-v;		
	return C.dot(C)+k.dot(k)/3+C.dot(k);
}
Template(T) T QStanMath<T>::distanceSquareBetween(MatrixXt& curve1, MatrixXt& curve2)
{
	T distanceSquare=0;
	for(int i=0; i<curve1.cols()-1; i++)
	{
		distanceSquare+=integrateDistanceSquare
		(
			curve1.col(i+0), curve1.col(i+1), 
			curve2.col(i+0), curve2.col(i+1) 
		);
	}
	return distanceSquare;
}
Template(T) Vector3t QStanMath<T>::sketchPoint(const Vector2t& point, const Vector4t& plane)
{
	Vector3t pixel=Vector3t(point(0), point(1), 0);
	Vector3t direction=(Vector3t(0, -0.1, -1)).normalized();
	return intersectPlane(pixel-direction, direction, plane);
}
Template(T) Vector2t QStanMath<T>::canvasPoint(const Vector3t& point)
{
	Vector4t plane=createPlane(0, 0, 1, 0);
	Vector3t direction=(Vector3t(0, 0.1, 1)).normalized();
	Vector3t intersectionPoint=intersectPlane(point, direction, plane);
	return Vector2t(intersectionPoint(0), intersectionPoint(1));
}
Template(T) Vector3t QStanMath<T>::antialiasPoint(Vector3t point, Vector3t yAxis, Vector4t groundPlane)
{
	Vector3t groundPoint=intersectPlane(point, -yAxis, groundPlane);
	return groundPoint+(point-groundPoint).norm()*getNormal(groundPlane);
}
Template(T) Vector2t QStanMath<T>::toVector2t(QVector2D vector2d)
{
	return Vector2t(vector2d.x(), vector2d.y());
}
Template(T) Vector3t QStanMath<T>::toVector3t(QVector3D vector3d)
{
	return Vector3t(vector3d.x(), vector3d.y(), vector3d.z());
}
Template(T) Vector4t QStanMath<T>::toVector4t(QVector4D vector4d)
{
	return Vector4t(vector4d.x(), vector4d.y(), vector4d.z(), vector4d.w());
}
Template(T) QVector2D QStanMath<T>::toVector2D(Vector2d vector2d)
{
	return QVector2D(vector2d(0), vector2d(1));
}
Template(T) QVector3D QStanMath<T>::toVector3D(Vector3d vector3d)
{
	return QVector3D(vector3d(0), vector3d(1), vector3d(2));
}
Template(T) QVector4D QStanMath<T>::toVector4D(Vector4d vector4d)
{
	return QVector4D(vector4d(0), vector4d(1), vector4d(2), vector4d(3));
}
Template(T) Vector3t* QStanMath<T>::getQuad(Vector4t plane, Vector3t space)
{
	T length=space(0), height=space(1), width=space(2);
	Vector3t p000=Vector3t(-length, -height, -width);
	Vector3t p001=Vector3t(-length, -height,  width);
	Vector3t p011=Vector3t(-length,  height,  width);
	Vector3t p010=Vector3t(-length,  height, -width);
	Vector3t p100=Vector3t( length, -height, -width);
	Vector3t p101=Vector3t( length, -height,  width);
	Vector3t p111=Vector3t( length,  height,  width);
	Vector3t p110=Vector3t( length,  height, -width);
	Vector3t normal=getNormal(plane);
	qreal xDotn=abs(xAxis.dot(normal));
	qreal yDotn=abs(yAxis.dot(normal));
	qreal zDotn=abs(zAxis.dot(normal));
	qreal maxDot=max(xDotn, max(yDotn, zDotn));
	if(xDotn==maxDot)return intersectPlane
	(
		plane, xAxis, new Vector3t[4]
		{
			p011, p001, p000, p010
		}
	);
	else if(yDotn==maxDot)return intersectPlane
	(
		plane, yAxis, new Vector3t[4]
		{
			p000, p001, p101, p100
		}
	);
	else return intersectPlane
	(
		plane, zAxis, new Vector3t[4]
		{
			p010, p000, p100, p110
		}
	);
}
Template(T) Vector3t* QStanMath<T>::intersectPlane(Vector4t plane, Vector3t axis, Vector3t* points)
{
	Vector3t* quad=new Vector3t[4];
	for(int i=0; i<4; i++)
	{
		quad[i]=intersectPlane(points[i], axis, plane);
	}
	return quad;
}
Template(Scalar) class Param
{
	public:
	Scalar step=0.1;
	Scalar epsilon=1e-5;
	int max_iterations=100;
};
Template(Scalar) class Solver
{
	public:
	Param<Scalar> param;
	Solver(Param<Scalar> p):param(p){}
	typedef Matrix<Scalar, Dynamic, 1> Vector;
	typedef Matrix<Scalar, Dynamic, Dynamic> Matrix;
	Template(Function) int minimize(Function& f, Vector& x, Scalar& fx)
	{
		Scalar zero=param.epsilon, step=param.step;
		int t=0, time=param.max_iterations; Vector dx;
		for
		(
			fx=f(x, dx); 
			dx.dot(dx)>zero&&t<time;
			x-=step*dx, fx=f(x, dx), t++
		);
		return t;
	}
};
#undef Template(T)
