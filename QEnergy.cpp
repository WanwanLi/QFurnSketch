#include <QFile>
#include <QDebug>
#include "QEnergy.h"
#include "QAnalyzer.h"
#include <QTextStream>
#define QVarMath QStanMath<var>
#define QDoubleMath QStanMath<double>

var QEnergy::depthEnergy(const VectorXv& planes)
{
	int planeIndex=sketchVector(2);
	Vector2i point=sketchVector.head(2);
	Vector4v plane=planes.segment(planeIndex*4, 4);
	var z=sketchPoint(point, plane)(2); return z*z;
}
var QEnergy::accuracyEnergy(const VectorXv& variables)
{
	var weight=0.01, error=0;
	int size=sketchVector.size()/3;
	for(int i=0, p=planeSize*4; i<size; i++)
	{
		var x0=sketchVector(i*3+0);
		var y0=sketchVector(i*3+1);
		var x1=variables(p+i*2+0);
		var y1=variables(p+i*2+1);
		var dx=x1-x0, dy=y1-y0;
		error+=dx*dx+dy*dy;
	}
//qDebug()<<"accuracyEnergy: "<<(weight*error).val();
	return weight*error/size;
}
var QEnergy::foreshorteningEnergy(const VectorXv& variables)
{
return 0;
	var weight=10, zSquare=0.0;
	int size=sketchVector.size()/3;
	for(int i=0; i<size; i++)
	{
		var z=sketchPoint(variables, i)(2); 
		zSquare+=z*z;
	}
//qDebug()<<"foreshorteningEnergy: "<<(weight*zSquare).val();
	return weight*zSquare/size;
}
var QEnergy::cosAngle(const VectorXv& variables, int i0, int i1, int i2)
{
	Vector3v p0=sketchPoint(variables, i0);
	Vector3v p1=sketchPoint(variables, i1);
	Vector3v p2=sketchPoint(variables, i2);
	Vector3v d1=(p0-p1).normalized();
	Vector3v d2=(p2-p1).normalized();
	return d1.dot(d2);
}
void addCosAngle(var cosA, var& sinS, var& cosS, int& size)
{
	var A=acos(cosA); sinS+=sin(A);  cosS+=cosA;  size++;
}
var QEnergy::stdDevAnglesEnergy(const VectorXv& variables, int start, int end)
{
	#define isValidCosAngle(a) (a>-0.99)
	int size=0; var sinS=0, cosS=0, cosA;
	for(int i=start+1; i<end; i++)
	{
		var cosA=cosAngle(variables, i-1, i, i+1);
		if(isValidCosAngle(cosA))
		addCosAngle(cosA, sinS, cosS, size);
	}
	cosA=cosAngle(variables, end, start, start+1);
	if(isValidCosAngle(cosA))
	addCosAngle(cosA, sinS, cosS, size);
	cosA=cosAngle(variables, end-1, end, start);
	if(isValidCosAngle(cosA))
	addCosAngle(cosA, sinS, cosS, size);
	sinS/=size; cosS/=size;
	return sqrt(-log(sinS*sinS+cosS*cosS));
	#undef isValidCosAngle(a)
}
var QEnergy::parallelEnergy(const Vector4v& plane1, const Vector4v& plane2)
{
	var dot=QVarMath::normalizedPlane(plane1).head(3).
	dot(QVarMath::normalizedPlane(plane2).head(3));
	return 1-dot*dot;
}
var QEnergy::distanceEnergy(const Vector2v& point1, const Vector4v& plane1, const Vector2v& point2, const Vector4v& plane2)
{
#ifdef QDB_DE 
qDebug()<<"distanceEnergy="<< (sketchPoint(point1, plane1)-sketchPoint(point2, plane2)).squaredNorm().val();
#endif
	return (sketchPoint(point1, plane1)-sketchPoint(point2, plane2)).squaredNorm();
}
var QEnergy::verticalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane)
{
	Vector3v start=sketchPoint(startPoint, plane);
	Vector3v end=sketchPoint(endPoint, plane);
	var dot=(end-start).normalized().dot(Vector3v(0, 1, 0));
	return 1-dot*dot;
}
var QEnergy::perpendicularEnergy(const Vector2v& leftPoint, const Vector2v& midPoint, const Vector2v& rightPoint, const Vector4v& plane)
{
return 0;
	Vector3v left=sketchPoint(leftPoint, plane);
	Vector3v mid=sketchPoint(midPoint, plane);
	Vector3v right=sketchPoint(rightPoint, plane);
	var dot=(left-mid).normalized().dot((right-mid).normalized());
	return dot*dot;
}
var QEnergy::parallelEnergy(const Vector2v& startPoint1, const Vector2v& endPoint1, const Vector4v& plane1, const Vector2v& startPoint2, const Vector2v& endPoint2, const Vector4v& plane2)
{
	Vector3v start1=sketchPoint(startPoint1, plane1);
	Vector3v end1=sketchPoint(endPoint1, plane1);
	Vector3v start2=sketchPoint(startPoint2, plane2);
	Vector3v end2=sketchPoint(endPoint2, plane2);
	var dot=(end1-start1).normalized().dot((end2-start2).normalized());
	return 1-dot*dot;
}
var QEnergy::coplanarEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& srcPlane, const Vector4v& destPlane)
{
	Vector3v start1=sketchPoint(startPoint, srcPlane);
	Vector3v end1=sketchPoint(endPoint, srcPlane);
	Vector3v start2=QVarMath::projectedPoint(start1, destPlane);
	Vector3v end2=QVarMath::projectedPoint(end1, destPlane);
	return QVarMath::integrateDistanceSquare(start1, end1, start2, end2);
}
var QEnergy::collinearEnergy(const MatrixXv& curve, const Vector4v& plane, int start, int end)
{
	MatrixXv subcurve=QVarMath::subCurve(curve, start ,end);
	MatrixXv projectedcurve=QVarMath::projectedCurve(subcurve, plane);
	return QVarMath::distanceSquareBetween(subcurve, projectedcurve);
}
//#define QDB_GCE 0
//#define QDB_GCE_D 0
var QEnergy::groundCoplanarEnergy(const Vector2v& startPoint1, const Vector2v& endPoint1, const Vector4v& plane1, const Vector2v& startPoint2, const Vector2v& endPoint2, const Vector4v& plane2)
{
	Vector3v start1=sketchPoint(startPoint1, plane1);
	Vector3v end1=sketchPoint(endPoint1, plane1);
#ifdef QDB_GCE_D
qDebug()<<"start1.y="<<start1(1).val();
qDebug()<<"end1.y="<<end1(1).val();
qDebug()<<"|d1.y|="<<abs(start1(1)-end1(1)).val();
#endif
	Vector3v start2=sketchPoint(startPoint2, plane2);
	Vector3v end2=sketchPoint(endPoint2, plane2);
#ifdef QDB_GCE_D
qDebug()<<"start2.y="<<start2(1).val();
qDebug()<<"end2.y="<<end2(1).val();
qDebug()<<"|d2.y|="<<abs(start2(1)-end2(1)).val();
#endif
	var weight=10, dy=start1(1)-end1(1), sum=dy*dy;
	dy=start2(1)-end2(1); sum+=dy*dy;
	dy=start1(1)-end2(1); sum+=dy*dy;
	dy=start2(1)-end1(1); 
#ifdef QDB_GCE 
qDebug()<<"groundCoplanarEnergy="<<(weight*(sum+dy*dy)).val();
#endif
return weight*(sum+dy*dy);
}
//#define  QDE 0
#ifdef QDE //QDebug For Energy Term
#define QD(s) qDebug()<<s<<" : tmp energy="<<energy.val()
#else 
#define QD(s) 0
#endif
var QEnergy::totalEnergy(const VectorXv& variables)
{
	#define planeOf(x) variables.segment(x*4, 4)
	#define pointOf(x) pointAt(variables, x)
	var energy=depthEnergy(variables);
	//energy+=accuracyEnergy(variables);
	//energy+=foreshorteningEnergy(variables);
	for(int i=0; i<regularity.size(); i++)
	{
		#define R QAnalyzer
		#define t(x) regularity(i+x)
		switch(t(0))
		{
			case R::SAME_POINTS: energy+=stdDevAnglesEnergy(variables, t(1), t(2)-1); QD("SDA"); break;
			case R::PARALLEL_PLANES: energy+=parallelEnergy(planeOf(t(1)), planeOf(t(2))); QD("PARP"); break;
			case R::VERTICAL: energy+=verticalEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3))); QD("VER"); break;
			case R::DISTANCE: energy+=distanceEnergy(pointOf(t(1)), planeOf(t(2)), pointOf(t(3)), planeOf(t(4))); QD("DIS"); break;
			case R::COPLANAR: energy+=coplanarEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3)), planeOf(t(4))); QD("COP"); break;
			case R::PERPENDICULAR: energy+=perpendicularEnergy(pointOf(t(1)), pointOf(t(2)), pointOf(t(3)), planeOf(t(4))); QD("PER"); break;
			case R::PARALLEL: energy+=parallelEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3)), pointOf(t(4)), pointOf(t(5)), planeOf(t(6))); QD("PAR"); break;
			case R::CONTACT_POINTS: energy+=groundCoplanarEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3)), pointOf(t(4)), pointOf(t(5)), planeOf(t(6))); QD("CON"); break;
		}
		i+=R::count(t(0));
		#undef R
		#undef t(x)
		#undef next()
	}
	#undef planeOf(x)
	#undef pointOf(x)
	return energy;
}
Vector3v QEnergy::sketchPoint(const Vector2i& point, const Vector4v& plane)
{
	return sketchPoint(Vector2v(point(0), point(1)), plane);
}
Vector3v QEnergy::sketchPoint(const Vector2v& point, const Vector4v& plane)
{
	Vector3v viewDirection=-forward;
	Vector3v eye=viewDirection*viewDistance;
	Vector3v focus=viewDirection*(viewDistance-focalLength);
	var x=(point(0)-sketchWidth/2)/(sketchWidth/2)/screenScale;
	var y=(point(1)-sketchHeight/2)/(sketchHeight/2)/aspectRatio/screenScale;
	return QVarMath::intersectPlane(eye, (focus+x*right-y*up)-eye, plane);
}
Vector3v QEnergy::sketchPoint(const VectorXv& variables, int sketchIndex)
{
	int planeIndex=sketchVector(sketchIndex*3+2);
	Vector4v plane=variables.segment(planeIndex*4, 4);
	return sketchPoint(pointAt(variables, sketchIndex), plane);
}
QPoint QEnergy::canvasPoint(double x, double y, double z)
{
	Vector3v point=Vector3v(x, y, z);
	Vector3v viewDirection=-forward;
	Vector3v eye=viewDirection*viewDistance;
	Vector3v focus=viewDirection*(viewDistance-focalLength);
	Vector4v viewPlane=QVarMath::createPlane(focus, viewDirection);
	Vector3v position=QVarMath::intersectPlane(eye, point-eye, viewPlane);
	x=screenScale*position.dot(right).val()*sketchWidth/2+sketchWidth/2; 
	y=screenScale*aspectRatio*position.dot(up).val()*sketchHeight/2+sketchHeight/2;
	return QPoint((int)x, sketchHeight-(int)y);
}
MatrixXv QEnergy::getSketchPoints(const VectorXd& variable)
{
	int size=sketchVector.size()/3;
	MatrixXv sketchPoints(3, size);
	for(int i=0; i<size; i++)
	{
		int pointIndex=planeSize*4+i*2;
		int planeIndex=sketchVector(i*3+2)*4;
		Vector2v point=toVector2v(variable, pointIndex);
		Vector4v plane=toVector4v(variable, planeIndex);
		sketchPoints.col(i)=sketchPoint(point, plane);
	}
	return sketchPoints;
}
MatrixXv QEnergy::sketchCurves(VectorXd& variables)
{
	int size=sketchVector.size()/3;
	MatrixXv curves(3, size);
	for(int i=0; i<size; i++)
	{
		int pointIndex=planeSize*4+i*2;
		int planeIndex=sketchVector(i*3+2)*4;
		Vector2v point=toVector2v(variables, pointIndex);
		Vector4v plane=toVector4v(variables, planeIndex);
		curves.col(i)=sketchPoint(point, plane);
	}
	return curves;
}
MatrixXd QEnergy::sketchPlanes(VectorXd& variables)
{
	Map<MatrixXd> planes(variables.data(), 4, planeSize);
	for(int i=0; i<planeSize; i++)
	{
		planes.col(i)=QDoubleMath::normalizedPlane(planes.col(i));
	}
	return planes;
}
void QEnergy::setupViewer(QViewer viewer)
{
	this->up=toVector3v(viewer.up);
	this->right=toVector3v(viewer.right);
	this->forward=toVector3v(viewer.forward);
	this->viewDistance=viewer.viewDistance;
	this->focalLength=viewer.focalLength;
	this->screenScale=viewer.screenScale;
	this->sketchWidth=viewer.width;
	this->sketchHeight=viewer.height;
	this->aspectRatio=viewer.aspectRatio;
}
QEnergy::QEnergy(veci path, veci sketch, int planesSize, veci regularity, QViewer viewer)
{
	this->path=path; this->planeSize=planesSize; this->setupViewer(viewer);
	std::vector<int> s=sketch.toStdVector(), r=regularity.toStdVector();
	this->regularity=Map<VectorXi>(r.data(), r.size());
	this->sketchVector=Map<VectorXi>(s.data(), s.size());
	vec variable; for(int i=0; i<planesSize; i++)variable<<0<<0<<1<<0;
	for(int i=0; i<sketch.size(); i+=3)variable<<sketch[i+0]<<sketch[i+1];
	std::vector<double> v=variable.toStdVector(); 
	this->variableVector=Map<VectorXd>(v.data(), v.size());
}
Vector2v QEnergy::pointAt(int index)
{
	int x=sketchVector(index*3+0);
	int y=sketchVector(index*3+1);
	return Vector2v(x, y);
}
Vector2v QEnergy::pointAt(const VectorXv& variable, int index)
{
	int i=planeSize*4+index*2;
	return variable.segment(i, 2);
}
void QEnergy::copySameGradients(VectorXd& grad)
{
	for(int i=0; i<samePoints.size(); i+=2)
	{
		int index0=samePoints[i+0];
		int index1=samePoints[i+1];
		int src=planeSize*4+index0*2;
		int dest=planeSize*4+index1*2;
		grad[dest+0]=grad[src+0]; 
		grad[dest+1]=grad[src+1]; 
	}
}
Vector3v QEnergy::toVector3v(QVector3D vector3D)
{
	return Vector3v(vector3D.x(), vector3D.y(), vector3D.z());
}
Vector2v QEnergy::toVector2v(const VectorXd& vectorXd, int startIndex)
{
	return Vector2v(vectorXd(startIndex+0), vectorXd(startIndex+1));
}
Vector4v QEnergy::toVector4v(const VectorXd& vectorXd, int startIndex)
{
	return Vector4v(vectorXd(startIndex+0), vectorXd(startIndex+1), vectorXd(startIndex+2), vectorXd(startIndex+3));
}
QVector2D QEnergy::toQVector2D(Vector2v vector2v)
{
	return QVector2D(vector2v(0).val(), vector2v(1).val());
}
QVector3D QEnergy::toQVector3D(Vector3v vector3v)
{
	return QVector3D(vector3v(0).val(), vector3v(1).val(), vector3v(2).val());
}
QVector4D QEnergy::toQVector4D(Vector4v vector4v)
{
	return QVector4D(vector4v(0).val(), vector4v(1).val(), vector4v(2).val(), vector4v(3).val());
}
veci QEnergy::toQVector(VectorXi vector)
{
	veci result; for(int i=0; i<vector.size(); i++)result<<vector[i]; return result;
}
vec QEnergy::toQVector(VectorXd vector)
{
	vec result; for(int i=0; i<vector.size(); i++)result<<vector[i]; return result;
}
vec QEnergy::toQVector(VectorXv vector)
{
	vec result; for(int i=0; i<vector.size(); i++)result<<vector[i].val(); return result;
}
