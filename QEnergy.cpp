#include <QFile>
#include <QDebug>
#include "QEnergy.h"
#include "QAnalyzer.h"
#include <QTextStream>
#define QVarMath QStanMath<var>
#define QDoubleMath QStanMath<double>

var QEnergy::depthEnergy(const VectorXv& planes)
{
	int planeIndex=sketch(2);
	Vector2d point=sketch.head(2);
	Vector4v plane=planes.segment(planeIndex*4, 4);
	var z=sketchPoint(point, plane)(2); return z*z;
}
var QEnergy::accuracyEnergy(const VectorXv& variables)
{
	var error=0;
	int size=sketch.size()/3;
	for(int i=0, p=planeSize*4; i<size; i++)
	{
		var x0=sketch(i*3+0);
		var y0=sketch(i*3+1);
		var x1=variables(p+i*2+0);
		var y1=variables(p+i*2+1);
		var dx=x1-x0, dy=y1-y0;
		error+=dx*dx+dy*dy;
	}
	return accuracy*error/size;
}
var QEnergy::foreshorteningEnergy(const VectorXv& variables)
{
return 0;
	var weight=10, zSquare=0.0;
	int size=sketch.size()/3;
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
	return QVarMath::cosAngle(p1, p0, p1, p2);
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
	/*
	var dot=QVarMath::normalizedPlane(plane1).head(3).
	dot(QVarMath::normalizedPlane(plane2).head(3));
	return 1-dot*dot;
	*/
	return QVarMath::distanceBetweenDirections(plane1.head(3), plane2.head(3));
}
var QEnergy::distanceEnergy(const Vector2v& point1, const Vector4v& plane1, const Vector2v& point2, const Vector4v& plane2)
{
	return (sketchPoint(point1, plane1)-sketchPoint(point2, plane2)).squaredNorm();
}
var QEnergy::verticalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane)
{
	Vector3v start=sketchPoint(startPoint, plane), end=sketchPoint(endPoint, plane);
	Vector4v ground=groundPlane(); Vector3v up=QVarMath::getNormal(ground);
	return QVarMath::distanceBetweenDirections(end-start, up);
}
var QEnergy::horizontalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane)
{
	Vector3v start=sketchPoint(startPoint, plane), end=sketchPoint(endPoint, plane);
	return QVarMath::distanceBetweenDirections(end-start, axis[0]);
}
var QEnergy::forwardEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane)
{
	Vector3v start=sketchPoint(startPoint, plane), end=sketchPoint(endPoint, plane);
	return QVarMath::distanceBetweenDirections(end-start, axis[2]);
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
	return QVarMath::distanceBetweenDirections(end1-start1, end2-start2);
}
var QEnergy::coplanarEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& srcPlane, const Vector4v& destPlane)
{
	Vector3v start1=sketchPoint(startPoint, srcPlane);
	Vector3v end1=sketchPoint(endPoint, srcPlane);
	Vector3v start2=QVarMath::projectedPoint(start1, destPlane);
	Vector3v end2=QVarMath::projectedPoint(end1, destPlane);
	var cosAngle=QVarMath::cosAngle(start1, end1, start2, end2);
	var directionDistance=1-cosAngle*cosAngle;
	return QVarMath::integrateDistanceSquare(start1, end1, start2, end2)+directionDistance;
}
var QEnergy::collinearEnergy(const MatrixXv& curve, const Vector4v& plane, int start, int end)
{
	MatrixXv subcurve=QVarMath::subCurve(curve, start ,end);
	MatrixXv projectedcurve=QVarMath::projectedCurve(subcurve, plane);
	return QVarMath::distanceSquareBetween(subcurve, projectedcurve);
}
//#define  QDE 0
#ifdef QDE //QDebug For Energy Term
#define QD(s) qDebug()<<s<<" : tmp energy="<<energy.val()
#else 
#define QD(s) 0
#endif
var QEnergy::weight(int type)
{
	#define R QAnalyzer
	switch(type)
	{
		case R::FORWARD: return 400;
		case R::HORIZONTAL: return 400;
		case R::DISTANCE: return isPlaneOnly?2:20;
		case R::COPLANAR: return isPlaneOnly?5:20;
		case R::VERTICAL: return isPlaneOnly?2:400;
		case R::CONTACT_POINTS: return isPlaneOnly?10:100;
	}
	return 1;
	#undef R
}
var QEnergy::totalEnergy(const VectorXv& variables)
{
	#define planeOf(x) variables.segment((x)*4, 4)
	#define pointOf(x) pointAt(variables, x)
	Vector4v ground=planeOf(planeSize-1);
	if(!isPlaneOnly)ground=groundPlane();
	var energy=depthEnergy(variables);
	ground=QVarMath::normalizedPlane(ground);
	isAddingAxis=isPlaneOnly&&axisIndices.size()==0;
	//if(!isPlaneOnly)energy+=accuracyEnergy(variables);
	for(int i=0; i<regularity.size(); i++)
	{
		#define R QAnalyzer
		#define t(x) regularity[i+x]
		switch(t(0))
		{
			case R::AXIS: if(isAddingAxis)addAxis(t(1), t(2), t(3), t(4)); break;
			case R::PARALLEL_PLANES: energy+=weight(t(0))*parallelEnergy(planeOf(t(1)), planeOf(t(2))); QD("PARP"); break;
			case R::VERTICAL: energy+=weight(t(0))*verticalEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3))); QD("VER"); break;
			case R::FORWARD:energy+=weight(t(0))*forwardEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3))); QD("FOR"); break;
			case R::HORIZONTAL: energy+=weight(t(0))*horizontalEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3))); QD("HOR"); break;
			case R::DISTANCE: energy+=weight(t(0))*distanceEnergy(pointOf(t(1)), planeOf(t(2)), pointOf(t(3)), planeOf(t(4))); QD("DIS"); break;
			case R::COPLANAR: energy+=weight(t(0))*coplanarEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3)), planeOf(t(4))); QD("COP"); break;
			case R::SAME_POINTS: energy+=isPlaneOnly?weight(t(0))*stdDevAnglesEnergy(variables, t(1), t(2)-1):0; samePoints<< t(1)<< t(2);  QD("SDA"); break;
			case R::CONTACT_POINTS: energy+=weight(t(0))*coplanarEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3)), ground); QD("GCOP"); break;
		//	case R::PERPENDICULAR: energy+=weight(t(0))*perpendicularEnergy(pointOf(t(1)), pointOf(t(2)), pointOf(t(3)), planeOf(t(4))); QD("PER"); break;
			case R::PARALLEL: energy+=weight(t(0))*parallelEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3)), pointOf(t(4)), pointOf(t(5)), planeOf(t(6))); QD("PAR"); break;
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
Vector4v QEnergy::groundPlane()
{
	return toVector4v(ground);
}
Vector4v QEnergy::getGroundPlane(const VectorXd& variables)
{
	return QVarMath::normalizedPlane(toVector4v(variables, (planeSize-1)*4));
}
Vector3v QEnergy::getLineDirection(const VectorXv& variables, int startIndex, int endIndex, int planeIndex)
{
	#define planeOf(x) variables.segment((x)*4, 4)
	#define pointOf(x) pointAt(variables, x)
	Vector2v p0=pointOf(startIndex);
	Vector2v p1=pointOf(endIndex);
	Vector4v plane=planeOf(planeIndex);
	Vector3v s0=sketchPoint(p0, plane);
	Vector3v s1=sketchPoint(p1, plane);
	return (s1-s0).normalized();
	#undef planeOf(x)
	#undef pointOf(x)
}
Vector3v QEnergy::addLineDirection(Vector3v dir1, Vector3v dir2)
{
	if(dir1.norm()==0)return dir2;
	if(dir1.dot(dir2)>0)return dir1+dir2; 
	else return dir1-dir2;
}
void QEnergy::addAxis(int axisIndex, int startIndex, int endIndex, int planeIndex)
{
	veci x, y, z; if(axis.size()==0)this->axisIndices<<x<<y<<z;
	this->axisIndices[axisIndex]<<startIndex<<endIndex<<planeIndex;
}
Vector3v getClosestDirection(QVector<Vector3v> dirs, Vector3v dir)
{
	var minDistance=1; int min=0;
	for(int i=0; i<dirs.size(); i++)
	{
		var distance=QVarMath::distanceBetweenDirections(dir, dirs[i]);
		if(distance<minDistance){minDistance=distance; min=i;}
	}
	if(dirs[min].dot(dir)<0)return -dirs[min];
	else return dirs[min];
}
void QEnergy::getAxis(const VectorXv& variables)
{
	if(!isPlaneOnly)return;
	#define k(t) axisIndices[i][j+t]
	QVector<Vector3v> axis;
	for(int i=0; i<3; i++)
	{
		axis<<Vector3v(0, 0, 0); Vector3v dir;
		for(int j=0; j<axisIndices[i].size(); j+=3)
		{
			dir=getLineDirection(variables, k(0), k(1), k(2));
			axis[i]=addLineDirection(axis[i], dir);
		}
		axis[i].normalize();
	}
	this->axis.clear();
	this->axis<<getClosestDirection(axis, Vector3v(1, 0, 0));
	this->axis<<getClosestDirection(axis, Vector3v(0, 1, 0));
	this->axis<<getClosestDirection(axis, Vector3v(0, 0, 1));
	#undef k(t)
}
Vector3v QEnergy::sketchPoint(const Vector2d& point, const Vector4v& plane)
{
	return sketchPoint(Vector2v(point(0), point(1)), plane);
}
Vector3v QEnergy::sketchPoint(const Vector2v& point, const Vector4v& plane)
{
	Vector3v pixel=Vector3v(point(0), point(1), 0);
	Vector3v	direction=(Vector3v(0, -0.1, -1)).normalized();
	return QVarMath::intersectPlane(pixel-direction, direction, plane);
}
Vector3v QEnergy::sketchPoint(const VectorXv& variables, int sketchIndex)
{
	int planeIndex=sketch(sketchIndex*3+2);
	Vector4v plane=variables.segment(planeIndex*4, 4);
	return sketchPoint(pointAt(variables, sketchIndex), plane);
}
MatrixXv QEnergy::getSketchPoints(const VectorXd& variable)
{
	int size=sketch.size()/3;
	MatrixXv sketchPoints(3, size);
	for(int i=0; i<size; i++)
	{
		int pointIndex=planeSize*4+i*2;
		int planeIndex=sketch(i*3+2)*4;
		Vector2v point=toVector2v(variable, pointIndex);
		Vector4v plane=toVector4v(variable, planeIndex);
		sketchPoints.col(i)=sketchPoint(point, plane);
	}
	return sketchPoints;
}
vec2 QEnergy::sketchPixel(int x, int y, QViewer viewer)
{
	return vec2
	(
		 (x-viewer.width/2+0.0)/(viewer.width/2),
		-(y-viewer.height/2+0.0)/(viewer.height/2)/viewer.aspectRatio
	);
}
QEnergy::QEnergy(veci path, veci sketch, int planeSize, veci regularity, QVector<vec4> planes, QVector<vec3> axis, QViewer viewer)
{
	this->path=path; this->point3D=sketch; 
	this->regularity=regularity; this->viewer=viewer;
	vec pixels, variables; 
	if(planes.size())
	{
		this->isPlaneOnly=false;
		this->planeSize=planes.size();
		for(int i=0; i<this->planeSize; i++)
		{
			variables<<planes[i].x()<<planes[i].y();
			variables<<planes[i].z()<<planes[i].w();
		}
		this->ground=planes[this->planeSize-1];
	}
	else
	{
		this->isPlaneOnly=true; this->planeSize=planeSize;
		for(int i=0; i<planeSize-1; i++)variables<<0<<0<<1<<0;
		variables<<ground.x()<<ground.y()<<ground.z()<<ground.w();
	}
	if(!isPlaneOnly)
	{
		this->axis=toVector3v(axis);
		this->axis[1]=QVarMath::getNormal(toVector4v(ground));
		this->axis[2]=this->axis[0].cross(this->axis[1]);
		this->axis[0]=this->axis[1].cross(this->axis[2]);
	}
	for(int i=0; i<sketch.size(); i+=3)
	{
		vec2 pixel=sketchPixel(sketch[i+0], sketch[i+1], viewer);
		pixels<<pixel.x()<<pixel.y()<<sketch[i+2];
		variables<<pixel.x()<<pixel.y();
	}
	this->sketch=toVectorXd(pixels);
	this->variables=toVectorXd(variables);
}
QVector<QVector3D> QEnergy::toVector3D(QVector<Vector3v> vector3v)
{
	QVector<QVector3D> vector3D;
	for(Vector3v v : vector3v)vector3D<<toQVector3D(v);
	return vector3D;
}
QVector<Vector3v> QEnergy::toVector3v(QVector<QVector3D> vector3D)
{
	QVector<Vector3v> vector3v;
	for(QVector3D v : vector3D)vector3v<<toVector3v(v);
	return vector3v;
}
void QEnergy::analyze()
{
	int size=sketch.size()/3; var error=0.05;
	VectorXv variables=toVectorXv(this->variables);
	this->horizontal.clear(); this->forward.clear(); 
	for(int i=0; i<size-1; i++)
	{
		int i0=i+0, i1=i+1;
		int plane0=sketch(i0*3+2);
		int plane1=sketch(i1*3+2);
		if(plane0!=plane1)continue;
		Vector3v p0=sketchPoint(variables, i0);
		Vector3v p1=sketchPoint(variables, i1);
		Vector3v direction=p1-p0, center=(p0+p1)/2;
		if(QVarMath::distanceBetweenDirections(direction, axis[0])<error)
		{
			this->regularity<<QAnalyzer::HORIZONTAL<<i0<<i1<<plane0;
			this->horizontal<<center(0).val()<<center(1).val()<<center(2).val();
		}
		else if(QVarMath::distanceBetweenDirections(direction, axis[2])<error)
		{
			this->regularity<<QAnalyzer::FORWARD<<i0<<i1<<plane0;
			this->forward<<center(0).val()<<center(1).val()<<center(2).val();
		}
	}
}
veci QEnergy::getPoint2D()
{
	veci point2D;
	for(int i=0; i<point3D.size(); i++)
	{
		if((i+1)%3!=0)point2D<<point3D[i];
	}
	return point2D;
}
QVector<vec4> QEnergy::getPlanes()
{
	QVector<vec4> planes;
	for(int i=0; i<planeSize; i++)
	{
		planes<<toQVector4D(toVector4v(variables, i*4));
	}
	return planes;
}
void QEnergy::save(QString fileName)
{
	this->analyze();
	QAnalyzer analyzer; 
	analyzer.analyze
	(
		path, this->getPoint2D(), regularity, 
		this->getPlanes(), toVector3D(axis), viewer
	);
	analyzer.save(fileName);
}
Vector2v QEnergy::pointAt(int index)
{
	double x=sketch(index*3+0);
	double y=sketch(index*3+1);
	return Vector2v(x, y);
}
Vector2v QEnergy::pointAt(const VectorXv& variable, int index)
{
	if(isPlaneOnly)return pointAt(index);
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
VectorXd QEnergy::toVectorXd(vec vector)
{
	std::vector<double> stdVector=vector.toStdVector();
	return Map<VectorXd>(stdVector.data(), stdVector.size());
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
Vector4v QEnergy::toVector4v(vec4 vector4D)
{
	return Vector4v(vector4D.x(), vector4D.y(), vector4D.z(), vector4D.w());
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
VectorXv QEnergy::toVectorXv(const VectorXd& vectorXd)
{
	VectorXv vectorXv(vectorXd.size());
	for(int i=0; i<vectorXd.size(); i++)
	vectorXv(i)=vectorXd(i);
	return vectorXv;
}
var QEnergy::decay(var distance, var speed)
{
	return 1-(distance*speed)*(distance*speed);
}
