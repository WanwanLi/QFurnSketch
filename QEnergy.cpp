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
var QEnergy::scaleAt(const VectorXv& variables, int index)
{
	int i0=index+0, i1=index+1;
	Vector2v c0=pointAt(i0), c1=pointAt(i1);
	Vector3v p0=sketchPoint(variables, i0);
	Vector3v p1=sketchPoint(variables, i1);
	var distance=QVarMath::distanceBetween(p0, p1);
	return distance/QVarMath::distanceBetween(c0, c1);
}
var QEnergy::stdDevScalesEnergy(const VectorXv& variables, int start, int end)
{
	var mean=0;
	QVector<var> scales;
	for(int i=start; i<end; i++)
	{
		var scale=scaleAt(variables, i);
		scales<<scale; mean+=scale;
	}
	mean/=scales.size();
	var stdDevScale=0;
	for(var scale : scales)
	{
		var dev=scale-mean;
		stdDevScale+=dev*dev;
	}
	return stdDevScale;
}
var QEnergy::accuracyEnergy(const VectorXv& variables, int start, int end)
{
	var error=0;
	for(int i=start; i<end; i++)
	{
		Vector3v p0=sketchPoint(variables, i+0);
		Vector3v p1=sketchPoint(variables, i+1);
		var distance=QVarMath::distanceBetween(p0, p1);
		error+=QVarMath::abs(distance-pointDistances[i]);
	}
	return error;
}
void QEnergy::addPointDistances(const VectorXv& variables, int start, int end)
{
	for(int i=start; i<end; i++)
	{
		Vector3v p0=sketchPoint(variables, i+0), p1=sketchPoint(variables, i+1);
		this->pointDistances<<QVarMath::distanceBetween(p0, p1).val();
	}
}
var QEnergy::parallelEnergy(const Vector4v& plane1, const Vector4v& plane2)
{
	return QVarMath::distanceBetweenDirections(plane1.head(3), plane2.head(3));
}
var QEnergy::distanceEnergy(const Vector2v& point1, const Vector4v& plane1, const Vector2v& point2, const Vector4v& plane2)
{
	return QVarMath::distanceBetween(QVarMath::sketchPoint(point1, plane1), QVarMath::sketchPoint(point2, plane2));
}
var QEnergy::verticalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane)
{
	Vector3v start=QVarMath::sketchPoint(startPoint, plane);
	Vector3v end=QVarMath::sketchPoint(endPoint, plane);
	Vector4v ground=groundPlane(); Vector3v up=QVarMath::getNormal(ground);
	return QVarMath::distanceBetweenDirections(end-start, up);
}
var QEnergy::horizontalEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane)
{
	Vector3v start=QVarMath::sketchPoint(startPoint, plane);
	Vector3v end=QVarMath::sketchPoint(endPoint, plane);
	return QVarMath::distanceBetweenDirections(end-start, axis[0]);
}
var QEnergy::forwardEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& plane)
{
	Vector3v start=QVarMath::sketchPoint(startPoint, plane);
	Vector3v end=QVarMath::sketchPoint(endPoint, plane);
	return QVarMath::distanceBetweenDirections(end-start, axis[2]);
}
var QEnergy::perpendicularEnergy(const Vector2v& leftPoint, const Vector2v& midPoint, const Vector2v& rightPoint, const Vector4v& plane)
{
	Vector3v left=QVarMath::sketchPoint(leftPoint, plane);
	Vector3v mid=QVarMath::sketchPoint(midPoint, plane);
	Vector3v right=QVarMath::sketchPoint(rightPoint, plane);
	return 1-QVarMath::distanceBetweenDirections(left-mid, right-mid);
}
var QEnergy::parallelEnergy(const Vector2v& startPoint1, const Vector2v& endPoint1, const Vector4v& plane1, const Vector2v& startPoint2, const Vector2v& endPoint2, const Vector4v& plane2)
{
	Vector3v start1=QVarMath::sketchPoint(startPoint1, plane1);
	Vector3v end1=QVarMath::sketchPoint(endPoint1, plane1);
	Vector3v start2=QVarMath::sketchPoint(startPoint2, plane2);
	Vector3v end2=QVarMath::sketchPoint(endPoint2, plane2);
	return QVarMath::distanceBetweenDirections(end1-start1, end2-start2);
}
var QEnergy::coplanarEnergy(const Vector2v& startPoint, const Vector2v& endPoint, const Vector4v& srcPlane, const Vector4v& destPlane)
{
	Vector3v start1=QVarMath::sketchPoint(startPoint, srcPlane);
	Vector3v end1=QVarMath::sketchPoint(endPoint, srcPlane);
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
		case R::FORWARD: return 0*100;
		case R::HORIZONTAL: return 0*100;
		case R::PARALLEL: return isPlaneOnly?2:0;
		case R::PARALLEL_PLANES: return isPlaneOnly?2:0;
		case R::DISTANCE: return isPlaneOnly?2:0*20;
		case R::COPLANAR: return isPlaneOnly?5:0*20;
		case R::VERTICAL: return isPlaneOnly?2:0*100;
		case R::PERPENDICULAR: return isPlaneOnly?0:0*0;
		case R::SAME_POINTS: return isPlaneOnly?4:0*200;
		case R::CONTACT_POINTS: return isPlaneOnly?0*10:0*100;
	}
	return isPlaneOnly?1:0;
	#undef R
}
var QEnergy::totalEnergy(const VectorXv& variables)
{
	if(!isPlaneOnly)return 0;
	#define planeOf(x) variables.segment((x)*4, 4)
	#define pointOf(x) pointAt(variables, x)
	Vector4v ground=planeOf(planeSize-1);
	if(!isPlaneOnly)ground=groundPlane();
	var energy=depthEnergy(variables);
	ground=QVarMath::normalizedPlane(ground);
	isAddingAxis=isPlaneOnly&&axisIndices.size()==0;
	isAddingSamePoints=samePoints.size()==0;
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
			case R::CONTACT_POINTS: energy+=weight(t(0))*distanceEnergy(pointOf(t(1)), planeOf(t(2)), pointOf(t(1)), ground); QD("GCOP"); break;
			case R::PERPENDICULAR: energy+=weight(t(0))*perpendicularEnergy(pointOf(t(1)), pointOf(t(2)), pointOf(t(3)), planeOf(t(4))); QD("PER"); break;
			case R::PARALLEL: energy+=weight(t(0))*parallelEnergy(pointOf(t(1)), pointOf(t(2)), planeOf(t(3)), pointOf(t(4)), pointOf(t(5)), planeOf(t(6))); QD("PAR"); break;
			case R::SAME_POINTS: 
				//if(isAddingSamePoints){this->samePoints<< t(1)<< t(2); if(!isPlaneOnly)this->addPointDistances(variables, t(1), t(2));}
				if(isPlaneOnly)
				{
					energy+=weight(t(0))*stdDevAnglesEnergy(variables, t(1), t(2)-1); QD("SDVA");
					energy+=0.1*stdDevScalesEnergy(variables, t(1), t(2)-1); QD("SDVS");
				}
				else {energy+=weight(t(0))*accuracyEnergy(variables, t(1), t(2)-1); QD("ACC");} break;
		}
		i+=R::count(t(0));
		#undef R
		#undef t(x)
		#undef next()
	}
	if(isAddingSamePoints&&!isPlaneOnly)qDebug()<<"PointDistances="<<pointDistances;
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
	Vector3v s0=QVarMath::sketchPoint(p0, plane);
	Vector3v s1=QVarMath::sketchPoint(p1, plane);
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
	return QVarMath::sketchPoint(Vector2v(point(0), point(1)), plane);
}
Vector3v QEnergy::sketchPoint(const VectorXv& variables, int sketchIndex)
{
	int planeIndex=sketch(sketchIndex*3+2);
	Vector4v plane=variables.segment(planeIndex*4, 4);
	return QVarMath::sketchPoint(pointAt(variables, sketchIndex), plane);
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
		sketchPoints.col(i)=QVarMath::sketchPoint(point, plane);
	}
	return sketchPoints;
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
		for(int i=0; i<planeSize; i++)variables<<0<<0<<1<<0;
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
		vec2 pixel=viewer.sketchPixel(sketch[i+0], sketch[i+1]);
		pixels<<pixel.x()<<pixel.y()<<sketch[i+2];
		variables<<pixel.x()<<pixel.y();
	}
	this->sketch=toVectorXd(pixels);
	this->variables=toVectorXd(variables);
}
QVector<QVector3D> QEnergy::toVector3D(QVector<Vector3v> vector3v)
{
	QVector<QVector3D> vector3D;
	for(Vector3v v : vector3v)
	vector3D<<toQVector3D(v);
	return vector3D;
}
QVector<Vector3v> QEnergy::toVector3v(QVector<QVector3D> vector3D)
{
	QVector<Vector3v> vector3v;
	for(QVector3D v : vector3D)
	vector3v<<toVector3v(v);
	return vector3v;
}
void insert(Vector3v point, int index, QVector<Vector3v>& lowestPoints, veci& indices)
{
	int i=0; while(i<lowestPoints.size()&&point(1)>lowestPoints[i](1))i++;
	lowestPoints.insert(i, point); indices.insert(i, index);
}
void QEnergy::analyze()
{
	veci groundIndices;
	this->groundPoints.clear();
	QVector<Vector3v> lowestPoints;
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
		insert(p0, i0, lowestPoints, groundIndices);
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
	this->groundPoints<<sketchPoint(variables, groundIndices[0]);
	this->groundPoints<<sketchPoint(variables, groundIndices[1]);
	this->groundPoints<<sketchPoint(variables, groundIndices[2]);
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
	for(int i=0; i<planeSize-1; i++)
	{
		planes<<toQVector4D(toVector4v(variables, i*4));
	}
	Vector4v groundPlane=QVarMath::createPlane(groundPoints);
	if(QVarMath::getNormal(groundPlane).dot(Vector3v(0, 1, 0))<0)
	groundPlane=-groundPlane; planes<<toQVector4D(groundPlane); return planes;
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
