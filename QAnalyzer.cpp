#include <QFile>
#include "QSketch.h"
#include "QStanMath.h"
#include <QtAlgorithms>
#define LINE sketch->LINE
#define MOVE sketch->MOVE
#define CUBIC sketch->CUBIC
#define INITIAL sketch->INITIALIZED
#define dot(x, y) vec3::dotProduct(x, y)
#define QDoubleMath QStanMath<double>

QString QAnalyzer::toString(int value)
{
	if(value==AXIS)return "AXIS";
	else if(value==JOINT)return "JON";
	else if(value==LOOP)return "LOOP";
	else if(value==VERTICAL)return "VER";
	else if(value==DISTANCE)return "DIS";
	else if(value==PARALLEL)return "PAR";
	else if(value==FORWARD)return "FOR";
	else if(value==COPLANAR)return "COP";
	else if(value==HORIZONTAL)return "HOR";
	else if(value==SAME_POINTS)return "SAME";
	else if(value==CONTACT_POINTS)return "CON";
	else if(value==PERPENDICULAR)return "PER";
	else if(value==PERPENDICULAR_PLANES)return "PERP";
	else if(value==GROUND_POINTS)return "GRND";
	else if(value==SYMMETRIC)return "SYM";
	else if(value==IDENTICAL)return "ID";
	else return "NULL";
}
int QAnalyzer::toValue(QString string)
{
	if(string=="AXIS")return AXIS;
	else if(string=="JON")return JOINT;
	else if(string=="LOOP")return LOOP;
	else if(string=="VER")return VERTICAL;
	else if(string=="DIS")return DISTANCE;
	else if(string=="PAR")return PARALLEL;
	else if(string=="FOR")return FORWARD;
	else if(string=="COP")return COPLANAR;
	else if(string=="HOR")return HORIZONTAL;
	else if(string=="SAME")return SAME_POINTS;
	else if(string=="CON")return CONTACT_POINTS;
	else if(string=="PER")return PERPENDICULAR;
	else if(string=="PERP")return PERPENDICULAR_PLANES;
	else if(string=="SYM")return SYMMETRIC;
	else if(string=="GRND")return GROUND_POINTS;
	else if(string=="ID")return IDENTICAL;
	else return -1;
}
int QAnalyzer::count(int value)
{
	switch(value)
	{
		case AXIS: return 4;
		case JOINT: return 3;
		case LOOP: return 2;
		case VERTICAL: return 3;
		case DISTANCE: return 4;
		case FORWARD: return 3;
		case PARALLEL: return 6;
		case COPLANAR: return 4;
		case HORIZONTAL: return 3;
		case SAME_POINTS: return 2;
		case PERPENDICULAR: return 4;
		case CONTACT_POINTS: return 2;
		case GROUND_POINTS: return 3;
		case PERPENDICULAR_PLANES: return 2;
	}
	return 0;
}
int QAnalyzer::toAxis(QString axis)
{
	return axis=="x"?0:(axis=="y"?1:2);
}
void QAnalyzer::initAxis()
{
	for(int i=0; i<3; i++)this->axis<<vec3(0, 0, 0);
}
void QAnalyzer::operator<<(QStringList& list)
{
	if(list[0]=="PLANES")this->planesSize=list[1].toInt();
	else if(list[0]=="p")
	{
		vec4 plane
		(
			list[1].toDouble(), list[2].toDouble(),
			list[3].toDouble(), list[4].toDouble()
		);
		this->planes<<plane;
	}
	else if(list[0]=="k")
	{
		int x=list[1].toInt();
		int y=list[2].toInt();
		int p=list[3].toInt();
		this->sketchVector<<x<<y<<p;
	}
	else if(list[0]=="x"||list[0]=="y"||list[0]=="z")
	{
		if(!axis.size())
		this->initAxis();
		vec3 vector
		(
			list[1].toDouble(), 
			list[2].toDouble(), 
			list[3].toDouble()
		);
		this->axis[toAxis(list[0])]=vector;
	}
	else 
	{
		int value=toValue(list[0]);
		if(value<0)return;
		int n=count(value);
		this->regularity<<value;
		for(int t=1; t<=n; t++)
		this->regularity<<list[t].toInt();
		if(value==JOINT)
		for(int t=1; t<=n; t++)
		this->joints<<list[t].toInt();
		else if(value==SAME_POINTS)
		for(int t=1; t<=n; t++)
		this->samePoints<<list[t].toInt();
	}
}
void QAnalyzer::load(QSketch* sketch)
{
	if(!sketch->is(INITIAL))return;
	this->sketch=new QSketch();
	this->sketch->path=sketch->path;
	this->sketch->point2D=sketch->point2D;
	this->sketch->point3D=sketch->point3D;
	this->sketch->viewer.resize(sketch->viewer.size());
}
void QAnalyzer::initializeSketchCurves()
{
#define path sketch->path
#define viewer sketch->viewer
#define point2D sketch->point2D
	QVector<veci> sketchPaths1, sketchPaths2;
	QVector<veci> sketchCurves1, sketchCurves2;
	for(int i=0, j=0; i<path.size(); i++)
	{
		if(path[i]==MOVE)
		{
			veci sketchPath;
			veci sketchCurve;
			sketchPath<<MOVE;
			sketchCurve<<point2D[j++];
			sketchCurve<<point2D[j++];
			for(i++; i<path.size()&&path[i]!=MOVE; i++)
			{
				if(path[i]==LINE)
				{
					sketchPath<<LINE;
					sketchCurve<<point2D[j++];
					sketchCurve<<point2D[j++];
				}
				else
				{
					sketchPath<<CUBIC;
					sketchCurve<<point2D[j++];
					sketchCurve<<point2D[j++];
					sketchPath<<CUBIC;
					sketchCurve<<point2D[j++];
					sketchCurve<<point2D[j++];
					sketchPath<<CUBIC;
					sketchCurve<<point2D[j++];
					sketchCurve<<point2D[j++];
				}
			}
			if(sketchCurve.size()>0)
			{
				if(sketchPath.size()>2)
				{
					sketchPaths1<<sketchPath;
					sketchCurves1<<sketchCurve;
				}
				else
				{
					sketchPaths2<<sketchPath;
					sketchCurves2<<sketchCurve;
				}

			}
			if(i<path.size())i--;
		}
	}
	this->sketchPaths=sketchPaths1+sketchPaths2;
	this->sketchCurves=sketchCurves1+sketchCurves2;
	for(int i=0; i<sketchCurves.size(); i++)
	{
		if(sketchCurves[i].size()==4)
		this->sketchTypes<<LINE_SEGMENT;
		else
		{
			if(equals(firstPoint(i), lastPoint(i)))
			{
				this->sketchTypes<<CLOSE_CURVE;
			}
			else this->sketchTypes<<OPEN_CURVE;
		}
	}
	for(int i=0; i<sketchTypes.size(); i++)
	{
		int planeIndex;
		this->sketchPlanes<<i;
		this->isJoint<<isJointType(i, planeIndex);
		if(isJoint[i])this->sketchPlanes[i]=planeIndex;
		if(isJoint[i]&&hasJointMarker)this->addJointMarker(i);
	}
	this->planesSize=max(sketchPlanes)+1;
	#undef path
	#undef viewer
	#undef point2D
}
int QAnalyzer::max(veci array)
{
	int result=array[0];
	for(int e : array)
	result=qMax(e, result);
	return result;
}
void QAnalyzer::addJointMarker(int jointIndex)
{
	vec2 p=getPoint(jointIndex, 0), q=getPoint(jointIndex, 1);
	vec2 d=p-q, r=vec2(d.y(), -d.x()).normalized()*5, p0, p1;
	p0=p+r; p1=p-r; this->markerLines<<p0.x()<<p0.y()<<p1.x()<<p1.y();
	p0=q+r; p1=q-r; this->markerLines<<p0.x()<<p0.y()<<p1.x()<<p1.y();
}
void QAnalyzer::copySketchCurves()
{
	this->sketch->path.clear();
	this->sketch->point3D.clear();
	this->sketchCurveSizes.clear();
	for(int i=0, size=0; i<sketchPaths.size(); i++)
	{
		veci path=sketchPaths[i];
		veci point2D=sketchCurves[i];
		for(int j=0, k=0; j<path.size(); j++)
		{
			this->sketch->path<<path[j];
			if(path[j]==MOVE||path[j]==LINE)
			{
				int x=point2D[k++], y=point2D[k++];
				this->sketch->point3D<<x<<y<<sketchPlanes[i];
			}
			else
			{
				int c1=point2D[k++], c2=point2D[k++]; j++;
				int c3=point2D[k++], c4=point2D[k++]; j++;
				int c5=point2D[k++], c6=point2D[k++];
				this->sketch->point3D<<c1<<c2<<sketchPlanes[i];
				this->sketch->point3D<<c3<<c4<<sketchPlanes[i];
				this->sketch->point3D<<c5<<c6<<sketchPlanes[i];
			}
		}
		this->sketchCurveSizes<<size; size+=path.size();
	}
}
void QAnalyzer::simplifySketchCurves()
{
	QVector<veci> paths;
	QVector<veci> curves;
	for(int i=0; i<sketchPaths.size(); i++)
	{
		if(sketchPaths[i].size()<=4)
		{
			paths<<sketchPaths[i];
			curves<<sketchCurves[i];
			continue;
		}
		paths<<veci(); curves<<veci();
		for(int j=0; j<sketchPaths[i].size(); j++)
		{
			if(sketchPaths[i][j]==MOVE||sketchPaths[i][j]==LINE)
			{
				paths[i]<<sketchPaths[i][j];
				curves[i]<<sketchCurves[i][j*2+0];
				curves[i]<<sketchCurves[i][j*2+1];
			}
			else
			{
				for
				(
				 j+=3; j<sketchPaths[i].size()
				 &&sketchPaths[i][j]==CUBIC; j+=3
				);
				j--;
				paths[i]<<LINE;
				curves[i]<<sketchCurves[i][j*2+0];
				curves[i]<<sketchCurves[i][j*2+1];
			}
		}
	}
	this->sketchPaths=paths;
	this->sketchCurves=curves;
}
void QAnalyzer::finalizeSketchCurves()
{
	for(int i=0; i<sketchTypes.size(); i++)
	{
		if(hasJointMarker&&isJoint[i])
		{
			vec2 p0=getPoint(i, 0), p1=getPoint(i, 1);
			this->markerPoints<<p0.x()<<p0.y()<<p1.x()<<p1.y();
		}
		if(hasCurveIndexMarker)
		{
			vec2 p0=getPoint(i, 0);  this->startPoints<<p0.x()<<p0.y();
		}
	}
}
void QAnalyzer::getSketchVector()
{
	this->sketchSizes.clear();
	this->sketchVector.clear();
	for(int i=0, size=0; i<sketchPaths.size(); i++)
	{
		veci path=sketchPaths[i];
		veci point2D=sketchCurves[i];
		for(int j=0, k=0; j<path.size(); j++)
		{
			if(path[j]==MOVE||path[j]==LINE)
			{
				int x=point2D[k++], y=point2D[k++];
				this->sketchVector<<x<<y<<sketchPlanes[i];
			}
			else
			{
				int c1=point2D[k++], c2=point2D[k++]; j++;
				int c3=point2D[k++], c4=point2D[k++]; j++;
				int c5=point2D[k++], c6=point2D[k++];
				this->sketchVector<<c1<<c2<<sketchPlanes[i];
				this->sketchVector<<c3<<c4<<sketchPlanes[i];
				this->sketchVector<<c5<<c6<<sketchPlanes[i];
			}
		}
		this->sketchSizes<<size; size+=path.size();
	}

}
void QAnalyzer::encloseSketchCurves(bool isLoop)
{
	for(int i=0; i<sketchTypes.size(); i++)
	{
		if(sketchTypes[i]==ENCLOSED_CURVE)
		this->sketchTypes[i]=CLOSE_CURVE;
		if(sketchTypes[i]==CLOSE_CURVE)
		{

			this->isSketchCurve=!isLoop;
			int first=indexOf(i, 0), size=sketchPaths[i].size();
			this->regularity<<(isLoop?LOOP:SAME_POINTS)<<first<<first+size-1;
		}
	}
}
void QAnalyzer::run()
{
	this->initializeSketchCurves();
	this->initializeJointGraph();
	do
	{
		for(int i=0; i<sketchCurves.size(); i++)
		if(sketchTypes[i]==OPEN_CURVE)
		this->completeOpenCurve(i);
		for(int i=0; i<sketchCurves.size(); i++)
		if(sketchTypes[i]==CLOSE_CURVE)
		this->addJointForCloseCurve(i, false);
		else if(sketchTypes[i]==ENCLOSED_CURVE)
		this->addJointForCloseCurve(i, true);
	}
	while(updateJoints());
	this->planesSize=max(sketchPlanes)+1;
	this->copySketchCurves();
	for(int i=0; i<sketchCurves.size(); i++)
	{
		this->getCurvatures(i);
		this->getSketchLengths(i);
	}
	this->encloseSketchCurves(false);
	this->simplifySketchCurves();
	this->getSketchVector();
	this->encloseSketchCurves(true);
	this->finalizeSketchCurves();
	for(int i=0; i<sketchCurves.size(); i++)
	this->getCurvesRegularity(i);
	for(int i=0; i<sketchCurves.size()-1; i++)
	for(int j=i+1; j<sketchCurves.size(); j++)
	this->getCurvesRegularity(i, j);
	this->getPlanesRegularity();
	this->updateRegularity();
}
void QAnalyzer::load(veci path, veci point3D, veci sketchVector, veci regularity, QVector<vec4> planes, QVector<vec3> axis, QViewer viewer)
{
	QSketch* sketch=new QSketch(path, point3D, viewer);
	this->load(sketch);
	this->axis=axis;
	this->planes=planes;
	this->regularity=regularity;
	this->sketchVector=sketchVector;
}
#define getVector3d(v) Vector3d(v(0), v(1), 0)
#define toVector2d(x) QDoubleMath::toVector2t(x)
#define toVector2D(x) QDoubleMath::toVector2D(x)
void QAnalyzer::getCurvatures(int curveIndex)
{
	int i=curveIndex, m=10; qreal l=2000;
	for(int j=1; j<sketchPaths[i].size(); j++)
	{
		if(sketchPaths[i][j]==CUBIC)
		{
			Vector2d p0=toVector2d(getPoint(i, j-1)), p1=toVector2d(getPoint(i, j-0));
			Vector2d p2=toVector2d(getPoint(i, j+1)), p3=toVector2d(getPoint(i, j+2));			
			for(qreal t=0; t<m; t++)
			{
				Vector2d p=QDoubleMath::cubicPointAt(p0, p1, p2, p3, t*1.0/m);
				Vector2d n=QDoubleMath::cubicNormalAt(p0, p1, p2, p3, t*1.0/m);
				qreal k=QDoubleMath::cubicCurvatureAt(p0, p1, p2, p3, t*1.0/m);
				Vector2d q=p+n*l*k; this->markerLines<<p(0)<<p(1)<<q(0)<<q(1);
			}
			Vector3d q0=getVector3d(p0), q1=getVector3d(p1);
			Vector3d q2=getVector3d(p2), q3=getVector3d(p3);
			qreal t=QDoubleMath::argmaxCurvatureOfCubic(q0, q1, q2, q3);
			Vector2d p=QDoubleMath::cubicPointAt(p0, p1, p2, p3, t);
			this->markerPoints<<p(0)<<p(1);
			j+=2;
		}
	}
}
#undef toVector2D(x)
#undef toVector2d(x)
#undef getVector3d(v)
void QAnalyzer::getSketchLengths(int curveIndex)
{
	vec lengths; lengths<<0;
	int i=curveIndex; qreal length=0;
	for(int j=1; j<sketchPaths[i].size(); j++)
	{
		vec2 p0=getPoint(i, j-1), p1=getPoint(i, j-0);
		length+=p0.distanceToPoint(p1); lengths<<length;
	}
	for(qreal& len : lengths)len/=length;
	vec sketchLengths; sketchLengths<<0;
	for(int i=1; i<lengths.size(); i++)
	sketchLengths<<lengths[i]-lengths[i-1];
	this->sketchLengths<<sketchLengths;
}

bool QAnalyzer::updateJoints()
{
	bool isUpdated=false;
	for(int i=0; i<sketchTypes.size(); i++)
	{
		if(isJoint[i])continue; int planeIndex;
		this->isJoint[i]=isJointType(i, planeIndex);
		if(isJoint[i])
		{
			this->sketchPlanes[i]=planeIndex;
			isUpdated=true;
		}
		if(isJoint[i]&&hasJointMarker)
		this->addJointMarker(i);
	}
	return isUpdated;
}
void QAnalyzer::initializeJointGraph()
{
	int size=this->sketchPlanes.size();
	this->jointGraph.resize(size);
	for(int i=0; i<size; i++)
	this->jointGraph[i]=vecb(size, false);
}
void QAnalyzer::getPlanesRegularity()
{
	for(int i=0; i<jointGraph.size(); i++)
	{
		for(int j=i+1; j<jointGraph[i].size(); j++)
		{
			if(jointGraph[i][j])
			{
				this->regularity<<PERPENDICULAR_PLANES<<i<<j;
			}
		}
	}
}
void QAnalyzer::updateRegularity()
{
	veci regularity; isSketchCurve=false;
	for(int i=0; i<3&&i<parallelLines.size(); i++)
	{
		for(int j=0; j<parallelLines[i].size(); j++)
		{
			vec4 line=parallelLines[i][j];
			int c=line.x(), k1=line.y(), k2=line.z(), planeIndex=line.w();
			regularity<<AXIS<<i<<indexOf(c, k1)<<indexOf(c, k2)<<planeIndex;
		}
	}
	for(int i=0; i<parallelLines.size(); i++)
	{
		for(int j=0, k=1; j<parallelLines[i].size()-1; j++, k++)
		{
			vec4 line1=parallelLines[i][j], line2=parallelLines[i][k];
			int c1=line1.x(), k1=line1.y(), k2=line1.z(), planeIndex1=line1.w();
			int c2=line2.x(), k3=line2.y(), k4=line2.z(), planeIndex2=line2.w(); 
			regularity<<PARALLEL<<indexOf(c1, k1)<<indexOf(c1, k2)<<planeIndex1;
			regularity<<indexOf(c2, k3)<<indexOf(c2, k4)<<planeIndex2;
		}
	}
	#define R this->regularity
	for(int i=0; i<R.size(); i++)
	{
		isSketchCurve=false;
		switch(R[i])
		{
			case VERTICAL: 
			{
				int c=R[++i], k1=R[++i], k2=R[++i], planeIndex=R[++i]; 
				regularity<<VERTICAL<<indexOf(c, k1)<<indexOf(c, k2)<<planeIndex; break;
			}
			case PERPENDICULAR: 
			{
				int c=R[++i], k1=R[++i], k2=R[++i], k3=R[++i], planeIndex=R[++i];
				regularity<<PERPENDICULAR<<indexOf(c, k1)<<indexOf(c, k2)<<indexOf(c, k3)<<planeIndex; break;
			}
			case PERPENDICULAR_PLANES:
			{
				int c1=R[++i], c2=R[++i];
				regularity<<PERPENDICULAR_PLANES;
				regularity<<c1<<c2; break;
			}
			case SAME_POINTS: 
			{
				int c1=R[++i], c2=R[++i];
				regularity<<SAME_POINTS;
				regularity<<c1<<c2; break;
			}
		case LOOP:
		{
			int c1=R[++i], c2=R[++i];
			regularity<<LOOP;
			regularity<<c1<<c2; break;
		}
			case DISTANCE:
			{
			isSketchCurve=true;
				int c1=R[++i], k1=R[++i], planeIndex1=R[++i], c2=R[++i], k2=R[++i], planeIndex2=R[++i];
				regularity<<DISTANCE<<indexOf(c1, k1)<<planeIndex1<<indexOf(c2, k2)<<planeIndex2; break;
			}
			case COPLANAR:
			{
			isSketchCurve=true;
				int c1=R[++i], k1=R[++i], k2=R[++i], planeIndex1=R[++i], planeIndex2=R[++i];
				regularity<<COPLANAR<<indexOf(c1, k1)<<indexOf(c1, k2)<<planeIndex1<<planeIndex2; break;
			}
		case JOINT:
		{
			isSketchCurve=true;
			int c=R[++i], k1=R[++i], k2=R[++i], planeIndex=R[++i];
			regularity<<JOINT<<indexOf(c, k1)<<indexOf(c, k2)<<planeIndex; break;
		}
		}
	}
	#undef R
	this->regularity.clear();
	this->regularity=regularity;
}
bool QAnalyzer::isShort(int curveIndex, int pointIndex)
{
	return sketchLengths[curveIndex][pointIndex]<minLength;
}
void QAnalyzer::getCurvesRegularity(int curveIndex)
{
	#define I(x) getIndex(curveIndex, x)
	veci path=sketchPaths[curveIndex];
	int planeIndex=sketchPlanes[curveIndex];
	for(int i=1; i<path.size(); i++)
	{
		if(path[i]==CUBIC){i+=2; continue;}
		if(isShort(curveIndex, i))continue;
		vec2 p0=getPoint(curveIndex, i-1);
		vec2 p1=getPoint(curveIndex, I(i));
		vec2 tangent=(p1-p0).normalized();
		if(isParallel(tangent, yAxis))
		{
			this->regularity<<VERTICAL<<curveIndex;
			this->regularity<<i-1<<I(i)<<planeIndex;
		}
	}
}
void QAnalyzer::getCurvesRegularity(int curveIndex1, int curveIndex2)
{
	#define I(x) getIndex(curveIndex1, x)
	#define J(x) getIndex(curveIndex2, x)
	veci path1=sketchPaths[curveIndex1];
	veci path2=sketchPaths[curveIndex2];
	int planeIndex1=sketchPlanes[curveIndex1];
	int planeIndex2=sketchPlanes[curveIndex2];
	for(int i=1; i<path1.size(); i++)
	{
		if(path1[i]==CUBIC){i+=2; continue;}
		if(isShort(curveIndex1, i))continue;
		vec2 p0=getPoint(curveIndex1, i-1);
		vec2 p1=getPoint(curveIndex1, I(i));
		vec2 tangent1=(p1-p0).normalized();
		for(int j=1; j<path2.size(); j++)
		{
			if(path2[j]==CUBIC){j+=2; continue;}
			if(isShort(curveIndex2, j))continue;
			vec2 q0=getPoint(curveIndex2, j-1);
			vec2 q1=getPoint(curveIndex2, J(j));
			vec2 tangent2=(q1-q0).normalized();
			if(isParallel(tangent1, tangent2))
			{
				vec4 line1=vec4(curveIndex1, i-1, I(i), planeIndex1);
				vec4 line2=vec4(curveIndex2, j-1, J(j), planeIndex2);
				this->addParallelLines(line1, line2);
			}
		}
	}
	#undef I(x)
	#undef J(x)
}
void QAnalyzer::getClosePoints(int curveIndex1, int curveIndex2)
{
	if(curveIndex1==curveIndex2)return;
	if(sketchTypes[curveIndex1]!=CLOSE_CURVE)return;
	if(sketchTypes[curveIndex2]!=CLOSE_CURVE)return;
	veci path1=sketchPaths[curveIndex1];
	veci path2=sketchPaths[curveIndex2];
	int planeIndex1=sketchPlanes[curveIndex1];
	int planeIndex2=sketchPlanes[curveIndex2];
	for(int i=0; i<path1.size()-1; i++)
	{
		if(path1[i]==CUBIC){i+=2; continue;}
		vec2 p1=getPoint(curveIndex1, i);
		for(int j=0; j<path2.size()-1; j++)
		{
			if(path2[j]==CUBIC){j+=2; continue;}
			vec2 p2=getPoint(curveIndex2, j);
			if((p1-p2).length()<minDistance)
			{
				this->regularity<<DISTANCE;
				this->regularity<<curveIndex1<<i<<planeIndex1;
				this->regularity<<curveIndex2<<j<<planeIndex2;
			}
		}
	}
}
int QAnalyzer::indexOf(int curveIndex, int pointIndex)
{
	return (isSketchCurve?sketchCurveSizes[curveIndex]:sketchSizes[curveIndex])+pointIndex;
}
vec2 QAnalyzer::getPoint(int index)
{
	int x=sketchVector[index*3+0];
	int y=sketchVector[index*3+1];
	return vec2(x, y);
}
void QAnalyzer::setPoint(int curveIndex, int pointIndex, vec2 point)
{
	this->sketchCurves[curveIndex][pointIndex*2+0]=point.x();
	this->sketchCurves[curveIndex][pointIndex*2+1]=point.y();
}
vec2 QAnalyzer::getPoint(int curveIndex, int pointIndex)
{
	int x=sketchCurves[curveIndex][pointIndex*2+0];
	int y=sketchCurves[curveIndex][pointIndex*2+1];
	return vec2(x, y);
}
int QAnalyzer::getIndex(int curveIndex, int pointIndex)
{
	if(sketchTypes[curveIndex]==CLOSE_CURVE)
	{
		if(pointIndex==-1)return sketchPaths[curveIndex].size()-2;
		else if(pointIndex==sketchPaths[curveIndex].size()-1)return 0;
		else return pointIndex;
	}
	else return pointIndex;
}
bool QAnalyzer::isParallel(const vec2& x, const vec2& y)
{
	return abs(1-abs(dot(x.normalized(), y.normalized())))<error;
}
void QAnalyzer::addParallelLines(vec4 line1, vec4 line2)
{
	#define has(x, y) x.indexOf(y)>=0
	for(int i=0; i<parallelLines.size(); i++)
	{
		if(has(parallelLines[i], line1)&&has(parallelLines[i], line2))return;
		else if(has(parallelLines[i], line1))
		{
			if(!isParallel(i, line2))return;
			for(int j=i+1; j<parallelLines.size(); j++)
			{
				if(has(parallelLines[j], line2))
				{
					parallelLines[i]+=parallelLines[j];
					parallelLines.removeAt(j);
					avgLineDirections.removeAt(j);
					updateAvgLineDirections(i); return;
				}
			}
			parallelLines[i]<<line2;
			updateAvgLineDirections(i); return;
		}
		else if(has(parallelLines[i], line2))
		{
			if(!isParallel(i, line1))return;
			for(int j=i+1; j<parallelLines.size(); j++)
			{
				if(has(parallelLines[j], line1))
				{
					parallelLines[i]+=parallelLines[j];
					parallelLines.removeAt(j);
					avgLineDirections.removeAt(j);
					updateAvgLineDirections(i); return;
				}
			}
			parallelLines[i]<<line1;
			updateAvgLineDirections(i); return;
		}
	}
	QVector<vec4> lines;
	lines<<line1<<line2;
	parallelLines<<lines;
	vec2 dir1=getLineDirection(line1);
	vec2 dir2=getLineDirection(line2);
	avgLineDirections<<addLineDirection(dir1, dir2).normalized();
}
void QAnalyzer::updateAvgLineDirections(int index)
{
	vec2 sum=vec2(0, 0);
	if(parallelLines[index].size()==0)return;
	for(vec4 line : parallelLines[index])
	{
		vec2 dir=getLineDirection(line);
		sum=addLineDirection(sum, dir);
	}
	avgLineDirections[index]=sum.normalized();
}
vec2 QAnalyzer::getLineDirection(vec4 line)
{
	int curve=line[0], point1=line[1], point2=line[2];
	vec2 direction=getPoint(curve, point1);
	direction-=getPoint(curve, point2);
	return direction.normalized();
}
vec2 QAnalyzer::addLineDirection(vec2 dir1, vec2 dir2)
{
	if(dir1.length()==0)return dir2;
	return dot(dir1.normalized(), dir2)>0?dir1+dir2:dir1-dir2;
}
bool QAnalyzer::isParallel(int lines, vec4 line)
{
	vec2 dir1=avgLineDirections[lines];
	vec2 dir2=getLineDirection(line);
	return isParallel(dir1, dir2);
}
bool QAnalyzer::equals(const veci& curve, int index1, int index2)
{
	int x1=curve[index1+0], y1=curve[index1+1];
	int x2=curve[index2+0], y2=curve[index2+1];
	return x1==x2&&y1==y2;
}
bool QAnalyzer::isJointType(int curveIndex, int& planeIndex)
{
	int k=curveIndex;
	if(sketchTypes[k]!=LINE_SEGMENT)return false;
	vec2 p1=getPoint(k, 0), p2=getPoint(k, 1);
	for(int i=0; i<sketchTypes.size(); i++)
	{
		vec positions;
		if(sketchTypes[i]==CLOSE_CURVE||sketchTypes[i]==ENCLOSED_CURVE)
		{
			int size=sketchCurves[i].size()/2;
			for(int j=0; j<size-1; j++)
			{
				vec2 p3=getPoint(i, j+0), p4=getPoint(i, j+1);
				if(isParallel(p1-p2, p3-p4))continue;
				vec2 p=intersect(p1, p2, p3, p4);
				qreal t=devide(p-p4, p3-p4);
				if(t>=0&&t<=1)positions<<devide(p-p1, p2-p1);
			}
		}
		if(isJointType(positions)){planeIndex=i; return true;}
	}
	return false;
}
vec2 QAnalyzer::intersect(vec2 p1, vec2 p2, vec2 p3, vec2 p4)
{
	qreal x=cross2(p1, p2)*(p3.x()-p4.x())-(p1.x()-p2.x())*cross2(p3, p4);
	qreal y=cross2(p1, p2)*(p3.y()-p4.y())-(p1.y()-p2.y())*cross2(p3, p4);
	qreal z=(p1.x()-p2.x())*(p3.y()-p4.y())-(p1.y()-p2.y())*(p3.x()-p4.x());
	return vec2(x/z, y/z);
}
bool QAnalyzer::isJointType(const vec& positions)
{
	if(positions.size()<2)return false;
	bool isLeftJoint=false, isRightJoint=false;
	for(qreal position : positions)
	{
		if(0<position&&position<1)return false;
		isLeftJoint=isLeftJoint||(position<=0);
		isRightJoint=isRightJoint||(position>=1);
	}
	return isLeftJoint&&isRightJoint;
}
bool QAnalyzer::intersectWithEdge(qreal& distance, vec2& leftPoint, vec2& rightPoint, QVector<vec2> points, bool isJoint)
{
	vec2 p1=points[0], p2=points[1], p3=points[2];
	vec2 p4=points[3], p5=points[4], p6=points[5];
	vec2 p=intersect(p1, p2, p5, p6);
	vec2 q=intersect(p3, p4, p5, p6);
	qreal l=devide(p-p5, p6-p5);
	qreal r=devide(q-p5, p6-p5);
	if((l>1&&r>1)||(l<0&&r<0))return false;
	qreal du=abs(l-0), dv=abs(r-1);
	if(abs(r-0)<du){du=abs(r-0); dv=abs(l-1);}
	du*=(p6-p5).length(); dv*=(p6-p5).length();
	distance=(du+dv)/(isJoint?3:2);
	leftPoint=p; rightPoint=q; return true;
}
void QAnalyzer::setFirstPoint(int curveIndex, vec2 point)
{
	this->sketchCurves[curveIndex][0]=point.x();
	this->sketchCurves[curveIndex][1]=point.y();
}
void QAnalyzer::setLastPoint(int curveIndex, vec2 point)
{
	int size=this->sketchCurves[curveIndex].size();
	this->sketchCurves[curveIndex][size-2]=point.x();
	this->sketchCurves[curveIndex][size-1]=point.y();
}
void QAnalyzer::addJointRegularity(int curveIndex, int jointIndex, int startIndex)
{
	int k=curveIndex, size=sketchCurves[k].size()/2, s=startIndex;
	this->regularity<<COPLANAR<<k<<0<<size-1;
	this->regularity<<sketchPlanes[k]<<sketchPlanes[jointIndex];
	this->regularity<<COPLANAR<<jointIndex<<s+0<<s+1;
	this->regularity<<sketchPlanes[jointIndex]<<sketchPlanes[k];
	this->regularity<<DISTANCE<<k<<0<<sketchPlanes[k];
	this->regularity<<jointIndex<<s+0<<sketchPlanes[jointIndex];
	this->regularity<<DISTANCE<<k<<size-1<<sketchPlanes[k];
	this->regularity<<jointIndex<<s+1<<sketchPlanes[jointIndex];
	if(size>2)
	{
		this->regularity<<JOINT<<k<<0<<1<<sketchPlanes[jointIndex];
		this->regularity<<JOINT<<k<<size-1<<size-2<<sketchPlanes[jointIndex];
	}
	this->jointGraph[sketchPlanes[curveIndex]][sketchPlanes[jointIndex]]=true;
	this->jointGraph[sketchPlanes[jointIndex]][sketchPlanes[curveIndex]]=true;
}
void QAnalyzer::completeOpenCurve(int curveIndex)
{
	int k=curveIndex;
	vec2 leftPoint, rightPoint, l, r;
	int size=sketchCurves[k].size()/2;
	qreal distance, d; vec2 jointIndex(-1, -1);
	vec2 p1=getPoint(k, 0), p2=getPoint(k, 1);
	vec2 p3=getPoint(k, size-2), p4=getPoint(k, size-1);
	QVector<vec2> p; vec2 O(0, 0); p<<p1<<p2<<p3<<p4<<O<<O;
	for(int i=0; i<sketchTypes.size(); i++)
	{
		if(isJoint[i]||sketchTypes[i]==CLOSE_CURVE)
		{
			for(int j=0, s=sketchCurves[i].size()/2; j<s-1; j++)
			{
					p[4]=getPoint(i, j+0); p[5]=getPoint(i, j+1);
					if(intersectWithEdge(d, l, r, p, isJoint[i]))
					if(jointIndex.x()<0||d<distance)
					{
						distance=d; jointIndex=vec2(i, j);
						leftPoint=l; rightPoint=r;
					}
			}
		}
	}
	if(jointIndex.x()>=0)
	{
		vec2 p0=leftPoint, p5=rightPoint;
		int i=jointIndex.x(), j=jointIndex.y();
		this->setFirstPoint(k, p0);
		this->setLastPoint(k, p5);
		if(isJoint[i])
		{
			this->setFirstPoint(i, p0);
			this->setLastPoint(i, p5);
			this->addJointRegularity(k, i, j);
		}
		else
		{
			this->addJoint(i, p0, p5);
			this->addJointRegularity(k, i, 0);
		}
		this->sketchPaths[k]<<LINE;
		this->sketchCurves[k]<<p0.x()<<p0.y();
		this->sketchTypes[k]=ENCLOSED_CURVE;
	}
}
qreal QAnalyzer::devide(vec2 vector1, vec2 vector2)
{
	return vector2.y()==0?(vector1/vector2).x():(vector1/vector2).y();
}
qreal QAnalyzer::intersectWithCloseCurve(vec2& leftPoint, vec2& rightPoint, int curveIndex, vec2 begin, vec2 end)
{
	int i=curveIndex, j, size=sketchCurves[i].size()/2;
	qreal leftMinDistance, rightMinDistance;
	for(leftPoint=vec2(-1, -1), rightPoint=vec2(-1, -1), j=0; j<size-1; j++)
	{
		vec2 point0=getPoint(i, j+0), point1=getPoint(i, j+1);
		vec2 point=intersect(point0, point1, begin, end);
		qreal t=devide(point-point1, point0-point1); if(t<0||t>1)continue;
		qreal leftDist=(point-begin).length(), rightDist=(point-end).length();
		if(leftDist<rightDist)
		{
			if(leftPoint.x()<0||leftDist<leftMinDistance)
			{
				leftMinDistance=leftDist; leftPoint=point;
			}
		}
		else
		{
			if(rightPoint.x()<0||rightDist<rightMinDistance)
			{
				rightMinDistance=rightDist; rightPoint=point;
			}
		}
	}
	if(leftPoint.x()<0||rightPoint.x()<0)return -1;
	return (leftMinDistance+rightMinDistance)/2;
}

bool QAnalyzer::isOnOutsideOrEqual(vec2 left, vec2 right, vec2 begin, vec2 end)
{
	bool isLeftOnOutside=(devide(left-end, begin-end)>1);
	bool isRightOnOutside=(devide(right-begin, end-begin)>1);
	bool isLeftEqual=equals(left, begin), isRightEqual=equals(right, end);
	return (isLeftOnOutside||isLeftEqual)&&(isRightOnOutside||isRightEqual);
}
void QAnalyzer::addJointForCloseCurve(int curveIndex, bool isEnclosedCurve)
{
	vec minDist(sketchCurves.size());
	veci pointIndex(sketchCurves.size());
	veci jointIndex(sketchCurves.size(), -1);
	int i=curveIndex; bool b=isEnclosedCurve;
	QVector<vec2> leftPoint(sketchCurves.size());
	QVector<vec2> rightPoint(sketchCurves.size());
	for(int j=0; j<sketchCurves[i].size()/2-(b?2:1); j++)
	{
		vec2 p0=getPoint(i, j+0), p1=getPoint(i, j+1), l, r;
		for(int k=0; k<sketchCurves.size(); k++)
		{
			if(jointGraph[sketchPlanes[i]][sketchPlanes[k]])continue;
			if(sketchTypes[k]==LINE_SEGMENT)continue;
			if(k==i||sketchTypes[k]==OPEN_CURVE)continue;
			qreal d=intersectWithCloseCurve(l, r, k, p0, p1);
			if(d>=0&&isOnOutsideOrEqual(l, r, p0, p1))
			if(jointIndex[k]<0||d<minDist[k])
			{
				jointIndex[k]=k; pointIndex[k]=j;
				leftPoint[k]=l; rightPoint[k]=r; minDist[k]=d;
			}
		}
	}
	for(int k=0; k<jointIndex.size(); k++)
	if(jointIndex[k]>=0)
	{
		int j=pointIndex[k];
		vec2 p0=getPoint(i, j+0);
		vec2 p1=getPoint(i, j+1);
		this->addJoint(jointIndex[k], p0, p1);
		this->addJointRegularity(jointIndex[k], i, j);
		if(hasJointMarker)
		{
			vec2 p=leftPoint[k], q=rightPoint[k];
			this->markerLines<<p.x()<<p.y()<<q.x()<<q.y();
			this->markerPoints<<p.x()<<p.y()<<q.x()<<q.y();
		}
	}
}
void QAnalyzer::addJoint(int& jointIndex, vec2 from, vec2 to)
{
	veci jointPath; jointPath<<MOVE<<LINE; veci jointLine;
	jointLine<<from.x()<<from.y()<<to.x()<<to.y();
	this->sketchPaths<<jointPath;
	this->sketchCurves<<jointLine;
	this->sketchTypes<<LINE_SEGMENT;
	this->sketchPlanes<<sketchPlanes[jointIndex];
	this->isJoint<<true; jointIndex=isJoint.size()-1;
	if(hasJointMarker)this->addJointMarker(jointIndex);
}
bool QAnalyzer::equals(vec2 p1, vec2 p2)
{
	return (p1-p2).length()<minDistance;
}
qreal QAnalyzer::cross2(vec2 p1, vec2 p2)
{
	return p1.x()*p2.y()-p1.y()*p2.x();
}
vec2 QAnalyzer::firstPoint(int curveIndex)
{
	int x=sketchCurves[curveIndex][0];
	int y=sketchCurves[curveIndex][1];
	return vec2(x, y);
}
vec2 QAnalyzer::lastPoint(int curveIndex)
{
	int size=sketchCurves[curveIndex].size();
	int x=sketchCurves[curveIndex][size-2];
	int y=sketchCurves[curveIndex][size-1];
	return vec2(x, y);
}
void QAnalyzer::save(QString fileName)
{
	QFile file(fileName); QString endl="\r\n";
	if(!file.open(QIODevice::WriteOnly))return;
	QTextStream textStream(&file); 
	textStream<<"s "<<sketch->viewer.width;
	textStream<<" "<<sketch->viewer.height<<endl;
	#define path sketch->path
	#define planes sketchPlanes
	#define point3D sketch->point3D
	for(int i=0, k=0; i<path.size(); i++)
	{
		if(path[i]==MOVE)
		{
			int x=point3D[k++], y=point3D[k++], z=point3D[k++];
			textStream<<"m "<<x<<" "<<y<<" "<<z<<endl;
		}
		else if(path[i]==LINE)
		{
			int x=point3D[k++], y=point3D[k++], z=point3D[k++];
			textStream<<"l "<<x<<" "<<y<<" "<<z<<endl;
		}
		else
		{
			int c1=point3D[k++], c2=point3D[k++], c3=point3D[k++];
			int c4=point3D[k++], c5=point3D[k++], c6=point3D[k++];
			int c7=point3D[k++], c8=point3D[k++], c9=point3D[k++];
			textStream<<"c "<<c1<<" "<<c2<<" "<<c3;
			textStream<<" "<<c4<<" "<<c5<<" "<<c6;
			textStream<<" "<<c7<<" "<<c8<<" "<<c9<<endl;
		}
	}
	#undef planes
	for(int i=0; i<sketchVector.size(); i+=3)
	{
		int x=sketchVector[i+0];
		int y=sketchVector[i+1];
		int p=sketchVector[i+2];
		textStream<<"k "<<x<<" "<<y<<" "<<p<<endl;
	}
	if(!planes.size())textStream<<"PLANES "<<planesSize+1<<endl;
	for(int i=0; i<regularity.size(); i++)
	{
		textStream<<QAnalyzer::toString(regularity[i]); 
		int n=QAnalyzer::count(regularity[i]);
		for(int t=0; t<n; t++)textStream<<" "<<regularity[++i];
		textStream<<endl;
	}
	for(vec4 p : planes)
	{
		textStream<<"p "<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<p.w()<<endl;
	}
	if(axis.size()==3)
	{
		this->axis=axis;
		QVector<QString> xyz; 
		xyz<<"x "<<"y "<<"z ";
		for(int i=0; i<3; i++)
		{
			textStream<<xyz[i]<<axis[i].x()<<" "<<axis[i].y()<<" "<<axis[i].z()<<endl;
		}
	}
}
void insert(QVector<QVector<vec2>>& pointsCluster, vec2 startPoint, vec2 endPoint)
{
	#define has(x, y) x.indexOf(y)>=0
	for(int i=0; i<pointsCluster.size(); i++)
	{
		if(has(pointsCluster[i], startPoint)&&has(pointsCluster[i], endPoint))return;
		else if(has(pointsCluster[i], startPoint))
		{
			for(int j=i+1; j<pointsCluster.size(); j++)
			{
				if(has(pointsCluster[j], endPoint))
				{
					pointsCluster[i]+=pointsCluster[j];
					pointsCluster.removeAt(j); return;
				}
			}
			pointsCluster[i]<<endPoint; return;
		}
		else if(has(pointsCluster[i], endPoint))
		{
			for(int j=i+1; j<pointsCluster.size(); j++)
			{
				if(has(pointsCluster[j], startPoint))
				{
					pointsCluster[i]+=pointsCluster[j];
					pointsCluster.removeAt(j); return;
				}
			}
			pointsCluster[i]<<startPoint; return;
		}
	}
	#undef has(x, y)
	QVector<vec2> points;
	points<<startPoint<<endPoint;
	pointsCluster<<points; 
}
void draw(QPainter& painter, QVector<QVector<vec2>> pointsCluster)
{
	QString label="I"; for(int i=0; i<pointsCluster.size(); i++, label+=" I")
	{
		for(int j=0; j<pointsCluster[i].size(); j++)
		{
			vec2 p=pointsCluster[i][j]+vec2(0, -5);
			painter.drawText(QPointF((int)p.x(), (int)p.y()), label);
		}
	}
}
void QAnalyzer::drawRegularity(QPainter& painter)
{
	QVector<QVector<vec2>> parallelCenters;
	for(int i=0; i<regularity.size(); i++)
	{
		switch(regularity[i])
		{
			case VERTICAL: 
			{
				int r1=regularity[i+1], r2=regularity[i+2], r3=regularity[i+3];
				vec2 p1=getPoint(r1), p2=getPoint(r2), m1=(p1+p2)/2, m2=m1+vec2(20, 0);
				painter.drawLine((int)m1.x(), (int)m1.y(), (int)m2.x(), (int)m2.y());
				painter.drawText(QPointF((int)m2.x(), (int)m2.y()), "VER"); break;
			}
			case PARALLEL:
			{
				int r1=regularity[i+1], r2=regularity[i+2], r3=regularity[i+3];
				int r4=regularity[i+4], r5=regularity[i+5], r6=regularity[i+6];
				vec2 p1=getPoint(r1), p2=getPoint(r2), p3=(p1+p2)/2;
				vec2 p4=getPoint(r4), p5=getPoint(r5), p6=(p4+p5)/2;
				insert(parallelCenters, p3, p6); break;
			}
			case PERPENDICULAR:
			{
				int r1=regularity[i+1], r2=regularity[i+2];
				int r3=regularity[i+3], r4=regularity[i+4]; vec2 p2=getPoint(r2);
				painter.drawText(QPointF((int)p2.x(), (int)p2.y()), "PER"); break;
			}
			case CONTACT_POINTS: 
			{
				int r1=regularity[i+1], r2=regularity[i+2], dy=15; vec2 p=getPoint(r1);
				painter.drawText(QPointF((int)p.x(), (int)p.y()+dy), "_|_"); break;
			}
		case LOOP:
		{
			if(hasCloseCurveCtrlPointsMarker)
			{
				for(int j=regularity[i+1]; j<regularity[i+2]; j++)
				{
					vec2 p0=getPoint(j+0),  p1=getPoint(j+1);
					painter.drawLine((int)p0.x(), (int)p0.y(), (int)p1.x(), (int)p1.y());
				}
			}
			break;
		}
		}
		i+=count(regularity[i]);
	}
	draw(painter, parallelCenters);
	#define dir avgLineDirections
	for(int i=0; i<dir.size(); i++)
	{ 
		qreal length=50;
		vec2 p0=vec2(100, 100);
		vec2 p1=p0+length*dir[i];
		painter.drawLine(p0.x(), p0.y(), p1.x(), p1.y());
		QString number=""; for(int j=0; j<=i; j++)number+="I ";
		painter.drawText(QPointF(p1.x(), p1.y()), number);
	}
	for(int i=0; i<sketchChords.size(); i+=2)
	{
		vec2 p1=getPoint(sketchChords[i+0]), p2=getPoint(sketchChords[i+1]);
		painter.drawLine((int)p1.x(), (int)p1.y(), (int)p2.x(), (int)p2.y());
	}
	#undef dir
}
void QAnalyzer::drawMarkers(QPainter& painter)
{
	for(int i=0; i<markerLines.size(); i+=4)
	{
		int x0=markerLines[i+0];
		int y0=markerLines[i+1];
		int x1=markerLines[i+2];
		int y1=markerLines[i+3];
		painter.drawLine(x0, y0, x1, y1);
	}
	for(int i=0; i<markerPoints.size(); i+=2)
	{
		int x=markerPoints[i+0];
		int y=markerPoints[i+1];
		painter.drawEllipse(QPoint(x, y), 2, 2);
	}
	for(int i=0; i<startPoints.size(); i+=2)
	{
		float x=startPoints[i+0]-10.0f;
		float y=startPoints[i+1]-10.0f;
		painter.drawText(QPointF(x, y), num(i/2));
	}
}
void QAnalyzer::clear()
{
	this->sketchPaths.clear();
	this->sketchCurves.clear();
	this->sketchTypes.clear();
	this->parallelLines.clear();
	this->avgLineDirections.clear();
	this->isJoint.clear();
	this->joints.clear();
	this->samePoints.clear();
	this->markerLines.clear(); 
	this->markerPoints.clear(); 
	this->startPoints.clear();
	this->planesSize=0;
	this->regularity.clear(); 
	this->contactPoints.clear();
	this->sketchLengths.clear();
	this->sketchPlanes.clear(); 
	this->sketchTypes.clear(); 
	this->sketchSizes.clear(); 
	this->sketchVector.clear();
	this->sketchCurveSizes.clear();
}
