#include <QFile>
#include<QDebug>
#include "QSketch.h"
#include "QOptimizer.h"
#include "QStanMath.h"

#define QDoubleMath QStanMath<double>
QString QSketch::sketch2DFile="QSketch2D.sky";
QString QSketch::sketch3DFile="QSketch3D.sky";
QString QSketch::sketchModelFile="QSketchMD.sky";

QSketch::QSketch()
{
	this->inflator=new QThread();
	this->normalizer=new QThread();
	this->optimizer2D=newQOptimizer(inflator, sketch2DFile, 4000);
	this->optimizer3D=newQOptimizer(normalizer, sketch3DFile, 600);
}
QSketch::QSketch(veci path, veci point3D, QViewer viewer)
{
	this->path=path;
	this->viewer=viewer;
	this->point3D=point3D;
}
QOptimizer* QSketch::newQOptimizer(QThread* thread, QString sketchFile, int iterations)
{
	QOptimizer* optimizer=new QOptimizer(thread);
	connect(optimizer, &QOptimizer::setValue, this, &QSketch::setValue);
	connect(optimizer, &QOptimizer::finished, this, &QSketch::finished);
	optimizer->sketchFile=sketchFile;
	optimizer->iterations=iterations;
	return optimizer;
}
void readPoint4D(QTextStream& textStream, vec& point4D)
{
	for(int i=0; i<4; i++)point4D<<textStream.readLine().toDouble();
}
void readVector3D(QTextStream& textStream, vec3& vector3D)
{
	for(int i=0; i<3; i++)vector3D[i]=textStream.readLine().toDouble();
}
void readVector4D(QTextStream& textStream, vec4& vector4D)
{
	for(int i=0; i<4; i++)vector4D[i]=textStream.readLine().toDouble();
}
void QSketch::drawStatusText(QPainter& painter)
{
	QString statusText;
	switch(state)
	{
		case INITIALIZED: statusText=""; break;
		case ANALYZED:  statusText="Sketh is analyzed."; break;
		case INFLATED:  statusText="Sketh is inflated."; break;
		case NORMALIZED:  statusText="Sketh is normalized"; break;
	}
	painter.drawText(20, 20, statusText);
}
void QSketch::drawLine(QPainter& painter, vec3 v0, vec3 v1)
{
	QPoint p0=viewer.lookAt(v0.x(), v0.y(), v0.z());
	QPoint p1=viewer.lookAt(v1.x(), v1.y(), v1.z());
	painter.drawLine(p0, p1);
}
void QSketch::drawRegularity3D(QPainter& painter, vec points, QString tag)
{
	for(int i=0; i<points.size(); i+=3)
	{
		qreal x=points[i+0];
		qreal y=points[i+1];
		qreal z=points[i+2];
		QPoint p=viewer.lookAt(x, y, z);
		painter.drawText(p, tag);
	}
}
void QSketch::drawRegularity3D(QPainter& painter)
{
	if(!displayRegularity3D)return;
	QFont font=painter.font();
	QFont newFont; newFont.setPointSize(30);
	painter.setFont(newFont);
	this->drawRegularity3D(painter, horizontal, "=");
	newFont.setPointSize(20);
	painter.setFont(newFont);
	this->drawRegularity3D(painter, forward, "ll");
	painter.setFont(font);
}
void QSketch::drawAxis(QPainter& painter)
{
	QPoint center=QPoint
	(
		-viewer.width/2+50,
		-viewer.height/2+100
	);
	QPoint center1=QPoint(0, 0);
	qreal length=0.25, length1=0.4;
	this->drawAxis(painter, xAxis, length, center);
	this->drawAxis(painter, yAxis, length, center);
	this->drawAxis(painter, zAxis, length, center);
	this->drawAxis(painter, viewer.xAxis, length1, center1);
	this->drawAxis(painter, viewer.yAxis, length1, center1);
	this->drawAxis(painter, viewer.zAxis, length1, center1);

}
void QSketch::drawAxis(QPainter& painter, vec3 axis, qreal length, QPoint center)
{
	if(!displayGroundPlane)return;
	vec3 dir=length*axis;
	QPoint startPoint=viewer.lookAt(0, 0, 0);
	QPoint endPoint=viewer.lookAt(dir.x(), dir.y(), dir.z());
	painter.drawLine(center+startPoint, center+endPoint);
}
void QSketch::drawGroundPlane(QPainter& painter)
{
	if(!displayGroundPlane)return;
	vec3 center=viewer.centerPoint(groundPlane);
	vec3* units=viewer.getTNBSpace(groundPlane);
	vec3 t=units[0], n=units[1], b=units[2];
	vec3 u0=center+planeSize/2*(t+b);
	vec3 u1=center+planeSize/2*(-t+b);
	vec3 v0=center+planeSize/2*(t-b);
	vec3 v1=center+planeSize/2*(-t-b);
	for(int i=0; i<planeGrids; i++)
	{
		qreal k=(i+0.0)/(planeGrids-1.0);
		drawLine(painter, u0*k+v0*(1-k), u1*k+v1*(1-k)); 
		drawLine(painter, u0*k+u1*(1-k), v0*k+v1*(1-k)); 
	}
}
int QSketch::getOptimizingState()
{
	return inflator->isRunning()?INFLATED:NORMALIZED;
}
void QSketch::setValue(int value)
{
	QFile file(QOptimizer::fileName+num(value)); 
	if(!file.open(QIODevice::ReadOnly))return;
	this->path.clear(); this->point4D.clear(); 
	QTextStream textStream(&file);
	this->analyzer.planesSize=textStream.readLine().toInt();
	readVector4D(textStream, groundPlane);
	readVector3D(textStream, xAxis);
	readVector3D(textStream, yAxis);
	readVector3D(textStream, zAxis);
	while(!textStream.atEnd())
	{
		int path=textStream.readLine().toInt();
		if(path==MOVE||path==LINE)
		readPoint4D(textStream, point4D);
		else for(int i=0; i<3; i++)
		readPoint4D(textStream, point4D);
		this->path<<path;
	}
	file.close(); file.remove(); 
	this->iterations=value;
	if(!isOptimizing())
	{
		while(painterPaths.size()<analyzer.planesSize)
		this->painterPaths<<QPainterPath();
	}
	this->state=getOptimizingState();
}
bool QSketch::isOnUpdated()
{
	if(isUpdated)
	{
		this->isUpdated=false;
		return true;
	}
	else return
	(
		this->inflator->isRunning()||
		this->normalizer->isRunning()
	);
}
vec QSketch::open(QString fileName)
{
	QFile file(fileName); vec vector;
	if(!file.open(QIODevice::ReadOnly))return vector;
	QTextStream textStream(&file);
	int size=textStream.readLine().toInt();
	for(int i=0; i<size; i++)vector<<textStream.readLine().toDouble();
	return vector;
}
void QSketch::finished()
{
	this->iterations=0;
	this->isUpdated=true;
	if(is(NORMALIZED))
	this->displayGroundPlane=true;
	if(!displayRegularity3D)
	{
		this->horizontal=open("horizontal");
		this->forward=open("forward");
	//	this->displayRegularity3D=true;
	}
}
QSketch::QSketch(QString fileName)
{
	this->isValid=true;
	QFile file(fileName);
	if(file.open(QIODevice::ReadOnly))
	{
		QTextStream textStream(&file); 
		if(!load(textStream, true))
		this->isValid=false;
		file.close();
	}
	else this->isValid=false;
}
bool QSketch::load(QString fileName)
{
	QFile file(fileName); 
	if(!file.open(QIODevice::ReadOnly))return false;
	QTextStream textStream(&file); this->clear(); 
	if(load(textStream, false))
	{
		this->state=INITIALIZED; this->iterations=0;
		this->painterPaths<<QPainterPath();
		this->paint(); file.close(); return true;
	}
	file.close(); return false;
}
bool QSketch::load(QTextStream& textStream, bool isPoint3D)
{
	bool isSketch=false;
	while(!textStream.atEnd())
	{
		QStringList list=textStream.readLine().split(" ");
		if(list.size()==0)continue;
		if(list[0]=="s")
		{
			if(list.size()<3)continue;
			this->viewer<<list;
			isSketch=true;
			continue;
		}
		if(!isSketch)continue;
		if
		(
			list[0]=="v"||list[0]=="u"||
			list[0]=="r"||list[0]=="f"
		)
		this->viewer<<list;
		else if(load(list, isPoint3D));
		else this->analyzer<<list;
	}
	return isSketch;
}
bool QSketch::load(QStringList& list, bool isPoint3D)
{
	if(isPoint3D)return load(list);
	if(list[0]=="m")
	{
		this->path<<MOVE;
		this->point2D<<list[1].toInt();
		this->point2D<<list[2].toInt();
	}
	else if(list[0]=="l")
	{
		this->path<<LINE;
		this->point2D<<list[1].toInt();
		this->point2D<<list[2].toInt();
	}
	else if(list[0]=="c")
	{
		this->path<<CUBIC;
		for(int i=1; i<=6; i++)
		this->point2D<<list[i].toInt();
	}
	else return false;
	return true;
}
bool QSketch::load(QStringList& list)
{
	if(list[0]=="m")
	{
		this->path<<MOVE;
		this->point3D<<list[1].toInt();
		this->point3D<<list[2].toInt();
		this->point3D<<list[3].toInt();
	}
	else if(list[0]=="l")
	{
		this->path<<LINE;
		this->point3D<<list[1].toInt();
		this->point3D<<list[2].toInt();
		this->point3D<<list[3].toInt();
	}
	else if(list[0]=="c")
	{
		this->path<<CUBIC;
		for(int i=1; i<=9; i++)
		this->point3D<<list[i].toInt();
	}
	else return false;
	return true;
}
QOptimizer* QSketch::optimizer()
{
	return inflator->isRunning()?optimizer2D:optimizer3D;
}
void QSketch::drawProgressBar(QPainter& painter)
{
	qreal t=this->iterations;
	qreal w=viewer.size().width();
	qreal m=optimizer()->iterations;
	painter.drawLine(0, 0, w*t/m, 0);
}
void QSketch::drawMarkers(QPainter& painter)
{
	if(is(ANALYZED))
	{
		this->analyzer.drawMarkers(painter);
		this->analyzer.drawRegularity(painter);
	}
}
bool QSketch::analyze()
{
	if(!is(INITIALIZED))return false;
	this->analyzer.clear();
	this->analyzer.load(this);
	this->analyzer.run();
	this->analyzer.save(sketch2DFile);
	this->state=ANALYZED;
	return true;
}
bool QSketch::inflate()
{
	if(inflator->isRunning())return false;
	this->inflator->start();
	return true;
}
bool QSketch::normalize()
{
	if(normalizer->isRunning())return false;
	this->normalizer->start();
	this->horizontal.clear();
	this->forward.clear();
	this->displayRegularity3D=false;
	return true;
}
QVector<vec3*> QSketch::getPoint4D()
{
	QVector<vec3*> quads; Vector3d space(5, 5, 5);
	for(int i=0; i<analyzer.planes.size()-1; i++)
	{
		Vector3d* quad3d=QDoubleMath::getQuad
		(
			QDoubleMath::toVector4t(analyzer.planes[i]), space
		);
		vec3* quad3D=new vec3[4]; for(int j=0; j<4; j++)
		quad3D[j]=QDoubleMath::toVector3D(quad3d[j]); quads<<quad3D;
	}
	for(int i=0; i<point3D.size(); i+=3)
	{
		vec2 sketchPixel=viewer.sketchPixel(point3D[i+0], point3D[i+1]);
		vec4 plane=analyzer.planes[point3D[i+2]];
		Vector3d sketchPoint=QDoubleMath::sketchPoint
		(
			QDoubleMath::toVector2t(sketchPixel), QDoubleMath::toVector4t(plane)
		);
		this->point4D<<sketchPoint(0)<<sketchPoint(1)<<sketchPoint(2)<<point3D[i+2];
	}
	return quads;
}
void insertPlanePoints(QVector<QVector<Vector3d>>& planePoints, Vector3d point, int planeIndex)
{
	int i=planeIndex; double error=0.001;
	if(planePoints[i].size()>=3)return;
	if(planePoints[i].size()==2)
	{
		Vector3d p0=planePoints[i][0], p1=planePoints[i][1], p2=point;
		Vector3d d0=(p1-p0).normalized(), d1=(p1-p2).normalized();
		if(QDoubleMath::distanceBetweenDirections(d0, d1)>error)planePoints[i]<<p2;
	}
	else planePoints[i]<<point;
}
vec3 QSketch::getPoint3D(int index)
{
	vec2 sketchPixel=viewer.sketchPixel
	(
				point3D[index+0], point3D[index+1]
	);
	vec4 plane=analyzer.planes[point3D[index+2]];
	return QDoubleMath::toVector3D
	(
				QDoubleMath::sketchPoint
				(
					QDoubleMath::toVector2t(sketchPixel),
					QDoubleMath::toVector4t(plane)
				)
	);
}
void QSketch::conncetJointToPlane()
{
	qDebug()<<analyzer.joints<<analyzer.samePoints;
	for(int i=0; i<analyzer.joints.size(); i+=3)
	{
		int c0=analyzer.joints[i+0], c1=analyzer.joints[i+1], p=analyzer.joints[i+2];
		Vector3d p0=QDoubleMath::toVector3t(getPoint3D(c0*3));
		Vector3d p1=QDoubleMath::toVector3t(getPoint3D(c1*3));
		Vector4d plane=QDoubleMath::toVector4t(analyzer.planes[p]);
		Vector3d sketchPoint=QDoubleMath::intersectPlane(p1, p0-p1, plane);
		Vector2d canvasPoint=QDoubleMath::canvasPoint(sketchPoint);
		this->setPoint3D(c0*3, viewer.canvasPixel(QDoubleMath::toVector2D(canvasPoint)));
	}
	for(int i=0; i<analyzer.samePoints.size(); i+=2)
	{
		int c0=analyzer.samePoints[i+0];
		int c1=analyzer.samePoints[i+1];
		int x=point3D[c0*3+0];
		int y=point3D[c0*3+1];
		this->setPoint3D(c1*3, vec2(x, y));
	}
}
void QSketch::antialias()
{

	if(!analyzer.planes.size())return;
	/*vec4 ground=analyzer.planes[analyzer.planes.size()-1];
	Vector4d groundPlane=QDoubleMath::toVector4t(ground);
	Vector3d yAxis=QDoubleMath::toVector3t(analyzer.axis[1]);
	QVector<QVector<Vector3d>> planePoints;
	for(int i=0; i<analyzer.planes.size()-1; i++)
	planePoints<<QVector<Vector3d>();
	for(int i=0; i<point3D.size(); i+=3)
	{
		vec2 sketchPixel=viewer.sketchPixel(point3D[i+0], point3D[i+1]);
		vec4 plane=analyzer.planes[point3D[i+2]];
		Vector3d sketchPoint=QDoubleMath::sketchPoint
		(
			QDoubleMath::toVector2t(sketchPixel), QDoubleMath::toVector4t(plane)
		);
		sketchPoint=QDoubleMath::antialiasPoint(sketchPoint, yAxis, groundPlane);
		insertPlanePoints(planePoints, sketchPoint, point3D[i+2]);
		Vector2d canvasPoint=QDoubleMath::canvasPoint(sketchPoint);
		this->setPoint3D(i, viewer.canvasPixel(QDoubleMath::toVector2D(canvasPoint)));
	}
	for(int i=0; i<analyzer.planes.size()-1; i++)
	{
		this->analyzer.planes[i]=QDoubleMath::toVector4D
		(
			QDoubleMath::createPlane(planePoints[i])
		);
	}
	*/
	this->conncetJointToPlane();
}
void QSketch::setPoint3D(int startIndex, vec2 point)
{
	this->point3D[startIndex+0]=point.x();
	this->point3D[startIndex+1]=point.y();
}
void QSketch::paint()
{
	for(int i=0, j=0; i<path.size(); i++)
	{
		if(path[i]==MOVE)
		{
			int x=point2D[j++];
			int y=point2D[j++];
			this->moveTo(x, y, 0);
		}
		else if(path[i]==LINE)
		{
			int x=point2D[j++];
			int y=point2D[j++];
			this->lineTo(x, y, 0);
		}
		else 
		{
			int x1=point2D[j++], y1=point2D[j++];
			int x2=point2D[j++], y2=point2D[j++];
			int x3=point2D[j++], y3=point2D[j++];
			this->cubicTo(x1, y1, x2, y2, x3, y3, 0);
		}
	}
}
void QSketch::update()
{
	this->updatePainterPaths();
	if(!isOptimizing()){paint(); return;}
	#define lookAt(x, y, z) viewer.lookAt(x, y, z)
	for(int i=0, j=0; i<path.size(); i++)
	{
		if(path[i]==MOVE)
		{
			qreal x=point4D[j++];
			qreal y=point4D[j++];
			qreal z=point4D[j++];
			qreal w=point4D[j++];
			QPoint p=lookAt(x, y, z);
			this->moveTo(p.x(), p.y(), (int)w);
		}
		else if(path[i]==LINE)
		{
			qreal x=point4D[j++];
			qreal y=point4D[j++];
			qreal z=point4D[j++];
			qreal w=point4D[j++];
			QPoint p=lookAt(x, y, z);
			this->lineTo(p.x(), p.y(), (int)w);
		}
		else if(path[i]==CUBIC)
		{
			qreal c1=point4D[j++], c2=point4D[j++], c3=point4D[j++]; j++;
			qreal c4=point4D[j++], c5=point4D[j++], c6=point4D[j++]; j++;
			qreal c7=point4D[j++], c8=point4D[j++], c9=point4D[j++], k=point4D[j++];
			QPoint p1=lookAt(c1, c2, c3), p2=lookAt(c4, c5, c6), p3=lookAt(c7, c8, c9);
			this->cubicTo(p1.x(), p1.y(), p2.x(), p2.y(), p3.x(), p3.y(), (int)k);
		}
	}	
}
bool QSketch::save(QString fileName)
{
	/*QFile file(fileName); QString endl="\r\n";
	if(!file.open(QIODevice::WriteOnly))return false;
	QTextStream textStream(&file); 
	textStream<<"s "<<sketch[0];
	textStream<<" "<<sketch[1]<<" ";
	textStream<<e.viewDistance<<endl;
	QVector<int> s=indices; int d=-1;
	double A=e.up(0).val(), B=e.up(1).val(), C=e.up(2).val();
	double D=e.right(0).val(), E=e.right(1).val(), F=e.right(2).val();
	double G=e.forward(0).val(), H=e.forward(1).val(), I=e.forward(2).val();
	textStream<<"u "<<A<<" "<<B<<" "<<C<<endl;
	textStream<<"r "<<D<<" "<<E<<" "<<F<<endl;
	textStream<<"f "<<G<<" "<<H<<" "<<I<<endl;
	for(int i=2; i<sketch.size(); i++)
	{
		if(sketch[i]==MOVE)
		{
			double x=sketch[++i], y=sketch[++i], z=sketch[++i];
			QPoint p=e.canvasPoint(x, y, z);
			textStream<<"m "<<(int)p.x()<<" ";
			textStream<<(int)p.y()<<" "<<s[d+=3]<<endl;
		}
		else if(sketch[i]==LINE)
		{	
			double x=sketch[++i], y=sketch[++i], z=sketch[++i];
			QPoint p=e.canvasPoint(x, y, z);
			textStream<<"l "<<(int)p.x()<<" ";
			textStream<<(int)p.y()<<" "<<s[d+=3]<<endl;
		}
		else if(sketch[i]==CUBIC)
		{
			double c1=sketch[++i], c2=sketch[++i], c3=sketch[++i];
			double c4=sketch[++i], c5=sketch[++i], c6=sketch[++i];
			double c7=sketch[++i], c8=sketch[++i], c9=sketch[++i];
			QPoint p1=e.canvasPoint(c1, c2, c3);
			QPoint p2=e.canvasPoint(c4, c5, c6);
			QPoint p3=e.canvasPoint(c7, c8, c9);
			textStream<<"c "<<(int)p1.x()<<" ";
			textStream<<(int)p1.y()<<" ";
			textStream<<s[d+=3]<<" ";
			textStream<<(int)p2.x()<<" ";
			textStream<<(int)p2.y()<<" ";
			textStream<<s[d+=3]<<" ";
			textStream<<(int)p3.x()<<" ";
			textStream<<(int)p3.y()<<" ";
			textStream<<s[d+=3]<<endl;
		}
	}
	for(int i=0; i<planes.size(); i+=4)
	{
		textStream<<"p "<<planes[i+0]<<" ";
		textStream<<planes[i+1]<<" ";
		textStream<<planes[i+2]<<" ";
		textStream<<planes[i+3]<<endl;
	}
	int jointDim=e.jointDim;
	for(int i=0; i<joints.size(); i+=jointDim)
	{
		textStream<<e.getJointName(joints[i]);
		for(int j=1; j<jointDim; j++)
		{
			textStream<<" "<<joints[i+j];
		}
		textStream<<endl;
	}*/
	return true;
}
void QSketch::clear()
{
	this->point2D.clear();
	this->point3D.clear();
	this->point4D.clear();
	this->painterPaths.clear();
}
QSize QSketch::size()
{
	return viewer.size();
}
void QSketch::updatePainterPaths()
{
	for(QPainterPath& painterPath : painterPaths)
	{
		QPainterPath newPainterPath=QPainterPath();
		painterPath.swap(newPainterPath);
	}
}
void QSketch::moveTo(int x, int y, int index)
{
	this->painterPaths[index].moveTo(x, y);
}
void QSketch::lineTo(int x, int y, int index)
{
	this->painterPaths[index].lineTo(x, y);
}
void QSketch::cubicTo(int x1, int y1, int x2, int y2, int x3, int y3, int index)
{
	this->painterPaths[index].cubicTo(x1, y1, x2, y2, x3, y3);
}
QPainterPath& QSketch::operator[](int index)
{
	return this->painterPaths[index];
}
bool QSketch::isOptimizing()
{
	return is(INFLATED)||is(NORMALIZED);
}
int QSketch::length()
{
	return this->painterPaths.size();
}
bool QSketch::is(int state)
{
	return this->state==state;
}
