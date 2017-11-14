#include <QFile>
#include<QDebug>
#include "QSketch.h"
#include "QOptimizer.h"

QSketch::QSketch()
{
	this->thread=new QThread();
	this->optimizer=new QOptimizer(thread);
	connect(optimizer, &QOptimizer::setValue, this, &QSketch::setValue);
}
void readPoint4D(QTextStream& textStream, vec& point4D)
{
	for(int i=0; i<4; i++)point4D<<textStream.readLine().toDouble();
}
void QSketch::drawLine(QPainter& painter, vec3 v0, vec3 v1)
{
	QPoint p0=viewer.lookAt(v0.x(), v0.y(), v0.z());
	QPoint p1=viewer.lookAt(v1.x(), v1.y(), v1.z());
	painter.drawLine(p0, p1);
}
void QSketch::drawPlane(QPainter& painter)
{
	if(!is(OPTIMIZED))return;
	vec4 plane=viewer.createPlane(planeCenter, vec3(0, 1, 0));
	vec3 h0=planeCenter-vec3(planeSize/2, plane.w(), planeSize/2), v0=h0;
	vec3 h1=h0+vec3(planeSize, 0, 0), v1=v0+vec3(0, 0, planeSize);
	qreal interval=planeSize/(planeGrids-1);
	for(int i=0; i<planeGrids; i++)
	{
		drawLine(painter, h0, h1); 
		drawLine(painter, v0, v1); 
		h0.setZ(h0.z()+interval);
		h1.setZ(h1.z()+interval);
		v0.setX(v0.x()+interval);
		v1.setX(v1.x()+interval);
	}
}
void QSketch::setValue(int value)
{
	QFile file(optimizer->fileName+num(value)); 
	if(!file.open(QIODevice::ReadOnly))return;
	this->path.clear(); this->point4D.clear(); 
	QTextStream textStream(&file);
	this->analyzer.planesSize=textStream.readLine().toInt();
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
	if(!is(OPTIMIZED))
	{
		while(painterPaths.size()<analyzer.planesSize)
		this->painterPaths<<QPainterPath();
		this->state=OPTIMIZED;
	}
}
bool QSketch::load()
{
	QFile file(analysisFile); 
	#define remove(f) {f.close(); }//f.remove();}
	if(!file.open(QIODevice::ReadOnly))return false;
	QTextStream textStream(&file); 
	if(load(textStream, true)){remove(file); return true;}
	else remove(file); return false;
	#undef remove(f)
}
bool QSketch::load(QString fileName)
{
	QFile file(fileName); 
	if(!file.open(QIODevice::ReadOnly))return false;
	QTextStream textStream(&file); this->clear(); 
	if(load(textStream, false))
	{
		this->state=INITIAL; this->iterations=0; 
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
void QSketch::drawProgressBar(QPainter& painter)
{
	qreal t=iterations+0.0;
	qreal w=viewer.size().width();
	qreal m=optimizer->iterations;
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
	if(!is(INITIAL))return false;
	this->analyzer.clear();
	this->analyzer.load(this);
	this->analyzer.run();
	this->analyzer.save(analysisFile);
	this->state=ANALYZED;
	return true;
}
bool QSketch::optimize()
{
	//if(!is(ANALYZED))return false;
	if(thread->isRunning())return false;
	else thread->start();
	return true;
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
	if(!is(OPTIMIZED)){paint(); return;}
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
int QSketch::length()
{
	return this->painterPaths.size();
}
bool QSketch::is(int state)
{
	return this->state==state;
}
