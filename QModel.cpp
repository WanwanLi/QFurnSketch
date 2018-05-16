#include <QDir>
#include <QFile>
#include "QModel.h"
#include "QSketch.h"
#define dot vec3::dotProduct
#define cross vec3::crossProduct

QModel::QModel(veci path, vec point4D, veci holes, QVector<vec3*> quads)
{
	for(int i=0, j=0; i<path.size(); i++)
	{
		if(path[i]==QSketch::MOVE)
		{
			vec coordinates;
			qreal x=point4D[j++], y=point4D[j++];
			qreal z=point4D[j++], w=point4D[j++];
			vec3 point0=vec3(x, y, z), point1, point2, point3;
			this->addPoint(coordinates, point0, quads[w]);
			for(i++; i<path.size()&&path[i]!=QSketch::MOVE; i++)
			{
				if(path[i]==QSketch::LINE)
				{
					x=point4D[j++]; y=point4D[j++];
					z=point4D[j++]; w=point4D[j++];
					vec3 point=vec3(x, y, z); point0=point;
					this->addPoint(coordinates, point, quads[w]);
				}
				else if(path[i]==QSketch::CUBIC)
				{
					x=point4D[j++]; y=point4D[j++];
					z=point4D[j++]; w=point4D[j++];
					point1=vec3(x, y, z);
					x=point4D[j++]; y=point4D[j++];
					z=point4D[j++]; w=point4D[j++];
					point2=vec3(x, y, z);
					x=point4D[j++]; y=point4D[j++];
					z=point4D[j++]; w=point4D[j++];
					point3=vec3(x, y, z);
					this->addCurve(coordinates,
					new vec3[4]{point0, point1,
					point2, point3}, quads[w]);
					point0=point3;
				}
			}
			QVector<vec> coords;
			coords<<coordinates;
			this->coords<<coords;
			this->quads<<quads[w];
			if(i<path.size())i--;
		}
	}
	for(int i=0; i<holes.size(); i+=2)
	{
		int dest=holes[i+0], src=holes[i+1];
		this->coords[dest]<<coords[src][0];
		this->coords[src][0].clear();
	}
	QVector<QVector<vec>> newCoords;
	QVector<vec3*> newQuads;
	for(int i=0; i<this->coords.size(); i++)
	{
		if(this->coords[i].size()==1)
		if(this->coords[i][0].size()<6)continue;
		newCoords<<this->coords[i];
		newQuads<<this->quads[i];
	}
	this->coords=newCoords;
	this->quads=newQuads;
}
void QModel::save(QString modelName)
{
	QDir dir(modelName);
	dir.removeRecursively();
	dir=QDir("."); dir.mkdir(modelName);
	qDebug()<<dir.absoluteFilePath(modelName);
	QString svgStart="<svg height=\"1000\" width=\"1000\" ";
	svgStart+="version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\">";
	QString svgEnd="\" fill=\"none\" stroke=\"blue\" stroke-width=\"1\"/></svg>";
	for(int i=0; i<coords.size(); i++)
	{
		if(this->coords[i].size()==1)
		if(this->coords[i][0].size()<6)continue;
		QString fileName=modelName;
		fileName+="\\closed_curve_"+num(i)+".svg";
		QFile file(fileName); QString endl="\r\n";
		if(!file.open(QIODevice::WriteOnly))continue;
		QTextStream textStream(&file);
		textStream<<svgStart<<endl;
		textStream<<"<path d=\""<<endl;
		qreal w=800, s=aspectRatio(i);
		for(int j=0; j<coords[i].size(); j++)
		{
			for(int k=0; k<coords[i][j].size(); k+=2)
			{
				textStream<<(k==0?"M ":"L ");
				qreal u=coords[i][j][k+0];
				qreal v=coords[i][j][k+1];
				textStream<<num((int)(u*w*s))<<" ";
				textStream<<num((int)(v*w))<<endl;
			}
			textStream<<" Z ";
		}
		textStream<<svgEnd<<endl;
		file.close();
	}
}
qreal QModel::aspectRatio(int quadIndex)
{
	int i=quadIndex;
	vec3 p00=quads[i][0];
	vec3 p01=quads[i][1];
	vec3 p10=quads[i][3];
	vec3 du=p10-p00, dv=p01-p00;
	vec3 y=cross(du, dv).normalized();
	vec3 x=dv.normalized(), z=cross(x, y);
	qreal width=dv.length();
	qreal height=dot(du, z);
	return height/width;
}
void QModel::addPoint(vec& coordinates, vec3 point, vec3* quad)
{
	vec3 p00=quad[0],  p01=quad[1], p10=quad[3];
	vec3 dr=point-p00, du=p10-p00, dv=p01-p00;
	vec3 y=cross(du, dv).normalized();
	vec3 x=dv.normalized(), z=cross(x, y);
	coordinates<<dot(dr, z)/dot(du, z)<<dot(dr, x)/dot(dv, x);
}
void QModel::addCurve(vec& coordinates, vec3* ctrlPoints,  vec3* quad)
{
	qreal totalLength=0.0; 
	for(int i=1; i<4; i++)
	{
		totalLength+=(ctrlPoints[i]-ctrlPoints[i-1]).length();
	}
	int size=totalLength/curveLength;
	size=size<10?10:size; qreal dt=1.0/(size-1);
	for(int i=0; i<size; i++)
	{
		vec3 point=pointAt(ctrlPoints, i*dt);
		this->addPoint(coordinates, point, quad);
	}
}
vec3 QModel::pointAt(vec3* ctrlPoints, qreal t)
{
	const int degree=3;
	vec3* B=new vec3[degree+1];
	for(int i=0; i<=degree; B[i]=ctrlPoints[i++]);
	for(int i=1; i<=degree; i++) 
	{	
		for(int j=0; j<=degree-i; j++)
		{
	    		B[j]=(1.0-t)*B[j]+t*B[j+1];
		}
	}
	return B[0];
}
