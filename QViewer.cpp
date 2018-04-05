#include "QViewer.h"
#define dot(x, y) vec3::dotProduct(x, y)
#define cross(x, y) vec3::crossProduct(x, y)

QSize QViewer::size()
{
	return QSize(width, height);
}
QPoint QViewer::lookAt(qreal x, qreal y, qreal z)
{
	vec3 point(x, y, z), viewDirection=-forward;
	vec3 eye=viewDirection*viewDistance;
	vec3 focus=viewDirection*(viewDistance-focalLength);
	vec4 viewPlane=createPlane(focus, viewDirection);
	vec3 position=intersectPlane(eye, point-eye, viewPlane);
	x=screenScale*dot(position, right)*width/2+width/2; 
	y=screenScale*aspectRatio*dot(position, up)*height/2+height/2;
	return QPoint((int)x, height-(int)y);
}
void QViewer::translate(qreal translation)
{
	this->viewDistance+=translateSpeed*translation;
	this->viewDistance=clamp(viewDistance, minDistance, maxDistance);
}
void QViewer::rotate(QPoint rotation)
{
	qreal rotUp=rotateSpeed*rotation.x();
	qreal rotRight=rotateSpeed*rotation.y();
	this->forward=rotate(forward, up, rotUp);
	this->right=rotate(right, up, rotUp);
	this->forward=rotate(forward, right, rotRight);
	this->up=rotate(up, right, rotRight);
}
void QViewer::operator<<(QStringList& stringList)
{
	if(stringList[0]=="s")
	{
		this->width=stringList[1].toDouble();
		this->height=stringList[2].toDouble();
	}
	else if(stringList[0]=="v")
	{
		this->viewDistance=stringList[1].toDouble();
		this->focalLength=stringList[2].toDouble();
		this->screenScale=stringList[3].toDouble();
	}
	else
	{
		qreal x=stringList[1].toDouble();
		qreal y=stringList[2].toDouble();
		qreal z=stringList[3].toDouble();
		vec3 vector=vec3(x, y, z).normalized();
		if(stringList[0]=="f")this->forward=vector;
		else if(stringList[0]=="u")this->up=vector;
		else if(stringList[0]=="r")this->right=vector;
	}
}
void QViewer::operator>>(QTextStream& textStream)
{
	textStream<<"v "<<viewDistance<<" "<<focalLength<<" "<<screenScale<<"\r\n";
	textStream<<"f "<<forward.x()<<" "<<forward.y()<<" "<<forward.z()<<"\r\n";
	textStream<<"r "<<right.x()<<" "<<right.y()<<" "<<right.z()<<"\r\n";
	textStream<<"u "<<up.x()<<" "<<up.y()<<" "<<up.z()<<"\r\n";
}
void QViewer::resize(QSize size)
{
	this->width=size.width()+0.0;
	this->height=size.height()+0.0;
	this->aspectRatio=width/height;
}
vec3 QViewer::rotate(vec3 vector, vec3 axis, qreal angle)
{
	QMatrix4x4 matrix4x4; matrix4x4.setToIdentity();
	matrix4x4.rotate(angle, axis.x(), axis.y(), axis.z());
	return matrix4x4.map(vector);
}
qreal QViewer::clamp(qreal value, qreal min, qreal max)
{
	return value<min?min:value>max?max:value;
}
vec4 QViewer::createPlane(vec3 point, vec3 normal)
{
	vec3 direction=normal.normalized();
	return vec4(direction, -dot(point, direction));
}
vec3 QViewer::intersectPlane(vec3 origin, vec3 direction, const vec4& plane)
{
	vec3 normal=plane.toVector3D();
	if(dot(normal, direction)==0)return origin;
	vec3 center=centerPoint(plane);
	vec3 connection=center-origin;
	qreal distance=dot(normal, connection)/dot(normal, direction);
	return origin+distance*direction;
}
vec3 QViewer::centerPoint(const vec4& plane)
{
	qreal A=plane.x(), B=plane.y(), C=plane.z(), D=plane.w(), E=1e-5; vec3 P(0, 0, 0);
	if(qAbs(A)>E)P[0]=-D/A; else if(qAbs(B)>E)P[1]=-D/B; else P[2]=-D/C; return P;
}
vec3* QViewer::getTNBSpace(const vec4& plane)
{
	vec3 x=vec3(1, 0, 0), y=vec3(0, 1, 0), z=vec3(0, 0, 1);
	vec3 normal=plane.toVector3D().normalized();
	qreal xDotn=qAbs(dot(x, normal));
	qreal yDotn=qAbs(dot(y, normal));
	qreal zDotn=qAbs(dot(z, normal));
	qreal min=qMin(xDotn, qMin(yDotn, zDotn));
	vec3 vector=xDotn==min?x:(yDotn==min?y:z);
	vec3 tangent=cross(normal, vector).normalized();
	vec3 bitangent=cross(normal, tangent).normalized();
	return new vec3[3]{tangent, normal, bitangent};
}
vec2 QViewer::sketchPixel(int x, int y)
{
	return vec2
	(
		 (x-width/2+0.0)/(width/2),
		-(y-height/2+0.0)/(height/2)/aspectRatio
	);
}
vec2 QViewer::canvasPixel(vec2 point)
{
	return vec2
	(
		point.x()*(width/2)+width/2,
		-point.y()*(height/2)*aspectRatio+height/2
	);
}