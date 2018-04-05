#include <QSize>
#include "QDefine.h"
#include <QMatrix4x4>
#include <QTextStream>

#ifndef Q_VIEWER
#define Q_VIEWER 0
class QViewer
{
	public:
	QSize size();
	qreal width, height;
	void resize(QSize size);
	vec3 xAxis=vec3(1, 0, 0);
	vec3 yAxis=vec3(0, 1, 0);
	vec3 zAxis=vec3(0, 0, 1);
	void rotate(QPoint rotation);
	vec2 sketchPixel(int x, int y);
	vec2 canvasPixel(vec2 point);
	void translate(qreal translation);
	vec3 centerPoint(const vec4& plane);
	QPoint lookAt(qreal x, qreal y, qreal z);
	void operator<<(QStringList& stringList);
	void operator>>(QTextStream& textStream);
	vec3* getTNBSpace(const vec4& plane);
	qreal rotateSpeed=0.4, translateSpeed=0.5;
	QPoint lookAt(int x, int y, const vec4& plane);
	qreal clamp(qreal value, qreal min, qreal max);
	vec4 createPlane(vec3 point, vec3 normal);
	qreal aspectRatio=1.0, screenScale=100.0, focalLength=0.1;
	vec3 rotate(vec3 vector, vec3 axis, qreal angle);
	qreal viewDistance=12.0, minDistance=6.0, maxDistance=24.0;
	vec3 forward=vec3(0, 0, -1), up=vec3(0, 1, 0), right=vec3(1, 0, 0);
	vec3 intersectPlane(vec3 origin, vec3 direction, const vec4& plane);
};
#endif
