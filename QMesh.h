#include "QDefine.h"

class QMesh
{
	public:
	QVector<vec> coords; QVector<vec3*> quads;
	QMesh(veci path, vec point4D, QVector<vec3*> quads);

	private:
	QVector<vec4> planes;
	void getCoordinates();
	qreal curveLength=0.05;
	vec3 pointAt(vec3* ctrlPoints, qreal t);
	void addPoint(vec& coordinates, vec3 point, vec3* quad);
	void addCurve(vec& coordinates, vec3* ctrlPoints,  vec3* quad);
};
