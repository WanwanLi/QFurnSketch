#include "QDefine.h"

class QModel
{
	public:
	void save(QString modelName);
	QVector<QVector<vec>> coords; QVector<vec3*> quads;
	QModel(veci path, vec point4D, veci holes, QVector<vec3*> quads);

	private:
	void getCoordinates();
	QVector<vec4> planes;
	qreal curveLength=0.05;
	qreal aspectRatio(int quadIndex);
	vec3 pointAt(vec3* ctrlPoints, qreal t);
	void addPoint(vec& coordinates, vec3 point, vec3* quad);
	void addCurve(vec& coordinates, vec3* ctrlPoints,  vec3* quad);
};
