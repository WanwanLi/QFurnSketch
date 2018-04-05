#include <QVector>
#include <QVector3D>
#include "QOpenGLObject.h"

class QEdgeNode
{
	public:
	qreal v, dv, maxU; QEdgeNode* next;
	QEdgeNode(qreal v, qreal dv, qreal maxU) : v(v), dv(dv), maxU(maxU), next(NULL){}
};
class QEdgeList
{
	public:
	qreal u;
	int length;
	QEdgeList* next;
	bool isNotEmpty();
	QEdgeNode* getFirst();
	QEdgeNode *first, *last;
	void insert(QEdgeNode* node);
	QEdgeList(qreal u) : u(u), length(0), next(NULL), first(NULL), last(NULL){}
};
class QEdgeListTable
{
	private:
	QEdgeList *first, *last;

	public:
	qreal u();
	void clear();
	bool isNotEmpty();
	QEdgeList* getFirst();
	void insert(QEdgeList* list);
	void insert(double u, QEdgeNode* node);
	QEdgeListTable() : first(NULL), last(NULL){}
};
class QOpenGLPolygon : public QOpenGLObject
{
	private:
	QColor color;
	QVector3D normal;
	qreal thickness=0.02;
	QEdgeListTable edgeListTable;
	void setQuad(QVector3D* quad);
	void createVertices(), addVertex(int p, int n, int t);
	QVector3D p00, p01, p10, p11, getPosition(qreal u, qreal v);
	void translateQuad(QVector3D* quad, QVector3D translation);
	void createEdgeListTable(const QVector<qreal>& coordinates);
	void loadPolygon(QVector3D* quad, const QVector<qreal>& coords);
	void addEdgeNode(qreal u0, qreal v0, qreal u, qreal v, qreal u1, qreal v1);
	void addPolygonSide(QVector3D* quad, QVector<qreal> coords, QVector3D normal);
	QVector<QVector3D> positions, normals; QVector<QColor> colors; QVector<QVector2D> texcoords;

	public:
	QOpenGLPolygon(QVector<QVector3D*> quads, QVector<QColor> colors, QVector<QVector<qreal>> coords);
};

