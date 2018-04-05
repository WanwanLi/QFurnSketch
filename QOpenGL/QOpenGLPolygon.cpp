#include "QOpenGLPolygon.h"
#define dot QVector3D::dotProduct
#define cross QVector3D::crossProduct

QOpenGLPolygon::QOpenGLPolygon(QVector<QVector3D*> quads, QVector<QColor> colors, QVector<QVector<qreal>> coords)
{
	for(int i=0; i<quads.size(); i++)
	{
		this->setQuad(quads[i]);
		this->color=colors[i%colors.size()];
		this->translateQuad(quads[i], normal);
		this->loadPolygon(quads[i], coords[i]);
		this->translateQuad(quads[i], -2*normal);
		this->loadPolygon(quads[i], coords[i]);
		this->addPolygonSide(quads[i], coords[i], normal);
	}
	this->initializeVertices(positions.size(), normals.size()*6);
	for(int i=0; i<normals.size(); i++)
	{
		this->addVertex(i*4+0, i, i*4+0);
		this->addVertex(i*4+1, i, i*4+1);
		this->addVertex(i*4+2, i, i*4+2);
		this->addVertex(i*4+3, i, i*4+3);
		this->addIndex(i*4+0);
		this->addIndex(i*4+1);
		this->addIndex(i*4+2);
		this->addIndex(i*4+2);
		this->addIndex(i*4+3);
		this->addIndex(i*4+0);
	}
	this->createBuffers();
}
void QOpenGLPolygon::addPolygonSide(QVector3D* quad, QVector<qreal> coords, QVector3D normal)
{
	for(int i=0, size=coords.size()/2; i<size; i++)
	{
		int i0=i+0, i1=(i+1)%size;
		QVector3D translate=normal*thickness;
		qreal u0=coords[i0*2+0], v0=coords[i0*2+1];
		qreal u1=coords[i1*2+0], v1=coords[i1*2+1];
		QVector3D p00=getPosition(u0, v0), p01=getPosition(u1, v1);
		QVector3D p10=p00+2*translate, p11=p01+2*translate;
		QVector3D normal=cross(p01-p00, p10-p00).normalized();
		this->texcoords<<QVector2D(u0, v0)<<QVector2D(u0, v0);
		this->texcoords<<QVector2D(u1, v1)<<QVector2D(u1, v1);
		this->positions<<p00<<p10<<p11<<p01;
		this->normals<<normal; this->colors<<color;
	}
}
void QOpenGLPolygon::translateQuad(QVector3D* quad, QVector3D normal)
{
	for(int i=0; i<4; i++)quad[i]+=normal*thickness;
}
void QOpenGLPolygon::addVertex(int p, int n, int t)
{
	GLfloat px=positions[p].x();
	GLfloat py=positions[p].y();
	GLfloat pz=positions[p].z();
	GLfloat nx=normals[n].x();
	GLfloat ny=normals[n].y();
	GLfloat nz=normals[n].z();
	GLfloat tx=texcoords[t].x();
	GLfloat ty=texcoords[t].y();
	this->addPosition(px, py, pz);
	this->addNormal(nx, ny, nz);
	this->addColor(colors[n]);
	this->addTexcoord(tx, ty);
}
void QOpenGLPolygon::createEdgeListTable(const QVector<qreal>& coordinates)
{
	this->edgeListTable.clear();
	int length=coordinates.size(), l=length/2, L=l;
	for(int i=0; i<L; i++)
	{
		int i0=(i-1+l)%l, i1=(i+1)%l;
		qreal u0=coordinates[i0*2+0];
		qreal v0=coordinates[i0*2+1];
		qreal u=coordinates[i*2+0];
		qreal v=coordinates[i*2+1];
		qreal u1=coordinates[i1*2+0];
		qreal v1=coordinates[i1*2+1];
		while(u0==u)
		{
			i=(i-1+l)%l; L--;
			i0=(i-1+l)%l; i1=(i+1)%l;
			u0=coordinates[i0*2+0];
			v0=coordinates[i0*2+1];
			u=coordinates[i*2+0];
			v=coordinates[i*2+1];
			u1=coordinates[i1*2+0];
			v1=coordinates[i1*2+1];
		}
		if(u1==u)
		{
			this->addEdgeNode(u0, v0, u, v, -1, -1);
			while(u1==u)
			{
				i=(i+1)%l; i1=(i+1)%l;
				u=coordinates[i*2+0];
				v=coordinates[i*2+1];
				u1=coordinates[i1*2+0];
				v1=coordinates[i1*2+1];
			}
			this->addEdgeNode(-1, -1, u, v, u1, v1);
		}
		else this->addEdgeNode(u0, v0, u, v, u1, v1);
	}
}
void QOpenGLPolygon::addEdgeNode(qreal u0, qreal v0, qreal u, qreal v, qreal u1, qreal v1)
{
	qreal dv0=(v0-v)/(u0-u), dv1=(v1-v)/(u1-u);
	if(u0>u&&u1>u)
	{
		this->edgeListTable.insert(u, new QEdgeNode(v, dv0, u0));
		this->edgeListTable.insert(u, new QEdgeNode(v, dv1, u1));
	}
	else if(u0<u&&u1<u);
	else
	{
		if(u1<u0)this->edgeListTable.insert(u, new QEdgeNode(v, dv0, u0));
		else this->edgeListTable.insert(u, new QEdgeNode(v, dv1, u1));
	}
}
void QOpenGLPolygon::setQuad(QVector3D* quad)
{
	this->p00=quad[0]; this->p01=quad[1];
	this->p11=quad[2]; this->p10=quad[3];
	this->normal=cross(p01-p00, p10-p00).normalized();
}
void QOpenGLPolygon::loadPolygon(QVector3D* quad, const QVector<qreal>& coords)
{
	this->setQuad(quad);
	this->createEdgeListTable(coords);
	QEdgeList* activeEdgeList=new QEdgeList(0.0);
	while(edgeListTable.isNotEmpty())
	{
		QEdgeList* tempEdgeList=edgeListTable.getFirst();
		QEdgeList* midEdgeList=NULL;
		qreal u0=tempEdgeList->u; 
		activeEdgeList->u=u0;
		while(tempEdgeList->isNotEmpty())
		{
			activeEdgeList->insert(tempEdgeList->getFirst());
		}
		if(activeEdgeList->isNotEmpty())
		{
			QEdgeList* newEdgeListTable=new QEdgeList(0.0);
			while(activeEdgeList->isNotEmpty())
			{
				QEdgeNode* n0=activeEdgeList->getFirst();
				QEdgeNode* n1=activeEdgeList->getFirst();
				qreal v00=n0->v, v01=n1->v, v10=v00, v11=v01;
				qreal u1=edgeListTable.u();
				qreal maxU0=n0->maxU, maxU1=n1->maxU;
				if(u1<=maxU0&&u1<=maxU1)
				{
					v10=v00+(u1-u0)*n0->dv;
					v11=v01+(u1-u0)*n1->dv;
					n0->v=v10;n1->v=v11;
					if(u1<maxU0)newEdgeListTable->insert(n0);
					if(u1<maxU1)newEdgeListTable->insert(n1);
				}
				else if(u1>maxU0&&maxU0==maxU1)
				{
					u1=maxU0;
					v10=v00+(u1-u0)*n0->dv;
					v11=v01+(u1-u0)*n1->dv;
					n0->v=v10; n1->v=v11;
				}
				else if(u1>maxU1)
				{
					u1=maxU1;
					v10=v00+(u1-u0)*n0->dv;
					v11=v01+(u1-u0)*n1->dv;
					n0->v=v10;
					if(midEdgeList==NULL)midEdgeList=new QEdgeList(u1);
					midEdgeList->insert(n0);
				}
				else if(u1>maxU0)
				{
					u1=maxU0;
					v10=v00+(u1-u0)*n0->dv;
					v11=v01+(u1-u0)*n1->dv;
					n1->v=v11;
					if(midEdgeList==NULL)midEdgeList=new QEdgeList(u1);
					midEdgeList->insert(n1);
				}
				this->texcoords<<QVector2D(u0, v00);
				this->texcoords<<QVector2D(u1, v10);
				this->texcoords<<QVector2D(u1, v11);
				this->texcoords<<QVector2D(u0, v01);
				this->positions<<getPosition(u0, v00);
				this->positions<<getPosition(u1, v10);
				this->positions<<getPosition(u1, v11);
				this->positions<<getPosition(u0, v01);
				this->normals<<normal; this->colors<<color;
			}
			activeEdgeList=newEdgeListTable;
		}
		if(midEdgeList!=NULL)this->edgeListTable.insert(midEdgeList);
	}
}
QVector3D QOpenGLPolygon::getPosition(qreal u, qreal v)
{
	QVector3D du=p10-p00, dv=p01-p00, y=normal;
	QVector3D x=dv.normalized(), z=cross(x, y);
	return p00+u*dot(du, z)*z+v*dot(dv, x)*x;
}
void QEdgeList::insert(QEdgeNode* node)
{
	if(first==NULL)
	{
		this->first=node;
		this->last=node;
	}
	else if(node->v<first->v||(node->v==first->v&&node->dv<first->dv))
	{
		node->next=first;
		this->first=node;
	}
	else if(node->v>last->v||(node->v==last->v&&node->dv>last->dv))
	{
		this->last->next=node;
		this->last=node;
	}
	else
	{
		QEdgeNode* m, *n;
		for(n=first, m=n; n&&(node->v>n->v||(node->v==n->v&&node->dv>n->dv)); m=n, n=n->next);
		node->next=n;
		m->next=node;
	}
	this->length++;
}
QEdgeNode* QEdgeList::getFirst()
{
	if(first==NULL)return NULL;
	QEdgeNode* node=first;
	this->first=first->next;
	node->next=NULL;
	this->length--;
	return node;
}
bool QEdgeList::isNotEmpty()
{
	return (first!=NULL);
}
void QEdgeListTable::insert(QEdgeList* list)
{
	if(first==NULL)
	{
		this->first=list;
		this->last=list;
	}
	else if(list->u>=last->u)
	{
		this->last->next=list;
		this->last=list;
	}
	else if(list->u<=first->u)
	{
		list->next=first;
		this->first=list;
	}
	else
	{
		QEdgeList *m, *n;
		for(n=first, m=n; list->u>n->u; m=n, n=n->next);
		list->next=n;
		m->next=list;
	}
}
void QEdgeListTable::insert(double u, QEdgeNode* node)
{
	QEdgeList* l;
	for(l=first; l&&l->u<u; l=l->next);
	if(!l||l->u!=u)
	{
		QEdgeList* l1=new QEdgeList(u);
		l1->insert(node);
		this->insert(l1);
	}
	else l->insert(node);
}
qreal QEdgeListTable::u()
{
	return first==NULL?1.0:first->u;
}
QEdgeList* QEdgeListTable::getFirst()
{
	if(first==NULL)return NULL;
	QEdgeList *list=first;
	this->first=first->next;
	list->next=NULL;
	return list;
}
bool QEdgeListTable::isNotEmpty()
{
	return (first!=NULL);
}
void QEdgeListTable::clear()
{
	this->first=NULL;
	this->last=NULL;
}
