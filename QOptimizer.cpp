#include <QFile>
#include <QDebug>
#include "QEnergy.h"
#include "QSketch.h"
#include "QOptimizer.h"
#include <QTextStream>

QString QOptimizer::fileName="QOptimizer.";
QOptimizer::QOptimizer(QThread* thread)
{
	this->moveToThread(thread);
	this->connectToThread(thread);
}
void QOptimizer::connectToThread(QThread* thread)
{
	connect(thread, &QThread::started, this, &QOptimizer::start);
	connect(this, &QOptimizer::finished, this, &QOptimizer::quit);
	connect(this, &QOptimizer::finished, thread, &QThread::quit);
}
void writeVector3v(QTextStream& textStream, Vector3v vector)
{
	textStream<<num(vector[0].val())<<"\n";
	textStream<<num(vector[1].val())<<"\n";
	textStream<<num(vector[2].val())<<"\n";
}
void writeVectorXv(QTextStream& textStream, VectorXv vector)
{
	for(int i=0; i<vector.size(); i++)textStream<<num(vector(i).val())<<"\n";
}
void QOptimizer::emitValueChanged(int value, const VectorXd& variable)
{
	#define path energy->path
	QFile file(fileName+num(value));
	if(!file.open(QIODevice::WriteOnly))return;
	#define planeIndex(i) energy->sketch(i*3+2)
	MatrixXv points=energy->getSketchPoints(variable);
	VectorXv variables=energy->toVectorXv(variable);
	energy->getAxis(variables);
	QTextStream textStream(&file);
	textStream<<num(energy->planeSize)<<"\n";
	writeVectorXv(textStream, energy->getGroundPlane(variable));
	writeVectorXv(textStream, energy->axis[0]);
	writeVectorXv(textStream, energy->axis[1]);
	writeVectorXv(textStream, energy->axis[2]);
	for(int i=0, j=0; i<path.size(); i++)
	{
		#define S QSketch
		textStream<<num(path[i])<<"\n";
		if(path[i]==S::MOVE||path[i]==S::LINE)
		{
			writeVector3v(textStream, points.col(j));
			textStream<<num(planeIndex(j++))<<"\n";
		}
		else
		{
			for(int k=0; k<3; k++)
			{
				writeVector3v(textStream, points.col(j));
				textStream<<num(planeIndex(j++))<<"\n";
			}
		}
		#undef S
	}
	file.close(); emit setValue(value);
	#undef planeIndex
	#undef path 
}
ISolver<QProblem, 1>  QOptimizer::QSolver()
{
	int solver=LBFGS_SOLVER;
	switch(solver)
	{
		case BFGS_SOLVER: return BfgsSolver<QProblem>();
	//	case LBFGS_SOLVER: return LbfgsSolver<QProblem>();
	//	case LBFGSB_SOLVER: return LbfgsbSolver<QProblem>();
	//	case CMAES_SOLVER: return CMAesSolver<QProblem>();
		//case CMAESB_SOLVER: return CMAEsbSolver<QProblem>();
		//case NELDERMEAD_SOLVER: return NeldermeadSolver<QProblem>();
//		case NEWTON_DESCENT_SOLVER: return NewtonDescentSolver<QProblem>();
		case GRADIENT_DESCENT_SOLVER: return GradientDescentSolver<QProblem>();
		case CONJUGATED_GRADIENT_DESCENT_SOLVER: return ConjugatedGradientDescentSolver<QProblem>();
	}
	return  GradientDescentSolver<QProblem>();
}
void QOptimizer::start()
{
	QSketch sketch(sketchFile);
	if(!sketch.isValid)
	{
		this->energy=NULL;
		emit finished(); return;
	}
	sketch.antialias();
	this->energy=new QEnergy
	(
		sketch.path, sketch.point3D,
		sketch.analyzer.planesSize,
		sketch.analyzer.regularity,
		sketch.analyzer.planes,
		sketch.analyzer.axis,
		sketch.viewer
	);
	QProblem f(energy);
	connect
	(
		&f, &QProblem::valueChanged, 
		this, &QOptimizer::valueChanged
	);
	VectorXd x=energy->variables;
	Criteria<double> criteria=Criteria<double>::defaults();
	criteria.iterations=iterations;
	GradientDescentSolver<QProblem> solver;
	solver.setStopCriteria(criteria);
	solver.minimize(f, x);
	energy->variables=x;
	emit finished();
}
void QOptimizer::valueChanged(int iterations, const VectorXd& variable)
{
	emitValueChanged(iterations, variable);
}
void QOptimizer::save(QString fileName, QVector<qreal> vector)
{
	QFile file(fileName);
	if(!file.open(QIODevice::WriteOnly))return;
	QTextStream textStream(&file);
	textStream<<num(vector.size())<<"\n";
	for(qreal x : vector)textStream<<num(x)<<"\n";
	file.close(); 
}
void QOptimizer::quit()
{
	if(!energy)return;
	if(energy->isPlaneOnly)
	{
		this->energy->save(QSketch::sketch3DFile);
		this->save("horizontal", energy->horizontal);
		this->save("forward", energy->forward);
	}
	else this->energy->save(QSketch::sketchModelFile);
}
