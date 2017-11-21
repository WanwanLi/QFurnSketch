#include <QFile>
#include <QDebug>
#include "QEnergy.h"
#include "QSketch.h"
#include "QOptimizer.h"
#include <QTextStream>

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
void QOptimizer::emitValueChanged(int value, const VectorXd& variable)
{
	#define path energy->path
	QFile file(fileName+num(value));
	if(!file.open(QIODevice::WriteOnly))return;
	#define planeIndex(i) energy->sketchVector(i*3+2)
	MatrixXv points=energy->getSketchPoints(variable);
	QTextStream textStream(&file);
	textStream<<num(energy->planeSize)<<"\n";
	Vector4v groundPlane=energy->getGroundPlane(variable);
	textStream<<num(groundPlane(0).val())<<"\n";
	textStream<<num(groundPlane(1).val())<<"\n";
	textStream<<num(groundPlane(2).val())<<"\n";
	textStream<<num(groundPlane(3).val())<<"\n";
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
	#define planeIndex(i)
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
	QSketch sketch;
	if(!sketch.load()){emit finished(); return;}
	this->energy=new QEnergy
	(
		sketch.path, sketch.point3D,
		sketch.analyzer.planesSize,
		sketch.analyzer.regularity,
		sketch.viewer
	);
	QProblem f(energy);
	connect
	(
		&f, &QProblem::valueChanged, 
		this, &QOptimizer::valueChanged
	);
	VectorXd x=energy->variableVector;
	Criteria<double> criteria=Criteria<double>::defaults();
	criteria.iterations=iterations;
	GradientDescentSolver<QProblem> solver;
	solver.setStopCriteria(criteria);
	solver.minimize(f, x);
	emit finished();
}
void QOptimizer::valueChanged(int iterations, const VectorXd& variable)
{
	emitValueChanged(iterations, variable);
}
void QOptimizer::quit()
{
	emit setValue(0);
}
