#include <QObject>
#include <QThread>
#include "QProblem.h"
#include <Eigen/Dense>
using namespace Eigen;

class QEnergy;
class QOptimizer : public QObject
{
	Q_OBJECT

	signals:
	void started();
	void finished();
	void setValue(int value);

	public slots:
	void quit();
	void start();
	void valueChanged(int iterations, const VectorXd& variable);

	public:
	int iterations;
	QEnergy* energy;
	QString sketchFile;
	static QString fileName;
	QOptimizer(QThread* thread);
	ISolver<QProblem, 1>  QSolver();
	void save(QString fileName, QVector<qreal> vector);
	void connectToThread(QThread* thread);
	void emitValueChanged(int value, const VectorXd& variable);
	static enum
	{
		BFGS_SOLVER,
		LBFGS_SOLVER,
		LBFGSB_SOLVER,
		CMAES_SOLVER,
		CMAESB_SOLVER,
		NELDERMEAD_SOLVER,
		NEWTON_DESCENT_SOLVER,
		GRADIENT_DESCENT_SOLVER,
		CONJUGATED_GRADIENT_DESCENT_SOLVER
	};
	int solver=GRADIENT_DESCENT_SOLVER;
};
