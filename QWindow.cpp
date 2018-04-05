#include "QWindow.h"
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>

QWindow::QWindow(QWidget* widget) : QWidget(widget)
{
	this->canvas=new QCanvas(this);
	this->setLayout(newQBoxLayout());
	this->setMinimumSize(100, 100);
	this->setWindowTitle("QProject");
	this->resize(500, 400);
}
QBoxLayout* QWindow::newQBoxLayout()
{
	QBoxLayout* VBoxLayout=new QVBoxLayout();
	VBoxLayout->setMenuBar(newQMenuBar());
	VBoxLayout->addWidget(canvas);
	VBoxLayout->setMargin(0);
	return VBoxLayout;
}
QMenuBar* QWindow::newQMenuBar()
{
	QMenuBar* menuBar=new QMenuBar();
	QMenu* menu=new QMenu("Image", this);
	QAction* action=menu->addAction("Open", this,  &QWindow::openImage, QKeySequence(tr("Ctrl+o")));
	action=menu->addAction("Save", this,  &QWindow::saveImage, QKeySequence(tr("Ctrl+s")));
	action=menu->addAction("Clear", canvas, &QCanvas::clear, QKeySequence(tr("Ctrl+c")));
	action=menu->addAction("Resize", canvas, &QCanvas::resizeImage, QKeySequence(tr("Ctrl+e")));
	action=menu->addAction("Reset View", canvas, &QCanvas::resetViewInfo, QKeySequence(tr("Ctrl+t")));
	action=menu->addAction("Inflate Sketch", canvas, &QCanvas::inflateSketch, QKeySequence(tr("Ctrl+i")));
	action=menu->addAction("Visualize 3D Model", canvas, &QCanvas::visualize, QKeySequence(tr("Ctrl+v")));
	action=menu->addAction("Analyze Sketch", canvas, &QCanvas::analyzeSketch, QKeySequence(tr("Ctrl+a")));
	action=menu->addAction("Normalize Sketch", canvas, &QCanvas::normalizeSketch, QKeySequence(tr("Ctrl+n")));
	action=menu->addAction("Set Camera Upward", canvas, &QCanvas::setCameraUpward, QKeySequence(tr("Ctrl+u")));
	action->setCheckable(true); action->setChecked(true); canvas->setCameraUpward();
	menuBar->addMenu(menu);
	return menuBar;
}
bool QWindow::openImage()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open Image", QDir::currentPath());
	if(fileName.isEmpty()||!this->canvas->loadImage(fileName))
	{
		QMessageBox::critical(this, "Error", "Can not open image."); return false;
	}
	return true;
}
bool QWindow::saveImage()
{
	QString fileName = QFileDialog::getSaveFileName(this, "Save Image", QString(), "FSK(*.fsk);; JEPG(*.jpg)");
	if(!fileName.isEmpty())
	{
		QStringList fileNames=fileName.split("."); 
		QByteArray fileFormat=fileNames[fileNames.size()-1].toLatin1();
		if(this->canvas->saveImage(fileName, fileFormat.data()))return true;
	}
	QMessageBox::critical(this, "Error", "Can not save image."); return false;
}
void QWindow::closeEvent(QCloseEvent* event)
{
	if(canvas->isModified()) 
	{
		QMessageBox::StandardButton button=QMessageBox::warning
		(
			this, "Close", "Save Image?", QMessageBox::Save|
			QMessageBox::Discard|QMessageBox::Cancel
		);
		if(button==QMessageBox::Save&&saveImage())event->accept();
		else if(button==QMessageBox::Discard)event->accept();
		else if(button==QMessageBox::Cancel)event->ignore();
		else event->ignore();
	}
	else event->accept();
}
