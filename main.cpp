#include "stdafx.h"
#include "zero.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Zero w;
	w.show();
	return a.exec();
}
