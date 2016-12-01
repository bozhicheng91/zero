/********************************************************************************
** Form generated from reading UI file 'zero.ui'
**
** Created by: Qt User Interface Compiler version 5.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ZERO_H
#define UI_ZERO_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qvtkwidget.h"

QT_BEGIN_NAMESPACE

class Ui_ZeroClass
{
public:
    QAction *actionOpenCloud;
    QAction *actionSaveCloud;
    QAction *actionQuit;
    QAction *actionMeasure;
    QAction *actionPolePoint;
    QAction *actionCenter;
    QAction *actionCentroid;
    QAction *actionPCLPossion;
    QAction *actionPCLFast;
    QAction *actionOuliterRemove;
    QAction *actionVoxeGridSimplify;
    QAction *actionUniformSimplify;
    QAction *actionUpSamplify;
    QAction *actionComputeNormal;
    QAction *actionSmoothNormal;
    QAction *actionOriginICP;
    QAction *actionNDTICP;
    QAction *actionOpenMesh;
    QAction *actionSaveMesh;
    QAction *actionCloudMessage;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QVTKWidget *pclviewerwidget;
    QProgressBar *progressBar;
    QMenuBar *menuBar;
    QMenu *menu;
    QMenu *menu_2;
    QMenu *menuSimplify;
    QMenu *menu_3;
    QMenu *menu_4;
    QMenu *menu_5;
    QMenu *menuAutoICP;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QDockWidget *filedockWidget;
    QWidget *dockWidgetContents;
    QTreeWidget *treeWidget;
    QDockWidget *paradockWidget;
    QWidget *dockWidgetContents_2;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;

    void setupUi(QMainWindow *ZeroClass)
    {
        if (ZeroClass->objectName().isEmpty())
            ZeroClass->setObjectName(QStringLiteral("ZeroClass"));
        ZeroClass->resize(776, 554);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(ZeroClass->sizePolicy().hasHeightForWidth());
        ZeroClass->setSizePolicy(sizePolicy);
        actionOpenCloud = new QAction(ZeroClass);
        actionOpenCloud->setObjectName(QStringLiteral("actionOpenCloud"));
        actionSaveCloud = new QAction(ZeroClass);
        actionSaveCloud->setObjectName(QStringLiteral("actionSaveCloud"));
        actionQuit = new QAction(ZeroClass);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        actionMeasure = new QAction(ZeroClass);
        actionMeasure->setObjectName(QStringLiteral("actionMeasure"));
        actionPolePoint = new QAction(ZeroClass);
        actionPolePoint->setObjectName(QStringLiteral("actionPolePoint"));
        actionCenter = new QAction(ZeroClass);
        actionCenter->setObjectName(QStringLiteral("actionCenter"));
        actionCentroid = new QAction(ZeroClass);
        actionCentroid->setObjectName(QStringLiteral("actionCentroid"));
        actionPCLPossion = new QAction(ZeroClass);
        actionPCLPossion->setObjectName(QStringLiteral("actionPCLPossion"));
        actionPCLFast = new QAction(ZeroClass);
        actionPCLFast->setObjectName(QStringLiteral("actionPCLFast"));
        actionOuliterRemove = new QAction(ZeroClass);
        actionOuliterRemove->setObjectName(QStringLiteral("actionOuliterRemove"));
        actionVoxeGridSimplify = new QAction(ZeroClass);
        actionVoxeGridSimplify->setObjectName(QStringLiteral("actionVoxeGridSimplify"));
        actionUniformSimplify = new QAction(ZeroClass);
        actionUniformSimplify->setObjectName(QStringLiteral("actionUniformSimplify"));
        actionUpSamplify = new QAction(ZeroClass);
        actionUpSamplify->setObjectName(QStringLiteral("actionUpSamplify"));
        actionComputeNormal = new QAction(ZeroClass);
        actionComputeNormal->setObjectName(QStringLiteral("actionComputeNormal"));
        actionSmoothNormal = new QAction(ZeroClass);
        actionSmoothNormal->setObjectName(QStringLiteral("actionSmoothNormal"));
        actionOriginICP = new QAction(ZeroClass);
        actionOriginICP->setObjectName(QStringLiteral("actionOriginICP"));
        actionNDTICP = new QAction(ZeroClass);
        actionNDTICP->setObjectName(QStringLiteral("actionNDTICP"));
        actionOpenMesh = new QAction(ZeroClass);
        actionOpenMesh->setObjectName(QStringLiteral("actionOpenMesh"));
        actionSaveMesh = new QAction(ZeroClass);
        actionSaveMesh->setObjectName(QStringLiteral("actionSaveMesh"));
        actionCloudMessage = new QAction(ZeroClass);
        actionCloudMessage->setObjectName(QStringLiteral("actionCloudMessage"));
        centralWidget = new QWidget(ZeroClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        pclviewerwidget = new QVTKWidget(centralWidget);
        pclviewerwidget->setObjectName(QStringLiteral("pclviewerwidget"));

        verticalLayout->addWidget(pclviewerwidget);

        progressBar = new QProgressBar(centralWidget);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setMaximum(10000);
        progressBar->setValue(0);

        verticalLayout->addWidget(progressBar);


        horizontalLayout->addLayout(verticalLayout);

        ZeroClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(ZeroClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 776, 23));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        menu_2 = new QMenu(menuBar);
        menu_2->setObjectName(QStringLiteral("menu_2"));
        menuSimplify = new QMenu(menu_2);
        menuSimplify->setObjectName(QStringLiteral("menuSimplify"));
        menu_3 = new QMenu(menuBar);
        menu_3->setObjectName(QStringLiteral("menu_3"));
        menu_4 = new QMenu(menuBar);
        menu_4->setObjectName(QStringLiteral("menu_4"));
        menu_5 = new QMenu(menuBar);
        menu_5->setObjectName(QStringLiteral("menu_5"));
        menuAutoICP = new QMenu(menu_5);
        menuAutoICP->setObjectName(QStringLiteral("menuAutoICP"));
        ZeroClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(ZeroClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        ZeroClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(ZeroClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        ZeroClass->setStatusBar(statusBar);
        filedockWidget = new QDockWidget(ZeroClass);
        filedockWidget->setObjectName(QStringLiteral("filedockWidget"));
        filedockWidget->setMinimumSize(QSize(220, 220));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        treeWidget = new QTreeWidget(dockWidgetContents);
        treeWidget->setObjectName(QStringLiteral("treeWidget"));
        treeWidget->setGeometry(QRect(9, 9, 200, 200));
        sizePolicy.setHeightForWidth(treeWidget->sizePolicy().hasHeightForWidth());
        treeWidget->setSizePolicy(sizePolicy);
        treeWidget->setColumnCount(0);
        filedockWidget->setWidget(dockWidgetContents);
        ZeroClass->addDockWidget(static_cast<Qt::DockWidgetArea>(1), filedockWidget);
        paradockWidget = new QDockWidget(ZeroClass);
        paradockWidget->setObjectName(QStringLiteral("paradockWidget"));
        paradockWidget->setMinimumSize(QSize(150, 150));
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName(QStringLiteral("dockWidgetContents_2"));
        gridLayoutWidget = new QWidget(dockWidgetContents_2);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 10, 201, 181));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setSizeConstraint(QLayout::SetMinimumSize);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        paradockWidget->setWidget(dockWidgetContents_2);
        ZeroClass->addDockWidget(static_cast<Qt::DockWidgetArea>(1), paradockWidget);

        menuBar->addAction(menu->menuAction());
        menuBar->addAction(menu_2->menuAction());
        menuBar->addAction(menu_5->menuAction());
        menuBar->addAction(menu_3->menuAction());
        menuBar->addAction(menu_4->menuAction());
        menu->addAction(actionOpenCloud);
        menu->addAction(actionOpenMesh);
        menu->addSeparator();
        menu->addAction(actionSaveCloud);
        menu->addAction(actionSaveMesh);
        menu->addSeparator();
        menu->addAction(actionQuit);
        menu_2->addAction(menuSimplify->menuAction());
        menu_2->addAction(actionOuliterRemove);
        menu_2->addAction(actionUpSamplify);
        menu_2->addAction(actionComputeNormal);
        menu_2->addAction(actionSmoothNormal);
        menuSimplify->addAction(actionVoxeGridSimplify);
        menuSimplify->addAction(actionUniformSimplify);
        menu_3->addAction(actionPCLPossion);
        menu_3->addAction(actionPCLFast);
        menu_4->addAction(actionMeasure);
        menu_4->addAction(actionPolePoint);
        menu_4->addAction(actionCenter);
        menu_4->addAction(actionCentroid);
        menu_4->addAction(actionCloudMessage);
        menu_5->addAction(menuAutoICP->menuAction());
        menuAutoICP->addAction(actionOriginICP);
        menuAutoICP->addAction(actionNDTICP);

        retranslateUi(ZeroClass);

        QMetaObject::connectSlotsByName(ZeroClass);
    } // setupUi

    void retranslateUi(QMainWindow *ZeroClass)
    {
        ZeroClass->setWindowTitle(QApplication::translate("ZeroClass", "Zero", 0));
        actionOpenCloud->setText(QApplication::translate("ZeroClass", "\346\211\223\345\274\200\347\202\271\344\272\221", 0));
        actionSaveCloud->setText(QApplication::translate("ZeroClass", "\344\277\235\345\255\230\347\202\271\344\272\221", 0));
        actionQuit->setText(QApplication::translate("ZeroClass", "\351\200\200\345\207\272", 0));
        actionMeasure->setText(QApplication::translate("ZeroClass", "\346\265\213\350\267\235", 0));
        actionPolePoint->setText(QApplication::translate("ZeroClass", "\346\236\201\347\202\271", 0));
        actionCenter->setText(QApplication::translate("ZeroClass", "\347\202\271\344\272\221\344\270\255\345\277\203", 0));
        actionCentroid->setText(QApplication::translate("ZeroClass", "\347\202\271\344\272\221\350\264\250\345\277\203", 0));
        actionPCLPossion->setText(QApplication::translate("ZeroClass", "\346\263\212\346\235\276\351\207\215\345\273\272", 0));
        actionPCLFast->setText(QApplication::translate("ZeroClass", "\345\277\253\351\200\237\351\207\215\345\273\272", 0));
        actionOuliterRemove->setText(QApplication::translate("ZeroClass", "\347\247\273\351\231\244\347\246\273\347\276\244\347\202\271", 0));
        actionVoxeGridSimplify->setText(QApplication::translate("ZeroClass", "\345\235\207\345\214\200\347\262\276\347\256\200", 0));
        actionUniformSimplify->setText(QApplication::translate("ZeroClass", "\347\273\237\344\270\200\347\262\276\347\256\200", 0));
        actionUpSamplify->setText(QApplication::translate("ZeroClass", "\344\270\212\351\207\207\346\240\267", 0));
        actionComputeNormal->setText(QApplication::translate("ZeroClass", "\350\256\241\347\256\227\346\263\225\345\220\221\351\207\217", 0));
        actionSmoothNormal->setText(QApplication::translate("ZeroClass", "\345\205\211\346\273\221\346\263\225\345\220\221\351\207\217", 0));
        actionOriginICP->setText(QApplication::translate("ZeroClass", "ICP", 0));
        actionNDTICP->setText(QApplication::translate("ZeroClass", "NDT", 0));
        actionOpenMesh->setText(QApplication::translate("ZeroClass", "\346\211\223\345\274\200\344\270\211\350\247\222\347\275\221\346\240\274", 0));
        actionSaveMesh->setText(QApplication::translate("ZeroClass", "\344\277\235\345\255\230\344\270\211\350\247\222\347\275\221\346\240\274", 0));
        actionCloudMessage->setText(QApplication::translate("ZeroClass", "\347\202\271\344\272\221\344\277\241\346\201\257", 0));
        menu->setTitle(QApplication::translate("ZeroClass", "\346\226\207\344\273\266", 0));
        menu_2->setTitle(QApplication::translate("ZeroClass", "\351\242\204\345\244\204\347\220\206", 0));
        menuSimplify->setTitle(QApplication::translate("ZeroClass", "\347\262\276\347\256\200", 0));
        menu_3->setTitle(QApplication::translate("ZeroClass", "\351\207\215\345\273\272", 0));
        menu_4->setTitle(QApplication::translate("ZeroClass", "\345\267\245\345\205\267", 0));
        menu_5->setTitle(QApplication::translate("ZeroClass", "\351\205\215\345\207\206", 0));
        menuAutoICP->setTitle(QApplication::translate("ZeroClass", "AutoICP", 0));
    } // retranslateUi

};

namespace Ui {
    class ZeroClass: public Ui_ZeroClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ZERO_H
