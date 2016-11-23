#include "stdafx.h"
#ifndef ZERO_H
#define ZERO_H

#include <vtkRenderWindow.h>

#include <QtWidgets/QMainWindow>
#include "ui_zero.h"
#include "src/modules/Zero_Common.h"
#include "src/modules/Zero_IO.h"
#include "src/modules/Zero_Pretreatment.h"
#include "src/modules/Zero_Registration.h"
#include "src/modules/Zero_Search.h"
#include "src/modules/Zero_Surface.h"

class Zero : public QMainWindow
{
	Q_OBJECT

public:
	Zero(QWidget *parent = 0);
	~Zero();

private:
	void Init();
	void Signals_Slots();

protected:
	void opencloudfilethread();
	void openmeshfilethread();
	void savecloudthread();
	void savemeshthread();
	void voxelgridsimplifythread();
	void uniformsimplifythread();
	void outlierremovethread();
	void upsamplifythread();
	void computernormalthread();
	void smoothnormalthread();
	void originicpthread();
	void ndticpthread();
	void pclpossisonthread();
	void pclfastthread();
	void measurethread();
	void polepointthread();
	void centerthread();
	void centroidthread();
	void cloudmessagethread();

	protected slots:
	void OpenCloudFileTriggered();
	void OpenMeshFileTriggered();
	void SaveCloudTriggered();
	void SaveMeshTriggered();
	void VoxelGridSimplifyTriggered();
	void UniformSimplifyTriggered();
	void OutlierRemoveTriggered();
	void UpSamplifyTriggered();
	void ComputerNormalTriggered();
	void SmoothNormalTriggered();
	void OriginICPTriggered();
	void NDTICPTriggered();
	void PCLPossisonTriggered();
	void PCLFastTriggered();
	void MeasureTriggered();
	void PolePointTriggered();
	void CenterTriggered();
	void CentroidTriggered();
	void CloudMessageTriggered();
	void IndexChoseClicked(QTreeWidgetItem *item, int count);
	void RefreshStarbar();
	

private:
	Ui::ZeroClass *ui;
	// PCL显示器
	boost::shared_ptr<PCLViewer> m_pclviewer;
	// 视图中添加的点云计数
	int m_model_count;
	// 打开点云
	bool m_opencloud;
	// 当前操作点云索引
	int m_choose_cloud_index;
	// 多个点云
	std::vector<PCTRGB::Ptr> m_clouds;
	// 打开mesh
	bool m_openmesh;
	// 当前操作三角网格索引
	int m_choose_mesh_index;
	// 多个三角网格
	std::vector<pcl::PolygonMesh::Ptr> m_meshs;
	// 打开文件的列表
	QStringList m_openfile_list;

	// 操作索引
	int m_opreator_index;
	// 当前路径
	QString m_currentPath;
	// 中文编码，支持中文路径
	QTextCodec *m_zhcode;
	// 状态栏刷新计时器
	QTimer *m_statusBartimer;
	// 数据流定向
	std::stringstream m_ss;
	// 日志信息
	std::string m_log_message;

};

#endif // ZERO_H
