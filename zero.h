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
	void savecloudthread(std::string filename);
	void savemeshthread(std::string filename);
	void voxelgridsimplifythread(double scale);
	void uniformsimplifythread();
	void outlierremovethread(int k, double threshold ,bool outlier_flag = false);
	void upsamplifythread(double kr , double ur , double stepsize );
	void computernormalthread();
	void smoothnormalthread();
	void originicpthread(int iterations, bool flag);
	void ndticpthread();
	void pclpossisonthread(int k, double r, bool flag, double scale);
	void pclfastthread(int k, double r, double scale, double d, bool flag);
	void measurethread();
	void polepointthread();
	void centerthread();
	void centroidthread();
	void cloudmessagethread();
	void DeleteModel();

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
	void YesTriggered();
	void NoTriggered();

protected:
	void keyPressEvent(QKeyEvent *keyevent);
	void keyReleaseEvent(QKeyEvent *keyevent);
	
	// 创建参数面板
protected:
	void VoxelGridSimplifyPanel();
	void UniformSimplifyPanel();
	void OutlierRemovePanel();
	void UpSamplifyPanel();
	void ComputerNormalPanel();
	void SmoothNormalPanel();
	void OriginICPPanel();
	void NDTICPPanel();
	void PCLPossionPanel();
	void PCLFastPanel();
	void YesButtonClicked();
	void NoButtonClicked();
	void ClearLayout(QLayout *layout);
	QLabel *AddLabel(std::string labelname, std::string text);
	QDoubleSpinBox *AddDoubleSpinBox(std::string objectname, double min, double max, double defaultvalue);
	QSpinBox *AddSpinBox(std::string objectname, int min, int max, int defaultvalue);
	QCheckBox *AddCheckBox(std::string objectname);
	QLineEdit *AddLineEdit(std::string objectname);
	void AddYesNoButton(int i);

private:
	void EmptyDataViewer();
	int GetModelTypeCount(std::string modeltype);
	void PCTRGB2PCT(PCTRGB& cloud_rgb, PCT& cloud);
	static bool endsWith(const std::string& str, const std::string& substr);
	void AddFilelist();
	

private:
	Ui::ZeroClass *ui;
	// PCL显示器
	PCLViewer::Ptr m_pclviewer;
	// 视图中添加的模型及类型
	std::map<int, string> m_models;
	// 所选择的模型索引
	int m_choose_model_index;
	// 当前操作点云索引
	int m_choose_cloud_index;
	// 多个点云
	std::vector<PCTRGB::Ptr> m_clouds;
	// 当前操作三角网格索引
	int m_choose_mesh_index;
	// 多个三角网格
	std::vector<pcl::PolygonMesh::Ptr> m_meshs;
	// 打开文件的列表
	QStringList m_openfile_list;
	// 极大值点、极小值点、质心、中心
	PCTRGB::Ptr m_points;

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

	QSpinBox *m_spinbox1;
	QDoubleSpinBox *m_doublespinbox1;
	QDoubleSpinBox *m_doublespinbox2;
	QDoubleSpinBox *m_doublespinbox3;
	QLineEdit *m_lineedit1, *m_lineedit2;
	QCheckBox *m_checkbox1;
	QPushButton *m_yesbutton;
	QPushButton *m_nobutton;
};

#endif // ZERO_H
