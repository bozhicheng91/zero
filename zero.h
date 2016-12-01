#include "stdafx.h"
#ifndef ZERO_H
#define ZERO_H

#include <vtkRenderWindow.h>

#include <QtWidgets/QMainWindow>
#include <QDebug>
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
	void voxelgridsimplifythread();
	void uniformsimplifythread();
	void outlierremovethread();
	void upsamplifythread();
	void computernormalthread();
	void smoothnormalthread();
	void originicpthread();
	void ndticpthread();
	void pclpossisonthread(int k, double r, bool flag, double scale);
	void pclfastthread();
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
	
	// �����������
protected:
	void VoxelGridSimplifyPanel();
	void UniformSimplifyPanel();
	void OutlierRemovePanel();
	void UpSamplifyPanel();
	void ComputerNormalPanel();
	void SmoothNormalPanel();
	void OriginICPPanel();
	void NDTICPPanel();
	void PCLPossisonPanel();
	void PCLFastPanel();
	void YesButtonClicked();
	void NoButtonClicked();
	void ClearLayout(QLayout *layout);
	QLabel *AddLabel(QString labelname, QString text, int i, int j);
	QDoubleSpinBox *AddDoubleSpinBox(QString objectname, double min, double max, double defaultValue, int i, int j);
	QSpinBox *AddSpinBox(QString objectname, int min, int max, int defaultValue, int i, int j);
	void AddYesNoButton(int i);

private:
	int GetModelTypeCount(std::string modeltype);
	void PCTRGB2PCT(PCTRGB& cloud_rgb, PCT& cloud);
	static bool endsWith(const std::string& str, const std::string& substr);
	

private:
	Ui::ZeroClass *ui;
	// PCL��ʾ��
	boost::shared_ptr<PCLViewer> m_pclviewer;
	// ��ͼ����ӵ�ģ�ͼ�����
	std::map<int, string> m_models;
	// ��ѡ���ģ������
	int m_choose_model_index;
	// ��ǰ������������
	int m_choose_cloud_index;
	// �������
	std::vector<PCTRGB::Ptr> m_clouds;
	// ��ǰ����������������
	int m_choose_mesh_index;
	// �����������
	std::vector<pcl::PolygonMesh::Ptr> m_meshs;
	// ���ļ����б�
	QStringList m_openfile_list;

	// ��������
	int m_opreator_index;
	// ��ǰ·��
	QString m_currentPath;
	// ���ı��룬֧������·��
	QTextCodec *m_zhcode;
	// ״̬��ˢ�¼�ʱ��
	QTimer *m_statusBartimer;
	// ����������
	std::stringstream m_ss;
	// ��־��Ϣ
	std::string m_log_message;
	
	//progressBar message
	int m_progressBarValue;
	int m_progressBarState;

	QSpinBox *m_spinbox1;
	QDoubleSpinBox *m_doublespinbox1;
	QDoubleSpinBox *m_doublespinbox2;
	QDoubleSpinBox *m_doublespinbox3;
	QCheckBox *m_checkbox1;
	QPushButton *m_yesbutton;
	QPushButton *m_nobutton;
};

#endif // ZERO_H
