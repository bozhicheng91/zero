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
	// PCL��ʾ��
	boost::shared_ptr<PCLViewer> m_pclviewer;
	// ��ͼ�����ӵĵ��Ƽ���
	int m_model_count;
	// �򿪵���
	bool m_opencloud;
	// ��ǰ������������
	int m_choose_cloud_index;
	// �������
	std::vector<PCTRGB::Ptr> m_clouds;
	// ��mesh
	bool m_openmesh;
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

};

#endif // ZERO_H