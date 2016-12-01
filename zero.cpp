#include "stdafx.h"
#include "zero.h"

static int opencloudfile_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int openmeshfile_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int savecloud_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int savemesh_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int voxelgridsim_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int uniformsim_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int outlierremove_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int upsamp_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int computenormal_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int smoothnormal_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int originicp_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int ndticp_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int pclpossison_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int pclfast_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int measure_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int polepoint_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int center_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static int centroid_success = -1; // -1 ��ʾʧ�ܣ�0 ��ʾ�����У� 1 ��ʾ�ɹ�
static bool global_flag = false;
static bool yesflag = false;
static bool noflag = false;

Zero::Zero(QWidget *parent)
	: QMainWindow(parent),
	ui(new Ui::ZeroClass)
{
	ui->setupUi(this);
	this->setWindowTitle(QString::fromLocal8Bit("��άZERO"));

	// ��ʼ��
	Init();

	// �ź����
	Signals_Slots();
}

Zero::~Zero()
{
	delete ui;
}

void Zero::Init()
{
	m_zhcode = QTextCodec::codecForName("gb18030");
	m_statusBartimer = new QTimer(this);
	m_currentPath = QApplication::applicationDirPath();
	m_choose_model_index = 0;
	m_choose_cloud_index = 0;
	m_choose_mesh_index = 0;
	m_opreator_index = 0;

	// �ļ�Ŀ¼��ʼ��
	ui->treeWidget->setColumnCount(1);
	ui->treeWidget->setHeaderLabel(QStringLiteral("ģ��"));

	// ��ʾ����ʼ��
	m_pclviewer.reset(new PCLViewer("pclviewer", false));
	m_pclviewer->setBackgroundColor(0.3, 0.3, 1);
	m_pclviewer->addText("", 5, 20, "Text");
	// Widget��ʼ��
	ui->filedockWidget->setWindowTitle(QStringLiteral("�ṹ��"));
	ui->filedockWidget->hide();
	ui->paradockWidget->hide();
	ui->pclviewerwidget->SetRenderWindow(m_pclviewer->getRenderWindow());
	m_pclviewer->setupInteractor(ui->pclviewerwidget->GetInteractor(), ui->pclviewerwidget->GetRenderWindow());
	ui->pclviewerwidget->update();
	qApp->processEvents();
}

void Zero::Signals_Slots()
{
	// �򿪵����ļ�
	connect(ui->actionOpenCloud, SIGNAL(triggered()), this, SLOT(OpenCloudFileTriggered()));
	// �� mesh �ļ�
	connect(ui->actionOpenMesh, SIGNAL(triggered()), this, SLOT(OpenMeshFileTriggered()));
	// ��������ļ�
	connect(ui->actionSaveCloud, SIGNAL(triggered()), this, SLOT(SaveCloudTriggered()));
	// ���� mesh �ļ�
	connect(ui->actionSaveMesh, SIGNAL(triggered()), this, SLOT(SaveMeshTriggered()));
	// �˳�
	connect(ui->actionQuit, SIGNAL(triggered()), this, SLOT(close()));

	// ���Ⱦ���
	connect(ui->actionVoxeGridSimplify, SIGNAL(triggered()), this, SLOT(VoxelGridSimplifyTriggered()));
	// ͳһ����
	connect(ui->actionUniformSimplify, SIGNAL(triggered()), this, SLOT(UniformSimplifyTriggered()));
	// �ϲ���
	connect(ui->actionUpSamplify, SIGNAL(triggered()), this, SLOT(UpSamplifyTriggered()));
	// ���㷨����
	connect(ui->actionComputeNormal, SIGNAL(triggered()), this, SLOT(ComputerNormalTriggered()));
	// �⻬������
	connect(ui->actionSmoothNormal, SIGNAL(triggered()), this, SLOT(SmoothNormalTriggered()));

	// �Զ�ICP
	connect(ui->actionOriginICP, SIGNAL(triggered()), this, SLOT(OriginICPTriggered()));
	// ����NDT��ICP
	connect(ui->actionNDTICP, SIGNAL(triggered()), this, SLOT(NDTICPTriggered()));

	// �����ؽ�
	connect(ui->actionPCLPossion, SIGNAL(triggered()), this, SLOT(PCLPossisonTriggered()));
	// �����ؽ�
	connect(ui->actionPCLFast, SIGNAL(triggered()), this, SLOT(PCLFastTriggered()));

	// ���
	connect(ui->actionMeasure, SIGNAL(triggered()), this, SLOT(MeasureTriggered()));
	// ����
	connect(ui->actionPolePoint, SIGNAL(triggered()), this, SLOT(PolePointTriggered()));
	// ����
	connect(ui->actionCenter, SIGNAL(triggered()), this, SLOT(CenterTriggered()));
	// ����
	connect(ui->actionCentroid, SIGNAL(triggered()), this, SLOT(CentroidTriggered()));
	// ������Ϣ
	connect(ui->actionCloudMessage, SIGNAL(triggered()), this, SLOT(CloudMessageTriggered()));

	// ˢ����ѡ����
	connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
		this, SLOT(IndexChoseClicked(QTreeWidgetItem *, int)));

	// ����״̬��
	connect(m_statusBartimer, SIGNAL(timeout()), this, SLOT(RefreshStarbar()));
	m_statusBartimer->start(100);
}

void Zero::opencloudfilethread()
{
	m_opreator_index = 1;
	opencloudfile_success = 0;
	//ui->progressBar->setRange(0,10000);
	m_progressBarValue = 100;
	// ���ƶ�ȡ
	m_progressBarState = 1;
	for (size_t i = 0; i < m_openfile_list.size(); i++)
	{
		PCTRGB::Ptr cloud_tmp(new PCTRGB);
		int return_status;
		std::string file = m_zhcode->fromUnicode(m_openfile_list[i]).data();
		// ���endsWith�ڶ���������Ĭ�ϵ�Qt::CaseSensitive�������ִ�Сд������Qt::CaseInsensitive������
		if (m_openfile_list[i].endsWith(".pcd", Qt::CaseInsensitive))
			return_status = pcl::io::loadPCDFile(file, *cloud_tmp);
		// toStdString����std::string����
		if (m_openfile_list[i].endsWith(".ply", Qt::CaseInsensitive))
			return_status = pcl::io::loadPLYFile(file, *cloud_tmp);
		//if (m_openfile_list[i].endsWith(".asc", Qt::CaseInsensitive))
		//	return_status = zero::zeroio::LoadASC(file, *cloud_tmp);
		if (m_openfile_list[i].endsWith(".gpd", Qt::CaseInsensitive))
			return_status = zero::zeroio::LoadGPD(file, *cloud_tmp);

		m_clouds.push_back(cloud_tmp);
		if (return_status != 0)
		{
			m_ss << "�޷��򿪵����ļ�" << m_openfile_list[i].toStdString() << std::endl;
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			opencloudfile_success = -1;
			return;
		}
		string modeltype = "cloud";
		m_models[i] = modeltype;

	
	}
	m_progressBarState = 2;

	opencloudfile_success = 1;
	
	
}

void Zero::openmeshfilethread()
{
	m_opreator_index = 2;
	openmeshfile_success = 0;
	// mesh ��ȡ
	for (size_t i = 0; i < m_openfile_list.size(); i++)
	{
		pcl::PolygonMesh::Ptr mesh_tmp(new pcl::PolygonMesh);
		bool return_status = true;
		std::string file = m_zhcode->fromUnicode(m_openfile_list[i]).data();
		// ���endsWith�ڶ���������Ĭ�ϵ�Qt::CaseSensitive�������ִ�Сд������Qt::CaseInsensitive������
		// toStdString����std::string����
		if (m_openfile_list[i].endsWith(".ply", Qt::CaseInsensitive)
			|| m_openfile_list[i].endsWith(".stl", Qt::CaseInsensitive)
			|| m_openfile_list[i].endsWith(".vtk", Qt::CaseInsensitive))
		{
			if (pcl::io::loadPolygonFile(file, *mesh_tmp) < 0)
			{
				return_status = false;
			}
		}

		m_meshs.push_back(mesh_tmp);
		if (!return_status)
		{
			m_ss << "�޷��� mesh �ļ�" << m_openfile_list[i].toStdString() << std::endl;
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			openmeshfile_success = -1;
			return;
		}
		string modeltype = "mesh";
		m_models[i] = modeltype;
	}

	openmeshfile_success = 1;
}

void Zero::savecloudthread(std::string filename)
{
	m_opreator_index = 3;
	savecloud_success = 0;

	int return_status = 1;
	if (endsWith(filename, ".pcd"))
	{
		return_status = pcl::io::savePCDFile(filename, *m_clouds[m_choose_cloud_index], true);
	}
	if (endsWith(filename, ".ply"))
	{
		return_status = pcl::io::savePLYFile(filename, *m_clouds[m_choose_cloud_index], true);
	}

	if (return_status != 0)
	{
		savecloud_success = -1;
	}

	savecloud_success = 1;
}

void Zero::savemeshthread(std::string filename)
{
	m_opreator_index = 4;
	savemesh_success = 0;

	int return_status = 1;
	if (endsWith(filename, ".stl") || endsWith(filename, ".ply"))
	{
		return_status = pcl::io::savePolygonFile(filename, *m_meshs[m_choose_mesh_index], true);
	}

	if (return_status != 0)
	{
		savemesh_success = -1;
	}

	savemesh_success = 1;
}

void Zero::voxelgridsimplifythread()
{
	m_opreator_index = 5;
	voxelgridsim_success = 0;


	voxelgridsim_success = 1;
}

void Zero::uniformsimplifythread()
{
	m_opreator_index = 6;
	uniformsim_success = 0;


	voxelgridsim_success = 1;
}

void Zero::outlierremovethread()
{
	m_opreator_index = 7;
	outlierremove_success = 0;



	outlierremove_success = 1;
}

void Zero::upsamplifythread()
{
	m_opreator_index = 8;
	upsamp_success = 0;


	upsamp_success = 1;
}

void Zero::computernormalthread()
{
	m_opreator_index = 9;
	computenormal_success = 0;

	computenormal_success = 1;
}

void Zero::smoothnormalthread()
{
	m_opreator_index = 10;
	smoothnormal_success = 0;


	smoothnormal_success = 1;
}

void Zero::originicpthread()
{
	m_opreator_index = 11;
	originicp_success = 0;


	originicp_success = 1;
}

void Zero::ndticpthread()
{
	m_opreator_index = 12;
	ndticp_success = 0;



	ndticp_success = 1;
}

void Zero::pclpossisonthread(int k, double r, bool flag, double scale)
{
	pclpossison_success = 0;

	PCT::Ptr cloud(new PCT);
	PCTRGB2PCT(*m_clouds[m_choose_cloud_index], *cloud);
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	if (zero::zerosurface::PCLPossionReconstruct(cloud, *mesh, k, r, flag, scale) != 0)
	{
		pclpossison_success = -1;
		return;
	}

	m_meshs.push_back(mesh);

	pclpossison_success = 1;
}

void Zero::pclfastthread()
{
	m_opreator_index = 14;
	pclfast_success = 0;


	pclfast_success = 1;
}

void Zero::measurethread()
{
	m_opreator_index = 15;
	measure_success = 0;


	measure_success = 1;
}

void Zero::polepointthread()
{
	m_opreator_index = 16;
	polepoint_success = 0;



	polepoint_success = 1;
}

void Zero::centerthread()
{
	m_opreator_index = 17;
	center_success = 0;
	

	center_success = 1;
}

void Zero::centroidthread()
{
	m_opreator_index = 18;
	centroid_success = 0;


	centroid_success = 1;
}

void Zero::DeleteModel()
{
	if (m_choose_model_index < 0)
	{
		return;
	}

	delete ui->treeWidget->topLevelItem(m_choose_model_index);

	int i;
	string modeltype = m_models[m_choose_model_index];
	if (modeltype.compare("cloud") == 0)
	{
		m_ss.str("");
		m_ss << "cloud" << m_choose_cloud_index;
		m_pclviewer->removePointCloud(m_ss.str());
		ui->pclviewerwidget->update();
		std::vector<PCTRGB::Ptr>::iterator Iter;
		i = 0;
		for (Iter = m_clouds.begin(); Iter != m_clouds.end(); Iter++)
		{
			if (i == m_choose_cloud_index)
			{
				m_clouds[m_choose_cloud_index]->clear();
				m_clouds.erase(Iter);
				break;
			}
			i++;
		}
	}
	if (modeltype.compare("mesh") == 0)
	{
		m_ss.str("");
		m_ss << "mesh" << m_choose_mesh_index;
		m_pclviewer->removePolygonMesh(m_ss.str());
		ui->pclviewerwidget->update();
		std::vector<pcl::PolygonMesh::Ptr>::iterator Iter;
		i = 0;
		for (Iter = m_meshs.begin(); Iter != m_meshs.end(); Iter++)
		{
			if (i == m_choose_mesh_index)
			{
				m_meshs.erase(Iter);
				break;
			}
			i++;
		}
	}
	
	map<int, string>::iterator mapiter;
	i = 0;
	for (mapiter = m_models.begin(); mapiter != m_models.end(); mapiter++)
	{
		if (i == m_choose_model_index)
		{
			m_models.erase(mapiter);
			break;
		}
		i++;
	}
}


/*�ۺ���*/
void Zero::OpenCloudFileTriggered()
{
	m_openfile_list.clear();

	// ���ڸ�����widget���ú����ṩ��һ��ģ̬�ļ��Ի���
	// ���parent��Ϊ0���Ի�����ʾ�ڸ����ڵ�����λ��
	m_openfile_list = QFileDialog::getOpenFileNames(this,  // QWidget *parent
		QStringLiteral("��һ�����������ļ�"), // 
		m_currentPath, // �򿪵��ļ�·���������򿪶���ļ�������;;�ָ�
		tr("data(*.pcd *.ply *.gpd *.asc)"),// ��ʾ�򿪵��ļ���ʽ
		&QString(""));

	if (m_openfile_list.isEmpty())
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QString::fromLocal8Bit("�ļ���Ϊ�գ�"));
		m_log_message = m_zhcode->fromUnicode(QString("�ļ���Ϊ�գ�")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	m_models.clear();
	std::map<int, string>().swap(m_models);
	for (size_t i = 0; i < m_clouds.size(); i++)
	{
		m_clouds[i]->clear();
	}
	std::vector<PCTRGB::Ptr>().swap(m_clouds);
	std::vector<pcl::PolygonMesh::Ptr>().swap(m_meshs);
	//m_pclviewer->removeAllPointClouds();
	m_pclviewer->removeAllShapes();
	ui->pclviewerwidget->update();

	QStringList paths = m_openfile_list[0].split(QRegExp("[\\/]"));
	for (size_t i = 0; i < paths.size() - 1; i++)
	{
		m_currentPath += paths[i];
	}

	while (ui->treeWidget->topLevelItemCount() > 0)
	{
		delete ui->treeWidget->topLevelItem(0);
	}

	//ui->progressBar->setRange(0, m_openfile_list.size());
	//�趨progressBar��ֵ
	
	ui->statusBar->clearMessage();
	ui->statusBar->showMessage(QString::fromLocal8Bit("���ڶ�ȡ����..."));
	m_log_message = m_zhcode->fromUnicode(QString("���ڶ�ȡ����...")).data();
	WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
	
	std::thread opencloudthread(&Zero::opencloudfilethread, this);
	opencloudthread.detach();

	// �����ļ�Ŀ¼
	for (int i = 0; i < m_openfile_list.size(); i++)
	{
		QStringList strlist = m_openfile_list[i].split(QRegExp("[\\/.]"));
		QString name = strlist[strlist.size() - 2];
		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(name));
		item->setCheckState(0, Qt::Checked);
	}
	
	
	
	ui->filedockWidget->show();
	ui->treeWidget->expandAll();
}

void Zero::OpenMeshFileTriggered()
{
	m_openfile_list.clear();

	// ���ڸ�����widget���ú����ṩ��һ��ģ̬�ļ��Ի���
	// ���parent��Ϊ0���Ի�����ʾ�ڸ����ڵ�����λ��
	m_openfile_list = QFileDialog::getOpenFileNames(this,  // QWidget *parent
		QStringLiteral("��һ���������������ļ�"), // 
		m_currentPath, // �򿪵��ļ�·���������򿪶���ļ�������;;�ָ�
		tr("data(*.stl *.ply *.vtk)"),// ��ʾ�򿪵��ļ���ʽ
		&QString(""));

	if (m_openfile_list.isEmpty())
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QString::fromLocal8Bit("�ļ���Ϊ�գ�"));
		m_log_message = m_zhcode->fromUnicode(QString("�ļ���Ϊ�գ�")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	int mesh_count = GetModelTypeCount("mesh");
	m_pclviewer->removeAllShapes();
	ui->pclviewerwidget->update();
	/*for (int i = 0; i < mesh_count; i++)
	{
		m_ss.str("");
		m_ss << "mesh" << i;
		m_pclviewer->removePolygonMesh(m_ss.str());
	}*/
	m_models.clear();
	std::map<int, string>().swap(m_models);
	for (size_t i = 0; i < m_clouds.size(); i++)
	{
		m_clouds[i]->clear();
	}
	std::vector<PCTRGB::Ptr>().swap(m_clouds);
	std::vector<pcl::PolygonMesh::Ptr>().swap(m_meshs);

	QStringList paths = m_openfile_list[0].split(QRegExp("[\\/]"));
	for (size_t i = 0; i < paths.size() - 1; i++)
	{
		m_currentPath += paths[i];
	}

	while (ui->treeWidget->topLevelItemCount() > 0)
	{
		delete ui->treeWidget->topLevelItem(0);
	}

	ui->progressBar->setRange(0, m_openfile_list.size());
	ui->statusBar->clearMessage();
	ui->statusBar->showMessage(QString::fromLocal8Bit("���ڶ�ȡ����..."));
	m_log_message = m_zhcode->fromUnicode(QString("���ڶ�ȡ����...")).data();
	WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);

	std::thread openmeshthread(&Zero::openmeshfilethread, this);
	openmeshthread.detach();

	// �����ļ�Ŀ¼
	for (int i = 0; i < m_openfile_list.size(); i++)
	{
		QStringList strlist = m_openfile_list[i].split(QRegExp("[\\/.]"));
		QString name = strlist[strlist.size() - 2];
		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(name));
		item->setCheckState(0, Qt::Checked);
	}
	ui->treeWidget->expandAll();
}

void Zero::SaveCloudTriggered()
{
	QString file = QFileDialog::getSaveFileName(this,
		QStringLiteral("��������ļ�"),
		m_currentPath,
		tr("data(*.pcd *.ply)"));

	if (file.isEmpty())
	{
		QMessageBox::warning(NULL, QStringLiteral("����"), QStringLiteral("�ļ���Ϊ�գ�"));
		m_log_message = m_zhcode->fromUnicode(QString("�ļ���Ϊ�գ�")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	m_currentPath = file;

	std::string filename = m_zhcode->fromUnicode(file).data();
	std::thread savecloudthd(&Zero::savecloudthread, this, filename);
	savecloudthd.detach();
}

void Zero::SaveMeshTriggered()
{
	QString file = QFileDialog::getSaveFileName(this,
		QStringLiteral("�������������ļ�"),
		m_currentPath,
		tr("data(*.stl *.ply)"));

	if (file.isEmpty())
	{
		QMessageBox::warning(NULL, QStringLiteral("����"), QStringLiteral("�ļ���Ϊ�գ�"));
		m_log_message = m_zhcode->fromUnicode(QString("�ļ���Ϊ�գ�")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	m_currentPath = file;

	std::string filename = m_zhcode->fromUnicode(file).data();
	std::thread savemeshthd(&Zero::savemeshthread, this, filename);
	savemeshthd.detach();
}

void Zero::VoxelGridSimplifyTriggered()
{

}

void Zero::UniformSimplifyTriggered()
{

}

void Zero::OutlierRemoveTriggered()
{

}

void Zero::UpSamplifyTriggered()
{

}

void Zero::ComputerNormalTriggered()
{

}

void Zero::SmoothNormalTriggered()
{

}

void Zero::OriginICPTriggered()
{

}

void Zero::NDTICPTriggered()
{

}

void Zero::PCLPossisonTriggered()
{
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QString::fromLocal8Bit("�ؽ�ʱ�������󣬵��Ƶ���С��3��"));
		m_log_message = m_zhcode->fromUnicode(QString("�ؽ�ʱ�������󣬵��Ƶ���С��3��")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	PCLPossisonPanel();

	m_opreator_index = 13;
}

void Zero::PCLFastTriggered()
{

}

void Zero::MeasureTriggered()
{

}

void Zero::PolePointTriggered()
{

}

void Zero::CenterTriggered()
{

}

void Zero::CentroidTriggered()
{

}

void Zero::CloudMessageTriggered()
{

}

void Zero::IndexChoseClicked(QTreeWidgetItem *item, int count)
{
	m_choose_model_index = ui->treeWidget->indexOfTopLevelItem(item);
	std::string modeltype = m_models[m_choose_model_index];
	m_ss.str("");
	int n = 0;
	if (modeltype.compare("cloud") == 0)
	{
		for (int i = 0; i <= m_choose_model_index; i++)
		{
			std::string mtype = m_models[i];
			if (mtype.compare("cloud") == 0)
			{
				n++;
			}
		}
		m_choose_cloud_index = n - 1;
		m_ss << "cloud" << m_choose_cloud_index;
	}
	if (modeltype.compare("mesh") == 0)
	{
		for (int i = 0; i <= m_choose_model_index; i++)
		{
			std::string mtype = m_models[i];
			if (mtype.compare("mesh") == 0)
			{
				n++;
			}
		}
		m_choose_mesh_index = n - 1;
		m_ss << "mesh" << m_choose_mesh_index;
	}
	

	if (item->checkState(0) == Qt::Unchecked)
	{
		if (modeltype.compare("cloud") == 0)
		{
			m_pclviewer->removePointCloud(m_ss.str());
		}
		if (modeltype.compare("mesh") == 0)
		{
			m_pclviewer->removePolygonMesh(m_ss.str());
		}
	}
	if (item->checkState(0) == Qt::Checked)
	{
		if (modeltype.compare("cloud") == 0)
		{
			m_pclviewer->addPointCloud(m_clouds[m_choose_cloud_index], m_ss.str());
		}
		if (modeltype.compare("mesh") == 0)
		{
			m_pclviewer->addPolygonMesh(*m_meshs[m_choose_mesh_index], m_ss.str());
		}
	}
	m_pclviewer->updateCamera();
}

void Zero::RefreshStarbar()
{
	if (m_opreator_index == 0)
	{
		return;
	}

	// �򿪵����ļ�
	if (m_opreator_index == 1)
	{


		
		if (m_progressBarState == 1 && m_progressBarValue < 7000)
		{

			int pau = 1000;
			//while (pau-- > 0);
			m_progressBarValue = m_progressBarValue + 10;
			ui->progressBar->setValue(m_progressBarValue);
			
		}	

		if (m_progressBarState == 2)
		{
		
			for (; m_progressBarValue < 9999; m_progressBarValue++)
			{
				int pau_2 = 1000;
				//while (pau_2-- > 0);
				ui->progressBar->setValue(m_progressBarValue);
			}
		}


		if (opencloudfile_success == 0)
		{			
			//ui->pclviewerwidget->update();
		}		
		if (opencloudfile_success == 1)
		{
			
			for (int i = 0; i < m_clouds.size(); i++)
			{
				// ��ʾ����
				m_ss.str("");
				m_ss << "cloud" << i;
				m_pclviewer->addPointCloud(m_clouds[i], m_ss.str());
			}
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("��ȡ���ݽ���!"));
			m_log_message = m_zhcode->fromUnicode(QString("��ȡ���ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			opencloudfile_success = 0;
			
			
			m_progressBarValue = 0;
			ui->progressBar->setValue(m_progressBarValue);
		}
		if (opencloudfile_success == -1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("��ȡ����ʧ��!"));
			m_log_message = m_zhcode->fromUnicode(QString("��ȡ���ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			opencloudfile_success = 0;
		}
	}

	// �����������ļ�
	if (m_opreator_index == 2)
	{
		if (openmeshfile_success == 0)
		{
			//ui->pclviewerwidget->update();
		}
		if (openmeshfile_success == 1)
		{
			for (int i = 0; i < m_meshs.size(); i++)
			{
				// ��ʾmesh
				m_ss.str("");
				m_ss << "mesh" << i;
				m_pclviewer->addPolygonMesh(*m_meshs[i], m_ss.str());
			}
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("��ȡ���ݽ���!"));
			m_log_message = m_zhcode->fromUnicode(QString("��ȡ���ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			openmeshfile_success = 0;
		}
		if (openmeshfile_success == -1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("��ȡ����ʧ��!"));
			m_log_message = m_zhcode->fromUnicode(QString("��ȡ���ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			openmeshfile_success = 0;
		}
	}

	// ��������ļ�
	if (m_opreator_index == 3)
	{
		if (savecloud_success == 0)
		{
			;
		}
		if (savecloud_success == 1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("����������ݽ���!"));
			m_log_message = m_zhcode->fromUnicode(QString("����������ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			savecloud_success = 0;
		}
		if (savecloud_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("�����������ʧ��!"));
			m_log_message = m_zhcode->fromUnicode(QString("����������ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			savecloud_success = 0;
		}
	}

	// �������������ļ�
	if (m_opreator_index == 4)
	{
		if (savemesh_success == 0)
		{
			;
		}
		if (savemesh_success == 1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("���������������ݽ���!"));
			m_log_message = m_zhcode->fromUnicode(QString("���������������ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			savemesh_success = 0;
		}
		if (savemesh_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("����������������ʧ��!"));
			m_log_message = m_zhcode->fromUnicode(QString("���������������ݽ���!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			savemesh_success = 0;
		}
	}


	// PCL�����ؽ�
	// �����������ļ�
	if (m_opreator_index == 13)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}

		if (noflag)
		{
			ui->paradockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}
		if (yesflag)
		{
			int k = m_spinbox1->text().toInt();
			double r = 0.0;
			if (k == 0)
			{
				r = m_doublespinbox1->text().toDouble();
			}
			bool flag = true;
			double scale = m_doublespinbox2->text().toDouble();
			if (m_checkbox1->isChecked())
			{
				flag = false;
			}
			ui->paradockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("�����ؽ� ..."));
			std::thread meshtd(&Zero::pclpossisonthread, this, k, r, flag, scale);
			meshtd.detach();
		}
		if (pclpossison_success == 0)
		{
			//ui->pclviewerwidget->update();
		}
		if (pclpossison_success == 1)
		{
			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "mesh";
			
			int n = GetModelTypeCount("mesh");
			m_ss.str("");
			m_ss << "mesh" << n - 1;
			m_pclviewer->addPolygonMesh(*m_meshs[m_meshs.size() - 1], m_ss.str());

			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("�����ؽ�����!"));
			m_log_message = m_zhcode->fromUnicode(QString("�����ؽ�����!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			pclpossison_success = 0;
			global_flag = false;
		}
		if (pclpossison_success == -1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("�����ؽ�ʧ��!"));
			m_log_message = m_zhcode->fromUnicode(QString("�����ؽ�����!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			pclpossison_success = 0;
			global_flag = false;
		}
	}
}

void Zero::YesTriggered()
{
	ui->paradockWidget->hide();
	yesflag = true;
}

void Zero::NoTriggered()
{
	ui->paradockWidget->hide();
	m_opreator_index = 0;
	noflag = true;
}

void Zero::keyPressEvent(QKeyEvent *keyevent)
{
	if (keyevent->key() == Qt::Key_Delete)
	{
	 	DeleteModel();
	}

	QMainWindow::keyPressEvent(keyevent);
}

void Zero::keyReleaseEvent(QKeyEvent *keyevent)
{
	QMainWindow::keyReleaseEvent(keyevent);
}

void Zero::VoxelGridSimplifyPanel()
{

}

void Zero::UniformSimplifyPanel()
{

}

void Zero::OutlierRemovePanel()
{

}

void Zero::UpSamplifyPanel()
{

}

void Zero::ComputerNormalPanel()
{

}

void Zero::SmoothNormalPanel()
{

}

void Zero::OriginICPPanel()
{

}

void Zero::NDTICPPanel()
{

}

void Zero::PCLPossisonPanel()
{
	ClearLayout(ui->gridLayout);

	ui->paradockWidget->setWindowTitle(QStringLiteral("poisson ��������"));

	QLabel *labelk = AddLabel("labelk", "k neighhors", 0, 0);
	m_spinbox1 = AddSpinBox("spinboxk", 0, 100,20, 0, 1);

	QLabel *labelr = AddLabel("labelk", "search radius", 1, 0);
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxr", 0.0, 1000.0, 5, 1, 1);
	

	QLabel *labelserrior = AddLabel("labelserrior", "closure", 2, 0);
	m_checkbox1 = new QCheckBox();
	m_checkbox1->setObjectName("serriorcheckbox");
	m_checkbox1->setText("");
	ui->gridLayout->addWidget(m_checkbox1, 2, 1);

	QLabel *labelscale = AddLabel("labelscale", "clipping factor", 3, 0);
	m_doublespinbox2 = AddDoubleSpinBox("doublespinboxscale", 0.0, 20.0,3, 3, 1);
	//ui->gridLayout->addWidget(labelr, 3, 0);
	//ui->gridLayout->addWidget(m_doublespinbox2, 3, 1);


	QLabel *labelNodeNum = AddLabel("labelNodeNum", "SamplesPerNode", 4, 0);
	m_doublespinbox3 = AddDoubleSpinBox("doublespinboxscale", 0.0, 20.0, 3, 4, 1);

	QLabel *setConfidence = AddLabel("setConfidence", "setConfidence", 5, 0);
	QComboBox *choice_type = new QComboBox();
	choice_type->addItem("True");
	choice_type->addItem("False");
	ui->gridLayout->addWidget(choice_type, 5, 1);

	AddYesNoButton(6);

	ui->paradockWidget->show();
}

void Zero::PCLFastPanel()
{

}



void Zero::NoButtonClicked()
{

}

void Zero::ClearLayout(QLayout *layout)
{
	QLayoutItem *item;
	while ((item = layout->takeAt(0)) != 0)
	{
		if (item->widget())
		{
			delete item->widget();
		}

		QLayout *childLayout = item->layout();
		if (childLayout)
		{
			ClearLayout(childLayout);
		}
		delete item;
	}
}

QLabel *Zero::AddLabel(QString labelname, QString text, int i, int j)
{
	QLabel *label = new QLabel();
	label->setObjectName(labelname);
	label->setText(QString::fromLocal8Bit(text.toLocal8Bit()));
	ui->gridLayout->addWidget(label, i, j,1, 1);
	return label;
}

QDoubleSpinBox *Zero::AddDoubleSpinBox(QString objectname, double min, double max, double defaultValue, int i, int j)
{
	QDoubleSpinBox *doublespinbox = new QDoubleSpinBox();
	doublespinbox->setObjectName(objectname);
	doublespinbox->setMinimum(min);
	doublespinbox->setMaximum(max);
	doublespinbox->setValue(defaultValue);
	ui->gridLayout->addWidget(doublespinbox, i, j); 
	return doublespinbox;
}

QSpinBox *Zero::AddSpinBox(QString objectname, int min, int max,int defaultValue, int i, int j)
{
	QSpinBox *spinbox = new QSpinBox();
	spinbox->setObjectName(objectname);
	spinbox->setMinimum(min);
	spinbox->setMaximum(max);
	spinbox->setValue(defaultValue);
	ui->gridLayout->addWidget(spinbox, i, j);
	return spinbox;
}

void Zero::AddYesNoButton(int i)
{
	m_yesbutton = new QPushButton();
	m_yesbutton->setObjectName("yesbutton");
	m_yesbutton->setText(QStringLiteral("ȷ��"));
	ui->gridLayout->addWidget(m_yesbutton, i, 0);

	m_nobutton = new QPushButton();
	m_nobutton->setObjectName("nobutton");
	m_nobutton->setText(QStringLiteral("ȡ��"));
	ui->gridLayout->addWidget(m_nobutton, i, 1);

	connect(m_yesbutton, SIGNAL(clicked()), this, SLOT(YesTriggered()));
	connect(m_nobutton, SIGNAL(clicked()), this, SLOT(NoTriggered()));
}

int Zero::GetModelTypeCount(std::string modeltype)
{
	int n = 0;
	for (int i = 0; i < m_models.size(); i++)
	{
		std::string mtype = m_models[i];
		if (mtype.compare(modeltype) == 0)
		{
			n++;
		}
	}

	return n;
}

void Zero::PCTRGB2PCT(PCTRGB& cloud_rgb, PCT& cloud)
{
	for (int i = 0; i < cloud_rgb.size(); i++)
	{
		PTRGB p = cloud_rgb.points[i];
		cloud.points.push_back(PT(p.x, p.y, p.z));
	}
}

bool Zero::endsWith(const std::string& str, const std::string& substr)
{
	return str.rfind(substr) == (str.length() - substr.length());
}
