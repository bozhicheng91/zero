#include "stdafx.h"
#include "zero.h"

static int opencloudfile_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int openmeshfile_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int savecloud_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int savemesh_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int voxelgridsim_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int uniformsim_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int outlierremove_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int upsamp_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int computenormal_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int smoothnormal_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int originicp_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int ndticp_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int pclpossison_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int pclfast_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int measure_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int polepoint_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int center_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static int centroid_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功

Zero::Zero(QWidget *parent)
	: QMainWindow(parent),
	ui(new Ui::ZeroClass)
{
	ui->setupUi(this);
	this->setWindowTitle(QString::fromLocal8Bit("三维ZERO"));

	// 初始化
	Init();

	// 信号与槽
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

	// 文件目录初始化
	ui->treeWidget->setColumnCount(1);
	ui->treeWidget->setHeaderLabel(QStringLiteral("场景"));

	// 显示器初始化
	m_pclviewer.reset(new PCLViewer("pclviewer", false));
	m_pclviewer->setBackgroundColor(0.5, 0.5, 1.0);
	m_pclviewer->addText("", 5, 20, "Text");
	// Widget初始化
	ui->pclviewerwidget->SetRenderWindow(m_pclviewer->getRenderWindow());
	m_pclviewer->setupInteractor(ui->pclviewerwidget->GetInteractor(), ui->pclviewerwidget->GetRenderWindow());
	ui->pclviewerwidget->update();
	qApp->processEvents();
}

void Zero::Signals_Slots()
{
	// 打开点云文件
	connect(ui->actionOpenCloud, SIGNAL(triggered()), this, SLOT(OpenCloudFileTriggered()));
	// 打开点云文件
	connect(ui->actionOpenMesh, SIGNAL(triggered()), this, SLOT(OpenMeshFileTriggered()));
	// 保存文件
	connect(ui->actionSaveCloud, SIGNAL(triggered()), this, SLOT(OpenCloudTriggered()));
	// 保存文件
	connect(ui->actionSaveMesh, SIGNAL(triggered()), this, SLOT(OpenMeshTriggered()));
	// 退出
	connect(ui->actionQuit, SIGNAL(triggered()), this, SLOT(close()));

	// 均匀精简
	connect(ui->actionVoxeGridSimplify, SIGNAL(triggered()), this, SLOT(VoxelGridSimplifyTriggered()));
	// 统一精简
	connect(ui->actionUniformSimplify, SIGNAL(triggered()), this, SLOT(UniformSimplifyTriggered()));
	// 上采样
	connect(ui->actionUpSamplify, SIGNAL(triggered()), this, SLOT(UpSamplifyTriggered()));
	// 计算法向量
	connect(ui->actionComputeNormal, SIGNAL(triggered()), this, SLOT(ComputerNormalTriggered()));
	// 光滑法向量
	connect(ui->actionSmoothNormal, SIGNAL(triggered()), this, SLOT(SmoothNormalTriggered()));

	// 自动ICP
	connect(ui->actionOriginICP, SIGNAL(triggered()), this, SLOT(OriginICPTriggered()));
	// 基于NDT的ICP
	connect(ui->actionNDTICP, SIGNAL(triggered()), this, SLOT(NDTICPTriggered()));

	// 泊松重建
	connect(ui->actionPCLPossion, SIGNAL(triggered()), this, SLOT(PCLPossisonTriggered()));
	// 快速重建
	connect(ui->actionPCLFast, SIGNAL(triggered()), this, SLOT(PCLFastTriggered()));

	// 测距
	connect(ui->actionMeasure, SIGNAL(triggered()), this, SLOT(MeasureTriggered()));
	// 极点
	connect(ui->actionPolePoint, SIGNAL(triggered()), this, SLOT(PolePointTriggered()));
	// 中心
	connect(ui->actionCenter, SIGNAL(triggered()), this, SLOT(CenterTriggered()));
	// 质心
	connect(ui->actionCentroid, SIGNAL(triggered()), this, SLOT(CentroidTriggered()));
	// 点云信息
	connect(ui->actionCloudMessage, SIGNAL(triggered()), this, SLOT(CloudMessageTriggered()));

	// 刷新所选索引
	connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
		this, SLOT(IndexChoseClicked(QTreeWidgetItem *, int)));

	// 更新状态栏
	connect(m_statusBartimer, SIGNAL(timeout()), this, SLOT(RefreshStarbar()));
	m_statusBartimer->start(100);
}

void Zero::opencloudfilethread()
{
	m_opreator_index = 1;
	opencloudfile_success = 0;
	m_pclviewer->removeAllShapes();

	// 点云读取
	for (size_t i = 0; i < m_openfile_list.size(); i++)
	{
		PCTRGB::Ptr cloud_tmp(new PCTRGB);
		int return_status;
		std::string file = m_zhcode->fromUnicode(m_openfile_list[i]).data();
		// 如果endsWith第二个参数是默认的Qt::CaseSensitive，则区分大小写，否则Qt::CaseInsensitive不区分
		if (m_openfile_list[i].endsWith(".pcd", Qt::CaseInsensitive))
			return_status = pcl::io::loadPCDFile(file, *cloud_tmp);
		// toStdString返回std::string类型
		if (m_openfile_list[i].endsWith(".ply", Qt::CaseInsensitive))
			return_status = pcl::io::loadPLYFile(file, *cloud_tmp);
		if (m_openfile_list[i].endsWith(".asc", Qt::CaseInsensitive))
			return_status = zero::zeroio::LoadASC(file, *cloud_tmp);
		if (m_openfile_list[i].endsWith(".gpd", Qt::CaseInsensitive))
			return_status = zero::zeroio::LoadGPD(file, *cloud_tmp);

		m_clouds.push_back(cloud_tmp);
		if (return_status != 0)
		{
			ui->statusBar->clearMessage();
			m_ss << "无法打开点云文件" << m_openfile_list[i].toStdString() << std::endl;
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			opencloudfile_success = -1;
			return;
		}
		string modeltype = "cloud";
		m_models[i] = modeltype;

		// 显示点云
		m_ss.str("");
		m_ss << "cloud" << i;
		m_pclviewer->addPointCloud(cloud_tmp, m_ss.str());
		m_pclviewer->resetCamera();
	}

	opencloudfile_success = 1;
}

void Zero::openmeshfilethread()
{
	m_pclviewer->removeAllShapes();
	m_opreator_index = 2;
	openmeshfile_success = 0;
	// mesh 读取
	for (size_t i = 0; i < m_openfile_list.size(); i++)
	{
		pcl::PolygonMesh::Ptr mesh_tmp(new pcl::PolygonMesh);
		bool return_status = true;
		std::string file = m_zhcode->fromUnicode(m_openfile_list[i]).data();
		// 如果endsWith第二个参数是默认的Qt::CaseSensitive，则区分大小写，否则Qt::CaseInsensitive不区分
		// toStdString返回std::string类型
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
			m_ss << "无法打开 mesh 文件" << m_openfile_list[i].toStdString() << std::endl;
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			openmeshfile_success = -1;
			return;
		}
		string modeltype = "mesh";
		m_models[i] = modeltype;

		// 显示mesh
		m_ss.str("");
		m_ss << "mesh" << i;
		m_pclviewer->addPolygonMesh(*mesh_tmp, m_ss.str());
		m_pclviewer->resetCamera();
	}

	openmeshfile_success = 1;
}

void Zero::savecloudthread()
{
	m_opreator_index = 3;
	savecloud_success = 0;


	savecloud_success = 1;
}

void Zero::savemeshthread()
{
	m_opreator_index = 4;
	savemesh_success = 0;


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

void Zero::pclpossisonthread()
{
	m_opreator_index = 13;
	pclpossison_success = 0;

	PCT::Ptr cloud(new PCT);
	PCTRGB2PCT(*m_clouds[m_choose_cloud_index], *cloud);
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	if (zero::zerosurface::PCLPossionReconstruct(cloud, *mesh) != 0)
	{
		pclpossison_success = -1;
		return;
	}

	m_meshs.push_back(mesh);
	int n = GetModelTypeCount("mesh");
	m_ss.str(""); 
	m_ss << "mesh" << n;
	m_pclviewer->addPolygonMesh(*mesh, m_ss.str());

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

void Zero::OpenCloudFileTriggered()
{
	m_openfile_list.clear();

	// 对于给定的widget，该函数提供了一个模态文件对话框
	// 如果parent不为0，对话框将显示在父窗口的中心位置
	m_openfile_list = QFileDialog::getOpenFileNames(this,  // QWidget *parent
		QStringLiteral("打开一个或多个点云文件"), // 
		m_currentPath, // 打开的文件路径，如果想打开多个文件，利用;;分割
		tr("data(*.pcd *.ply *.gpd *.asc)"),// 显示打开的文件格式
		&QString(""));

	if (m_openfile_list.isEmpty())
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QString::fromLocal8Bit("文件名为空！"));
		m_log_message = m_zhcode->fromUnicode(QString("文件名为空！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	m_models.clear();
	std::map<int, string>().swap(m_models);
	m_pclviewer->removeAllPointClouds();
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

	ui->progressBar->setRange(0, m_openfile_list.size());
	ui->statusBar->clearMessage();
	ui->statusBar->showMessage(QString::fromLocal8Bit("正在读取数据..."));
	m_log_message = m_zhcode->fromUnicode(QString("正在读取数据...")).data();
	WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);

	std::thread opencloudthread(&Zero::opencloudfilethread, this);
	opencloudthread.detach();

	// 增加文件目录
	for (int i = 0; i < m_openfile_list.size(); i++)
	{
		QStringList strlist = m_openfile_list[i].split(QRegExp("[\\/.]"));
		QString name = strlist[strlist.size() - 2];
		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(name));
		item->setCheckState(0, Qt::Checked);
	}
	ui->treeWidget->expandAll();
}

void Zero::OpenMeshFileTriggered()
{
	m_openfile_list.clear();

	// 对于给定的widget，该函数提供了一个模态文件对话框
	// 如果parent不为0，对话框将显示在父窗口的中心位置
	m_openfile_list = QFileDialog::getOpenFileNames(this,  // QWidget *parent
		QStringLiteral("打开一个或多个三角网格文件"), // 
		m_currentPath, // 打开的文件路径，如果想打开多个文件，利用;;分割
		tr("data(*.stl *.ply *.vtk)"),// 显示打开的文件格式
		&QString(""));

	if (m_openfile_list.isEmpty())
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QString::fromLocal8Bit("文件名为空！"));
		m_log_message = m_zhcode->fromUnicode(QString("文件名为空！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	int mesh_count = GetModelTypeCount("mesh");
	for (int i = 0; i < mesh_count; i++)
	{
		m_ss.str("");
		m_ss << "mesh" << i;
		m_pclviewer->removePolygonMesh(m_ss.str());
	}
	m_models.clear();
	std::map<int, string>().swap(m_models);

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
	ui->statusBar->showMessage(QString::fromLocal8Bit("正在读取数据..."));
	m_log_message = m_zhcode->fromUnicode(QString("正在读取数据...")).data();
	WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);

	std::thread openmeshthread(&Zero::openmeshfilethread, this);
	openmeshthread.detach();

	// 增加文件目录
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

}

void Zero::SaveMeshTriggered()
{

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
		ui->statusBar->showMessage(QString::fromLocal8Bit("重建时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QString("重建时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	ui->statusBar->clearMessage();
	ui->statusBar->showMessage(QString::fromLocal8Bit("正在重建 ..."));
	std::thread meshtd(&Zero::pclpossisonthread, this);
	meshtd.detach();
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
			if (mtype.compare("cloud") == 0)
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

	// 打开点云文件
	if (m_opreator_index == 1)
	{
		if (opencloudfile_success == 0)
		{			
			ui->pclviewerwidget->update();
		}		
		if (opencloudfile_success == 1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("读取数据结束!"));
			m_log_message = m_zhcode->fromUnicode(QString("读取数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			opencloudfile_success = 0;
		}
		if (opencloudfile_success == -1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("读取数据失败!"));
			m_log_message = m_zhcode->fromUnicode(QString("读取数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			opencloudfile_success = 0;
		}
	}

	// 打开三角网格文件
	if (m_opreator_index == 2)
	{
		if (openmeshfile_success == 0)
		{
			ui->pclviewerwidget->update();
		}
		if (openmeshfile_success == 1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("读取数据结束!"));
			m_log_message = m_zhcode->fromUnicode(QString("读取数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			openmeshfile_success = 0;
		}
		if (openmeshfile_success == -1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("读取数据失败!"));
			m_log_message = m_zhcode->fromUnicode(QString("读取数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			openmeshfile_success = 0;
		}
	}





	// PCL泊松重建
	// 打开三角网格文件
	if (m_opreator_index == 13)
	{
		if (pclpossison_success == 0)
		{
			ui->pclviewerwidget->update();
		}
		if (pclpossison_success == 1)
		{
			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "mesh";
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("泊松重建结束!"));
			m_log_message = m_zhcode->fromUnicode(QString("泊松重建结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			pclpossison_success = 0;
		}
		if (pclpossison_success == -1)
		{
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("泊松重建失败!"));
			m_log_message = m_zhcode->fromUnicode(QString("泊松重建结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_opreator_index = 0;
			pclpossison_success = 0;
		}
	}
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
