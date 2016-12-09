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
static int cloudmessage_success = -1; // -1 表示失败，0 表示运行中， 1 表示成功
static bool global_flag = false;
static bool yesflag = false;
static bool noflag = false;

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
	ui->treeWidget->setHeaderLabel(QStringLiteral("模型"));

	m_points.reset(new PCTRGB);
	// 显示器初始化
	m_pclviewer.reset(new PCLViewer("pclviewer", false));
	m_pclviewer->setBackgroundColor(0.5, 0.5, 1.0);
	m_pclviewer->addText("", 5, 20, "Text");
	// Widget初始化
	ui->parameterdockWidget->hide();
	ui->pclviewerwidget->SetRenderWindow(m_pclviewer->getRenderWindow());
	m_pclviewer->setupInteractor(ui->pclviewerwidget->GetInteractor(), ui->pclviewerwidget->GetRenderWindow());
	ui->pclviewerwidget->update();
	qApp->processEvents();
}

void Zero::Signals_Slots()
{
	// 打开点云文件
	connect(ui->actionOpenCloud, SIGNAL(triggered()), this, SLOT(OpenCloudFileTriggered()));
	// 打开 mesh 文件
	connect(ui->actionOpenMesh, SIGNAL(triggered()), this, SLOT(OpenMeshFileTriggered()));
	// 保存点云文件
	connect(ui->actionSaveCloud, SIGNAL(triggered()), this, SLOT(SaveCloudTriggered()));
	// 保存 mesh 文件
	connect(ui->actionSaveMesh, SIGNAL(triggered()), this, SLOT(SaveMeshTriggered()));
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
	//移除离群点
	connect(ui->actionOuliterRemove, SIGNAL(triggered()), this, SLOT(OutlierRemoveTriggered()));
	// 自动ICP
	connect(ui->actionOriginICP, SIGNAL(triggered()), this, SLOT(OriginICPTriggered()));
	// 基于NDT的ICP
	connect(ui->actionNDTICP, SIGNAL(triggered()), this, SLOT(NDTICPTriggered()));

	// 泊松重建
	connect(ui->actionPCLPossion, SIGNAL(triggered()), this, SLOT(PCLPossisonTriggered()));
	// 快速重建
	connect(ui->actionPCLFast, SIGNAL(triggered()), this, SLOT(PCLFastTriggered()));

	// 测距
	connect(ui->actionMeasureD, SIGNAL(triggered()), this, SLOT(MeasureTriggered()));
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
	opencloudfile_success = 0;

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
			m_ss << "无法打开点云文件" << m_openfile_list[i].toStdString() << std::endl;
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			opencloudfile_success = -1;
			return;
		}
		string modeltype = "cloud";
		m_models[i] = modeltype;
	}

	opencloudfile_success = 1;
}

void Zero::openmeshfilethread()
{
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
	}

	openmeshfile_success = 1;
}

void Zero::savecloudthread(std::string filename)
{
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

void Zero::voxelgridsimplifythread(double scale)
{
	
	PCTRGB::Ptr cloud(new PCTRGB);
	if (zero::zeropretreatment::VoxelGridSimplify(*m_clouds[m_choose_cloud_index], *cloud, scale) != 0)
	{
		voxelgridsim_success = -1;
		return;
	}
	m_clouds.push_back(cloud);
	voxelgridsim_success = 1;
}

void Zero::uniformsimplifythread(double r)
{
	
	PCTRGB::Ptr cloud(new PCTRGB);
	if (zero::zeropretreatment::UniformSimplify(*m_clouds[m_choose_cloud_index], *cloud, r) != 0)
	{
		uniformsim_success = -1;
		return;
	}
	m_clouds.push_back(cloud);
	uniformsim_success = 1;
}

void Zero::outlierremovethread(int k, double threshold, bool outlier_flag )
{
	PCTRGB::Ptr cloud(new PCTRGB);
	//PCT *cloud(new PCT);
	//PCTRGB2PCT(*m_clouds[m_choose_cloud_index], *cloud);
	if (zero::zeropretreatment::StaticalPOutlierRemoval(*m_clouds[m_choose_cloud_index], *cloud, k, threshold, outlier_flag) != 0)
	{
		outlierremove_success = -1;
		return;
	}
	m_clouds.push_back(cloud);
	outlierremove_success = 1;
}

void Zero::upsamplifythread(double kr = 0.01,double ur = 0.01,double stepsize = 0.01)
{
	PCTRGB::Ptr cloud(new PCTRGB);

	if (zero::zeropretreatment::UpSampling(*m_clouds[m_choose_cloud_index], *cloud, kr, ur, stepsize) != 0)
	{
		outlierremove_success = -1;
		return;
	}
	m_clouds.push_back(cloud);
	upsamp_success = 1;
	
}

void Zero::computernormalthread(int k, double r)
{
	
	PCTN::Ptr normal(new PCTN);
	PCTRGBN::Ptr cloud_with_normal(new PCTRGBN);
	if (zero::zeropretreatment::ComputeCloudNormal(*m_clouds[m_choose_cloud_index], *normal, k, r) != 0)
	{
		computenormal_success = -1;
		return;
	}

	pcl::concatenateFields(*m_clouds[m_choose_cloud_index], *normal, *cloud_with_normal);
	m_clouds_with_normals.push_back(cloud_with_normal);
	computenormal_success = 1;
}

void Zero::smoothnormalthread(bool normal_f, bool polynomialfit, double r)
{
	PCTRGBN::Ptr cloud_with_normal(new PCTRGBN);
	//PCT::Ptr cloud(new PCT);
	//PCTRGB2PCT(*m_clouds[m_choose_cloud_index], *cloud);

	if (zero::zeropretreatment::SmoothingNormal(*m_clouds[m_choose_cloud_index], *cloud_with_normal, normal_f, polynomialfit, r) != 0)
	{
		smoothnormal_success = -1;
		return;
	}
	m_clouds_with_normals.push_back(cloud_with_normal);
	smoothnormal_success = 1;
}

void Zero::originicpthread(int iterations, bool flag)
{
	if (flag)
	{
		zero::zeroregistration::PretreatSourceTargetICP(*m_clouds[0], *m_clouds[1], iterations);
	}
	else
	{
		zero::zeroregistration::PretreatSourceTargetICP(*m_clouds[1], *m_clouds[0], iterations);
	}
	
	originicp_success = 1;
}

void Zero::ndticpthread()
{
	ndticp_success = 0;



	ndticp_success = 1;
}

void Zero::pclpossisonthread(int k, double r, bool flag, double scale)
{
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

void Zero::pclfastthread(int k, double r, double scale, double d, bool flag)
{
	PCT::Ptr cloud(new PCT);
	PCTRGB2PCT(*m_clouds[m_choose_cloud_index], *cloud);
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	if (zero::zerosurface::SpeedTriangulation(cloud, *mesh, k, r, scale, d, flag) != 0)
	{
		pclfast_success = -1;
		return;
	}

	m_meshs.push_back(mesh);

	pclfast_success = 1;
}

void Zero::measurethread()
{
	measure_success = 0;


	measure_success = 1;
}

void Zero::polepointthread()
{
	if (m_clouds[m_choose_cloud_index]->size() == 0)
	{
		polepoint_success = -1;
	}
	PTRGB polemax, polemin;
	zero::zerocommon::ComputeMinMax(*m_clouds[m_choose_cloud_index], polemin, polemax);
	m_points->push_back(polemax);
	m_points->push_back(polemin);

	polepoint_success = 1;
}

void Zero::centerthread()
{
	if (m_clouds[m_choose_cloud_index]->size() == 0)
	{
		center_success = -1;
	}
	PTRGB center;
	zero::zerocommon::Compute3dCenter(*m_clouds[m_choose_cloud_index], center);
	m_points->push_back(center);
	center_success = 1;
}

void Zero::centroidthread()
{
	if (m_clouds[m_choose_cloud_index]->size() == 0)
	{
		centroid_success = -1;
	}
	Eigen::Vector4f centroid;
	zero::zerocommon::Compute3dCentroid(*m_clouds[m_choose_cloud_index], centroid);
	PTRGB Centroid;
	Centroid.x = centroid[0];
	Centroid.y = centroid[1];
	Centroid.z = centroid[2];
	Centroid.r = 0;
	Centroid.g = 0;
	Centroid.b = 0;

	m_points->push_back(Centroid);

	centroid_success = 1;
}

void Zero::cloudmessagethread()
{
	if (m_clouds[m_choose_cloud_index]->size() == 0)
	{
		cloudmessage_success = -1;
	}

	PTRGB polemax, polemin;
	zero::zerocommon::ComputeMinMax(*m_clouds[m_choose_cloud_index], polemin, polemax);
	m_points->push_back(polemax);
	m_points->push_back(polemin);

	PTRGB center;
	zero::zerocommon::Compute3dCenter(*m_clouds[m_choose_cloud_index], center);
	m_points->push_back(center);

	Eigen::Vector4f centroid;
	zero::zerocommon::Compute3dCentroid(*m_clouds[m_choose_cloud_index], centroid);
	PTRGB Centroid;
	Centroid.x = centroid[0];
	Centroid.y = centroid[0];
	Centroid.z = centroid[0];
	Centroid.r = 0;
	Centroid.g = 0;
	Centroid.b = 0;
	m_points->push_back(Centroid);

	cloudmessage_success = 1;

	ofstream fout("./CloudMessage.txt");
	fout << "\n";
	fout << "点云极大值：(" << m_points->points[0].x << "," << m_points->points[0].y << "," << m_points->points[0].z << ")\n";
	fout << "    极小值：(" << m_points->points[1].x << "," << m_points->points[1].y << "," << m_points->points[1].z << ")\n";
	fout << "      中心：(" << m_points->points[2].x << "," << m_points->points[2].y << "," << m_points->points[2].z << ")\n";
	fout << "      质心：(" << m_points->points[3].x << "," << m_points->points[3].y << "," << m_points->points[3].z << ")\n";
	fout.close();
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
	if (modeltype.compare("cloud") == 0)//所选索引为点模型
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
	if (modeltype.compare("mesh") == 0)//所选索引为网格模型
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
	
	if (modeltype.compare("normal") == 0)//所选索引为法向模型
	{
		m_ss.str("");
		m_ss << "normal" << m_choose_normal_index;
		m_pclviewer->removePointCloud(m_ss.str());
		ui->pclviewerwidget->update();
		std::vector<PCTRGBN::Ptr>::iterator Iter;
		i = 0;
		for (Iter = m_clouds_with_normals.begin(); Iter != m_clouds_with_normals.end(); Iter++)
		{
			if (i == m_choose_normal_index)
			{
				m_clouds_with_normals.erase(Iter);
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
		ui->statusBar->showMessage(QStringLiteral("文件名为空！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("文件名为空！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}	

	QStringList paths = m_openfile_list[0].split(QRegExp("[\\/]"));
	for (size_t i = 0; i < paths.size() - 1; i++)
	{
		m_currentPath += paths[i];
		m_currentPath += "/";
	}

	EmptyDataViewer();

	ui->progressBar->setRange(0, m_openfile_list.size());
	ui->statusBar->clearMessage();
	ui->statusBar->showMessage(QStringLiteral("正在读取数据..."));
	m_log_message = m_zhcode->fromUnicode(QStringLiteral("正在读取数据...")).data();
	WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);

	m_opreator_index = 1;
	std::thread opencloudthread(&Zero::opencloudfilethread, this);
	opencloudthread.detach();	
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
		ui->statusBar->showMessage(QStringLiteral("文件名为空！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("文件名为空！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	QStringList paths = m_openfile_list[0].split(QRegExp("[\\/]"));
	for (size_t i = 0; i < paths.size() - 1; i++)
	{
		m_currentPath += paths[i];
		m_currentPath += "/";
	}
	EmptyDataViewer();

	ui->progressBar->setRange(0, m_openfile_list.size());
	ui->statusBar->clearMessage();
	ui->statusBar->showMessage(QStringLiteral("正在读取数据..."));
	m_log_message = m_zhcode->fromUnicode(QStringLiteral("正在读取数据...")).data();
	WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);

	m_opreator_index = 2;
	std::thread openmeshthread(&Zero::openmeshfilethread, this);
	openmeshthread.detach();
}

void Zero::SaveCloudTriggered()
{
	QString file = QFileDialog::getSaveFileName(this,
		QStringLiteral("保存点云文件"),
		m_currentPath,
		tr("data(*.pcd *.ply)"));

	if (file.isEmpty())
	{
		QMessageBox::warning(NULL, QStringLiteral("警告"), QStringLiteral("文件名为空！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("文件名为空！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	m_currentPath = file;

	m_opreator_index = 3;
	std::string filename = m_zhcode->fromUnicode(file).data();
	std::thread savecloudthd(&Zero::savecloudthread, this, filename);
	savecloudthd.detach();	
}

void Zero::SaveMeshTriggered()
{
	QString file = QFileDialog::getSaveFileName(this,
		QStringLiteral("保存三角网格文件"),
		m_currentPath,
		tr("data(*.stl *.ply)"));

	if (file.isEmpty())
	{
		QMessageBox::warning(NULL, QStringLiteral("警告"), QStringLiteral("文件名为空！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("文件名为空！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	m_currentPath = file;

	m_opreator_index = 4;
	std::string filename = m_zhcode->fromUnicode(file).data();
	std::thread savemeshthd(&Zero::savemeshthread, this, filename);
	savemeshthd.detach();
}

void Zero::VoxelGridSimplifyTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("均匀精简时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("均匀精简时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	m_opreator_index = 5;
	VoxelGridSimplifyPanel();
}

void Zero::UniformSimplifyTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("统一精简时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("统一精简时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}
	m_opreator_index = 6;
	UniformSimplifyPanel();
}

void Zero::OutlierRemoveTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("移除离散点时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("移除离散点时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}
	m_opreator_index = 7;
	OutlierRemovePanel();
	
}

void Zero::UpSamplifyTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("上采样时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("上采样时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}
	m_opreator_index = 8;
	UpSamplifyPanel();
}

void Zero::ComputerNormalTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("法向估计时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("法向估计时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}
	m_opreator_index = 9;
	ComputerNormalPanel();
}

void Zero::SmoothNormalTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("光滑点云法向时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("光滑点云法向时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}
	m_opreator_index = 10;
	SmoothNormalPanel();
}

void Zero::OriginICPTriggered()
{
	if (m_clouds.size() <= 1)
	{
		return;
	}
	if (m_clouds[0]->size() <= 3 || m_clouds[1]->size() <= 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("配准时发生错误，点云点数过少！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("配准时发生错误，点云点数过少！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	OriginICPPanel();
	m_opreator_index = 11;
}

void Zero::NDTICPTriggered()
{

	m_opreator_index = 12;
}

void Zero::PCLPossisonTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("重建时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("重建时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	PCLPossionPanel();
	m_opreator_index = 13;
}

void Zero::PCLFastTriggered()
{
	if (m_clouds.size() == 0)
	{
		return;
	}
	if (m_clouds[m_choose_cloud_index]->size() < 3)
	{
		ui->statusBar->clearMessage();
		ui->statusBar->showMessage(QStringLiteral("重建时发生错误，点云点数小于3！"));
		m_log_message = m_zhcode->fromUnicode(QStringLiteral("重建时发生错误，点云点数小于3！")).data();
		WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		return;
	}

	PCLFastPanel();
	m_opreator_index = 14;
}

void Zero::MeasureTriggered()
{

	m_opreator_index = 15;
}

void Zero::PolePointTriggered()
{
	polepoint_success = 0;
	m_opreator_index = 16;

	m_points->clear();
	std::thread polepointthd(&Zero::polepointthread, this);
	polepointthd.detach();	
}

void Zero::CenterTriggered()
{
	center_success = 0;
	m_opreator_index = 17;

	m_points->clear();
	std::thread centerthd(&Zero::centerthread, this);
	centerthd.detach();
}

void Zero::CentroidTriggered()
{
	centroid_success = 0; 
	m_opreator_index = 18;

	m_points->clear();
	std::thread centroidthd(&Zero::centroidthread, this);
	centroidthd.detach();
}

void Zero::CloudMessageTriggered()
{
	cloudmessage_success = 0;
	m_opreator_index = 19;

	m_points->clear();
	std::thread cloudmessagethd(&Zero::cloudmessagethread, this);
	cloudmessagethd.detach();	
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
	if (modeltype.compare("normal") == 0)
	{
		for (int i = 0; i <= m_choose_model_index; i++)
		{
			std::string mtype = m_models[i];
			if (mtype.compare("normal") == 0)
			{
				n++;
			}
		}
		m_choose_normal_index = n - 1;
		m_ss << "normal" << m_choose_normal_index;
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
		if (modeltype.compare("normal") == 0)
		{
			m_pclviewer->removePointCloud(m_ss.str());
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
		if (modeltype.compare("normal") == 0)
		{
			m_pclviewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(m_clouds_with_normals[m_choose_normal_index]->makeShared(), 10, m_meanDistance, m_ss.str());
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
			return;
		}		
		if (opencloudfile_success == 1)
		{
			AddFilelist();
			for (int i = 0; i < m_clouds.size(); i++)
			{
				// 显示点云
				m_ss.str("");
				m_ss << "cloud" << i;
				m_pclviewer->addPointCloud(m_clouds[i], m_ss.str());
			}
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("读取数据结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("读取数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (opencloudfile_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("读取数据失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("读取数据失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		opencloudfile_success = -1;
		return;
	}

	// 打开三角网格文件
	if (m_opreator_index == 2)
	{
		if (openmeshfile_success == 0)
		{
			return;
		}
		if (openmeshfile_success == 1)
		{
			AddFilelist();
			for (int i = 0; i < m_meshs.size(); i++)
			{
				// 显示mesh
				m_ss.str("");
				m_ss << "mesh" << i;
				m_pclviewer->addPolygonMesh(*m_meshs[i], m_ss.str());
			}
			m_pclviewer->resetCamera();
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("读取数据结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("读取数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (openmeshfile_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("读取数据失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("读取数据失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		openmeshfile_success = -1;
		return;
	}

	// 保存点云文件
	if (m_opreator_index == 3)
	{
		if (savecloud_success == 0)
		{
			return;
		}
		if (savecloud_success == 1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("保存点云数据结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("保存点云数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (savecloud_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("保存点云数据失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("保存点云数据失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		savecloud_success = -1;
		return;
	}

	// 保存三角网格文件
	if (m_opreator_index == 4)
	{
		if (savemesh_success == 0)
		{
			return;
		}
		if (savemesh_success == 1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("保存三角网格数据结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("保存三角网格数据结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (savemesh_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("保存三角网格数据失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("保存三角网格数据失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		savemesh_success = -1;
		return;
	}

	//均匀精简
	if (m_opreator_index == 5)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}
		if (noflag)
		{
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}
		if (yesflag)
		{
			double scale = m_doublespinbox1->text().toDouble();
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在均匀精简 ..."));
			std::thread meshtd(&Zero::voxelgridsimplifythread, this, scale);
			meshtd.detach();
		}
		if (voxelgridsim_success == 0)
		{
			return;
		}
		if (voxelgridsim_success == 1)
		{
			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "cloud";


			int n = GetModelTypeCount("cloud");
			m_ss.str("");
			m_ss << "cloud" << n - 1;
			m_pclviewer->addPointCloud(m_clouds[m_clouds.size() - 1], m_ss.str());

			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("均匀精简结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("均匀精简结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (voxelgridsim_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("均匀精简失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("均匀精简失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);


		}

		m_opreator_index = 0;
		voxelgridsim_success = -1;
		global_flag = false;
		return;

	}
	//统一精简
	if (m_opreator_index == 6)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}
		if (noflag)
		{
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}
		if (yesflag)
		{
			double r = m_doublespinbox1->text().toDouble();
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在统一精简 ..."));
			std::thread meshtd(&Zero::uniformsimplifythread, this, r);
			meshtd.detach();
		}
		if (uniformsim_success == 0)
		{
			return;
		}
		if (uniformsim_success == 1)
		{
			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "cloud";


			int n = GetModelTypeCount("cloud");
			m_ss.str("");
			m_ss << "cloud" << n - 1;
			m_pclviewer->addPointCloud(m_clouds[m_clouds.size() - 1], m_ss.str());

			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("统一精简结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("统一精简结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (uniformsim_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("统一精简失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("统一精简失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);


		}

		m_opreator_index = 0;
		uniformsim_success = -1;
		global_flag = false;
		return;




	}
	// 移除离群点
	if (m_opreator_index == 7)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}
		if (noflag)
		{
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}
		if (yesflag)
		{
			int k = m_spinbox1->text().toInt();
			double threshold = m_doublespinbox1->text().toDouble();
			bool outlier_flag;
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在移除离群点 ..."));
			std::thread meshtd(&Zero::outlierremovethread, this, k, threshold, outlier_flag );
			meshtd.detach();
		}
		if (outlierremove_success == 0)
		{
			return;
		}
		if (outlierremove_success == 1)
		{
			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "cloud";


			int n = GetModelTypeCount("cloud");
			m_ss.str("");
			m_ss << "cloud" << n - 1;
			m_pclviewer->addPointCloud(m_clouds[m_clouds.size() - 1], m_ss.str());

			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("移除离群点结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("移除离群点结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (outlierremove_success == -1)
		{
				ui->statusBar->clearMessage();
				ui->statusBar->showMessage(QStringLiteral("移除离群点失败!"));
				m_log_message = m_zhcode->fromUnicode(QStringLiteral("移除离群点失败!")).data();
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);

				
		}

		m_opreator_index = 0;
		outlierremove_success = -1;
		global_flag = false;
		return;
	}
	//上采样
	if (m_opreator_index == 8)
	{

		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}
		if (noflag)
		{
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}

		if (yesflag)
		{
			double kr = m_doublespinbox1->text().toDouble();
			double ur = m_doublespinbox2->text().toDouble();
			double step = m_doublespinbox3->text().toDouble();
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在对所选点集进行上采样 ..."));
			std::thread meshtd(&Zero::upsamplifythread, this, kr, ur,step);
			meshtd.detach();
		}
		if (upsamp_success == 0)
		{
			return;
		}
		if (upsamp_success == 1)
		{

			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "cloud";


			int n = GetModelTypeCount("cloud");
			m_ss.str("");
			m_ss << "cloud" << n - 1;
			m_pclviewer->addPointCloud(m_clouds[m_clouds.size() - 1], m_ss.str());

			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("上采样结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("上采样结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}

		if (upsamp_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("上采样失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("上采样")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);


		}
		m_opreator_index = 0;
		upsamp_success = -1;
		global_flag = false;
		return;
		

	}
	//计算法向
	if (m_opreator_index == 9)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}

		if (noflag)
		{
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}
		if (yesflag)
		{
			double k = m_spinbox1->text().toInt();
			double r = m_doublespinbox1->text().toDouble();
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在对所选点集计算法向 ..."));
			std::thread meshtd(&Zero::computernormalthread, this, k, r);
			meshtd.detach();
		}
		if (computenormal_success == 0)
		{
			return;
		}
		if (computenormal_success == 1)
		{

			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "normal";


			int n = GetModelTypeCount("normal");
			m_ss.str("");
			m_ss << "normal" << n - 1;

			//求出点云平均距离
			m_meanDistance = zero::zerocommon::pointcloudmeand(*m_clouds[m_clouds.size() - 1]);
			
			m_pclviewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(m_clouds_with_normals[m_clouds_with_normals.size() - 1], 10, 5 * m_meanDistance, m_ss.str());

			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("法向估计结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("法向估计结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}

		if (computenormal_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("法向估计失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("法向估计失败")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);


		}
		m_opreator_index = 0;
		computenormal_success = -1;
		global_flag = false;
		return;
	}
	//光顺法向
	if (m_opreator_index == 10)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}

		if (noflag)
		{
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}
		if (yesflag)
		{
			bool norm_flag = false;
			if (m_checkbox1->isChecked())
			{
				norm_flag = true;
			}
			bool polynorm_flag = false;
			if (m_checkbox2->isChecked())
			{
				polynorm_flag = true;
			}
		
			double r = m_doublespinbox1->text().toDouble();
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在对所选点集进行光顺计算 ..."));
			std::thread meshtd(&Zero::smoothnormalthread, this, norm_flag, polynorm_flag, r);
			meshtd.detach();
		}
		if (smoothnormal_success == 0)
		{
			return;
		}
		if (smoothnormal_success == 1)
		{

			QTreeWidgetItem *choose_item = ui->treeWidget->topLevelItem(m_choose_model_index);
			QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget, QStringList(choose_item->text(0)));
			item->setCheckState(0, Qt::Checked);
			m_models[m_models.size()] = "normal";


			int n = GetModelTypeCount("normal");
			m_ss.str("");
			m_ss << "normal" << n - 1;

			//求出点云平均距离
			m_meanDistance = zero::zerocommon::pointcloudmeand(*m_clouds[m_clouds.size() - 1]);

			m_pclviewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(m_clouds_with_normals[m_clouds_with_normals.size() - 1], 10, 5 * m_meanDistance, m_ss.str());

			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("光顺过程结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("光顺过程结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}

		if (smoothnormal_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("光顺过程失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("光顺过程失败")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);


		}
		m_opreator_index = 0;
		smoothnormal_success = -1;
		global_flag = false;
		return;
	}

			
	// ICP配准
	if (m_opreator_index == 11)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}

		if (noflag)
		{
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			m_opreator_index = 0;
			global_flag = false;
			return;
		}
		if (yesflag)
		{
			QTreeWidgetItem *item1 = ui->treeWidget->topLevelItem(0);
			if (item1->text(0) != m_lineedit1->text() && item1->text(0) != m_lineedit1->text())
			{
				ui->statusBar->clearMessage();
				ui->statusBar->showMessage(QStringLiteral("配准点云名称不正确，请检查!"));
				m_log_message = m_zhcode->fromUnicode(QStringLiteral("配准点云名称不正确，请检查!")).data();
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
				return;
			}
			
			bool flag = true;			
			if (item1->text(0) != m_lineedit1->text())
			{
				flag = false;
			}
			int num = m_spinbox1->text().toInt();

			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在配准 ..."));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("正在配准 ...")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			std::thread meshtd(&Zero::originicpthread, this, num, flag);
			meshtd.detach();
		}
		if (originicp_success == 0)
		{
			return;
		}
		if (originicp_success == 1)
		{
			m_pclviewer->updatePointCloud(m_clouds[1], "cloud1");
			ui->pclviewerwidget->update();
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("配准结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("配准结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (originicp_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("配准失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("配准失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		originicp_success = -1;
		global_flag = false;
		return;
	}

	// PCL泊松重建
	if (m_opreator_index == 13)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}

		if (noflag)
		{
			ui->parameterdockWidget->hide();
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
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在重建 ..."));
			std::thread meshtd(&Zero::pclpossisonthread, this, k, r, flag, scale);
			meshtd.detach();
		}
		if (pclpossison_success == 0)
		{
			return;
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
			ui->statusBar->showMessage(QStringLiteral("泊松重建结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("泊松重建结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (pclpossison_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("泊松重建失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("泊松重建失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		pclpossison_success = -1;
		global_flag = false;
		return;
	}

	// 快速三角形重建
	if (m_opreator_index == 14)
	{
		if (!global_flag && !yesflag && !noflag)
		{
			return;
		}

		if (noflag)
		{
			ui->parameterdockWidget->hide();
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
			double scale = m_doublespinbox2->text().toDouble();
			double max_angle = m_doublespinbox3->text().toDouble();
			bool flag = false;
			if (m_checkbox1->isChecked())
			{
				flag = true;
			}
			ui->parameterdockWidget->hide();
			noflag = false;
			yesflag = false;
			global_flag = true;
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("正在重建 ..."));
			std::thread meshtd(&Zero::pclfastthread, this, k, r, scale, max_angle, flag);
			meshtd.detach();
		}
		if (pclfast_success == 0)
		{
			return;
		}
		if (pclfast_success == 1)
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
			ui->statusBar->showMessage(QStringLiteral("快速重建结束!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("快速重建结束!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		if (pclfast_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("快速重建失败!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("快速重建失败!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		pclfast_success = -1;
		global_flag = false;
		return;
	}


	// 极大值极小值点
	if (m_opreator_index == 16)
	{
		if (polepoint_success == 0)
		{
			return;
		}
		if (polepoint_success == 1)
		{
			m_ss.str("");
			m_ss << "极大值点为(" << m_points->points[0].x << "," << m_points->points[0].y << "," << m_points->points[0].z << ")" << ";";
			m_ss << "极小值点为(" << m_points->points[1].x << "," << m_points->points[1].y << "," << m_points->points[1].z << ")";
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit(m_ss.str().c_str()));
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_ss.str("");
			m_ss << "poles" << m_choose_cloud_index;
			m_pclviewer->removePointCloud(m_ss.str());
			pcl::visualization::PointCloudColorHandlerCustom<PTRGB> red(m_points, 255, 0, 0);
			m_pclviewer->addPointCloud(m_points, red, m_ss.str());
			m_pclviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, m_ss.str());
		}
		if (polepoint_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("获取极值点失败，点云点数为0!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("获取极值点失败，点云点数为0!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		polepoint_success = -1;
		return;
	}

	// 中心
	if (m_opreator_index == 17)
	{
		if (center_success == 0)
		{
			return;
		}
		if (center_success == 1)
		{
			m_ss.str("");
			m_ss << "中心点为(" << m_points->points[0].x << "," << m_points->points[0].y << "," << m_points->points[0].z << ")";
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit(m_ss.str().c_str()));
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_ss.str("");
			m_ss << "center" << m_choose_cloud_index;
			m_pclviewer->removePointCloud(m_ss.str());
			pcl::visualization::PointCloudColorHandlerCustom<PTRGB> red(m_points, 255, 0, 0);
			m_pclviewer->addPointCloud(m_points, red, m_ss.str());
			m_pclviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, m_ss.str());
		}
		if (center_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("获取中心点失败，点云点数为0!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("获取中心点失败，点云点数为0!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		center_success = -1;
		return;
	}

	// 质心
	if (m_opreator_index == 18)
	{
		if (centroid_success == 0)
		{
			return;
		}
		if (centroid_success == 1)
		{
			m_ss.str("");
			m_ss << "质心为(" << m_points->points[0].x << "," << m_points->points[0].y << "," << m_points->points[0].z << ")";
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit(m_ss.str().c_str()));
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit(m_ss.str().c_str())).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			m_ss.str("");
			m_ss << "centroid" << m_choose_cloud_index;
			m_pclviewer->removePointCloud(m_ss.str());
			pcl::visualization::PointCloudColorHandlerCustom<PTRGB> red(m_points, 255, 0, 0);
			m_pclviewer->addPointCloud(m_points, red, m_ss.str());
			m_pclviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, m_ss.str());
		}
		if (centroid_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("获取质心失败，点云点数为0!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("获取质心失败，点云点数为0!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		centroid_success = -1;
		return;
	}

	// 点云信息
	if (m_opreator_index == 19)
	{
		if (cloudmessage_success == 0)
		{
			return;
		}
		if (cloudmessage_success == 1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QString::fromLocal8Bit("获取点云信息成功!"));
			m_log_message = m_zhcode->fromUnicode(QString::fromLocal8Bit("获取点云信息成功!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
			QProcess* pro = new QProcess;
			pro->start("notepad.exe", QStringList(QString(".\\CloudMessage.txt")));
		}
		if (cloudmessage_success == -1)
		{
			ui->statusBar->clearMessage();
			ui->statusBar->showMessage(QStringLiteral("获取点云信息失败，点云点数为0!"));
			m_log_message = m_zhcode->fromUnicode(QStringLiteral("获取点云信息失败，点云点数为0!")).data();
			WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", m_log_message);
		}
		m_opreator_index = 0;
		cloudmessage_success = -1;
		return;
	}
}

void Zero::YesTriggered()
{
	ui->parameterdockWidget->hide();
	yesflag = true;
}

void Zero::NoTriggered()
{
	ui->parameterdockWidget->hide();
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
	ClearLayout(ui->gridLayout);
	QLabel *labelscale = AddLabel("labelscale", "精简参数");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 0.5);
	ui->gridLayout->addWidget(labelscale, 0, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 0, 1);

	AddYesNoButton(1);
	voxelgridsim_success = 0;
	ui->parameterdockWidget->show();

}

void Zero::UniformSimplifyPanel()
{
	ClearLayout(ui->gridLayout);
	QLabel *labelr = AddLabel("labelr", "采样半径");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 0.5);
	ui->gridLayout->addWidget(labelr, 0, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 0, 1);

	AddYesNoButton(1);
	uniformsim_success = 0;
	ui->parameterdockWidget->show();
}

void Zero::OutlierRemovePanel()
{
	ClearLayout(ui->gridLayout);

	QLabel *labelk = AddLabel("labelk", "搜索近邻点数");
	ui->gridLayout->addWidget(labelk, 0, 0);
	m_spinbox1 = AddSpinBox("spinboxk", 0, 100, 50);
	ui->gridLayout->addWidget(m_spinbox1, 0, 1);

	QLabel *labelr = AddLabel("labelscale", "离群点判断阈值");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 1.0);
	ui->gridLayout->addWidget(labelr, 1, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 1, 1);

	QLabel *labelsetNegative = AddLabel("labelsetNegative", "保留离群点");
	m_checkbox1 = AddCheckBox("checkboxserrior");
	ui->gridLayout->addWidget(labelsetNegative, 2, 0);
	ui->gridLayout->addWidget(m_checkbox1, 2, 1);

	AddYesNoButton(3);
	outlierremove_success = 0;
	ui->parameterdockWidget->show();
}

void Zero::UpSamplifyPanel()
{
	ClearLayout(ui->gridLayout);
	QLabel *labelkr = AddLabel("labelkr", "近邻搜索半径");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 1.0);
	ui->gridLayout->addWidget(labelkr, 0, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 0, 1);

	QLabel *labelur = AddLabel("labelur", "局部采样平面半径");
	m_doublespinbox2 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 1.0);
	ui->gridLayout->addWidget(labelur, 1, 0);
	ui->gridLayout->addWidget(m_doublespinbox2, 1, 1);


	QLabel *labelstep = AddLabel("labelstep", "采样步长");
	m_doublespinbox3 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 1.0);
	ui->gridLayout->addWidget(labelstep, 2, 0);
	ui->gridLayout->addWidget(m_doublespinbox3, 2, 1);

	AddYesNoButton(3);
	upsamp_success = 0;
	ui->parameterdockWidget->show();

}

void Zero::ComputerNormalPanel()
{
	ClearLayout(ui->gridLayout);
	QLabel *labelk = AddLabel("labelk", "搜索近邻点数");
	ui->gridLayout->addWidget(labelk, 0, 0);
	m_spinbox1 = AddSpinBox("spinboxk", 0, 100, 20);
	ui->gridLayout->addWidget(m_spinbox1, 0, 1);

	QLabel *labelr = AddLabel("labelscale", "搜索半径");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 1.0);
	ui->gridLayout->addWidget(labelr, 1, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 1, 1);

	AddYesNoButton(2);

	computenormal_success = 0;
	ui->parameterdockWidget->show();

}

void Zero::SmoothNormalPanel()
{
	ClearLayout(ui->gridLayout);
	QLabel *labelkr = AddLabel("labelkr", "近邻搜索半径");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 1.0);
	ui->gridLayout->addWidget(labelkr, 0, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 0, 1);

	QLabel *labelnormal_f = AddLabel("labelnormal_f", "是否保存法向信息");
	m_checkbox1 = AddCheckBox("labelnormal_f");
	ui->gridLayout->addWidget(labelnormal_f, 1, 0);
	ui->gridLayout->addWidget(m_checkbox1, 1, 1);

	QLabel *labelpolynomialfit_f = AddLabel("labelpolynomialfit_f", "是否封闭");
	m_checkbox2 = AddCheckBox("labelpolynomialfit_f");
	ui->gridLayout->addWidget(labelpolynomialfit_f, 2, 0);
	ui->gridLayout->addWidget(m_checkbox2, 2, 1);

	AddYesNoButton(3);
	smoothnormal_success = 0;
	m_opreator_index = 10;
	ui->parameterdockWidget->show();

}

void Zero::OriginICPPanel()
{
	ClearLayout(ui->gridLayout);

	QLabel *labelsource = AddLabel("labelsource", "固定点云");
	ui->gridLayout->addWidget(labelsource, 0, 0);
	m_lineedit1 = AddLineEdit("sourcecloud");
	ui->gridLayout->addWidget(m_lineedit1, 0, 1);

	QLabel *labeltarget = AddLabel("labelk", "变换点云");
	ui->gridLayout->addWidget(labeltarget, 1, 0);
	m_lineedit2 = AddLineEdit("targetcloud");
	ui->gridLayout->addWidget(m_lineedit2, 1, 1);

	QLabel *labelnum = AddLabel("labelnum", "最大迭代次数");
	ui->gridLayout->addWidget(labelnum, 2, 0);
	m_spinbox1 = AddSpinBox("spinboxnum", 0, 10000, 100);
	ui->gridLayout->addWidget(m_spinbox1, 2, 1);

	AddYesNoButton(3);

	originicp_success = 0;
	ui->parameterdockWidget->show();
}

void Zero::NDTICPPanel()
{

}

void Zero::PCLPossionPanel()
{
	ClearLayout(ui->gridLayout);

	QLabel *labelk = AddLabel("labelk", "搜索近邻点数");
	ui->gridLayout->addWidget(labelk, 0, 0);
	m_spinbox1 = AddSpinBox("spinboxk", 0, 100, 30);
	ui->gridLayout->addWidget(m_spinbox1, 0, 1);

	QLabel *labelr = AddLabel("labelr", "搜索半径");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxr", 0.0, 1000.0, 0.0);
	ui->gridLayout->addWidget(labelr, 1, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 1, 1);

	QLabel *labelserrior = AddLabel("labelserrior", "是否封闭");
	m_checkbox1 = AddCheckBox("checkboxserrior");
	ui->gridLayout->addWidget(labelserrior, 2, 0);
	ui->gridLayout->addWidget(m_checkbox1, 2, 1);

	QLabel *labelscale = AddLabel("labelscale", "裁剪因子");
	m_doublespinbox2 = AddDoubleSpinBox("doublespinboxscale", 0.00, 20.0, 0.0);
	ui->gridLayout->addWidget(labelscale, 3, 0);
	ui->gridLayout->addWidget(m_doublespinbox2, 3, 1);

	AddYesNoButton(4);

	pclpossison_success = 0;
	ui->parameterdockWidget->show();
}

void Zero::PCLFastPanel()
{
	ClearLayout(ui->gridLayout);

	QLabel *labelk = AddLabel("labelk", "搜索近邻点数");
	ui->gridLayout->addWidget(labelk, 0, 0);
	m_spinbox1 = AddSpinBox("spinboxk", 0, 100, 30);
	ui->gridLayout->addWidget(m_spinbox1, 0, 1);

	QLabel *labelr = AddLabel("labelr", "搜索半径");
	m_doublespinbox1 = AddDoubleSpinBox("doublespinboxr", 0.0, 1000.0, 0.0);
	ui->gridLayout->addWidget(labelr, 1, 0);
	ui->gridLayout->addWidget(m_doublespinbox1, 1, 1);

	QLabel *labelscale = AddLabel("labelscale", "连接点的最大距离因子");
	m_doublespinbox2 = AddDoubleSpinBox("doublespinboxscale", 0.01, 2000.0, 10.0);
	ui->gridLayout->addWidget(labelscale, 3, 0);
	ui->gridLayout->addWidget(m_doublespinbox2, 3, 1);

	QLabel *labelangle = AddLabel("labelscale", "最大角度");
	m_doublespinbox3 = AddDoubleSpinBox("doublespinboxscale", 0.0, 150.0, 90.0);
	ui->gridLayout->addWidget(labelangle, 4, 0);
	ui->gridLayout->addWidget(m_doublespinbox3, 4, 1);

	QLabel *labelkeepnormal = AddLabel("labelkeepnormal", "是否保持法向量");
	m_checkbox1 = AddCheckBox("checkboxkeepnormal");
	ui->gridLayout->addWidget(labelkeepnormal, 5, 0);
	ui->gridLayout->addWidget(m_checkbox1, 5, 1);

	AddYesNoButton(6);

	pclfast_success = 0;
	ui->parameterdockWidget->show();
}

void Zero::YesButtonClicked()
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

QLabel *Zero::AddLabel(std::string objectname, std::string text)
{
	QLabel *label = new QLabel();
	label->setObjectName(QString::fromStdString(objectname));
	label->setText(QString::fromLocal8Bit(text.c_str()));
	return label;
}

QDoubleSpinBox *Zero::AddDoubleSpinBox(std::string objectname, double min, double max, double defaultvalue)
{
	QDoubleSpinBox *doublespinbox = new QDoubleSpinBox();
	doublespinbox->setObjectName(QString::fromStdString(objectname));
	doublespinbox->setMinimum(min);
	doublespinbox->setMaximum(max);
	doublespinbox->setValue(defaultvalue);
	return doublespinbox;
}

QSpinBox *Zero::AddSpinBox(std::string objectname, int min, int max, int defaultvalue)
{
	QSpinBox *spinbox = new QSpinBox();
	spinbox->setObjectName(QString::fromStdString(objectname));
	spinbox->setMinimum(min);
	spinbox->setMaximum(max);
	spinbox->setValue(defaultvalue);
	return spinbox;
}

QCheckBox * Zero::AddCheckBox(std::string objectname)
{
	QCheckBox *checkbox = new QCheckBox();
	checkbox->setObjectName(QString::fromStdString(objectname));
	checkbox->setText("");

	return checkbox;
}

QLineEdit * Zero::AddLineEdit(std::string objectname)
{
	QLineEdit *linedit = new QLineEdit();
	linedit->setObjectName(QString::fromStdString(objectname));
	linedit->setText("");

	return linedit;
}

void Zero::AddYesNoButton(int i)
{
	m_yesbutton = new QPushButton();
	m_yesbutton->setObjectName("yesbutton");
	m_yesbutton->setText(QStringLiteral("确定"));
	ui->gridLayout->addWidget(m_yesbutton, i, 0);

	m_nobutton = new QPushButton();
	m_nobutton->setObjectName("nobutton");
	m_nobutton->setText(QStringLiteral("取消"));
	ui->gridLayout->addWidget(m_nobutton, i, 1);

	connect(m_yesbutton, SIGNAL(clicked()), this, SLOT(YesTriggered()));
	connect(m_nobutton, SIGNAL(clicked()), this, SLOT(NoTriggered()));
}

void Zero::EmptyDataViewer()
{
	m_models.clear();
	std::map<int, string>().swap(m_models);
	for (size_t i = 0; i < m_clouds.size(); i++)
	{
		m_clouds[i]->clear();
	}
	for (size_t i = 0; i < m_clouds_with_normals.size(); i++)
	{
		m_clouds_with_normals[i]->clear();
	}

	std::vector<PCTRGB::Ptr>().swap(m_clouds);
	std::vector<pcl::PolygonMesh::Ptr>().swap(m_meshs);
	std::vector<PCTRGBN::Ptr>().swap(m_clouds_with_normals);
	m_pclviewer->removeAllPointClouds();
	m_pclviewer->removeAllShapes();
	ui->pclviewerwidget->update();
}
//获取模型类型数量
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

void Zero::AddFilelist()
{
	while (ui->treeWidget->topLevelItemCount() > 0)
	{
		delete ui->treeWidget->topLevelItem(0);
	}
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
