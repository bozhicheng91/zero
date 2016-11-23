#include "stdafx.h"
#ifndef ZERO_IO_H_
#define  ZERO_IO_H_

#include "src/modules/Zero_exports.h"
#include <string>

namespace zero
{
	class ZERO_EXPORTS ZEROReader
	{
	public:
		ZEROReader() {}
		~ZEROReader() {}
		// 加载ASC格式文件
		template<typename PointT>
		int ascload(const std::string &filename, pcl::PointCloud<PointT> &cloud)
		{
			std::fstream fs;
			std::string line;
			std::vector<std::string> st;

			if (filename == "" || !boost::filesystem::exists(filename))
			{
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Could not find file: %s", filename.c_str());
				return (-1);
			}

			fs.open(filename.c_str(), std::ios::in);
			if (!fs.is_open() || fs.fail())
			{
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Could not find file: %s", filename.c_str());
				fs.close();
				return (-1);
			}

			try
			{
				while (!fs.eof())
				{
					getline(fs, line);
					if (line == "")
						continue;

					boost::trim(line);
					boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

					std::stringstream sstream(line);
					sstream.imbue(std::locale::classic());

					std::string line_type;
					sstream >> line_type;

					if (line_type.substr(0, 1) == "#")
						continue;

					cloud.height = 1;
					float x, y, z;
					sstream >> x >> y >> z;
					PTRGB p;
					p.x = x; p.y = y; p.z = z;
					p.r = 255; p.g = 255; p.b = 255;
					cloud.points.push_back(p);
				}
				cloud.width = static_cast<uint32_t>(cloud.points.size());
				if (cloud.width < 1)
					throw filename;
			}
			catch (const char *exception)
			{
				if (cloud.size() == 0)
				{
					WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------File %s have no any data!\n", exception);
					fs.close();
					return (-1);
				}
			}

			fs.close();
			return (0);
		}
		// 加载GPD格式文件
		template<typename PointT>
		int gpdload(std::string filename, pcl::PointCloud<PointT> &cloud)
		{
			std::fstream fs;

			if (filename == "" || !boost::filesystem::exists(filename))
			{
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Could not find file: %s", filename.c_str());
				return (-1);
			}

			fs.open(filename.c_str(), std::ios::in | std::ios::binary);
			if (!fs.is_open() || fs.fail())
			{
				stringstream ss;
				ss << "[pcl::PCDReader::readHeader] Could not open file" << filename << "!Error :" << strerror(errno);
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", ss.str().c_str());
				fs.close();
				return (-1);
			}

			long index = 0;

			// 读取GPD
			char gpd[4];
			fs.seekg(index, ios::beg);
			fs.read(gpd, sizeof(gpd));
			index += 4;

			// 读取GPD版本
			short ver;
			fs.seekg(index, ios::beg);
			fs.read((char *)&ver, 2);
			index += 2;

			// 读取点数
			long pn;
			fs.seekg(index, ios::beg);
			fs.read((char *)&pn, 4);
			index += 4;

			// 读取color flag
			bool color_f;
			fs.seekg(index, ios::beg);
			fs.read((char *)&color_f, 1);
			index += 1;

			// 读取UV flag
			bool uv_f;
			fs.seekg(index, ios::beg);
			fs.read((char *)&uv_f, 1);
			index += sizeof(uv_f);

			// 读取normal flag
			bool normal_f;
			fs.seekg(index, ios::beg);
			fs.read((char *)&normal_f, 1);
			index += sizeof(normal_f);

			// 读取display units
			bool display_f;
			fs.seekg(index, ios::beg);
			fs.read((char *)&display_f, 1);
			index += sizeof(display_f);

			// 读取targets
			short targets;
			fs.seekg(index, ios::beg);
			fs.read((char *)&targets, 2);
			index += sizeof(targets);

			// 读取reserved
			char reserved[16];
			fs.seekg(index, ios::beg);
			fs.read(reserved, 16);
			index += sizeof(reserved);

			// 读取第一个点
			std::vector<float> vec;
			std::vector<short> vec_int;

			cloud.points.resize(pn);
			int i;
			for (i = 0; i < pn; i++)
			{
				readanumber(fs, vec, index, 3);
				cloud.points[i].x = vec[0];
				cloud.points[i].y = vec[1];
				cloud.points[i].z = vec[2];
				vec.clear();
				if (uv_f)
				{
					readanumber(fs, vec_int, index, 2);
					vec_int.clear();
				}
				if (color_f)
				{
					readanumber(fs, vec, index, 3);
					vec.clear();
				}
			}
			std::vector<float>(vec).swap(vec);
			std::vector<short>(vec_int).swap(vec_int);

			return (0);
		}

	private:
		template<typename T>
		void readanumber(std::fstream &fs, std::vector<T> &vec, long &index, int c)
		{
			T p;
			for (size_t i = 0; i < c; i++)
			{
				fs.seekg(index, ios::beg);
				fs.read((char *)&p, sizeof(p));
				vec.push_back(p);
				index += sizeof(p);
			}
		}
	};

	class ZERO_EXPORTS ZEROWriter
	{
	public:
		ZEROWriter() {}
		~ZEROWriter() {}
		// 输出ASC格式文件
		template<typename PointT> 
		int ascasciisave(const std::string &file_name, 
			const pcl::PointCloud<PointT> &cloud,
			const int precision = 8)
		{
			if (cloud.empty())
			{
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", "[zero::ZEROWriter::zerowriteascii] Input point cloud has no data!\n");
				return (-1);
			}

			if (cloud.width * cloud.height != cloud.points.size())
			{
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", "[zero::ZEROWriter::zerowriteascii] Number of points different than width * height!");
				return (-1);
			}

			std::ofstream fs;
			fs.open(file_name);

			if (!fs.is_open() || fs.fail())
			{
				WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", "[zero::ZEROWriter::zerowriteascii] could not open file for writing!");
				return (-1);
			}

			boost::interprocess::file_lock file_lock;
			zero::ZEROWriter::zerosetLockingPermissions(file_name, file_lock);

			fs.precision(precision);
			fs.imbue(std::locale::classic());

			for (size_t i = 0; i < cloud.points.size(); i++)
			{
				fs << cloud.points[i].x << cloud.points[i].y << cloud.points[i].z << std::endl;
			}

			fs.close();
			zeroresetLockingPermissions(file_name, file_lock);
			return (0);
		}
		template<typename PointT> 
		inline int ascsave(const std::string &filename,
			const pcl::PointCloud<PointT> &cloud,
			const bool binary = false)
		{
			if (!binary)
				return (writeASCascii<PointT>(filename, cloud));
		}	
	
	protected:
		void zerosetLockingPermissions(const std::string &filename,
			boost::interprocess::file_lock &lock);
		void zeroresetLockingPermissions(const std::string &filename,
			boost::interprocess::file_lock &lock);
	};

	namespace zeroio
	{
		// load Data
		// 从文件中读取ASC点云
		template<typename PointT>
		inline int LoadASC(const std::string &filename, pcl::PointCloud<PointT> &cloud)
		{
			zero::ZEROReader r;
			return (r.ascload(filename, cloud));
		}

		// 从文件中读取GPD点云
		template<typename PointT>
		inline int LoadGPD(const std::string &filename, pcl::PointCloud<PointT> &cloud)
		{
			zero::ZEROReader r;
			return (r.gpdload(filename, cloud));
		}
	}
}

#endif 