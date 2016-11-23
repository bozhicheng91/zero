#include "stdafx.h"
#include "src/modules/Zero_IO.h"
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

// ÉèÖÃÎÄ¼þËø
void zero::ZEROWriter::zerosetLockingPermissions(const std::string &filename,
	boost::interprocess::file_lock &lock)
{
	(void)filename;
	(void)lock;
#ifndef WIN32
#if BOOST_VERSION >= 104900
	lock = boost::interprocess::file_lock(filename.c_str());
	if (lock.try_lock())
		DEBUG("[zero::ZEROWriter::zerosetLockingPermissions] File %s locked successfully.\n", filename.c_str());
	else
		DEBUG("[zero::ZEROWriter::zerosetLockingPermissions] File %s locked successfully.\n", filename.c_str());

	namespace fs = boost::filesystem;
	try
		fs::permissions(fs::path(filename), fs::add_perms | fs::set_gid_on_exe);
	catch (const std::exception &e)
		DEBUG("[zero::ZEROWriter::zerosetLockingPermissions] Permissions on %s could not be set.\n", filename.c_str());

#endif
#endif
}

void zero::ZEROWriter::zeroresetLockingPermissions(const std::string &filename,
	boost::interprocess::file_lock &lock)
{
	(void)filename;
	(void)lock;
#ifndef WIN32
#if BOOST_VERSION >= 104900
	(void)filename;
	namespace fs = boost::filesystem;
	try
		fs::permissions(fs::path(filename), fs::remove_perms | fs::set_gid_on_exe);
	catch (const std::exception &e)
		DEBUG("[zero::ZEROWriter::zeroresetLockingPermissions] Permissions on %s could not be set.\n", filename.c_str());
	lock.unlock();
#endif
#endif
}