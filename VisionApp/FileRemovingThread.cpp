#include "FileRemovingThread.h"
#include "Logger.h"
#include "AuditLog.h"


FileRemovingThread::FileRemovingThread()
{
}

void FileRemovingThread::setSetting(bool autoRemoving, QStringList clearingPathList, int storageLimit)
{
	_autoRemoving = autoRemoving;
	_clearingPathList = clearingPathList;
	_storageLimit = storageLimit;
}

void FileRemovingThread::run()
{
	//ct::logger::info("[QThread] File removing thread started");
	cleanUpFolder();
}

void FileRemovingThread::cleanUpFolder()
{
	for (auto cP : _clearingPathList)
	{
		manageRunResultFolder(cP);
	}

}

void FileRemovingThread::manageRunResultFolder(QString resultPath)
{
	struct FolderData {
		QString path = "";
		int days_created = 0;
	};

	QDateTime dt = QDateTime::currentDateTime();
	QDirIterator it(resultPath, QDir::Files | QDir::AllDirs | QDir::NoDotAndDotDot);
	std::vector<FolderData> folders;

	while (it.hasNext()) {
		QFileInfo info(it.next());

		QString prepath = resultPath.replace("\\", "/");
		if (!info.absoluteFilePath().contains(prepath)) continue; //do not remove anything outside of this location

		QDateTime c = info.lastModified();

		FolderData folder;
		folder.path = info.absoluteFilePath();
		folder.days_created = c.daysTo(dt);
		folders.emplace_back(folder);

	}

	std::sort(std::rbegin(folders), std::rend(folders), [](const FolderData & a, const FolderData & b) -> bool
	{
		return a.days_created < b.days_created;
	});



	auto cleanup_folders = [=](const FolderData& folder) {
		qDebug() << "Delete Production Folder: " << folder.path;
		AuditLog::instance().log(QStringLiteral("FOLDER_AUTO_PURGE"), folder.path, QStringLiteral("system"));
		QDir dir(folder.path);
		dir.removeRecursively();
	};

	auto manage_folders = [=]() {

		for (auto folder : folders) {
			if (isDriveFullPercentageWise(resultPath, _storageLimit))
			{

				QFileInfo f(folder.path);

				if (_autoRemoving)
				{
					cleanup_folders(folder);
				}

			}

		}
	};
	QtConcurrent::run(manage_folders);
}


bool FileRemovingThread::isDriveFullPercentageWise(QString drivePath, double percentage)
{
	QStorageInfo storage;
	int byteToGB = 1000000000;
	storage.setPath(drivePath);
	double totalSpace = storage.bytesTotal() / byteToGB;
	double availableSpace = storage.bytesAvailable() / byteToGB;
	double percentageFilled = 100 - (availableSpace / totalSpace * 100);

	_percentageFilled = percentageFilled;
	_availableSpace = availableSpace;

	emit updateDriveSpace();
	

	if (percentageFilled >= percentage) return true;
	return false;
}

FileRemovingThread::~FileRemovingThread()
{
}
