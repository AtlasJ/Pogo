#include "VisionApp.h"

void VisionApp::generateRecipeSetupTemplate()
{
	QString recipeFolder = Common::Directory::RecipePath() + "\\Template";
	Common::Directory::createDir(recipeFolder);

	QVector<QString> filesToCopy = {
		"barcode.json", "fiducial.json", "islandInfo.json", "linescans.json", "optics.json", "plane.json"
	};
	
	for (const auto& file : filesToCopy) {
		auto srcPath = Common::Directory::getRecipeCurrentPath() + file;
		auto dstPath = recipeFolder + file;
		util::copyTo(srcPath, dstPath);
	}

	openRecipe("Template");
}