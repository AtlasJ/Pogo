#pragma once

#include <QWidget>
#include <QMessageBox>
#include "ui_TemplateLibraryTab.h"
#include "AlgoGraphList.h"

class TemplateLibraryTab : public QWidget
{
	Q_OBJECT

public:
	TemplateLibraryTab(QWidget *parent = Q_NULLPTR);
	~TemplateLibraryTab();

	QString currentTemplateId();
	QString currentTemplateName();
	QColor currentTemplateColor();
	QColor getTemplateColor(QString& templateID);
	AlgoGraph* currentAlgoGraph();
	AlgoGraph*getAlgoGraph(QString& templateID);
	void setTemplateImagePath(const QString& id, const QString& templateImagePath, const QSize& size);

	QStringList getAllTemplateID();
	void releaseAlgoGraphs();

private:
	Ui::TemplateLibraryTab ui;

	AlgoGraphList _algoGraphList;
	QHash<QString, int> _tagNameCountHash;
	QStringList colorPallete = {
		"#FF0028","#FF4200","#FFAE00","#E3FF00","#76FF00","#0AFF00","#00FF60",
		"#00FFCC","#00C5FF","#0058FF","#1300FF","#8000FF","#ED00FF","#FF0023","#FF4800",
		"#FFB400","#DDFF00","#71FF00","#05FF00","#00FF66","#00FFD1","#00C0FF","#0053FF",
		"#1900FF","#8500FF","#F200FF","#FF001E","#FF4D00","#FFB900","#D8FF00","#6CFF00",
		"#00FF00","#00FF6B","#00FFD7","#00BAFF","#004DFF","#1E00FF","#8B00FF","#F800FF",
		"#FF0018","#FF5200","#FFBF00","#D2FF00","#66FF00","#00FF05","#00FF70","#00FFDC",
		"#00B5FF","#0048FF","#2400FF","#9000FF","#FD00FF","#FF0013","#FF5800","#FFC400",
		"#CDFF00","#61FF00","#00FF0A","#00FF76","#00FFE1","#00AFFF","#0043FF","#2900FF",
		"#9600FF","#FF00FB","#FF000E","#FF5D00","#FFC900","#C7FF00","#5BFF00","#00FF10",
		"#00FF7B","#00FFE7","#00AAFF","#003DFF","#2F00FF","#9B00FF","#FF00F5","#FF0008",
		"#FF6300","#FFCF00","#C2FF00","#56FF00","#00FF15","#00FF81","#00FFEC","#00A4FF",
		"#0038FF","#3400FF","#A100FF","#FF00F0","#FF0003","#FF6800","#FFD400","#BDFF00",
		"#51FF00","#00FF1A","#00FF86","#00FFF1","#009FFF","#0032FF","#3900FF","#A600FF",
		"#FF00EA","#FF0100","#FF6E00","#FFDA00","#B7FF00","#4BFF00","#00FF20","#00FF8B",
		"#00FFF7","#009AFF","#002DFF","#3F00FF","#AC00FF","#FF00E5","#FF0700","#FF7300",
		"#FFDF00","#B2FF00","#46FF00","#00FF25","#00FF91","#00FFFC","#0094FF","#0027FF",
		"#4400FF","#B100FF","#FF00DF","#FF0C00","#FF7800","#FFE400","#ACFF00","#40FF00",
		"#00FF2B","#00FF96","#00FBFF","#008FFF","#0022FF","#4A00FF","#B600FF","#FF00DA",
		"#FF1200","#FF7E00","#FFEA00","#A7FF00","#3BFF00","#00FF30","#00FF9B","#00F6FF",
		"#0089FF","#001DFF","#4F00FF","#BC00FF","#FF00D4","#FF1700","#FF8300","#FFEF00",
		"#A2FF00","#36FF00","#00FF35","#00FFA1","#00F1FF","#0084FF","#0017FF","#5500FF",
		"#C100FF","#FF00CF","#FF1C00","#FF8900","#FFF500","#9CFF00","#30FF00","#00FF3B",
		"#00FFA6","#00EBFF","#007EFF","#0012FF","#5A00FF","#C700FF","#FF00CA","#FF2200",
		"#FF8E00","#FFFA00","#97FF00","#2BFF00","#00FF40","#00FFAC","#00E6FF","#0079FF",
		"#000CFF","#5F00FF","#CC00FF","#FF00C4","#FF2700","#FF9300","#FEFF00","#91FF00","#25FF00",
		"#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#FF00FF", "#00FFFF", "#000000",
		"#800000", "#008000", "#000080", "#808000", "#800080", "#008080", "#808080",
		"#C00000", "#00C000", "#0000C0", "#C0C000", "#C000C0", "#00C0C0", "#C0C0C0",
		"#400000", "#004000", "#000040", "#404000", "#400040", "#004040", "#404040",
		"#200000", "#002000", "#000020", "#202000", "#200020", "#002020", "#202020",
		"#600000", "#006000", "#000060", "#606000", "#600060", "#006060", "#606060",
		"#A00000", "#00A000", "#0000A0", "#A0A000", "#A000A0", "#00A0A0", "#A0A0A0",
		"#E00000", "#00E000", "#0000E0", "#E0E000", "#E000E0", "#00E0E0", "#E0E0E0" };

	bool templateNameExist(const QString & templateName);
	QString getUnusedColor();
	bool checkColourUsed(QColor colour);

	void updateTemplateLibraryTableWidget();

public Q_SLOTS:
	void addTemplate();
	void editTemplate();
	void deleteTemplate();
	void duplicateTemplate();
	void renameTemplate();
	void generateVIDIImages(bool enablePreprocess);
	void addPadding();
	void resizeTemplate();
	void saveRefImage();

	void updateUniformBoxFlag();
	void updateUniformBoxFlag(bool flag);
	void backupTemplateList();
	void saveTemplateList();
	void loadTemplateList();
	void tableWidgetTemplateLibCellClicked(int row, int column);
	void customContextMenuRequested(QPoint point);
	void changeTemplateColor();
	void updateTemplateListSettings(QString color, QString templateName);
	void selectTemplateTableRow(const QString & templateName);
	void loadAlgoGraphListMask();
	void reloadAlgoGraphListMetaData();
	void setTagNameCount(QHash<QString, int> tagNameCountHash);
	void displayTagNameCount();
	void saveGoldenTemplateList();
	void loadGoldenTemplateList();

signals:
	void updateVisionObjectTemplate(AlgoGraph* algoGraph);
	void deleteVisionObjectTemplate(const QString &templateId);
	void updateVisionObjectSize(AlgoGraph* algoGraph);
	void updateVisionObjectColor(AlgoGraph* algoGraph);
	void generateVIDIImages(AlgoGraph* algoGraph, bool enablePreprocess);
	void addVisionObjectPadding(AlgoGraph* algoGraph, int padding);
	void showMsg(const QString& msg, QMessageBox::StandardButtons buttons = QMessageBox::Close);
	void editTemplateSignal();
	void saveTemplateReferenceImage(AlgoGraph* algoGraph);
	void signalOpenGoldenRecipeDialog();


};
