#include "QView.h"

QView::QView()
{
}

QView::~QView()
{
}

ZStackInfo::ZStackInfo()
{
}

ZStackInfo::~ZStackInfo()
{
}

ZStackInfo& ZStackInfo::operator=(const ZStackInfo& other)
{
	if (this != &other) { // Avoid self-assignment
		acq_type = other.acq_type;
		step_um = other.step_um;
		preset_iteration = other.preset_iteration;
		encoder_range_um = other.encoder_range_um;
		time_interval_ms = other.time_interval_ms;
		generate_2D_stack = other.generate_2D_stack;
		generate_3D_stack = other.generate_3D_stack;
	}
	return *this;
}

bool ZStackInfo::operator==(const ZStackInfo& other) const
{
	return acq_type == other.acq_type &&
		step_um == other.step_um &&
		preset_iteration == other.preset_iteration &&
		encoder_range_um == other.encoder_range_um &&
		time_interval_ms == other.time_interval_ms &&
		generate_2D_stack == other.generate_2D_stack &&
		generate_3D_stack == other.generate_3D_stack;
}

ZStackInfo::ZStackInfo(const ZStackInfo& other)
{
	acq_type = other.acq_type;
	step_um = other.step_um;
	preset_iteration = other.preset_iteration;
	encoder_range_um = other.encoder_range_um;
	time_interval_ms = other.time_interval_ms;
	generate_2D_stack = other.generate_2D_stack;
	generate_3D_stack = other.generate_3D_stack;
}

PreprocessViewInfo& PreprocessViewInfo::operator=(const PreprocessViewInfo& other)
{
	if (this != &other) { // Avoid self-assignment
		crop = other.crop;
		cropRect = other.cropRect;
		resize = other.resize;
		resizeRect = other.resizeRect;
		flatfield = other.flatfield;
	}
	return *this;
}

PreprocessViewInfo::PreprocessViewInfo(const PreprocessViewInfo& other)
{
	crop = other.crop;
	cropRect = other.cropRect;
	resize = other.resize;
	resizeRect = other.resizeRect;
	flatfield = other.flatfield;
}

bool PreprocessViewInfo::operator==(const PreprocessViewInfo& other) const
{
	return crop == other.crop &&
		cropRect == other.cropRect &&
		resize == other.resize &&
		resizeRect == other.resizeRect&&
	flatfield == other.flatfield;
}

PreprocessViewInfo::PreprocessViewInfo()
{
}

PreprocessViewInfo::~PreprocessViewInfo()
{
}
