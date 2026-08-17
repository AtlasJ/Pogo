#pragma once
#include <string>
#include <vector>
#include <functional>

#include <QImage>
#include "mtrx.h"
#include "FrameInfo.h"
#include "MessageQue.h"

#include "Gocator\Include\GoSdk\GoSdk.h"

extern TMessageQue<FrameInfo> g_imageQueue;

namespace ct{

	struct GoInfo {
		QString id = "";
		GoMode mode = GO_MODE_SURFACE;
		double resolution_x_mm = 0.0;
		double resolution_y_mm = 0.0;
		double resolution_z_mm = 0.0;
		MIL_ID mHeightMap = M_NULL;
		MIL_ID mIntensity = M_NULL;
		std::vector<double> profiles;
		std::function<void()> fnc;

		void reset() {
			id = "";
			resolution_x_mm = 0.0;
			resolution_y_mm = 0.0;
			resolution_z_mm = 0.0;
			mtrx::free_buffer(mHeightMap);
			mtrx::free_buffer(mIntensity);
			profiles.clear();
		}
	};
}

class IGocator {
public:
	static IGocator& instance();

	bool init(std::string ip);
	bool release();

	//controls
	bool flush();//x
	bool reset_sensor();//x
	bool refresh();//x
	bool reconnect();//i/
	bool start();//v/
	bool stop();//v/
	bool enable_data_channel(bool enable);//i/
	bool set_fixed_length(double mm);//v/
	bool set_exposure(int us);//v/
	bool enable_intensity(bool enable);//v/
	bool snapshot();//v/

	//file system
	std::string default_job();//x
	std::string loaded_job();//i/
	bool load_job(std::string filename);//v: loadconfig/
	bool save_job(); //save current loaded job //x
	bool set_default_job(std::string filename);

	bool clear_recordings(); //x
	bool record(bool on); //x

	bool download(std::string filename, std::string dst_path); //.job .rec .log //x
	bool upload(std::string filename, std::string src_path); //.job .rec .log //x
	bool copy(std::string src_file, std::string dst_file); //.job .rec .log //x
	bool delete_file(std::string filename);//.job .rec .log //x

	//layout 
	GoOrientation orientation(); //x
	bool set_orientation(GoOrientation); //x
	void set_rotationAngle(double rotationAngle); //x

	//info
	bool is_connected(); //v/

	bool file_exists(std::string filename); //.job .rec .log //x
	std::vector<std::string> get_filenames(); //x

	std::string version(); //v
	std::string part_number(); //v
	std::string error(); //v

	ct::GoInfo& go_info();
	
	template <typename Func, typename... Args>
	void data_received(Func&& func, Args&&... args) {
		go_info().fnc = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
	}

	void test();

	FrameInfo* getFrame();

private:
	IGocator();
	~IGocator();
	IGocator(const IGocator&) = delete;
	IGocator& operator=(const IGocator&) = delete;

	static IGocator m_instance;

	kStatus m_status;
	kAssembly m_api = kNULL;
	GoSystem m_system = kNULL;
	GoSensor m_sensor = kNULL;
	GoSetup m_setup = kNULL;
	GoLayout m_layout = kNULL;
	GoAccelerator m_accelerator = kNULL;
	kIpAddress m_ipAddress;
	
	ct::GoInfo m_goInfo;

	kChar m_ip_string[FILENAME_MAX];
	kChar m_filename[FILENAME_MAX];
	kChar m_path[FILENAME_MAX];

	std::string m_error_msg = "";

	std::string get_status_msg(kStatus status);
	bool set_scan_mode(GoMode mode);
	bool safe_guard();
	bool verify_connection();
};

/*
To scan for surface data using encoder:
- Encoder mm per pulse must be correct, if not nothing will generate out: 0.001 mm/pulse
- Fixed length, sequential 
- Ensure output is tick for any type of data like surface, surface integrity
*/