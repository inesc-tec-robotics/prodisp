#include "configuration.h"
#include "LoggerSys.hpp"
#include "FileSystemOperations.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <cstring>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/this_node.h>
#include <ros/package.h>


namespace fs = boost::filesystem;
using namespace std;

struct TextExep : public std::exception
{
	std::string s;
	TextExep(std::string ss) : s(ss) {}
	~TextExep() throw () {} // Updated
	const char* what() const throw() { return s.c_str(); }
};


// Singleton instance
template<> Configuration* Singleton<Configuration>::msInstance = 0;


void Configuration::initialize(int argc, char* argv[])
{
	FDEBUG("Configuration::initialize");

	// Parse command line parameters
	string config_file = "";
	if (argc > 1)
	{
		// Find and load configuration file:
		for (int i = 0; i < argc; i++)
		{
			string p = argv[i];
			if (p.find(".cfg") != string::npos)
			{
				config_file = p;
				break;
			}
		}
	}

	if (config_file == "")
	{
		FWARN("No configuration file was specified as input! Attempting to use std 'prodisp.cfg'");
		config_file = "prodisp.cfg";
	}

	// Load:
	load(config_file);
}

std::string Configuration::getPath()
{
	FDEBUG("Configuration::getPath");

//	string path = FileSystemOperations::getPath("/prodisp");

	// Get path:
	string name = ros::this_node::getName();
	string path = ros::package::getPath(name.substr(1));

	return path;
}

void Configuration::load(const string& config_file)
{
	FDEBUG("Configuration::load");

	// Construct path to config file:
	fs::path config_path = getPath();
	config_path /= config_file;


	// Parse command line parameters
	bool file_read = false;
	ifstream in;
	if (config_file != "" || config_file.find(".cfg") != string::npos)
	{
		in.open(config_path.c_str());
		if (in)
		{
			file_read = true;
		}
	}

	if (!file_read)
	{
		FERROR("Configuration: Error opening configuration file " << config_file);
		throw runtime_error("could not load configuration file");
	}

	// Read lines:
	vector<string> lines;
	string line;
	while (getline(in, line))
	{
		// Skip empty lines and comments:
		if (line.empty() || line[0] == '#')
			continue;

		lines.push_back(line);
	}

	for (vector<string>::iterator line_it = lines.begin(); line_it != lines.end(); ++line_it)
	{
		// Remove trailing comments:
		size_t pos_end = line_it->find_first_of('#');
		if (pos_end != string::npos)
		{
			*line_it = line_it->substr(0, pos_end);
		}

		// Remove trailing spaces:
		pos_end = line_it->find_last_not_of(" \t");
		*line_it = line_it->substr(0, pos_end+1);
	}

	// Merge connected lines:
	for (int i = 0; i < (int)lines.size()-1; i++)
	{
		string this_line = lines[i];
		string next_line = lines[i+1];
		if (this_line.find_last_of(";,") == this_line.size()-1 && next_line.find_first_of(" \t") == 0)
		{
			lines[i] += lines[i+1];
			lines.erase(lines.begin() + i + 1);
			i--;
		}
	}

	// Parse the lines:
	for (int i = 0; i < (int)lines.size(); i++)
	{
		string line = lines[i];
		size_t pos_key_last =  line.find_first_of(" \t");
		string key = line.substr(0, pos_key_last);
		string val = line.substr(pos_key_last+1, string::npos);

		// Remove first spaces/tabs:
		size_t pos_val_first = val.find_first_not_of(" \t");
		val = val.substr(pos_val_first, string::npos);
		if (pos_val_first == string::npos || !parseParam(key, val))
		{
			FERROR("Configuration: Invalid line in configuration file "
					 << config_file << ": " << line);
			throw runtime_error("could not parse configuration file");
		}
	}

	// Notify of load complete
	postLoad();
}

cv::Mat Configuration::parseMatrix(std::string matrix)
{
	string matrix_input = matrix;
	cv::Mat retVal;

	try
	{
		// Trim brackets:
		size_t pos_start = matrix.find_last_of('[');
		size_t pos_stop = matrix.find_first_of(']');
		matrix = matrix.substr(pos_start+1, pos_stop-pos_start-1);

		vector< vector<double> > values;
		size_t pos_newline;
		do
		{
			pos_newline = matrix.find(';');
			string line = matrix.substr(0, pos_newline);

			// Trim white spaces:
			line = line.substr(line.find_first_not_of(" \t"));
			line = line.substr(0, line.find_last_not_of(" \t")+1);

			vector<double> line_values;
			size_t pos_newword;
			do
			{
				string word;
				pos_newword = line.find_first_of(", \t");
				if (pos_newword != string::npos)
				{
					pos_newword = line.find_first_not_of(", \t", pos_newword);
					word = line.substr(0, pos_newword);
					line_values.push_back(atof(word.c_str()));
					line = line.substr(pos_newword);
				}
				else
				{
					line_values.push_back(atof(line.c_str()));
				}
			}
			while(pos_newword != string::npos);

			values.push_back(line_values);
			matrix = matrix.substr(pos_newline+1);
		}
		while(pos_newline != string::npos);

		int rows = values.size();
		int cols = values[0].size();

		// Check that all rows have same number of entries:
		for (int r = 0; r < rows; r++)
		{
			if (cols != (int)values[r].size())
			{
				cerr << "Rows does not have equal number of entries for matrix: " << matrix << endl;
				throw runtime_error("Error parsing matrix");
			}
		}

		// Fill matrix:
		retVal = cv::Mat_<double>(rows, cols);
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < cols; c++)
			{
				retVal.at<double>(r,c) = values[r][c];
			}
		}

	}
	catch (...)
	{
		cerr << "Exception while parsing matrix: '" << matrix_input << "'" << endl;
		throw runtime_error("could not parse configuration file");
	}

	return retVal;
}

cv::Matx44d Configuration::parseMatrix44d(std::string matrix)
{
	cv::Matx44d ret_val;
	cv::Mat mat = parseMatrix(matrix);
	if (mat.rows != 4 || mat.cols != 4)
	{
		cerr << "Invalid matrix specified - should be 4x4: " << matrix << endl;
		return ret_val;
	}

	for (int col = 0; col < mat.cols; col++)
	{
		for (int row = 0; row < mat.rows; row++)
		{
			ret_val(row,col) = mat.at<double>(row,col);
		}
	}

	return ret_val;
}

Vector3 Configuration::parseVector3(std::string vector)
{
	Vector3 ret_val;
	cv::Mat mat = parseMatrix(vector);
	if (mat.rows != 1 || mat.cols != 3)
	{
		cerr << "Invalid vector specified: " << vector << endl;
		throw runtime_error("could not parse configuration file");
	}

	for (int col = 0; col < 3; col++)
	{
		ret_val.c[col] = mat.at<double>(0,col);
	}

	return ret_val;
}

std::vector<double> Configuration::parseVector(std::string vector)
{
	std::vector<double> ret_val;
	cv::Mat mat = parseMatrix(vector);
	if (mat.rows != 1)
	{
		cerr << "Invalid vector specified: " << vector << endl;
		return ret_val;
	}

	for (int col = 0; col < mat.cols; col++)
	{
		ret_val.push_back(mat.at<double>(0,col));
	}

	return ret_val;
}

bool Configuration::parseParam(const string key, const string value)
{
	if (key == "model")		models_.push_back(value);

	// Camera parameters:
	else if (key == "proj_matrix")			proj_matrix_data_		= parseMatrix(value);
	else if (key == "proj_distortions")		proj_distortions_		= parseMatrix(value);
	else if (key == "proj_resolution")		proj_res					= parseVector(value);
//	else if (key == "proj2cam")
//	{
//		cam2proj_ = MathOp::Transform( parseMatrix(value) );
//		proj2cam_ = cam2proj_.inverse();
//	}

	// GUI parameters:
	else if (key == "full_screen")			full_screen_			= atoi(value.c_str());
	else if (key == "enable_at_start")		enable_at_start_		= atoi(value.c_str());
	else if (key == "init_cam_at_distance")init_cam_at_distance_= atoi(value.c_str());
	else if (key == "disp_wall_coordsys")	disp_wall_coordsys_	= atoi(value.c_str());
	else if (key == "window_height")			window_height_			= atoi(value.c_str());

	// Model handling parameters
	else if (key == "SharpAngleMin")			sharp_angle_min_		= atof(value.c_str());
	else if (key == "PlaneAngleMax")			plane_angle_max_		= atof(value.c_str());

	else
	{
		// Invalid option
		return false;
	}

	return true;
}

void Configuration::postLoad(void)
{
	FDEBUG("Configuration::postLoad");

//	 Validate:
	if (window_height_ <= 0)
		throw runtime_error("Invalid window dimensions");

	if (proj_res.size() != 2)
		throw runtime_error(string("Invalid projector resolution"));

	// Determine projector's aspect ratio:
	proj_aspect_ = getProjResX()/getProjResY();

	// Retrieve parameters for glFrustum:
	double fy = proj_matrix_data_.at<double>(1,1);
	double fx = proj_matrix_data_.at<double>(0,0);
	double px = proj_matrix_data_.at<double>(0,2);
	double py = proj_matrix_data_.at<double>(1,2);

	// Determine projector's FoV:
	proj_fov_y_ = 2*atan(getProjResY()/(2*fy)) * 180/M_PI;

	// Calculate parameters for glFrustum:
	proj_near_	= 0.1;
	proj_far_	= 100;
	proj_left_	= -proj_near_ * px/fx;
	proj_right_	=  proj_near_ * (getProjResX()-px)/fx;
	proj_top_	=  proj_near_ * py/fy;
	proj_bottom_= -proj_near_ * (getProjResY()-py)/fy;

	// 2D projection:
	proj2d_limits_ = Rectf(-1.0 * proj_aspect_, 1.0, 1.0*proj_aspect_, -1.0);
}

void Configuration::setProjMatrix(GLdouble proj_matrix[16])
{
	FDEBUG("Configuration::setProjMatrix");
	cv::Matx44d temp;
	for (int i = 0; i < 16; i++)
	{
		proj_matrix_[i] = proj_matrix[i];
		temp(i) = proj_matrix[i];
	}
	proj_matrix_inv_cv_ = temp.inv();
}

void Configuration::getProjMatrixGl(GLdouble proj_matrix[16]) const
{
	for (int i = 0; i < 16; i++)
		proj_matrix[i] = proj_matrix_[i];
}

//void Configuration::getProjMatrixInvGl(GLdouble proj_matrix[16]) const
//{
//	for (int i = 0; i < 16; i++)
//		proj_matrix[i] = proj_matrix_[i];
//}

