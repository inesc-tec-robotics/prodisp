#include "configuration_parser.h"

#include <string>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <exception>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

using namespace std;
namespace fs = boost::filesystem;


ConfigurationParser::~ConfigurationParser()
{
}

void ConfigurationParser::initialize(int argc, char* argv[])
{
	// Reset configuration before loading
	reset();

	// Get path of above /bin/executable:
	fs::path config_path = fs::system_complete( fs::path( argv[0] ) );
	config_path.remove_leaf().remove_leaf();

	string config_file = "";
	// Parse command line parameters
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
		cout << "No configuration file was specified as input! Attempting to use std 'prodisp.cfg'" << endl;
		config_file = "prodisp.cfg";
	}

	// Load:
	config_path /= config_file;
	load(config_path.string());

//	throw TextExep("No configuration file was specified as input! The config file must end with .cfg.");
}

cv::Mat ConfigurationParser::parseMatrix(std::string matrix)
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

Vector3 ConfigurationParser::parseVector3(std::string vector)
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

std::vector<double> ConfigurationParser::parseVector(std::string vector)
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

void ConfigurationParser::load(string filename)
{
	ifstream in(filename.c_str());
	if (!in)
	{
		cerr << "ConfigurationParser: Error opening configuration file "
				  << filename << endl;
		throw runtime_error("could not load configuration file");
	}

	vector<string> lines;
	string line;
	while (getline(in, line))
	{
		// Skip empty lines and comments
		if (line.empty() || line[0] == '#')
			continue;

		lines.push_back(line);
	}

	for (int i = 0; i < (int)lines.size()-1; i++)
	{
		// Remove trailing comments:
		size_t pos_end = line.find_first_of('#');
		line = line.substr(0, pos_end-1);

		// Remove trailing spaces:
		pos_end = line.find_last_not_of(" \t");
		line = line.substr(0, pos_end-1);
	}


	// Merge connected lines:
	for (int i = 0; i < (int)lines.size()-1; i++)
	{
		string this_line = lines[i];
		string next_line = lines[i+1];
		if (this_line.find_last_of(";,") == this_line.size()-1 && next_line.find_first_of(" \t") == 0)
//		if (*this_line.rbegin() == ';' && (*next_line.begin() == '\t' || *next_line.begin() == ' '))
		{
			lines[i] += lines[i+1];
			lines.erase(lines.begin() + i + 1);
			i--;
		}
	}


	// Parse the line
	for (int i = 0; i < (int)lines.size()-1; i++)
	{
		if (!parseLine(lines[i]))
		{
			cerr << "ConfigurationParser: Invalid line in configuration file "
				 << filename << ":\n" << lines[i] << endl;
			throw runtime_error("could not parse configuration file");
		}
	}

	// Notify of load complete
	postLoad();
}

void ConfigurationParser::loadDefaults()
{
}

void ConfigurationParser::postLoad()
{
}
