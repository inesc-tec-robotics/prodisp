#ifndef ConfigurationParser_H__
#define ConfigurationParser_H__

#include "prerequisites.h"
#include "opencv2/highgui/highgui.hpp"

#include <boost/noncopyable.hpp>

class ConfigurationParser : boost::noncopyable
{
public:
	virtual ~ConfigurationParser();

	void initialize(int argc, char* argv[]);

protected:
	virtual void reset() = 0;
	virtual void loadDefaults();

	virtual bool parseLine(const std::string& line) = 0;
	cv::Mat parseMatrix(std::string matrix);
	Vector3 parseVector3(std::string vector);
	std::vector<double> parseVector(std::string vector);

	virtual void postLoad();

private:
	void load(std::string filename);
};

#endif // ConfigurationParser_H__
