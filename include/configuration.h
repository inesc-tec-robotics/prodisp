#ifndef Configuration_H__
#define Configuration_H__

#include "prerequisites.h"
#include "configuration_parser.h"
#include "singleton.h"

#include <string>
#include <vector>

class Configuration : public Singleton<Configuration>
{
public:
	void				initialize(int argc, char* argv[]);
	std::string		getPath();

private:
	void				load(const std::string &config_file);
	cv::Mat			parseMatrix(std::string matrix);
	cv::Matx44d		parseMatrix44d(std::string matrix);
	Vector3			parseVector3(std::string vector);
	std::vector<double> parseVector(std::string vector);
	bool				parseParam(const std::string key, const std::string value);
	void				postLoad(void);

public:
	std::vector<std::string> getModels() const { return models_; }

	// Camera parameters:
	void			setProjMatrix(GLdouble proj_matrix[16]);
	void			getProjMatrixGl(GLdouble proj_matrix[16]) const;
//	void			getProjMatrixInvGl(GLdouble proj_matrix[16]) const;
	cv::Matx44d getProjMatrixInvCv() const { return proj_matrix_inv_cv_; }
	GLdouble		getProjLeft() const			{ return proj_left_; }
	GLdouble		getProjRight() const			{ return proj_right_; }
	GLdouble		getProjBottom() const		{ return proj_bottom_; }
	GLdouble		getProjTop() const			{ return proj_top_; }
	GLdouble		getProjNear() const			{ return proj_near_; }
	GLdouble		getProjFar() const			{ return proj_far_; }
	cv::Mat		getProjDistortions() const { return proj_distortions_; }
	double		getProjFovY() const			{ return proj_fov_y_; }		// RSA_TODO
	double		getProjAspectRatio() const { return proj_aspect_; }		// RSA_TODO
	double		getProjResX()					{ return proj_res[0]; }
	double		getProjResY()					{ return proj_res[1]; }
	MathOp::Transform getProj2Cam() const	{ return proj2cam_; }
	MathOp::Transform getCam2Proj() const	{ return cam2proj_; }
	Rectf			getProj2dEyeArea()	const		{ return proj2d_limits_; }	// left: -1*aspect, bot: -1


	// GUI parameters:
	bool		getFullScreen() const		{ return full_screen_; }
	void		toggleFullScreen()			{ full_screen_ = !full_screen_; }
	bool		getEnableAtStart() const	{ return enable_at_start_; }
	bool		initCamAtDistance() const	{ return init_cam_at_distance_; }
	bool		dispWallCoordsys() const{ return disp_wall_coordsys_; }
	unsigned getInitWindowHeight() const{ return window_height_; }
	unsigned getInitWindowWidth() const	{ return window_height_*proj_aspect_; }
	bool		getGuiHelp() const			{ return gui_help_; }
	void		toggleGuiHelp()				{ gui_help_ = !gui_help_; }
	
	// Model handling parameters:
	float getSharpAngleMin() { return sharp_angle_min_; }
	float getPlaneAngleMax() { return plane_angle_max_; }

private:
	std::vector<std::string> models_;

	cv::Mat		proj_matrix_data_;
	GLdouble		proj_matrix_[16];
	cv::Matx44d	proj_matrix_inv_cv_;
	GLdouble		proj_left_;
	GLdouble		proj_right_;
	GLdouble		proj_bottom_;
	GLdouble		proj_top_;
	GLdouble		proj_near_;
	GLdouble		proj_far_;
	cv::Mat		proj_distortions_;	//	[k1 k2 p1 p2 k3], where k's are radial and p's are tangential components
	std::vector<double> proj_res;
	MathOp::Transform proj2cam_;
	MathOp::Transform cam2proj_;
	double		proj_aspect_;
	float			proj_fov_y_;
	Rectf			proj2d_limits_;

	// GUI parameters
	bool			full_screen_;
	bool			enable_at_start_;
	bool			init_cam_at_distance_;
	bool			disp_wall_coordsys_;
	bool			gui_help_;
	unsigned		window_height_;
	
	// Model handling parameters
	float			sharp_angle_min_;		// Degrees
	float			plane_angle_max_;		// Degrees
};

#endif // Configuration_H__
