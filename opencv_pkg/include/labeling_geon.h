#pragma once

#include "BlobLabeling.h"
#include "opencv2/imgproc/imgproc.hpp"

class CLabeling
{
private :
	CBlobLabeling mblob;
	int max_label_idx;
public :
	bool set_Label(cv::Mat, int param);
	bool set_MaxLabel();
	void draw_MaxLabel(cv::Mat, cv::Scalar, int);
	int get_MaxLabel_Size();
	int get_MaxLabel_width();
	int get_MaxLabel_height();
	int get_Label_num();
	cv::Point get_MaxLabel_FirstPt();

	CLabeling(void);
	~CLabeling(void);




};
