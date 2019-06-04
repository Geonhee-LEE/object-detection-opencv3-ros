#include <rnd_ws/labeling_geon.h>

bool CLabeling::set_Label(cv::Mat mat, int param)
{
	try
	{
		//cv::line(mat,cv::Point(10,100),cv::Point(100,200),CV_RGB(0,0,255),2,8,5);
		cv::Mat gray = cv::Mat::zeros(mat.size(), CV_8UC1);
		mblob.SetParam(mat,param,1);
		mblob.DoLabeling(gray);
		return true;

	}
	catch(cv::Exception& e)
	{
		const char* err =e.what();
		std::cout << "exception caught: " <<err <<std::endl;
		return false;
	}
}

bool CLabeling::set_MaxLabel()
{
	int max_square=0;
	
	for( int i=0; i<mblob.m_nBlobs; i++ )
         {
            	 if(mblob.m_recBlobs[i].width*mblob.m_recBlobs[i].height >= max_square) 
                 {
                    	 max_square = mblob.m_recBlobs[i].width*mblob.m_recBlobs[i].height;
                         max_label_idx = i;
                 }
         }
	
	if(max_square!=0)
		return true;
	else 
		return false;
}

void CLabeling::draw_MaxLabel(cv::Mat mat, cv::Scalar color, int thickness=1)
{
	
	cv::Rect rect(mblob.m_recBlobs[max_label_idx].x,mblob.m_recBlobs[max_label_idx].y,mblob.m_recBlobs[max_label_idx].width,mblob.m_recBlobs[max_label_idx].height);

	cv::rectangle(mat,rect,color,thickness);
}

int CLabeling::get_MaxLabel_Size()
{
	return mblob.m_recBlobs[max_label_idx].width*mblob.m_recBlobs[max_label_idx].height;
}

int CLabeling::get_Label_num()
{
	return mblob.m_nBlobs;
}

int CLabeling::get_MaxLabel_width()
{
	return mblob.m_recBlobs[max_label_idx].width;
}

int CLabeling::get_MaxLabel_height()
{
	return mblob.m_recBlobs[max_label_idx].height;
}

cv::Point CLabeling::get_MaxLabel_FirstPt()
{
	return cv::Point(mblob.m_recBlobs[max_label_idx].x, mblob.m_recBlobs[max_label_idx].y);
}



