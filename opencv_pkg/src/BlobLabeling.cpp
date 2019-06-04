#include "rnd_ws/BlobLabeling.h"

/// @brief CBlobLabeling ������
/// @remarks Labeling�� ���Ǵ� ���� ���� �ʱ�ȭ
CBlobLabeling::CBlobLabeling(void)
{
    m_nThreshold    = 0;
	m_DiffThreshold	= 0;
    m_nBlobs        = _DEF_MAX_BLOBS;
    //m_Image         = NULL;

    m_vPoint        = NULL;

    m_recBlobs      = NULL;
    m_intBlobs      = NULL;

	m_nPixels		= NULL;

	m_nAvgValue		= NULL;
}

/// @brief CBlobLabeling �Ҹ���
/// @remarks Labeling�� ���� ���� ���� ����
CBlobLabeling::~CBlobLabeling(void)
{
    /*if( m_Image != NULL )
        cvReleaseImage( &m_Image );*/

    if( m_recBlobs != NULL )
    {
        delete m_recBlobs;
        m_recBlobs = NULL;
    }

    if( m_intBlobs != NULL )
    {
        delete m_intBlobs;
        m_intBlobs = NULL;
    }

	if( m_nPixels != NULL)
	{
		delete m_nPixels;
		m_nPixels = NULL;
	}

	if( m_nAvgValue != NULL)
	{
		delete m_nAvgValue;
		m_nAvgValue = NULL;
	}
}

/// @brief Parameter ����
/// @remarks Labeling�� ���Ǵ� �̹����� ���� ũ�� ������ ����
/// @param[in] image        Labeling�� ����� �̹���. 0 �Ǵ� 255�� ���� ������ 1ch �̹���.
/// @param[in] nThreshold   Labeling ��� �� Blob�� �ȼ� ���� nThreshold ������ ��� ����
void CBlobLabeling::SetParam(cv::Mat image, int nThreshold, int DiffThreshold)
{
    if( m_recBlobs != NULL )
    {
        delete m_recBlobs;

        m_recBlobs	= NULL;
        m_nBlobs	= _DEF_MAX_BLOBS;
    }

    if( m_intBlobs != NULL )
    {
        delete m_intBlobs;

        m_intBlobs	= NULL;
        m_nBlobs	= _DEF_MAX_BLOBS;
    }

    //if( m_Image != NULL )	cvReleaseImage( &m_Image );

    m_Image         = image.clone();
    m_nThreshold	= nThreshold;
	m_DiffThreshold	= DiffThreshold;

	if( m_nPixels != NULL)
	{
		delete m_nPixels;

		m_nPixels	= NULL;
		m_nBlobs	= _DEF_MAX_BLOBS;
	}

	if( m_nAvgValue != NULL)
	{
		delete m_nAvgValue;

		m_nAvgValue	= NULL;
		m_nBlobs	= _DEF_MAX_BLOBS;
	}
}

/// @brief Labeling ����
void CBlobLabeling::DoLabeling(cv::Mat add_image)
{
    m_nBlobs = Labeling(m_Image, add_image, m_nThreshold, m_DiffThreshold);
}

/// @brief Labeling ����
/// @param[in] image        Labeling�� ����� �̹���. 0 �Ǵ� 255�� ���� ������ 1ch �̹���.
/// @param[in] nThreshold   Labeling ��� �� Blob�� �ȼ� ���� nThreshold ������ ��� ����
/// @return Label ����
int CBlobLabeling::Labeling(cv::Mat image, cv::Mat add_image, int nThreshold, int DiffThreshold)
{
	if( image.channels() != 1 ) 	return 0;

    int nNumber;

    int nWidth  = image.cols;
	int nHeight = image.rows;

    unsigned char* tmpBuf = new unsigned char [nWidth * nHeight];

    int i,j;

	for(j=0;j<nHeight;j++){
		unsigned char *tmpData = image.ptr<uchar>(j);
		for(i=0;i<nWidth ;i++){
			tmpBuf[j*nWidth+i] = *tmpData++;
			//tmpBuf[j*nWidth+i] = (unsigned char)image->imageData[j*image->widthStep+i];
		}
	}

	m_nPixels = new int [255];
	m_nAvgValue = new int [255];

    // ���̺��� ���� ����Ʈ �ʱ�ȭ
    InitvPoint(nWidth, nHeight);

    // ���̺�
    nNumber = _Labeling(add_image, tmpBuf, nWidth, nHeight, nThreshold, DiffThreshold);

    // ����Ʈ �޸� ����
    DeletevPoint();

    if( nNumber != _DEF_MAX_BLOBS )     m_recBlobs = new CvRect [nNumber];

    if( nNumber != _DEF_MAX_BLOBS )     m_intBlobs = new int [nNumber];

    if( nNumber != 0 )	DetectLabelingRegion(nNumber, tmpBuf, nWidth, nHeight);

	for(j=0;j<nHeight;j++){
		unsigned char *tmpData = image.ptr<uchar>(j);
		for(i=0;i<nWidth ;i++){
			*tmpData++ = tmpBuf[j*nWidth+i];
            //image->imageData[j*image->widthStep+i] = tmpBuf[j*nWidth+i];
		}
	}
    delete tmpBuf;

    return nNumber;
}

/// @brief m_vPoint �ʱ�ȭ
void CBlobLabeling::InitvPoint(int nWidth, int nHeight)
{
    int nX, nY;

    m_vPoint = new Visited [nWidth * nHeight];

    for(nY = 0; nY < nHeight; nY++)
    {
        for(nX = 0; nX < nWidth; nX++)
        {
            m_vPoint[nY * nWidth + nX].bVisitedFlag		= false;
            m_vPoint[nY * nWidth + nX].ptReturnPoint.x	= nX;
            m_vPoint[nY * nWidth + nX].ptReturnPoint.y	= nY;
        }
    }
}

/// @brief m_vPoint ����
void CBlobLabeling::DeletevPoint()
{
    delete m_vPoint;
    m_vPoint = NULL;
}

/// @brief Labeling ���� �Լ�
/// @remarks Size�� nWidth�̰� nHeight�� DataBuf���� nThreshold���� ���� ������ ������ �������� blob���� ȹ��
/// @param[in] DataBuf      Labeling�� ����� �̹��� ������ ����. 0 �Ǵ� 255�� ���� ����.
/// @param[in] nWidth       ������ ������ ���� ����
/// @param[in] nHeight      ������ ������ ���� ����
/// @param[in] nThreshold   Labeling ��� �� Blob�� �ȼ� ���� nThreshold ������ ��� ����
/// @return Label ����
int CBlobLabeling::_Labeling(cv::Mat add_image, unsigned char* DataBuf, int nWidth, int nHeight, int nThreshold, int DiffThreshold)
{
    int num = 0;
    int nX, nY, k, l;
    int StartX , StartY, EndX , EndY;
	int a = 0;
	int avg = 0;

    // Find connected components
    for(nY = 0; nY < nHeight; nY++)
    {
		unsigned char* _data = add_image.ptr<uchar>(nY);
        for(nX = 0; nX < nWidth; nX++)
        {
			unsigned char tmpVal = *_data++;
            if(DataBuf[nY * nWidth + nX] !=0 && tmpVal == 0)    // Is this a new component?, Object != 0
            {
				int InitValue = DataBuf[nY * nWidth + nX];
                num++;

                DataBuf[nY * nWidth + nX] = num;

                StartX = nX, StartY = nY, EndX = nX, EndY= nY;

                avg = __NRFIndNeighbor(add_image, DataBuf, InitValue, DiffThreshold, nWidth, nHeight, nX, nY, &StartX, &StartY, &EndX, &EndY);

                if((a=__Area(DataBuf, StartX, StartY, EndX, EndY, nWidth, num)) < nThreshold)//���� �ɷ���
                {
                    for(k = StartY; k <= EndY; k++)
                    {
                        for(l = StartX; l <= EndX; l++)
                        {
                            if(DataBuf[k * nWidth + l] == num)
                                DataBuf[k * nWidth + l] = 0;
                        }
                    }
                    --num;

                    if(num > 250)
                        return  0;
                }
				else
				{
					if( num != 0 )
					{
						m_nPixels[num-1] = a;
						m_nAvgValue[num-1] = avg;
					}
				}
            }
        }
    }

    return num;
}

/// @brief Labeling ����� Blob ������ ���ϴ� �Լ�
/// @param[in] nLabelNumber �ش� Blob�� Label
/// @param[in] DataBuf      Labeling Result
/// @param[in] nWidth       DataBuf ���� ũ��
/// @param[in] nHeight      DataBuf ���� ũ��
void CBlobLabeling::DetectLabelingRegion(int nLabelNumber, unsigned char *DataBuf, int nWidth, int nHeight)
{
    int nX, nY;
    int nLabelIndex ;

    bool bFirstFlag[255] = {false,};

    for(nY = 0; nY < nHeight; nY++)
    {
        for(nX = 0; nX < nWidth; nX++)
        {
            nLabelIndex = DataBuf[nY * nWidth + nX];

            if(nLabelIndex != 0)	// Is this a new component?, 255 == Object
            {
                if(bFirstFlag[nLabelIndex] == false)
                {
                    m_recBlobs[nLabelIndex-1].x 	= nX;
                    m_recBlobs[nLabelIndex-1].y     	= nY;
                    m_recBlobs[nLabelIndex-1].width 	= 0;
                    m_recBlobs[nLabelIndex-1].height	= 0;

                    bFirstFlag[nLabelIndex] = true;
                }
                else
                {
                    int left	= m_recBlobs[nLabelIndex-1].x;
                    int right	= left + m_recBlobs[nLabelIndex-1].width;
                    int top     = m_recBlobs[nLabelIndex-1].y;
                    int bottom	= top + m_recBlobs[nLabelIndex-1].height;

                    if( left   >= nX )	left	= nX;
                    if( right  <= nX )	right	= nX+1;
                    if( top    >= nY )	top 	= nY;
                    if( bottom <= nY )	bottom	= nY+1;

                    m_recBlobs[nLabelIndex-1].x     	= left;
                    m_recBlobs[nLabelIndex-1].y     	= top;
                    m_recBlobs[nLabelIndex-1].width 	= right - left;
                    m_recBlobs[nLabelIndex-1].height	= bottom - top;

                    m_intBlobs[nLabelIndex-1]       	= nLabelIndex;
                }
            }
        }
    }
}

/// @brief Blob Labeling�� ���� ���ϴ� �Լ�
/// @param[in] DataBuf   Labeling�� ����� �̹��� ������ ����. 0 �Ǵ� 255�� ���� ����.
///				DiffThreshold	����threshold
/// @param[in] nWidth   ������ ������ ���� ����
/// @param[in] nHeight  ������ ������ ���� ����
/// @param[in] nPosX    ���� Ž������ X��ǥ
/// @param[in] nPosY    ���� Ž������ Y��ǥ
/// @param[out] StartX  Ž�� �������� X��ǥ
/// @param[out] StartY  Ž�� �������� Y��ǥ
/// @param[out] EndX    Ž�� �������� X��ǥ
/// @param[out] EndY    Ž�� �������� Y��ǥ
///				MinValue
///				MaxValue
/// @return 0
int CBlobLabeling::__NRFIndNeighbor(cv::Mat add_image, unsigned char *DataBuf, int InitValue, int DiffThreshold , int nWidth, int nHeight, int nPosX, int nPosY, int *StartX, int *StartY, int *EndX, int *EndY)
{
    CvPoint CurrentPoint;

    CurrentPoint.x = nPosX;
    CurrentPoint.y = nPosY;

    m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x].bVisitedFlag    = true;
    m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x].ptReturnPoint.x = nPosX;
    m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x].ptReturnPoint.y = nPosY;

	int MinValue = 0;
	int MaxValue = 0;

	CompMin = InitValue - DiffThreshold;
	CompMax = InitValue + DiffThreshold;
	LimitMinMax();

	MinValue = MaxValue = InitValue;

    while(1)
    {
		int Cdatabuf=0;

		if(CurrentPoint.x != 0)
			Cdatabuf = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x - 1];
        if( (CurrentPoint.x != 0) && (Cdatabuf !=0) && (Cdatabuf >= CompMin) && (Cdatabuf <= CompMax) )   // -X ����
        {
			if(MinValue > Cdatabuf)
			{
				MinValue = Cdatabuf;
				CompMin = Cdatabuf - DiffThreshold;
				LimitMinMax();
			}
			if(Cdatabuf > MaxValue)
			{
				MaxValue = Cdatabuf;
				CompMax = Cdatabuf + DiffThreshold;
				LimitMinMax();
			}
            if( m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x - 1].bVisitedFlag == false )
            {
                DataBuf[CurrentPoint.y  * nWidth + CurrentPoint.x  - 1]                 = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
                m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x - 1].bVisitedFlag	= true;
                m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x - 1].ptReturnPoint	= CurrentPoint;
				unsigned char* CurPData = add_image.ptr<uchar>(CurrentPoint.y);
				*(CurPData + (CurrentPoint.x - 1)) = (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
				//add_image->imageData[CurrentPoint.y  * nWidth + CurrentPoint.x  - 1]	= (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
                CurrentPoint.x--;

                if(CurrentPoint.x <= 0)
                    CurrentPoint.x = 0;

                if(*StartX >= CurrentPoint.x)
                    *StartX = CurrentPoint.x;

                continue;
            }
        }


		if(CurrentPoint.x != nWidth - 1)
			Cdatabuf = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x + 1];
        if( (CurrentPoint.x != nWidth - 1) && (Cdatabuf !=0) && (Cdatabuf >= CompMin) && (Cdatabuf <= CompMax) )   //+X ����
        {
			if(MinValue > Cdatabuf)
			{
				MinValue = Cdatabuf;
				CompMin = Cdatabuf - DiffThreshold;
				LimitMinMax();
			}
			if(Cdatabuf > MaxValue)
			{
				MaxValue = Cdatabuf;
				CompMax = Cdatabuf + DiffThreshold;
				LimitMinMax();
			}
            if( m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x + 1].bVisitedFlag == false )
            {
                DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x + 1]       		= DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
                m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x + 1].bVisitedFlag	= true;
                m_vPoint[CurrentPoint.y * nWidth +  CurrentPoint.x + 1].ptReturnPoint	= CurrentPoint;
				unsigned char* CurPData2 = add_image.ptr<uchar>(CurrentPoint.y);
				*(CurPData2 + (CurrentPoint.x + 1)) = (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
				//add_image->imageData[CurrentPoint.y  * nWidth + CurrentPoint.x + 1]		= (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
                CurrentPoint.x++;

                if(CurrentPoint.x >= nWidth - 1)
                    CurrentPoint.x = nWidth - 1;

                if(*EndX <= CurrentPoint.x)
                    *EndX = CurrentPoint.x;

                continue;
            }
        }

		if(CurrentPoint.y != 0)
			Cdatabuf = DataBuf[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x];
        if( (CurrentPoint.y != 0) && (Cdatabuf !=0) && (Cdatabuf >= CompMin) && (Cdatabuf <= CompMax) )   // -Y ����
        {
			if(MinValue > Cdatabuf)
			{
				MinValue = Cdatabuf;
				CompMin = Cdatabuf - DiffThreshold;
				LimitMinMax();
			}
			if(Cdatabuf > MaxValue)
			{
				MaxValue = Cdatabuf;
				CompMax = Cdatabuf + DiffThreshold;
				LimitMinMax();
			}
            if( m_vPoint[(CurrentPoint.y - 1) * nWidth +  CurrentPoint.x].bVisitedFlag == false )
            {
                DataBuf[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x]                 = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
                m_vPoint[(CurrentPoint.y - 1) * nWidth +  CurrentPoint.x].bVisitedFlag	= true;
                m_vPoint[(CurrentPoint.y - 1) * nWidth +  CurrentPoint.x].ptReturnPoint = CurrentPoint;
				unsigned char* CurPData3 = add_image.ptr<uchar>(CurrentPoint.y - 1);
				*(CurPData3 + CurrentPoint.x) = (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
				//add_image->imageData[(CurrentPoint.y - 1) * nWidth + CurrentPoint.x]	= (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
                CurrentPoint.y--;

                if(CurrentPoint.y <= 0)
                    CurrentPoint.y = 0;

                if(*StartY >= CurrentPoint.y)
                    *StartY = CurrentPoint.y;

                continue;
            }
        }


		if(CurrentPoint.y != nHeight - 1)
			Cdatabuf = DataBuf[(CurrentPoint.y + 1) * nWidth + CurrentPoint.x];
        if( (CurrentPoint.y != nHeight - 1) && (Cdatabuf !=0) && (Cdatabuf >= CompMin) && (Cdatabuf <= CompMax) )   // +Y ����
        {
			if(MinValue > Cdatabuf)
			{
				MinValue = Cdatabuf;
				CompMin = Cdatabuf - DiffThreshold;
				LimitMinMax();
			}
			if(Cdatabuf > MaxValue)
			{
				MaxValue = Cdatabuf;
				CompMax = Cdatabuf + DiffThreshold;
				LimitMinMax();
			}
            if( m_vPoint[(CurrentPoint.y + 1) * nWidth +  CurrentPoint.x]. bVisitedFlag == false )
            {
                DataBuf[(CurrentPoint.y + 1) * nWidth + CurrentPoint.x]                 = DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x];	// If so, mark it
                m_vPoint[(CurrentPoint.y + 1) * nWidth +  CurrentPoint.x].bVisitedFlag	= true;
                m_vPoint[(CurrentPoint.y + 1) * nWidth +  CurrentPoint.x].ptReturnPoint = CurrentPoint;
				unsigned char* CurPData4 = add_image.ptr<uchar>(CurrentPoint.y + 1);
				*(CurPData4 + CurrentPoint.x) = (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
				//add_image->imageData[(CurrentPoint.y + 1) * nWidth +  CurrentPoint.x]	= (DataBuf[CurrentPoint.y * nWidth + CurrentPoint.x] + 1);
                CurrentPoint.y++;

                if(CurrentPoint.y >= nHeight - 1)
                    CurrentPoint.y = nHeight - 1;

                if(*EndY <= CurrentPoint.y)
                    *EndY = CurrentPoint.y;

                continue;
            }
        }

        if(     (CurrentPoint.x == m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint.x)
            &&	(CurrentPoint.y == m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint.y) )
        {
            break;
        }
        else
        {
            CurrentPoint = m_vPoint[CurrentPoint.y * nWidth + CurrentPoint.x].ptReturnPoint;
        }
    }

	int avg = (MinValue + MaxValue)>>1;
    return avg;
}

/// @brief Blob ���� �� ������ Label�� ���� ������ ũ��(�ȼ� ��)�� ���ϴ� �Լ�
/// @param[in] DataBuf   Labeling�� ����� �̹��� ������ ����
/// @param[in] StartX   Ž�� �������� X��ǥ
/// @param[in] StartY   Ž�� �������� Y��ǥ
/// @param[in] EndX     Ž�� �������� X��ǥ
/// @param[in] EndY     Ž�� �������� Y��ǥ
/// @param[in] nWidth   ������ ������ ���� ����
/// @param[in] nLevel   ������ Blob�� Label
/// @return Ž������ ������ ������ Label�� �ȼ� ��
int CBlobLabeling::__Area(unsigned char *DataBuf, int StartX, int StartY, int EndX, int EndY, int nWidth, int nLevel)
{
    int nArea = 0;
    int nX, nY;

    for (nY = StartY; nY < EndY; nY++)
        for (nX = StartX; nX < EndX; nX++)
            if (DataBuf[nY * nWidth + nX] == nLevel)
                ++nArea;

    return nArea;
}

/// @brief ����/���� ������ ������ �������� ���� Blob ����
/// @remarks Width/Hight �� fRatio���� ���� ���, �װ͵��� ������, ����� rect�� ����\n
/// Input���� ���� rect�� �װ��� ���� nRecNumber�� ������, ����� �� ��
/// @param[in] fRatio ����/���� ����
void CBlobLabeling::BlobWidthHeightSmallRatioConstraint(float fRatio)
{
    m_nBlobs = _BlobWidthHeightSmallRatioConstraint(fRatio, m_nBlobs, m_recBlobs, m_intBlobs, m_nPixels, m_nAvgValue);					//, m_nPixels GY����
}

/// @brief ����/���� ������ ������ �������� ���� Blob �����ϴ� ���� �Լ�
/// @param[in] fRatio   ����/���� ����
/// @param[in] rect     Blob ���� ����
/// @param[in] blobs    Blob Index ����
/// @param[in] nRecNumber Blob ����
/// @return ������ Blob ����
int CBlobLabeling::_BlobWidthHeightSmallRatioConstraint(float fRatio, int nRecNumber, CvRect* rect, int* blobs, int* pixel, int* AvgValue)	//, int* pixel GY����
{
    if(nRecNumber == 0)	return 0;

    int nX;
    int ntempRec = 0;

    CvRect *temp = new CvRect[nRecNumber];
    int *labeled = new int[nRecNumber];
	int	*temp_pixel = new int[nRecNumber];								// GY����
	int	*temp_AvgValue = new int[nRecNumber];

    for(nX = 0; nX < nRecNumber; nX++)
    {
        temp[nX]    = rect[nX];
        labeled[nX] = blobs[nX];
		temp_pixel[nX]	= pixel[nX];									// GY����
		temp_AvgValue[nX]	= AvgValue[nX];
    }

    for(nX = 0; nX < nRecNumber; nX++)
    {
        if( (float)rect[nX].height / rect[nX].width > fRatio )
        {
            rect[ntempRec] = temp[nX];
            blobs[ntempRec] = labeled[nX];
			pixel[ntempRec] = temp_pixel[nX];							// GY����
			AvgValue[ntempRec] = temp_AvgValue[nX];

            ntempRec++;
        }
    }

    delete temp;
    delete labeled;
	delete temp_pixel;													// GY����
	delete temp_AvgValue;

    return ntempRec;
}
/// @brief ����/���� ������ ������ �������� ū Blob ����
/// @remarks Width/Hight �� fRatio���� Ŭ�� ���, �װ͵��� ������, ����� rect�� ����\n
/// Input���� ���� rect�� �װ��� ���� nRecNumber�� ������, ����� �� ��
/// @param[in] fRatio ����/���� ����
void CBlobLabeling::BlobWidthHeightBigRatioConstraint(float fRatio)
{
    m_nBlobs = _BlobWidthHeightBigRatioConstraint(fRatio, m_nBlobs, m_recBlobs, m_intBlobs, m_nPixels, m_nAvgValue);					//, m_nPixels GY����
}

/// @brief ����/���� ������ ������ �������� ���� Blob �����ϴ� ���� �Լ�
/// @param[in] fRatio   ����/���� ����
/// @param[in] rect     Blob ���� ����
/// @param[in] blobs    Blob Index ����
/// @param[in] nRecNumber Blob ����
/// @return ������ Blob ����
int CBlobLabeling::_BlobWidthHeightBigRatioConstraint(float fRatio, int nRecNumber, CvRect* rect, int* blobs, int* pixel, int* AvgValue)	//, int* pixel GY����
{
    if(nRecNumber == 0)	return 0;

    int nX;
    int ntempRec = 0;

    CvRect *temp = new CvRect[nRecNumber];
    int *labeled = new int[nRecNumber];
	int	*temp_pixel = new int[nRecNumber];								// GY����
	int	*temp_AvgValue = new int[nRecNumber];

    for(nX = 0; nX < nRecNumber; nX++)
    {
        temp[nX]    = rect[nX];
        labeled[nX] = blobs[nX];
		temp_pixel[nX]	= pixel[nX];									// GY����
		temp_AvgValue[nX]	= AvgValue[nX];
    }

    for(nX = 0; nX < nRecNumber; nX++)
    {
        if( (float)rect[nX].height / rect[nX].width <= fRatio )
        {
            rect[ntempRec] = temp[nX];
            blobs[ntempRec] = labeled[nX];
			pixel[ntempRec] = temp_pixel[nX];							// GY����
			AvgValue[ntempRec] = temp_AvgValue[nX];

            ntempRec++;
        }
    }

    delete temp;
    delete labeled;
	delete temp_pixel;													// GY����
	delete temp_AvgValue;

    return ntempRec;
}

/// @brief ������ nWidth�� nHeight���� ���� Blob�� ��� ����
/// @param[in] nWidth   ���� ũ�� Threshold
/// @param[in] nHeight  ���� ũ�� Threshold
void CBlobLabeling::BlobSmallSizeConstraint(int nWidth, int nHeight)
{
    m_nBlobs = _BlobSmallSizeConstraint(nWidth, nHeight, m_nBlobs, m_recBlobs, m_intBlobs, m_nPixels, m_nAvgValue);						//, m_nPixels GY����
}

/// @brief ������ nWidth�� nHeight���� ���� Blob�� ��� �����ϴ� ���� �Լ�
/// @param[in] nWidth       ���� ũ�� Threshold
/// @param[in] nHeight      ���� ũ�� Threshold
/// @param[in] nRecNumber   Blob ����
/// @param[in] rect         Blob ���� ����
/// @param[in] blobs        Blob Index ����
/// @return ������ Blob ����
int CBlobLabeling::_BlobSmallSizeConstraint(int nWidth, int nHeight, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue)	//, int* pixel GY����
{
    if(nRecNumber == 0)	return 0;

    int nX;
    int ntempRec = 0;

    CvRect* temp = new CvRect[nRecNumber];
    int* labeled = new int[nRecNumber];
	int	*temp_pixel = new int[nRecNumber];									// GY����
	int	*temp_AvgValue = new int[nRecNumber];
	
	
	

    for(nX = 0; nX < nRecNumber; nX++)
    {
        temp[nX]    = rect[nX];
        labeled[nX] = label[nX];
		temp_pixel[nX]	= pixel[nX];										// GY����
		temp_AvgValue[nX]	= AvgValue[nX];
    }

    for(nX = 0; nX < nRecNumber; nX++)
    {
        if( (rect[nX].width >= nWidth) && (rect[nX].height >= nHeight) )
        {
            temp[ntempRec] = rect[nX];
            labeled[ntempRec] = label[nX];
			temp_pixel[ntempRec] = pixel[nX];								// GY����
			temp_AvgValue[ntempRec] = AvgValue[nX];

            ntempRec++;
        }
    }

    for(nX = 0; nX < ntempRec; nX++)
    {
        rect[nX] = temp[nX];
        label[nX] = labeled[nX];
		pixel[nX] = temp_pixel[nX];											// GY����
		AvgValue[nX] = temp_AvgValue[nX];
		
    }

    delete temp;
    delete labeled;
	delete temp_pixel;														// GY����
	delete temp_AvgValue;

    return ntempRec;
}

/// @brief ������ nWidth�� nHeight���� ū Blob�� ��� ����
/// @param[in] nWidth   ���� ũ�� Threshold
/// @param[in] nHeight  ���� ũ�� Threshold
void CBlobLabeling::BlobBigSizeConstraint(int nWidth, int nHeight)
{
    m_nBlobs = _BlobBigSizeConstraint(nWidth, nHeight, m_nBlobs, m_recBlobs, m_intBlobs, m_nPixels, m_nAvgValue);						//, m_nPixels GY����
}

/// @brief ������ nWidth�� nHeight���� ū Blob�� ��� �����ϴ� ���� �Լ�
/// @param[in] nWidth       ���� ũ�� Threshold
/// @param[in] nHeight      ���� ũ�� Threshold
/// @param[in] nRecNumber   Blob ����
/// @param[in] rect         Blob ���� ����
/// @param[in] blobs        Blob Index ����
/// @return ������ Blob ����
int CBlobLabeling::_BlobBigSizeConstraint(int nWidth, int nHeight, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue)	//, int* pixel GY����
{
    if(nRecNumber == 0)	return 0;

    int nX;
    int ntempRec = 0;

    CvRect* temp = new CvRect [nRecNumber];
    int* labeled = new int [nRecNumber];
	int	*temp_pixel = new int[nRecNumber];									// GY����
	int	*temp_AvgValue = new int[nRecNumber];
	
    for(nX = 0; nX < nRecNumber; nX++)
    {
        temp[nX] = rect[nX];
        labeled[nX] = label[nX];
		temp_pixel[nX]	= pixel[nX];										// GY����
		temp_AvgValue[nX]	= AvgValue[nX];
    }

    for(nX = 0; nX < nRecNumber; nX++)
    {
        if( (rect[nX].width < nWidth) && (rect[nX].height < nHeight) )
        {
            temp[ntempRec] = rect[nX];
            labeled[ntempRec] = label[nX];
			temp_pixel[ntempRec] = pixel[nX];								// GY����
			temp_AvgValue[ntempRec] = AvgValue[nX];

            ntempRec++;
        }
    }

    for(nX = 0; nX < ntempRec; nX++)
    {
        rect[nX] = temp[nX];
        label[nX] = labeled[nX];
		pixel[nX] = temp_pixel[nX];											// GY����
		AvgValue[nX] = temp_AvgValue[nX];
    }

    delete temp;
    delete labeled;
	delete temp_pixel;														// GY����
	delete temp_AvgValue;

    return ntempRec;
}

/// @brief �̹��� �ܰ� �κ����κ��� ������ �Ÿ� �ȿ� �ִ�(�̹��� �ܰ��� ����) Blob�� ����
/// @param[in] marginX  ���� �Ÿ� Threshold
/// @param[in] marginY  ���� �Ÿ� Threshold
void CBlobLabeling::BlobEdgeConstraint(int marginX, int marginY)
{
    m_nBlobs = _BlobEdgeConstraint(marginX, marginY, m_nBlobs, m_recBlobs, m_intBlobs, m_nPixels, m_nAvgValue);							//, m_nPixels GY����
}

/// @brief �̹��� �ܰ� �κ����κ��� ������ �Ÿ� �ȿ� �ִ� Blob�� �����ϴ� ���� �Լ�
/// @param[in] marginX      ���� �Ÿ� Threshold
/// @param[in] marginY      ���� �Ÿ� Threshold
/// @param[in] nRecNumber   Blob ����
/// @param[in] rect         Blob ���� ����
/// @param[in] blobs        Blob Index ����
/// @return ������ Blob ����
int CBlobLabeling::_BlobEdgeConstraint(int marginX, int marginY, int nRecNumber, CvRect *rect, int *label, int* pixel, int* AvgValue)		//, int* pixel GY����
{
    if(nRecNumber == 0)	return 0;

    int nX;
    int ntempRec = 0;

    CvRect* temp = new CvRect [nRecNumber];
    int* labeled = new int [nRecNumber];
	int	*temp_pixel = new int[nRecNumber];										// GY����
	int	*temp_AvgValue = new int[nRecNumber];
	
    for(nX = 0; nX < nRecNumber; nX++)
    {
            temp[nX] = rect[nX];
            labeled[nX] = label[nX];
			temp_pixel[nX]	= pixel[nX];										// GY����
			temp_AvgValue[nX]	= AvgValue[nX];
    }

    for(nX = 0; nX < nRecNumber; nX++)
    {
        int l = rect[nX].x;
        int t = rect[nX].y;
        int r = rect[nX].x + rect[nX].width;
        int b = rect[nX].y + rect[nX].height;

		if(     l > marginX && r < m_Image.cols - marginX
            &&  t > marginY && b < m_Image.rows - marginY )
        {
                temp[ntempRec] = rect[nX];
                labeled[ntempRec] = label[nX];
				temp_pixel[ntempRec] = pixel[nX];								// GY����
				temp_AvgValue[ntempRec] = AvgValue[nX];

                ntempRec++;
        }
    }

    for(nX = 0; nX < ntempRec; nX++)
    {
            rect[nX] = temp[nX];
            label[nX] = labeled[nX];
			pixel[nX] = temp_pixel[nX];											// GY����
			AvgValue[nX] = temp_AvgValue[nX];
    }

    delete temp;
    delete labeled;
	delete temp_pixel;															// GY����
	delete temp_AvgValue;

    return ntempRec;
}

/// @brief �̹����� �ѷ� �� ���� ������ ���� ��ü�� �����ϴ� ������ ������ �� ���� ���� Blob�� ����
/// @param[in] ratio ��ü�� �����ϴ� ���� ����ġ
void CBlobLabeling::BlobIncludeRatioConstraint(float ratio)
{
    m_nBlobs = _BlobIncludeRatioConstraint(ratio, m_nBlobs, m_recBlobs, m_intBlobs, m_nPixels, m_nAvgValue);								//, m_nPixels GY����
}

/// @brief �̹����� �ѷ� �� ���� ������ ���� ��ü�� �����ϴ� ������ ������ �� ���� ���� Blob�� �����ϴ� ���� �Լ�
/// @param[in] ratio        ��ü�� �����ϴ� ���� ����ġ
/// @param[in] nRecNumber   Blob ����
/// @param[in] rect         Blob ���� ����
/// @param[in] blobs        Blob Index ����
/// @return ������ Blob ����
int CBlobLabeling::_BlobIncludeRatioConstraint(float ratio, int nRecNumber, CvRect *rect, int *label, int* pixel, int* AvgValue)			//, int* pixel GY����
{
    if(nRecNumber == 0)	return 0;

    int nX;
    int ntempRec = 0;

    CvRect* temp = new CvRect [nRecNumber];
    int* labeled = new int [nRecNumber];
	int	*temp_pixel = new int[nRecNumber];										// GY����
	int	*temp_AvgValue = new int[nRecNumber];

    for(nX = 0; nX < nRecNumber; nX++)
    {
        temp[nX] = rect[nX];
        labeled[nX] = label[nX];
		temp_pixel[nX]	= pixel[nX];											// GY����
		temp_AvgValue[nX]	= AvgValue[nX];
    }

    for(nX = 0; nX < nRecNumber; nX++)
    {
        int cnt = 0;

		for( int j = 0; j < rect[nX].height; j++ ){
			unsigned char *m_data = m_Image.ptr<uchar>(j + rect[nX].y);
			for( int i = 0; i < rect[nX].width;  i++ )
			{
				unsigned char val = *(m_data + (i + rect[nX].x));
				//unsigned char val = (unsigned char)m_Image->imageData[ (j + rect[nX].y) * m_Image->widthStep + (i + rect[nX].x) ];

				if( val == label[nX] )	cnt++;
			}

			float constraint = (float)cnt / (rect[nX].width * rect[nX].height);

			if( constraint > ratio )
			{
					temp[ntempRec] = rect[nX];
					labeled[ntempRec] = label[nX];
					temp_pixel[ntempRec] = pixel[nX];								// GY����
					temp_AvgValue[ntempRec] = AvgValue[nX];

					ntempRec++;
			}
		}
    }

    for(nX = 0; nX < ntempRec; nX++)
    {
            rect[nX] = temp[nX];
            label[nX] = labeled[nX];
			pixel[nX] = temp_pixel[nX];											// GY����
			AvgValue[nX] = temp_AvgValue[nX];
    }

    delete temp;
    delete labeled;
	delete temp_pixel;															// GY����
	delete temp_AvgValue;

    return ntempRec;
}

/// @brief ���ϴ� ����� �̹��� ������ �����´�
/// @remarks ���ϴ� ����� �̹����� �����ϹǷ�, �Լ� ���� ������ �̹��������� �ʱ�ȭ �Ǿ�� ��.
/// @param[out] dest    ��� �̹���
/// @param[in] nLabel   ���ϴ� Blob�� Label
/// @param[in] nX       image offset x��ǥ
/// @param[in] nY       image offset y��ǥ
void CBlobLabeling::GetBlobImage(cv::Mat dest, int nLabel, int nX, int nY)
{
    CvRect rect = m_recBlobs[ nLabel ];
    int nNum	= m_intBlobs[ nLabel ];

	 //rect.height<20;
	//rect.width<20;
    for( int j = 0; j < rect.height; j++ )
	{
		unsigned char *dstData = m_Image.ptr<uchar>(j + rect.y);
		unsigned char *ndstData = dest.ptr<uchar>(nY + j);
		for( int i = 0; i < rect.width;  i++ )
		{
			unsigned char val = *(dstData + (i + rect.x));
			//unsigned char val = (unsigned char)m_Image->imageData[ (j + rect.y) * m_Image->widthStep + (i + rect.x) ];

			if( val == nNum )	*(ndstData + (nX + i)) = (unsigned char)255;   
			//dest->imageData[ (nY + j) * dest->widthStep + (nX + i) ] = (unsigned char)255;
			else				*(ndstData + (nX + i)) = (unsigned char)0;
			//dest->imageData[ (nY + j) * dest->widthStep + (nX + i) ] = (unsigned char)0;
		}
	}
}

void CBlobLabeling::LimitMinMax()
{
	if(CompMin<=0)			CompMin = 1;
	else if(CompMax>=255)	CompMax = 255;
}
