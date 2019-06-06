/// @file BlobLabeling.h
/// @brief Labeling ���� �Լ� ����
/// @author �����(http://martinblog.net) - �λ���б� ��ǻ�Ͱ��а� NR-Lab

#pragma once

#include <cv.h>
#include <highgui.h>

#define HORIZONTAL	0
#define	VERTICAL	1

#define _DEF_MAX_BLOBS	10000   ///<    �ִ� BLOB ����

#define max(a, b)	(a > b ? a : b)
#define min(a, b)	(a < b ? a : b)

/// @struct Visited
/// @brief  Labeling �� �湮���θ� �����ϴ� ����ü
typedef struct
{
        bool	bVisitedFlag;
        CvPoint ptReturnPoint;
} Visited;

/// @class CBlobLabeling
/// @brief Labeling ���� �� �� Blob�� ���� �̿��� ���� Class
class  CBlobLabeling
{
public:
	CBlobLabeling(void);
	~CBlobLabeling(void);

public:
	cv::Mat	m_Image;        ///< ���̺��� ���� �̹���(0 �Ǵ� 255 ���� ������ 1ch �̹���)
        int		m_nThreshold;   ///< ���̺� Threshold(�ȼ��� ��)
		int		m_DiffThreshold;	/// ������� Threshold
        Visited*	m_vPoint;       ///< ���̺��� �湮����
        int		m_nBlobs;	///< ���̺��� ����
	
        CvRect*		m_recBlobs;	///< �� ���̺� ��������
        int*		m_intBlobs;	///< �� ���̺� �ε���

		int*		m_nPixels;			//�ش� lable�� �ȼ���
		int*		m_nAvgValue;

		int CompMin;
		int CompMax;
		void LimitMinMax();

public:
	// ���̺� �̹��� ����
	void	SetParam(cv::Mat image, int nThreshold, int DiffThreshold);

	// ���̺�(����)
	void	DoLabeling(cv::Mat image);

private:
	// ���̺�(����)
	int     Labeling(cv::Mat image, cv::Mat add_image, int nThreshold, int DiffThreshold);
        void    DetectLabelingRegion(int nLabelNumber, unsigned char *DataBuf, int nWidth, int nHeight);

	// ����Ʈ �ʱ�ȭ
        void	InitvPoint(int nWidth, int nHeight);
        void	DeletevPoint();

	// ���̺�(���� �˰���)
		int     _Labeling(cv::Mat add_image, unsigned char* DataBuf, int nWidth, int nHeight, int nThreshold, int DiffThreshold);
	
	// _Labling ���� ��� �Լ�
		int     __NRFIndNeighbor(cv::Mat add_image, unsigned char *DataBuf, int InitValue, int DiffThreshold , int nWidth, int nHeight, int nPosX, int nPosY, int *StartX, int *StartY, int *EndX, int *EndY);
        int     __Area(unsigned char *DataBuf, int StartX, int StartY, int EndX, int EndY, int nWidth, int nLevel);


	// ����-���κ� �񱳰� ���� ���� ���̺� ����
public:
	void	BlobWidthHeightSmallRatioConstraint(float fRatio);
private:
        int	_BlobWidthHeightSmallRatioConstraint(float fRatio, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY����
	// ����-���κ� �񱳰� ���� ū ���̺� ����
public:
	void	BlobWidthHeightBigRatioConstraint(float fRatio);
private:
        int	_BlobWidthHeightBigRatioConstraint(float fRatio, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY����

	// ����, ���� ũ�Ⱑ �񱳰� ���� ���� ���̺� ����
public:
	void	BlobSmallSizeConstraint(int nWidth, int nHeight);
		//int nWidth>11;
		//int nHeight>11;
private:
        int	_BlobSmallSizeConstraint(int nWidth, int nHeight, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY����

	// ����, ���� ũ�Ⱑ �񱳰� ���� ū ���̺� ����
public:
	void	BlobBigSizeConstraint(int nWidth, int nHeight);
		//int nWidth=20;	
		//int nHeight=20;
		
private:
        int	_BlobBigSizeConstraint(int nWidth, int nHeight, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY����

        // �߽����� �̹����� �𼭸��� �����ϴ� ���̺� ����
public:
        void    BlobEdgeConstraint(int marginX, int marginY);
private:
        int    _BlobEdgeConstraint(int marginX, int marginY, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY����

        // ��ü�� ȭ�� ���� ũ�� ������ �񱳰� ���� ���� ���̺� ����
public:
        void    BlobIncludeRatioConstraint(float ratio);
private:
        int    _BlobIncludeRatioConstraint(float ratio, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY����


	// ���ϴ� ����� �̹��� ������ �����´�.
public:
	void	GetBlobImage(cv::Mat dest, int nLabel, int nX = 0, int nY = 0);


};

