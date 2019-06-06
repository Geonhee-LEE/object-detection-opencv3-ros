/// @file BlobLabeling.h
/// @brief Labeling 관련 함수 모음
/// @author 구경모(http://martinblog.net) - 부산대학교 컴퓨터공학과 NR-Lab

#pragma once

#include <cv.h>
#include <highgui.h>

#define HORIZONTAL	0
#define	VERTICAL	1

#define _DEF_MAX_BLOBS	10000   ///<    최대 BLOB 개수

#define max(a, b)	(a > b ? a : b)
#define min(a, b)	(a < b ? a : b)

/// @struct Visited
/// @brief  Labeling 시 방문여부를 저장하는 구조체
typedef struct
{
        bool	bVisitedFlag;
        CvPoint ptReturnPoint;
} Visited;

/// @class CBlobLabeling
/// @brief Labeling 수행 및 각 Blob의 정보 이용을 위한 Class
class  CBlobLabeling
{
public:
	CBlobLabeling(void);
	~CBlobLabeling(void);

public:
	cv::Mat	m_Image;        ///< 레이블링을 위한 이미지(0 또는 255 값을 가지는 1ch 이미지)
        int		m_nThreshold;   ///< 레이블링 Threshold(픽셀의 수)
		int		m_DiffThreshold;	/// 밝기차이 Threshold
        Visited*	m_vPoint;       ///< 레이블링시 방문정보
        int		m_nBlobs;	///< 레이블의 개수
	
        CvRect*		m_recBlobs;	///< 각 레이블 영역정보
        int*		m_intBlobs;	///< 각 레이블 인덱스

		int*		m_nPixels;			//해당 lable의 픽셀수
		int*		m_nAvgValue;

		int CompMin;
		int CompMax;
		void LimitMinMax();

public:
	// 레이블링 이미지 선택
	void	SetParam(cv::Mat image, int nThreshold, int DiffThreshold);

	// 레이블링(실행)
	void	DoLabeling(cv::Mat image);

private:
	// 레이블링(동작)
	int     Labeling(cv::Mat image, cv::Mat add_image, int nThreshold, int DiffThreshold);
        void    DetectLabelingRegion(int nLabelNumber, unsigned char *DataBuf, int nWidth, int nHeight);

	// 포인트 초기화
        void	InitvPoint(int nWidth, int nHeight);
        void	DeletevPoint();

	// 레이블링(실제 알고리즘)
		int     _Labeling(cv::Mat add_image, unsigned char* DataBuf, int nWidth, int nHeight, int nThreshold, int DiffThreshold);
	
	// _Labling 내부 사용 함수
		int     __NRFIndNeighbor(cv::Mat add_image, unsigned char *DataBuf, int InitValue, int DiffThreshold , int nWidth, int nHeight, int nPosX, int nPosY, int *StartX, int *StartY, int *EndX, int *EndY);
        int     __Area(unsigned char *DataBuf, int StartX, int StartY, int EndX, int EndY, int nWidth, int nLevel);


	// 가로-세로비가 비교값 보다 작은 레이블 제거
public:
	void	BlobWidthHeightSmallRatioConstraint(float fRatio);
private:
        int	_BlobWidthHeightSmallRatioConstraint(float fRatio, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY삽입
	// 가로-세로비가 비교값 보다 큰 레이블 제거
public:
	void	BlobWidthHeightBigRatioConstraint(float fRatio);
private:
        int	_BlobWidthHeightBigRatioConstraint(float fRatio, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY삽입

	// 가로, 세로 크기가 비교값 보다 작은 레이블 제거
public:
	void	BlobSmallSizeConstraint(int nWidth, int nHeight);
		//int nWidth>11;
		//int nHeight>11;
private:
        int	_BlobSmallSizeConstraint(int nWidth, int nHeight, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY삽입

	// 가로, 세로 크기가 비교값 보다 큰 레이블 제거
public:
	void	BlobBigSizeConstraint(int nWidth, int nHeight);
		//int nWidth=20;	
		//int nHeight=20;
		
private:
        int	_BlobBigSizeConstraint(int nWidth, int nHeight, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY삽입

        // 중심점이 이미지의 모서리에 존재하는 레이블 제거
public:
        void    BlobEdgeConstraint(int marginX, int marginY);
private:
        int    _BlobEdgeConstraint(int marginX, int marginY, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY삽입

        // 객체의 화소 수대 크기 비율이 비교값 보다 작은 레이블 제거
public:
        void    BlobIncludeRatioConstraint(float ratio);
private:
        int    _BlobIncludeRatioConstraint(float ratio, int nRecNumber, CvRect* rect, int* label, int* pixel, int* AvgValue);			//, int* pixel GY삽입


	// 원하는 블롭의 이미지 정보만 가져온다.
public:
	void	GetBlobImage(cv::Mat dest, int nLabel, int nX = 0, int nY = 0);


};

