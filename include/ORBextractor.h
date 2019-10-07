
#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <cstdint>

namespace orb {

template<typename _Tp>
class Point_
{
public:
	typedef _Tp value_type;

	Point_();
	Point_(_Tp _x, _Tp _y);
	Point_(const Point_& pt);

	Point_& operator=(const Point_& pt);

	_Tp x;
	_Tp y;
};

typedef Point_<int> Point2i;
typedef Point_<int64_t> Point2l;
typedef Point_<float> Point2f;
typedef Point_<double> Point2d;
typedef Point2i Point;

class KeyPoint
{
public:
	KeyPoint();

	KeyPoint(Point2f _pt, float _size, float _angle = -1,
		float _response = 0, int _octave = 0, int _class_id = -1);
	KeyPoint(float x, float y, float _size, float _angle = -1,
		float _response = 0, int _octave = 0, int _class_id = -1);

	Point2f pt; //!< coordinates of the keypoints
	float size; //!< diameter of the meaningful keypoint neighborhood
	float angle; //!< computed orientation of the keypoint (-1 if not applicable);
							//!< it's in [0,360) degrees and measured relative to
							//!< image coordinate system, ie in clockwise.
	float response; //!< the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
	int octave; //!< octave (pyramid layer) from which the keypoint has been extracted
	int class_id; //!< object 
};

class Mat
{
public:
	Mat();
	Mat(int rows, int cols, int type);
	Mat(const Mat& m);
	Mat(int rows, int cols, int type, void* data, size_t step = 0);
	~Mat();
	Mat& operator=(const Mat& m);
	Mat rowRange(int startrow, int endrow) const;
	Mat colRange(int startcol, int endcol) const;
	Mat clone() const;
};

class ExtractorNode
{
public:
	ExtractorNode() :bNoMore(false) {}

	void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

	std::vector<KeyPoint> vKeys;
	Point2i UL, UR, BL, BR;
	std::list<ExtractorNode>::iterator lit;
	bool bNoMore;
};

class ORBextractor
{
public:

	enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

	ORBextractor(int nfeatures, float scaleFactor, int nlevels,
		int iniThFAST, int minThFAST);

	~ORBextractor() {}

	// Compute the ORB features and descriptors on an image.
	// ORB are dispersed on the image using an octree.
	// Mask is ignored in the current implementation.
	void operator()(Mat image, Mat mask,
		std::vector<KeyPoint>& keypoints,
		Mat descriptors);

	int inline GetLevels() {
		return nlevels;
	}

	float inline GetScaleFactor() {
		return scaleFactor;
	}

	std::vector<float> inline GetScaleFactors() {
		return mvScaleFactor;
	}

	std::vector<float> inline GetInverseScaleFactors() {
		return mvInvScaleFactor;
	}

	std::vector<float> inline GetScaleSigmaSquares() {
		return mvLevelSigma2;
	}

	std::vector<float> inline GetInverseScaleSigmaSquares() {
		return mvInvLevelSigma2;
	}

	std::vector<Mat> mvImagePyramid;

protected:

	void ComputePyramid(Mat image);
	void ComputeKeyPointsOctTree(std::vector<std::vector<KeyPoint> >& allKeypoints);
	std::vector<KeyPoint> DistributeOctTree(const std::vector<KeyPoint>& vToDistributeKeys, const int &minX,
		const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

	void ComputeKeyPointsOld(std::vector<std::vector<KeyPoint> >& allKeypoints);
	std::vector<Point> pattern;

	int nfeatures;
	double scaleFactor;
	int nlevels;
	int iniThFAST;
	int minThFAST;

	std::vector<int> mnFeaturesPerLevel;

	std::vector<int> umax;

	std::vector<float> mvScaleFactor;
	std::vector<float> mvInvScaleFactor;
	std::vector<float> mvLevelSigma2;
	std::vector<float> mvInvLevelSigma2;
};

}

#endif

