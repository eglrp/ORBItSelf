#ifndef _COMMON_HPP
#define _COMMON_HPP 

#include <cstdint>

#define PI 3.141592653589793238462643383
#define OUT
#define IN

typedef unsigned char uchar;

template<typename _Tp> class Point_
{
public:
	typedef _Tp value_type;

	Point_();
	Point_(_Tp _x, _Tp _y);
	Point_(const Point_& pt);
	Point_(Point_&& pt) noexcept;

	Point_& operator = (const Point_& pt);
	Point_& operator = (Point_&& pt) noexcept;

	
	_Tp x; //!< x coordinate of the point
	_Tp y; //!< y coordinate of the point
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

	KeyPoint(Point2f _pt, float _size, float _angle = -1, float _response = 0, int _octave = 0, int _class_id = -1);

	KeyPoint(float x, float y, float _size, float _angle = -1, float _response = 0, int _octave = 0, int _class_id = -1);

	Point2f pt; //!< coordinates of the keypoints
	float size; //!< diameter of the meaningful keypoint neighborhood
	float angle; //!< computed orientation of the keypoint (-1 if not applicable);
							//!< it's in [0,360) degrees and measured relative to
							//!< image coordinate system, ie in clockwise.
	float response; //!< the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
	int octave; //!< octave (pyramid layer) from which the keypoint has been extracted
	int class_id; //!< object class (if the keypoints need to be clustered by an object they belong to)
};

class Mat
{
public:
	Mat();
	Mat(int rows, int cols, int type);
	~Mat();
	Mat& operator = (const Mat& m);
	int rows, cols;
	uchar* data;
protected:
private:
};

#endif