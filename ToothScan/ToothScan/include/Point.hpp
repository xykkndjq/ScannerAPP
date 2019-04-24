#ifndef ORTH_POINT_HPP
#define ORTH_POINT_HPP


//////////////////////////////// Point3_ ////////////////////////////////

/** @brief Template class for 3D points specified by its coordinates `x`, `y` and `z`.

The following Point3_\<\> aliases are available:
@code
typedef Point3_<int> Point3i;
typedef Point3_<float> Point3f;
typedef Point3_<double> Point3d;
@endcode
*/

namespace orth
{

	template<typename _Tp> class Point3_
	{
	public:
		typedef _Tp value_type;

		//! default constructor
		Point3_();
		Point3_(_Tp _x, _Tp _y, _Tp _z);
		//_Tp Point3_(const int i);
		Point3_(const Point3_& pt);
		//explicit Point3_(const Point_<_Tp>& pt);

		Point3_& operator = (const Point3_& pt);
		//! conversion to another data type
		template<typename _Tp2> operator Point3_<_Tp2>() const;

		//! dot product
		_Tp dot(const Point3_& pt) const;
		//! dot product computed in double-precision arithmetics
		double ddot(const Point3_& pt) const;
		//! cross product of the 2 3D points
		Point3_ cross(const Point3_& pt) const;
		_Tp x; //!< x coordinate of the 3D point
		_Tp y; //!< y coordinate of the 3D point
		_Tp z; //!< z coordinate of the 3D point
			   //_Tp* data = &x;

			   //data buffer's address
		_Tp* data();

		//normalization current vector
		void normalize();

		//template<typename _Tp> static inline
		//	Point3_<_Tp> operator - (const Point3_<_Tp>& a, const Point3_<_Tp>& b);

		//template<typename _Tp> static inline
		//	Point3_<_Tp> operator - (const Point3_<_Tp>& a, const Point3_<_Tp>& b);


	};

	typedef Point3_<int> Point3i;
	typedef Point3_<float> Point3f;
	typedef Point3_<double> Point3d;
	typedef Point3_<unsigned char> Point3uc;
	typedef Point3_<unsigned short> Point3us;
	typedef Point3_<unsigned int> Point3ui;

	template<typename _Tp> inline
		_Tp* Point3_<_Tp>::data()
	{
		return &x;
	}

	template<typename _Tp> inline
		void Point3_<_Tp>::normalize()
	{
		double length = sqrt(x*x + y*y + z*z);
		if (length == 0)
		{
			std::cout << " zero value in normalization " << std::endl;
			return;
		}
		x = (_Tp)(x / length);
		y = (_Tp)(y / length);
		z = (_Tp)(z / length);
	}

	template<typename _Tp> static inline
		std::ostream& operator << (std::ostream& out, const Point3_<_Tp>& p)
	{
		out << "[" << p.x << ", " << p.y << ", " << p.z << "]";
		return out;
	}

	template<typename _Tp> inline
		Point3_<_Tp>::Point3_()
		: x(0), y(0), z(0) {}

	template<typename _Tp> inline
		Point3_<_Tp>::Point3_(_Tp _x, _Tp _y, _Tp _z)
		: x(_x), y(_y), z(_z) {}

	//template<typename _Tp> inline
	//	_Tp Point3_<_Tp>::Point3_(const int i)
	//	{
	//	return *(&x+i);
	//}

	template<typename _Tp> inline
		Point3_<_Tp>::Point3_(const Point3_& pt)
		: x(pt.x), y(pt.y), z(pt.z) {}



	template<typename _Tp> template<typename _Tp2> inline
		Point3_<_Tp>::operator Point3_<_Tp2>() const
	{
		return Point3_<_Tp2>((x), (y), (z));
	}


	template<typename _Tp> inline
		Point3_<_Tp>& Point3_<_Tp>::operator = (const Point3_& pt)
	{
		x = pt.x; y = pt.y; z = pt.z;
		return *this;
	}

	template<typename _Tp> inline
		_Tp Point3_<_Tp>::dot(const Point3_& pt) const
	{
		return (x*pt.x + y*pt.y + z*pt.z);
	}

	template<typename _Tp> inline
		double Point3_<_Tp>::ddot(const Point3_& pt) const
	{
		return (double)x*pt.x + (double)y*pt.y + (double)z*pt.z;
	}

	template<typename _Tp> inline
		Point3_<_Tp> Point3_<_Tp>::cross(const Point3_<_Tp>& pt) const
	{
		return Point3_<_Tp>(y*pt.z - z*pt.y, z*pt.x - x*pt.z, x*pt.y - y*pt.x);
	}


	template<typename _Tp> static inline
		Point3_<_Tp>& operator += (Point3_<_Tp>& a, const Point3_<_Tp>& b)
	{
		a.x += b.x;
		a.y += b.y;
		a.z += b.z;
		return a;
	}

	template<typename _Tp> static inline
		Point3_<_Tp>& operator -= (Point3_<_Tp>& a, const Point3_<_Tp>& b)
	{
		a.x -= b.x;
		a.y -= b.y;
		a.z -= b.z;
		return a;
	}

	template<typename _Tp> static inline
		Point3_<_Tp>& operator *= (Point3_<_Tp>& a, int b)
	{
		a.x = (a.x * b);
		a.y = (a.y * b);
		a.z = (a.z * b);
		return a;
	}

	template<typename _Tp> static inline
		Point3_<_Tp>& operator *= (Point3_<_Tp>& a, float b)
	{
		a.x = (a.x * b);
		a.y = (a.y * b);
		a.z = (a.z * b);
		return a;
	}

	template<typename _Tp> static inline
		Point3_<_Tp>& operator *= (Point3_<_Tp>& a, double b)
	{
		a.x = (a.x * b);
		a.y = (a.y * b);
		a.z = (a.z * b);
		return a;
	}

	template<typename _Tp> static inline
		Point3_<_Tp>& operator /= (Point3_<_Tp>& a, int b)
	{
		a.x = (a.x / b);
		a.y = (a.y / b);
		a.z = (a.z / b);
		return a;
	}

	template<typename _Tp> static inline
		Point3_<_Tp>& operator /= (Point3_<_Tp>& a, float b)
	{
		a.x = (a.x / b);
		a.y = (a.y / b);
		a.z = (a.z / b);
		return a;
	}

	template<typename _Tp> static inline
		Point3_<_Tp>& operator /= (Point3_<_Tp>& a, double b)
	{
		a.x = (a.x / b);
		a.y = (a.y / b);
		a.z = (a.z / b);
		return a;
	}

	template<typename _Tp> static inline
		double norm(const Point3_<_Tp>& pt)
	{
		return std::sqrt((double)pt.x*pt.x + (double)pt.y*pt.y + (double)pt.z*pt.z);
	}

	template<typename _Tp> static inline
		bool operator == (const Point3_<_Tp>& a, const Point3_<_Tp>& b)
	{
		return a.x == b.x && a.y == b.y && a.z == b.z;
	}

	template<typename _Tp> static inline
		bool operator != (const Point3_<_Tp>& a, const Point3_<_Tp>& b)
	{
		return a.x != b.x || a.y != b.y || a.z != b.z;
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator + (const Point3_<_Tp>& a, const Point3_<_Tp>& b)
	{
		return Point3_<_Tp>((a.x + b.x), (a.y + b.y), (a.z + b.z));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator - (const Point3_<_Tp>& a, const Point3_<_Tp>& b)
	{
		return Point3_<_Tp>((a.x - b.x), (a.y - b.y), (a.z - b.z));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator - (const Point3_<_Tp>& a)
	{
		return Point3_<_Tp>((-a.x), (-a.y), (-a.z));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator * (const Point3_<_Tp>& a, int b)
	{
		return Point3_<_Tp>((a.x*b), (a.y*b), (a.z*b));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator * (int a, const Point3_<_Tp>& b)
	{
		return Point3_<_Tp>((b.x * a), (b.y * a), (b.z * a));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator * (const Point3_<_Tp>& a, float b)
	{
		return Point3_<_Tp>((a.x * b), (a.y * b), (a.z * b));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator * (float a, const Point3_<_Tp>& b)
	{
		return Point3_<_Tp>((b.x * a), (b.y * a), (b.z * a));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator * (const Point3_<_Tp>& a, double b)
	{
		return Point3_<_Tp>((a.x * b), (a.y * b), (a.z * b));
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator * (double a, const Point3_<_Tp>& b)
	{
		return Point3_<_Tp>((b.x * a), (b.y * a), (b.z * a));
	}


	template<typename _Tp> static inline
		Point3_<_Tp> operator / (const Point3_<_Tp>& a, int b)
	{
		Point3_<_Tp> tmp(a);
		tmp /= b;
		return tmp;
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator / (const Point3_<_Tp>& a, float b)
	{
		Point3_<_Tp> tmp(a);
		tmp /= b;
		return tmp;
	}

	template<typename _Tp> static inline
		Point3_<_Tp> operator / (const Point3_<_Tp>& a, double b)
	{
		Point3_<_Tp> tmp(a);
		tmp /= b;
		return tmp;
	}
}



#endif // !ORTH_POINT_HPP

