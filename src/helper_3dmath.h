// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef HELPER_3DMATH_H_
#define HELPER_3DMATH_H_

#include <type_traits>

class Quaternion
{
    public:
        float w;
        float x;
        float y;
        float z;
        
        Quaternion();
        
        Quaternion(float nw, float nx, float ny, float nz);

        Quaternion getProduct(const Quaternion q) const;

        Quaternion getConjugate() const;
        
        float getMagnitude() const;
        
        void normalize();
        
        Quaternion getNormalized() const;
};

template<
    typename T,
    typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr
>
class Vector
{
	public:
	T x;
	T y;
	T z;

	Vector() : x(0), y(0), z(0) {}

	Vector(T nx, T ny, T nz) : x(nx), y(ny), z(nz) {}

	T getMagnitude() const
	{
		return sqrt(x * x + y * y + z * z);
	}

	void normalize()
	{
		T m = getMagnitude();
		x /= m;
		y /= m;
		z /= m;
	}

	Vector<T> getNormalized() const
	{
		Vector<T> r(x, y, z);
		r.normalize();
		return r;
	}

};

class VectorFloat {
    public:
        float x;
        float y;
        float z;

        VectorFloat();
        
        VectorFloat(float nx, float ny, float nz);

        float getMagnitude() const;

        void normalize();
        
        VectorFloat getNormalized();
        
        void rotate(Quaternion* q);

        VectorFloat getRotated(Quaternion* q);
};

#endif /* HELPER_3DMATH_H_ */