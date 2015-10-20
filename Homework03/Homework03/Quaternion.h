#include <math.h>
#include <vector>
#include <fvec.h>

//! Quaternion class for representing rotations.
		/** It provides cheap combinations and avoids gimbal locks.
		Also useful for interpolations. */
		class quaternion
		{
		public:

			//! Default Constructor
			quaternion() : X(0.0f), Y(0.0f), Z(0.0f), W(1.0f) {}

			//! Constructor
			quaternion(float x, float y, float z, float w) : X(x), Y(y), Z(z), W(w) { }

			//! Constructor which converts euler angles (radians) to a quaternion
			quaternion(float x, float y, float z);

			//! Constructor which converts euler angles (radians) to a quaternion
			quaternion(const vector3df& vec);

#ifndef IRR_TEST_BROKEN_QUATERNION_USE
			//! Constructor which converts a matrix to a quaternion
			quaternion(const matrix4& mat);
#endif

			//! Equalilty operator
			bool operator==(const quaternion& other) const;

			//! inequality operator
			bool operator!=(const quaternion& other) const;

			//! Assignment operator
			inline quaternion& operator=(const quaternion& other);

#ifndef IRR_TEST_BROKEN_QUATERNION_USE
			//! Matrix assignment operator
			inline quaternion& operator=(const matrix4& other);
#endif

			//! Add operator
			quaternion operator+(const quaternion& other) const;

			//! Multiplication operator
			//! Be careful, unfortunately the operator order here is opposite of that in CMatrix4::operator*
			quaternion operator*(const quaternion& other) const;

			//! Multiplication operator with scalar
			quaternion operator*(float s) const;

			//! Multiplication operator with scalar
			quaternion& operator*=(float s);

			//! Multiplication operator
			vector3df operator*(const vector3df& v) const;

			//! Multiplication operator
			quaternion& operator*=(const quaternion& other);

			//! Calculates the dot product
			inline float dotProduct(const quaternion& other) const;

			//! Sets new quaternion
			inline quaternion& set(float x, float y, float z, float w);

			//! Sets new quaternion based on euler angles (radians)
			inline quaternion& set(float x, float y, float z);

			//! Sets new quaternion based on euler angles (radians)
			inline quaternion& set(const vector3df& vec);

			//! Sets new quaternion from other quaternion
			inline quaternion& set(const quaternion& quat);

			//! returns if this quaternion equals the other one, taking floating point rounding errors into account
			inline bool equals(const quaternion& other,
				const float tolerance = ROUNDING_ERROR_float) const;

			//! Normalizes the quaternion
			inline quaternion& normalize();

#ifndef IRR_TEST_BROKEN_QUATERNION_USE
			//! Creates a matrix from this quaternion
			matrix4 getMatrix() const;
#endif

			//! Creates a matrix from this quaternion
			void getMatrix(matrix4 &dest, const vector3df &translation = vector3df()) const;

			/*!
			Creates a matrix from this quaternion
			Rotate about a center point
			shortcut for
			quaternion q;
			q.rotationFromTo ( vin[i].Normal, forward );
			q.getMatrixCenter ( lookat, center, newPos );

			matrix4 m2;
			m2.setInverseTranslation ( center );
			lookat *= m2;

			matrix4 m3;
			m2.setTranslation ( newPos );
			lookat *= m3;

			*/
			void getMatrixCenter(matrix4 &dest, const vector3df &center, const vector3df &translation) const;

			//! Creates a matrix from this quaternion
			inline void getMatrix_transposed(matrix4 &dest) const;

			//! Inverts this quaternion
			quaternion& makeInverse();

			//! Set this quaternion to the linear interpolation between two quaternions
			/** \param q1 First quaternion to be interpolated.
			\param q2 Second quaternion to be interpolated.
			\param time Progress of interpolation. For time=0 the result is
			q1, for time=1 the result is q2. Otherwise interpolation
			between q1 and q2.
			*/
			quaternion& lerp(quaternion q1, quaternion q2, float time);

			//! Set this quaternion to the result of the spherical interpolation between two quaternions
			/** \param q1 First quaternion to be interpolated.
			\param q2 Second quaternion to be interpolated.
			\param time Progress of interpolation. For time=0 the result is
			q1, for time=1 the result is q2. Otherwise interpolation
			between q1 and q2.
			\param threshold To avoid inaccuracies at the end (time=1) the
			interpolation switches to linear interpolation at some point.
			This value defines how much of the remaining interpolation will
			be calculated with lerp. Everything from 1-threshold up will be
			linear interpolation.
			*/
			quaternion& slerp(quaternion q1, quaternion q2,
				float time, float threshold = .05f);

			//! Create quaternion from rotation angle and rotation axis.
			/** Axis must be unit length.
			The quaternion representing the rotation is
			q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k).
			\param angle Rotation Angle in radians.
			\param axis Rotation axis. */
			quaternion& fromAngleAxis(float angle, const vector3df& axis);

			//! Fills an angle (radians) around an axis (unit vector)
			void toAngleAxis(float &angle, vector3df& axis) const;

			//! Output this quaternion to an euler angle (radians)
			void toEuler(vector3df& euler) const;

			//! Set quaternion to identity
			quaternion& makeIdentity();

			//! Set quaternion to represent a rotation from one vector to another.
			quaternion& rotationFromTo(const vector3df& from, const vector3df& to);

			//! Quaternion elements.
			float X; // vectorial (imaginary) part
			float Y;
			float Z;
			float W; // real part
		};


		// Constructor which converts euler angles to a quaternion
		inline quaternion::quaternion(float x, float y, float z)
		{
			set(x, y, z);
		}


		// Constructor which converts euler angles to a quaternion
		inline quaternion::quaternion(const vector3df& vec)
		{
			set(vec.X, vec.Y, vec.Z);
		}

#ifndef IRR_TEST_BROKEN_QUATERNION_USE
		// Constructor which converts a matrix to a quaternion
		inline quaternion::quaternion(const matrix4& mat)
		{
			(*this) = mat;
		}
#endif

		// equal operator
		inline bool quaternion::operator==(const quaternion& other) const
		{
			return ((X == other.X) &&
				(Y == other.Y) &&
				(Z == other.Z) &&
				(W == other.W));
		}

		// inequality operator
		inline bool quaternion::operator!=(const quaternion& other) const
		{
			return !(*this == other);
		}

		// assignment operator
		inline quaternion& quaternion::operator=(const quaternion& other)
		{
			X = other.X;
			Y = other.Y;
			Z = other.Z;
			W = other.W;
			return *this;
		}

#ifndef IRR_TEST_BROKEN_QUATERNION_USE
		// matrix assignment operator
		inline quaternion& quaternion::operator=(const matrix4& m)
		{
			const float diag = m[0] + m[5] + m[10] + 1;

			if (diag > 0.0f)
			{
				const float scale = sqrtf(diag) * 2.0f; // get scale from diagonal

													  // TODO: speed this up
				X = (m[6] - m[9]) / scale;
				Y = (m[8] - m[2]) / scale;
				Z = (m[1] - m[4]) / scale;
				W = 0.25f * scale;
			}
			else
			{
				if (m[0]>m[5] && m[0]>m[10])
				{
					// 1st element of diag is greatest value
					// find scale according to 1st element, and double it
					const float scale = sqrtf(1.0f + m[0] - m[5] - m[10]) * 2.0f;

					// TODO: speed this up
					X = 0.25f * scale;
					Y = (m[4] + m[1]) / scale;
					Z = (m[2] + m[8]) / scale;
					W = (m[6] - m[9]) / scale;
				}
				else if (m[5]>m[10])
				{
					// 2nd element of diag is greatest value
					// find scale according to 2nd element, and double it
					const float scale = sqrtf(1.0f + m[5] - m[0] - m[10]) * 2.0f;

					// TODO: speed this up
					X = (m[4] + m[1]) / scale;
					Y = 0.25f * scale;
					Z = (m[9] + m[6]) / scale;
					W = (m[8] - m[2]) / scale;
				}
				else
				{
					// 3rd element of diag is greatest value
					// find scale according to 3rd element, and double it
					const float scale = sqrtf(1.0f + m[10] - m[0] - m[5]) * 2.0f;

					// TODO: speed this up
					X = (m[8] + m[2]) / scale;
					Y = (m[9] + m[6]) / scale;
					Z = 0.25f * scale;
					W = (m[1] - m[4]) / scale;
				}
			}

			return normalize();
		}
#endif


		// multiplication operator
		inline quaternion quaternion::operator*(const quaternion& other) const
		{
			quaternion tmp;

			tmp.W = (other.W * W) - (other.X * X) - (other.Y * Y) - (other.Z * Z);
			tmp.X = (other.W * X) + (other.X * W) + (other.Y * Z) - (other.Z * Y);
			tmp.Y = (other.W * Y) + (other.Y * W) + (other.Z * X) - (other.X * Z);
			tmp.Z = (other.W * Z) + (other.Z * W) + (other.X * Y) - (other.Y * X);

			return tmp;
		}


		// multiplication operator
		inline quaternion quaternion::operator*(float s) const
		{
			return quaternion(s*X, s*Y, s*Z, s*W);
		}


		// multiplication operator
		inline quaternion& quaternion::operator*=(float s)
		{
			X *= s;
			Y *= s;
			Z *= s;
			W *= s;
			return *this;
		}

		// multiplication operator
		inline quaternion& quaternion::operator*=(const quaternion& other)
		{
			return (*this = other * (*this));
		}

		// add operator
		inline quaternion quaternion::operator+(const quaternion& b) const
		{
			return quaternion(X + b.X, Y + b.Y, Z + b.Z, W + b.W);
		}

#ifndef IRR_TEST_BROKEN_QUATERNION_USE
		// Creates a matrix from this quaternion
		inline matrix4 quaternion::getMatrix() const
		{
			matrix4 m;
			getMatrix(m);
			return m;
		}
#endif

		/*!
		Creates a matrix from this quaternion
		*/
		inline void quaternion::getMatrix(matrix4 &dest,
			const vector3df &center) const
		{
			dest[0] = 1.0f - 2.0f*Y*Y - 2.0f*Z*Z;
			dest[1] = 2.0f*X*Y + 2.0f*Z*W;
			dest[2] = 2.0f*X*Z - 2.0f*Y*W;
			dest[3] = 0.0f;

			dest[4] = 2.0f*X*Y - 2.0f*Z*W;
			dest[5] = 1.0f - 2.0f*X*X - 2.0f*Z*Z;
			dest[6] = 2.0f*Z*Y + 2.0f*X*W;
			dest[7] = 0.0f;

			dest[8] = 2.0f*X*Z + 2.0f*Y*W;
			dest[9] = 2.0f*Z*Y - 2.0f*X*W;
			dest[10] = 1.0f - 2.0f*X*X - 2.0f*Y*Y;
			dest[11] = 0.0f;

			dest[12] = center.X;
			dest[13] = center.Y;
			dest[14] = center.Z;
			dest[15] = 1.f;

			dest.setDefinitelyIdentityMatrix(false);
		}


		/*!
		Creates a matrix from this quaternion
		Rotate about a center point
		shortcut for
		quaternion q;
		q.rotationFromTo(vin[i].Normal, forward);
		q.getMatrix(lookat, center);

		matrix4 m2;
		m2.setInverseTranslation(center);
		lookat *= m2;
		*/
		inline void quaternion::getMatrixCenter(matrix4 &dest,
			const vector3df &center,
			const vector3df &translation) const
		{
			dest[0] = 1.0f - 2.0f*Y*Y - 2.0f*Z*Z;
			dest[1] = 2.0f*X*Y + 2.0f*Z*W;
			dest[2] = 2.0f*X*Z - 2.0f*Y*W;
			dest[3] = 0.0f;

			dest[4] = 2.0f*X*Y - 2.0f*Z*W;
			dest[5] = 1.0f - 2.0f*X*X - 2.0f*Z*Z;
			dest[6] = 2.0f*Z*Y + 2.0f*X*W;
			dest[7] = 0.0f;

			dest[8] = 2.0f*X*Z + 2.0f*Y*W;
			dest[9] = 2.0f*Z*Y - 2.0f*X*W;
			dest[10] = 1.0f - 2.0f*X*X - 2.0f*Y*Y;
			dest[11] = 0.0f;

			dest.setRotationCenter(center, translation);
		}

		// Creates a matrix from this quaternion
		inline void quaternion::getMatrix_transposed(matrix4 &dest) const
		{
			dest[0] = 1.0f - 2.0f*Y*Y - 2.0f*Z*Z;
			dest[4] = 2.0f*X*Y + 2.0f*Z*W;
			dest[8] = 2.0f*X*Z - 2.0f*Y*W;
			dest[12] = 0.0f;

			dest[1] = 2.0f*X*Y - 2.0f*Z*W;
			dest[5] = 1.0f - 2.0f*X*X - 2.0f*Z*Z;
			dest[9] = 2.0f*Z*Y + 2.0f*X*W;
			dest[13] = 0.0f;

			dest[2] = 2.0f*X*Z + 2.0f*Y*W;
			dest[6] = 2.0f*Z*Y - 2.0f*X*W;
			dest[10] = 1.0f - 2.0f*X*X - 2.0f*Y*Y;
			dest[14] = 0.0f;

			dest[3] = 0.f;
			dest[7] = 0.f;
			dest[11] = 0.f;
			dest[15] = 1.f;

			dest.setDefinitelyIdentityMatrix(false);
		}


		// Inverts this quaternion
		inline quaternion& quaternion::makeInverse()
		{
			X = -X; Y = -Y; Z = -Z;
			return *this;
		}


		// sets new quaternion
		inline quaternion& quaternion::set(float x, float y, float z, float w)
		{
			X = x;
			Y = y;
			Z = z;
			W = w;
			return *this;
		}


		// sets new quaternion based on euler angles
		inline quaternion& quaternion::set(float x, float y, float z)
		{
			f64 angle;

			angle = x * 0.5;
			const f64 sr = sin(angle);
			const f64 cr = cos(angle);

			angle = y * 0.5;
			const f64 sp = sin(angle);
			const f64 cp = cos(angle);

			angle = z * 0.5;
			const f64 sy = sin(angle);
			const f64 cy = cos(angle);

			const f64 cpcy = cp * cy;
			const f64 spcy = sp * cy;
			const f64 cpsy = cp * sy;
			const f64 spsy = sp * sy;

			X = (float)(sr * cpcy - cr * spsy);
			Y = (float)(cr * spcy + sr * cpsy);
			Z = (float)(cr * cpsy - sr * spcy);
			W = (float)(cr * cpcy + sr * spsy);

			return normalize();
		}

		// sets new quaternion based on euler angles
		inline quaternion& quaternion::set(const vector3df& vec)
		{
			return set(vec.X, vec.Y, vec.Z);
		}

		// sets new quaternion based on other quaternion
		inline quaternion& quaternion::set(const quaternion& quat)
		{
			return (*this = quat);
		}


		//! returns if this quaternion equals the other one, taking floating point rounding errors into account
		inline bool quaternion::equals(const quaternion& other, const float tolerance) const
		{
			return equals(X, other.X, tolerance) &&
				equals(Y, other.Y, tolerance) &&
				equals(Z, other.Z, tolerance) &&
				equals(W, other.W, tolerance);
		}


		// normalizes the quaternion
		inline quaternion& quaternion::normalize()
		{
			const float n = X*X + Y*Y + Z*Z + W*W;

			if (n == 1)
				return *this;

			//n = 1.0f / sqrtf(n);
			return (*this *= reciprocal_squareroot(n));
		}


		// set this quaternion to the result of the linear interpolation between two quaternions
		inline quaternion& quaternion::lerp(quaternion q1, quaternion q2, float time)
		{
			const float scale = 1.0f - time;
			return (*this = (q1*scale) + (q2*time));
		}


		// set this quaternion to the result of the interpolation between two quaternions
		inline quaternion& quaternion::slerp(quaternion q1, quaternion q2, float time, float threshold)
		{
			float angle = q1.dotProduct(q2);

			// make sure we use the short rotation
			if (angle < 0.0f)
			{
				q1 *= -1.0f;
				angle *= -1.0f;
			}

			if (angle <= (1 - threshold)) // spherical interpolation
			{
				const float theta = acosf(angle);
				const float invsintheta = reciprocal(sinf(theta));
				const float scale = sinf(theta * (1.0f - time)) * invsintheta;
				const float invscale = sinf(theta * time) * invsintheta;
				return (*this = (q1*scale) + (q2*invscale));
			}
			else // linear interpolation
				return lerp(q1, q2, time);
		}


		// calculates the dot product
		inline float quaternion::dotProduct(const quaternion& q2) const
		{
			return (X * q2.X) + (Y * q2.Y) + (Z * q2.Z) + (W * q2.W);
		}


		//! axis must be unit length, angle in radians
		inline quaternion& quaternion::fromAngleAxis(float angle, const vector3df& axis)
		{
			const float fHalfAngle = 0.5f*angle;
			const float fSin = sinf(fHalfAngle);
			W = cosf(fHalfAngle);
			X = fSin*axis.X;
			Y = fSin*axis.Y;
			Z = fSin*axis.Z;
			return *this;
		}


		inline void quaternion::toAngleAxis(float &angle, vector3df &axis) const
		{
			const float scale = sqrtf(X*X + Y*Y + Z*Z);

			if (iszero(scale) || W > 1.0f || W < -1.0f)
			{
				angle = 0.0f;
				axis.X = 0.0f;
				axis.Y = 1.0f;
				axis.Z = 0.0f;
			}
			else
			{
				const float invscale = reciprocal(scale);
				angle = 2.0f * acosf(W);
				axis.X = X * invscale;
				axis.Y = Y * invscale;
				axis.Z = Z * invscale;
			}
		}

		inline void quaternion::toEuler(vector3df& euler) const
		{
			const f64 sqw = W*W;
			const f64 sqx = X*X;
			const f64 sqy = Y*Y;
			const f64 sqz = Z*Z;
			const f64 test = 2.0 * (Y*W - X*Z);

			if (equals(test, 1.0, 0.000001))
			{
				// heading = rotation about z-axis
				euler.Z = (float)(-2.0*atan2(X, W));
				// bank = rotation about x-axis
				euler.X = 0;
				// attitude = rotation about y-axis
				euler.Y = (float)(PI64 / 2.0);
			}
			else if (equals(test, -1.0, 0.000001))
			{
				// heading = rotation about z-axis
				euler.Z = (float)(2.0*atan2(X, W));
				// bank = rotation about x-axis
				euler.X = 0;
				// attitude = rotation about y-axis
				euler.Y = (float)(PI64 / -2.0);
			}
			else
			{
				// heading = rotation about z-axis
				euler.Z = (float)atan2(2.0 * (X*Y + Z*W), (sqx - sqy - sqz + sqw));
				// bank = rotation about x-axis
				euler.X = (float)atan2(2.0 * (Y*Z + X*W), (-sqx - sqy + sqz + sqw));
				// attitude = rotation about y-axis
				euler.Y = (float)asin(clamp(test, -1.0, 1.0));
			}
		}


		inline vector3df quaternion::operator* (const vector3df& v) const
		{
			// nVidia SDK implementation

			vector3df uv, uuv;
			vector3df qvec(X, Y, Z);
			uv = qvec.crossProduct(v);
			uuv = qvec.crossProduct(uv);
			uv *= (2.0f * W);
			uuv *= 2.0f;

			return v + uv + uuv;
		}

		// set quaternion to identity
		inline quaternion& quaternion::makeIdentity()
		{
			W = 1.f;
			X = 0.f;
			Y = 0.f;
			Z = 0.f;
			return *this;
		}

		inline quaternion& quaternion::rotationFromTo(const vector3df& from, const vector3df& to)
		{
			// Based on Stan Melax's article in Game Programming Gems
			// Copy, since cannot modify local
			vector3df v0 = from;
			vector3df v1 = to;
			v0.normalize();
			v1.normalize();

			const float d = v0.dotProduct(v1);
			if (d >= 1.0f) // If dot == 1, vectors are the same
			{
				return makeIdentity();
			}
			else if (d <= -1.0f) // exactly opposite
			{
				vector3df axis(1.0f, 0.f, 0.f);
				axis = axis.crossProduct(v0);
				if (axis.getLength() == 0)
				{
					axis.set(0.f, 1.f, 0.f);
					axis = axis.crossProduct(v0);
				}
				// same as fromAngleAxis(PI, axis).normalize();
				return set(axis.X, axis.Y, axis.Z, 0).normalize();
			}

			const float s = sqrtf((1 + d) * 2); // optimize inv_sqrt
			const float invs = 1.f / s;
			const vector3df c = v0.crossProduct(v1)*invs;
			return set(c.X, c.Y, c.Z, s * 0.5f).normalize();
		}
