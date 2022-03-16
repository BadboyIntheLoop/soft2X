/**
*
* @class Numbers
*
* @brief Class includes auxiliary functions used in numbers calculations.
*
* 2009 Witold Kruczek @ Flytronic
*/

#ifndef NUMBERS_H
#define NUMBERS_H

// Tung sing edit
#define M_PI            3.1415926535897932385f
#define M_2PI			M_PI*2
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)
// Tung sinh end
#define RadiansToCentiDegrees(x) static_cast<int32_t>(static_cast<float>(x) * RAD_TO_DEG * static_cast<float>(100))
class Numbers
{
public:

	// Tung sinh edit
	// degrees -> radians
	static float radians(float deg)
	{
		return deg*DEG_TO_RAD;
	}

	// radians -> degrees
	// static float degrees(float rad)
	// {
	// 	return rad * RAD_TO_DEG;
	// }
	// radians -> degrees
	static inline float degrees(float rad)
	{
		return rad * RAD_TO_DEG;
	}
	static bool is_negative(float f)
	{
		return (f <= (-1.0 * FLT_EPSILON));
	}

	static bool is_positive(float f)
	{
		return (f >= FLT_EPSILON);
	}

	static float safe_sqrt(float f)
	{
		float ret = sqrtf(f);
		if (isnan(ret)) {
			return 0;
		}
		return ret;
	}

	static float wrap_PI(float f)
	{
		float res = wrap_2PI(f);
		if (res > M_PI) {
			res -= M_2PI;
		}
		return res;
	}

	static float wrap_2PI(float f)
	{
		float res = fmodf(f, M_2PI);
		if (res < 0) {
			res += M_2PI;
		}
		return res;
	}

	// Tung sing end
	static int32_t wrap_180_cd(const int32_t angle)
	{
		auto res = wrap_360_cd(angle);
		if (res > 18000)
		{
			res -= 36000;
		}
		return res;
	}

	static int32_t wrap_360_cd(const int32_t angle)
	{
		double res = fmod(angle, 36000.0f);
		if (res < 0) {
			res += 36000;
		}
		return (int32_t)res;
	}

	static bool isValid(float f)
	{
		return (fabsf(f) < FLT_MAX);
	}

	static bool assure(float& f, float altValue)
	{
		if (isValid(f))
			return true;

		f = altValue;
		return false;
	}

	static void limit(float& f, float minValue, float maxValue)
	{
		if (f < minValue)
			f = minValue;

		if (f > maxValue)
			f = maxValue;
	}
	
	static float constrain_float1(const float f, const float min, const float max)
	{
		if (f < min) {
			return min;
		}
		else if (f > max) {
			return max;
		}
		else {
			return f;
		}
	}

	template <typename T>
	static bool is_zero(T f)
	{
		return ((f < FLT_EPSILON) && (f > -FLT_EPSILON));
	}

	template <typename T>
	static bool is_equal(T x, T y)
	{
		return (is_zero(x - y));
	}
    
    static bool constrain_float (float& f, float minValue, float maxValue)
    {
        if (f < minValue) {
            f = minValue;
            return false;
        }
        else if (f > maxValue) {
            f = maxValue;  
            return false;
        }
        else {
            return true;
        }
    }

	static float maxF(float in1, float in2)
	{
		return (in1 > in2) ? in1 : in2;
	}

	static float minF(float in1, float in2)
	{
		return (in1 < in2) ? in1 : in2;
	}

	static float norm(float in1, float in2)
	{
		return sqrtf(in1 * in1 + in2 * in2);
	}

	// Convert to [-1 1] from [min, trim, max]
	static float covertPWM2Unity(int input, int min, int trim, int max)
	{
		if (input >= trim)
			return (float)(input - trim) / (max - trim);
		else
			return (float)(input - trim) / (trim - min);
	}

	static float fitBatteryLine (float f)
    {
        if (f <= 3.6f)
            return 0.0f;
        else        
            return (3793.294f * f * f * f * f - 59560.284f * f * f * f + 350268.654f * f * f - 914214.727f * f + 893407.853f);
    }
};

typedef union Byte
{
	struct bits
	{
		bool bit0 : 1;
		bool bit1 : 1;
		bool bit2 : 1;
		bool bit3 : 1;
		bool bit4 : 1;
		bool bit5 : 1;
		bool bit6 : 1;
		bool bit7 : 1;
	} bits;
	char byte;
} Byte;

class Bits
{
public:

	static bool triggerBit(bool& newBit, bool& oldBit)
	{
		return (newBit != oldBit);
	}

	static bool triggerHighEdge(bool& newBit, bool& oldBit)
	{
		return (triggerBit(newBit, oldBit) && newBit);
	}

	static bool triggerLowEdge(bool& newBit, bool& oldBit)
	{
		return (triggerBit(newBit, oldBit) && !newBit);
	}

	static char setByte(bool bit0 = false, bool bit1 = false, bool bit2 = false, bool bit3 = false,
		bool bit4 = false, bool bit5 = false, bool bit6 = false, bool bit7 = false)
	{
		return  ((bit0 & 0x0001) |
			((bit1 & 0x0001) << 1) |
			((bit2 & 0x0001) << 2) |
			((bit3 & 0x0001) << 3) |
			((bit4 & 0x0001) << 4) |
			((bit5 & 0x0001) << 5) |
			((bit6 & 0x0001) << 6) |
			((bit7 & 0x0001) << 7));
	}
};

#endif  //NUMBERS_H
