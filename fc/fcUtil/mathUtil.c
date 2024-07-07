#include "mathUtil.h"

#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0

float sinApprox(float x) {
	int32_t xint = x;
	if (xint < -32 || xint > 32) {
		return 0.0f;
	}               // Stop here on error input (5 * 360 Deg)
	while (x > PI_Val) {
		x -= TwoPI;
	}              // always wrap input angle to -PI..PI
	while (x < -PI_Val) {
		x += TwoPI;
	}
	if (x > HalfPI) {
		x = (HalfPI) - (x - HalfPI); // We just pick -90..+90 Degree
	} else if (x < -HalfPI) {
		x = -(HalfPI) - (HalfPI + x);
	}
	float x2 = x * x;
	return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cosApprox(float x) {
	return sinApprox(x + HalfPI);
}

float acosApprox(float x) {
	float xa = fabsf(x);
	float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
	if (x < 0.0f) {
		return PI_Val - result;
	} else {
		return result;
	}
}

#define atan2PolyCoef1  3.14551665884836e-07f
#define atan2PolyCoef2  0.99997356613987f
#define atan2PolyCoef3  0.14744007058297684f
#define atan2PolyCoef4  0.3099814292351353f
#define atan2PolyCoef5  0.05030176425872175f
#define atan2PolyCoef6  0.1471039133652469f
#define atan2PolyCoef7  0.6444640676891548f

float atan2Approx(float y, float x) {
	float res, absX, absY;
	absX = fabsf(x);
	absY = fabsf(y);
	res = MAX(absX, absY);
	if (res) {
		res = MIN(absX, absY) / res;
	} else {
		res = 0.0f;
	}
	res = -((((atan2PolyCoef5 * res - atan2PolyCoef4) * res - atan2PolyCoef3) * res - atan2PolyCoef2) * res - atan2PolyCoef1) / ((atan2PolyCoef7 * res + atan2PolyCoef6) * res + 1.0f);
	if (absY > absX) {
		res = HalfPI - res;
	}
	if (x < 0) {
		res = PI_Val - res;
	}
	if (y < 0) {
		res = -res;
	}
	return res;
}

// Coefficients for the polynomial approximation
const float atanPolyCoef1 = 0.9998660f;
const float atanPolyCoef2 = -0.3302995f;
const float atanPolyCoef3 = 0.1801410f;
const float atanPolyCoef4 = -0.0851330f;
const float atanPolyCoef5 = 0.0208351f;

float atanApprox(float x) {
	float x2 = x * x;
	return x * (atanPolyCoef1 + x2 * (atanPolyCoef2 + x2 * (atanPolyCoef3 + x2 * (atanPolyCoef4 + x2 * atanPolyCoef5))));
}

int mapToRange(int inValue, int minInRange, int maxInRange, int minOutRange, int maxOutRange) {
	float inputRatio = (float) (inValue - minInRange) / (float) (maxInRange - minInRange);
	int outValue = minOutRange + ((float) (maxOutRange - minOutRange) * inputRatio);
	return outValue;
}

float mapToRangeFloat(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange) {
	float inputRatio = (float) (inValue - minInRange) / (float) (maxInRange - minInRange);
	float outValue = minOutRange + ((float) (maxOutRange - minOutRange) * inputRatio);
	return outValue;
}

double mapToRangeDouble(double inValue, double minInRange, double maxInRange, double minOutRange, double maxOutRange) {
	double inputRatio = (double) (inValue - minInRange) / (double) (maxInRange - minInRange);
	double outValue = minOutRange + ((double) (maxOutRange - minOutRange) * inputRatio);
	return outValue;
}

double power2(double x) {
	return x * x;
}

double power3(double x) {
	return x * x * x;
}
union FAST_INV_DATA fastInvData;
float fastInvSqrtf(float x) {
	fastInvData.f = x;
	fastInvData.i = 0x5f3759df - (fastInvData.i >> 1);
	fastInvData.f *= 1.5f - (x * 0.5f * fastInvData.f * fastInvData.f);
	return fastInvData.f;
}

double invSqrt(double x) {
	return 1.0f / sqrt(x);
}

float invSqrtf(float x) {
	return 1.0f / sqrtf(x);
}

float power2f(float x) {
	return x * x;
}

float power3f(float x) {
	return x * x * x;
}

//Converts Radians to Degrees
float convertRadToDeg(float rads) {
	return rads * OneEightyByPI;
}

//Converts Pizels to CMeters
float convertPixelToCm(float pixs) {
	return pixs * 0.0264583333f;
}

//Converts Pizels to CMeters
float convertPixelToMts(float pixs) {
	return pixs * 0.000264583333f;
}

//Converts Radians to Degrees
float convertDegToRad(float deg) {
	return deg * PIByOneEighty;
}

//Handles bit shifting of -ve numbers
int16_t shiftBitsRight(int16_t value, int16_t nBits) {
	int16_t newValue = (int16_t) (value >> nBits);
	if (newValue < 0) {
		newValue++;
	}
	return newValue;
}

//Handles bit shifting of -ve numbers
int16_t shiftBitsLeft(int16_t value, int16_t nBits) {
	if (value < 0) {
		return (-((-value) << nBits));
	} else {
		return (value << nBits);
	}
}

//Handles bit shifting of -ve numbers
int32_t shiftBitsRight32(int32_t value, int32_t nBits) {
	int32_t newValue = (int32_t) (value >> nBits);
	if (newValue < 0) {
		newValue++;
	}
	return newValue;
}

//Handles bit shifting of -ve numbers
int32_t shiftBitsLeft32(int32_t value, int32_t nBits) {
	if (value < 0) {
		return (-((-value) << nBits));
	} else {
		return (value << nBits);
	}
}

/****************************************************************************/
/* Constraints the values to the boundaries of the applicable ranges        */
/****************************************************************************/
int16_t constrainToRange(int32_t rawValue, int32_t minValue, int32_t maxValue) {
	if (rawValue < minValue) {
		return minValue;
	} else if (rawValue > maxValue) {
		return maxValue;
	} else {
		return rawValue;
	}
}

/****************************************************************************/
/* Constraints the values to the boundaries of the applicable ranges        */
/****************************************************************************/
float constrainToRangeF(float rawValue, float minValue, float maxValue) {
	if (rawValue < minValue) {
		return minValue;
	} else if (rawValue > maxValue) {
		return maxValue;
	} else {
		return rawValue;
	}
}

/****************************************************************************/
/* Checks value in the boundaries of the applicable ranges                 */
/****************************************************************************/
uint8_t isInAbsRange(int32_t rawValue, uint32_t absValue) {
	if (abs(rawValue) <= absValue) {
		return 1;
	} else {
		return 0;
	}
}

/****************************************************************************/
/* Checks value in the boundaries of the applicable ranges                 */
/****************************************************************************/
uint8_t isInAbsRangeF(float rawValue, float absValue) {
	if (fabs(rawValue) <= absValue) {
		return 1;
	} else {
		return 0;
	}
}
/****************************************************************************/
/* Checks value in the boundaries of the applicable ranges                 */
/****************************************************************************/
uint8_t isInRange(int16_t rawValue, int16_t minValue, int16_t maxValue) {
	if (rawValue >= minValue && rawValue <= maxValue) {
		return 1;
	} else {
		return 0;
	}
}

/****************************************************************************/
/* Checks value in the boundaries of the applicable ranges                 */
/****************************************************************************/
uint8_t isInRangeF(float rawValue, float minValue, float maxValue) {
	if (rawValue >= minValue && rawValue <= maxValue) {
		return 1;
	} else {
		return 0;
	}
}

/************************************************************************/
/*Convert 2`s complement bytes to 16 bit integer                        */
/************************************************************************/
int16_t convert2CBytesToInt16(char msb, char lsb) {
	int16_t outPut = (int16_t) (((int16_t) msb << 8) | lsb);
	outPut = ~(outPut);
	outPut += 1;
	return outPut;
}

/************************************************************************/
/*Convert 2`s complement bytes to 16 bit unsigned integer                        */
/************************************************************************/
uint16_t convert2CBytesToUInt16(char msb, char lsb) {
	uint16_t outPut = (uint16_t) (((uint16_t) msb << 8) | lsb);
	outPut = ~(outPut);
	outPut += 1;
	return outPut;
}

/************************************************************************/
/*Convert bytes to 16 bit integer                                       */
/************************************************************************/
int16_t convertBytesToInt16(char msb, char lsb) {
	int16_t outPut = (int16_t) (((int16_t) msb << 8) | lsb);
	return outPut;
}

/************************************************************************/
/*Convert bytes to 16 bit integer                                       */
/************************************************************************/
int convertBytesToInt(char msb, char lsb) {
	int outPut = (int) (((int) msb << 8) | lsb);
	return outPut;
}

/************************************************************************/
/*Convert bytes to 16 bit unsigned integer                                       */
/************************************************************************/
uint16_t convertBytesToUInt16(char msb, char lsb) {
	uint16_t outPut = (uint16_t) (((uint16_t) msb << 8) | lsb);
	return outPut;
}

/************************************************************************/
/* Convert to 16 bit integer to inverted byte array                              */
/************************************************************************/
char* convertInt16ToBytesInverted(int16_t data, char *buffer) {
	buffer[0] = (data >> 8);
	buffer[1] = (data & 0xFF);
	return buffer;
}

/************************************************************************/
/* Convert to 16 bit integer to byte array                              */
/************************************************************************/
char* convertInt16ToBytes(int16_t data, char *buffer) {
	buffer[1] = (data >> 8);
	buffer[0] = (data & 0xFF);
	return buffer;
}

/************************************************************************/
/* Applies dead band for int16_t                                         */
/************************************************************************/
int16_t applyDeadBandInt16(int16_t neutralValue, int16_t value, int16_t boundary) {
	int16_t delta = value - neutralValue;
	if (abs(delta) >= boundary) {
		if (delta < 0.0f) {
			return value + boundary;
		} else {
			return value - boundary;
		}
	} else {
		return neutralValue;
	}
}

/************************************************************************/
/* Applies dead band for floats                                         */
/************************************************************************/
float applyDeadBandFloat(float neutralValue, float value, float boundary) {
	float delta = value - neutralValue;
	if (fabsf(delta) >= boundary) {
		if (delta < 0.0f) {
			return value + boundary;
		} else {
			return value - boundary;
		}
	} else {
		return neutralValue;
	}
}

/************************************************************************/
/* Applies dead band  for double                                        */
/************************************************************************/
double applyDeadBandDouble(double neutralValue, double value, double boundary) {
	double delta = value - neutralValue;
	if (fabs(delta) >= boundary) {
		if (delta < 0.0f) {
			return value + boundary;
		} else {
			return value - boundary;
		}
	} else {
		return neutralValue;
	}
}

float applyHigherDeadBandFloat(float value, float boundary) {
	if (value >= boundary) {
		return value - boundary;
	} else {
		return 0;
	}
}

float applyLowerDeadBandFloat(float value, float boundary) {
	if (value <= boundary) {
		return value - boundary;
	} else {
		return 0;
	}
}

// reverses a string 'str' of length 'len'
void reverseString(char *str, int len) {
	int i = 0, j = len - 1, temp;
	while (i < j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int convertIntToString(int x, char str[], int d) {
	int i = 0;
	while (x) {
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}
	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d) {
		str[i++] = '0';
	}
	reverseString(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void convertFloatToString(float n, char *res, int afterpoint) {
	// Extract integer part
	int ipart = (int) n;
	// Extract floating part
	float fpart = n - (float) ipart;
	// convert integer part to string
	int i = convertIntToString(ipart, res, 0);
	// check for display option after point
	if (afterpoint != 0) {
		res[i] = '.';  // add dot
		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);
		convertIntToString((int) fpart, res + i + 1, afterpoint);
	}
}

/************************************************************************/
/* Gets the index of .                                                  */
/************************************************************************/
int8_t getDecimalIndex(char *dataStr) {
	uint8_t len = strlen(dataStr);
	for (uint8_t indx = 0; indx < len; indx++) {
		if (dataStr[indx] == '.') {
			return indx;
		}
	}
	return -1;
}

/************************************************************************/
/* Convert String  to integer                                           */
/************************************************************************/
int convertStringToInt(char *data) {
	return atoi(data);
}

/************************************************************************/
/* Convert string to Float                                              */
/************************************************************************/
float convertStringToFloat(char *data) {
	float floatData = 0;
	if (data != 0) {
		int8_t decimalIndex = getDecimalIndex(data);
		int8_t dataLen = strlen(data);
		if (decimalIndex > 0) {
			int8_t dataLenMinusDecIndx = dataLen - decimalIndex;
			char fraction[9];
			char whole[9];
			strncpy(fraction, &data[decimalIndex + 1], dataLenMinusDecIndx);
			fraction[dataLenMinusDecIndx] = '\0';
			strncpy(whole, data, decimalIndex);
			whole[decimalIndex] = '\0';
			int wholeInt = convertStringToInt(whole);
			int fractionInt = convertStringToInt(fraction);
			floatData = wholeInt + ((float) fractionInt) / pow(10, dataLenMinusDecIndx - 1);
		} else {
			floatData = convertStringToInt(data);
		}
	}
	return floatData;
}

