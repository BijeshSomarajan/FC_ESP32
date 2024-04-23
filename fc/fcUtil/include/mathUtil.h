#ifndef UTILS_MATHUTILS_H__
#define UTILS_MATHUTILS_H__

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#define PI_Val 3.14159265358979323846f
#define OneEightyByPI 180.0f / PI_Val
#define PIByOneEighty PI_Val / 180.0f
#define HalfPI PI_Val/2.0f
#define TwoPI PI_Val*2.0f

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })

#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })

float sinApprox(float x);
float cosApprox(float x);
float acosApprox(float x);
float atan2Approx(float y, float x);

double power3(double x);
double power2(double x);
double invSqrt(double x);

float power3f(float x);
float power2f(float x);

union FAST_INV_DATA {
	float f;
	uint32_t i;
};
float fastInvSqrtf(float x);
float invSqrtf(float x);

//Byte operations
int16_t shiftBits(int16_t value, int16_t nBits);
int16_t convert2CBytesToInt16(char lsb, char msb);
uint16_t convert2CBytesToUInt16(char lsb, char msb);
int16_t convertBytesToInt16(char lsb, char msb);
uint16_t convertBytesToUInt16(char msb, char lsb);

char* convertInt16ToBytesInverted(int16_t data, char *buffer);
char* convertInt16ToBytes(int16_t data, char *buffer);
int16_t shiftBitsRight(int16_t value, int16_t nBits);
int32_t shiftBitsRight32(int32_t value, int32_t nBits);
int16_t shiftBitsLeft(int16_t value, int16_t nBits);
int32_t shiftBitsLeft32(int32_t value, int32_t nBits);

//Value Range Operations
int16_t constrainToRange(int32_t rawValue, int32_t minValue, int32_t maxValue);
float constrainToRangeF(float rawValue, float minValue, float maxValue);
uint8_t isInRange(int16_t rawValue, int16_t minValue, int16_t maxValue);
uint8_t isInRangeF(float rawValue, float minValue, float maxValue);
uint8_t isInAbsRange(int32_t rawValue, uint32_t absValue);
uint8_t isInAbsRangeF(float rawValue, float absValue);

/**
 * Maps to range
 */
int mapToRange(int inValue, int minInRange, int maxInRange, int minOutRange, int maxOutRange);
float mapToRangeFloat(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange);
double mapToRangeDouble(double inValue, double minInRange, double maxInRange, double minOutRange, double maxOutRange);

//Angle conversion Operations
float convertRadToDeg(float rads);
float convertDegToRad(float deg);
float convertPixelToCm(float pixs);
float convertPixelToMts(float pixs);

int16_t applyDeadBandInt16(int16_t value, int16_t boundary);
float applyDeadBandFloat(float value, float boundary);
float applyHigherDeadBandFloat(float value, float boundary);
float applyLowerDeadBandFloat(float value, float boundary);

double applyDeadBandDouble(double value, double boundary);

void reverseString(char *str, int len);
int convertIntToString(int x, char str[], int d);
void convertFloatToString(float n, char *res, int afterpoint);
int8_t getDecimalIndex(char *dataStr);
int convertStringToInt(char *data);
float convertStringToFloat(char *data);

#endif
