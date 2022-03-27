#include <opencv2/core.hpp>
#include <vector>
//#include <Eigen/Core>
#include <gmpxx.h>
#include <gmp.h>

typedef int64_t FIXP_INT_SCALAR_TYPE;
typedef float FIXP_SCALAR_TYPE;

void testout();
/**
 * Fixed point data configuration.
 * Include sign, bit_width, shift.
 */
class FixedPointConfig {
public:

	FixedPointConfig(
		int sign = 0,
		int bit_width = 0,
		int shift = 0) :
		sign(sign),
		bit_width(bit_width),
		shift(shift) {};

	// Copy constructor
	FixedPointConfig(const FixedPointConfig &object) {
		sign = object.sign;
		bit_width = object.bit_width;
		shift = object.shift;
	};

    // assignment
    FixedPointConfig& operator= (const FixedPointConfig& object) {
		sign = object.sign;
		bit_width = object.bit_width;
		shift = object.shift;
        return *this;
    };

	// destructor
	~FixedPointConfig() {};

	bool operator==(const FixedPointConfig& object) {
		if (sign == object.sign && bit_width == object.bit_width && shift == object.shift)
			return true;
		else
			return false;
	}

	int sign; // variable can be negative or not
	int bit_width; // max value < 2^bit_width
	int shift;  // 2^shift larger than float version
};

/**
 * Fixed point data type template.
 */
template<typename Floating>
class FixedPointType {
public:

	FixedPointType(
		Floating value_floating,
		FixedPointConfig config = FixedPointConfig(0, 0, 0)) :
		value_floating(value_floating),
		config(config) {
                mpz_init(big_value);
        };

	// Copy constructor
	FixedPointType(const FixedPointType &object) {
		value_floating = object.value_floating;
		config = object.config;
                mpz_init(big_value);
	        mpz_set(big_value, object.big_value);
	};

	// destructor
	~FixedPointType() {
               mpz_clear(big_value);
        };

	bool enable_check_bit_width = true;
	Floating value_floating;
	FixedPointConfig config;
        mpz_t big_value;
};

/**
 * Fixed point Scalar data type.
 */
class FixedPointScalar: public FixedPointType<FIXP_SCALAR_TYPE> {
public:

	FixedPointScalar();
	FixedPointScalar(
		FIXP_SCALAR_TYPE value_floating = 0,
		FixedPointConfig config = FixedPointConfig(0, 0, 0));

	// Copy constructor
	FixedPointScalar(const FixedPointScalar &object);

	// destructor
	~FixedPointScalar() {};

	void check_bit_width(int op);

	void set_bit_width(int bit_width);

	// Overload operator =
	FixedPointScalar& operator= (const FixedPointScalar &object);

	// Overload operator +
	FixedPointScalar operator + (const FixedPointScalar &object);
	
	// Overload operator +=
	void operator += (const FixedPointScalar &object);

	// Overload operator -
	FixedPointScalar operator - (const FixedPointScalar &object);

	// Overload operator -=
	void operator -= (const FixedPointScalar &object);

	// Overload operator *
	FixedPointScalar operator * (const FixedPointScalar &object);

	// Overload operator /
	FixedPointScalar operator / (const FixedPointScalar &object);

	FixedPointScalar sqrt();

	FixedPointScalar abs();

	FIXP_SCALAR_TYPE to_floating();

	void print_big_value();
};

class FixedPointVector {
public:
	FixedPointVector(const FixedPointScalar &x, const FixedPointScalar &y, const FixedPointScalar &z);
        ~FixedPointVector() {};

        FixedPointScalar x;
        FixedPointScalar y;
        FixedPointScalar z;
};

std::vector<FixedPointScalar> f_Mat2Vec(const cv::Mat& in_mat, FixedPointConfig config);
cv::Mat Vec2Mat_f(const std::vector<FixedPointScalar>& in_vec, int rows, int cols);
cv::Mat PVec2Mat_f(const std::vector<FixedPointVector>& in_vec, int rows, int cols);
std::vector<FixedPointVector> f_PMat2Vec(const cv::Mat& in_mat, FixedPointConfig config);

