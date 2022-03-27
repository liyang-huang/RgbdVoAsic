#include "fixed_point_util.hpp"
#include <opencv2/core/utility.hpp>
#include <iostream>
#include <gmpxx.h>
#include <gmp.h>

using namespace cv;

float test_max = 0.0;
float test_min = 0.0;

void testout(){
    std::cout << "max_value " << test_max << std::endl;
    std::cout << "min_value " << test_min << std::endl;
}

FixedPointScalar::FixedPointScalar(
	FIXP_SCALAR_TYPE value_floating,
	FixedPointConfig config) :
	FixedPointType<FIXP_SCALAR_TYPE>(value_floating, config) {
        mpz_set_si(big_value, static_cast<FIXP_INT_SCALAR_TYPE>(value_floating * (1LL << config.shift)));
	
        check_bit_width(0);
}

FixedPointScalar::FixedPointScalar(
	const FixedPointScalar &object): 
	FixedPointType<FIXP_SCALAR_TYPE>(object){
	value_floating = object.value_floating; 
        mpz_set(big_value, object.big_value);
	config.sign = object.config.sign;
	config.bit_width = object.config.bit_width;
	config.shift = object.config.shift;

        check_bit_width(01);
}

FixedPointVector::FixedPointVector(const FixedPointScalar &_x, const FixedPointScalar &_y, const FixedPointScalar &_z)
                                   : x(_x), y(_y), z(_z)
{}

void FixedPointScalar::check_bit_width(int op) {
	if (!enable_check_bit_width)
	//if (enable_check_bit_width)
		return;
        
	if (config.sign == 0) {
                mpz_t max_value;
                mpz_init(max_value);
	        mpz_set_si(max_value, (int64_t)1);
                mpz_mul_2exp(max_value, max_value, config.bit_width);
                int comp0 = mpz_cmp(max_value, big_value); // < max value
                if(comp0 <= 0)
                {
                    std::cout << "liyang value_floating " << value_floating << std::endl;
                    std::cout << "liyang shift " << config.shift << std::endl;
                    std::cout << "liyang bitwidth " << config.bit_width << std::endl;
                    std::cout << "liyang op " << op << std::endl;
                    std::cout << "liyang sign " << config.sign << std::endl;
                    gmp_printf("%Zd\n", max_value);
                    gmp_printf("%Zd\n", big_value);
                }
	        assert(!(comp0 <= 0));
                mpz_clear(max_value);
	}
	else {
                if ((test_max<=value_floating) && !std::isinf(value_floating))
                    test_max = value_floating;
                if ((test_min>=value_floating) && !std::isinf(value_floating))
                    test_min = value_floating;
                mpz_t max_value;
                mpz_init(max_value);
	        mpz_set_si(max_value, (int64_t)1);
                mpz_mul_2exp(max_value, max_value, (config.bit_width - 1));
                mpz_t min_value;
                mpz_init(min_value);
	        mpz_set_si(min_value, (int64_t)-1);
                mpz_mul_2exp(min_value, min_value, (config.bit_width - 1));
                int comp1 = mpz_cmp(max_value, big_value); // < max value
                int comp2 = mpz_cmp(min_value, big_value); // >= min value
                if((comp1 <= 0) || (comp2 > 0))
                {
                     std::cout << "value_floating " << value_floating << std::endl;
                     std::cout << "liyang shift " << config.shift << std::endl;
                     std::cout << "liyang bitwidth " << config.bit_width << std::endl;
                     std::cout << "liyang sign " << config.sign << std::endl;
                     std::cout << "liyang op " << op << std::endl;
                     gmp_printf("%Zd\n", big_value);
                     gmp_printf("%Zd\n", max_value);
                     gmp_printf("%Zd\n", min_value);
                     std::cout << "comp1 " << comp1 << std::endl;
                     std::cout << "comp2 " << comp2 << std::endl;
                }
	        assert(!(comp1 <= 0));
	        assert(!(comp2 > 0));
                mpz_clear(max_value);
                mpz_clear(min_value);
	}
        
}

void FixedPointScalar::set_bit_width(int bit_width){
	this->config.bit_width = bit_width;
	check_bit_width(03);
}

// Ref: https://www.learncpp.com/cpp-tutorial/overloading-the-assignment-operator/
FixedPointScalar& FixedPointScalar::operator= (const FixedPointScalar &object)
{
	// self-assignment guard
	if (this == &object)
		return *this;

	// do the copy
	value_floating = object.value_floating;
        ////////mpz_init(big_value);
        mpz_set(big_value, object.big_value);
	config.sign = object.config.sign;
	config.bit_width = object.config.bit_width;
	config.shift = object.config.shift;

	check_bit_width(123321);
	// return the existing object so we can chain this operator
	return *this;
}

// Ref: https://www.geeksforgeeks.org/operator-overloading-c/
FixedPointScalar FixedPointScalar::operator + (const FixedPointScalar &object) {
	assert(config.shift == object.config.shift);
	FixedPointScalar return_object(*this);
	return_object.value_floating = value_floating + object.value_floating;
        mpz_add(return_object.big_value, big_value, object.big_value);
	return_object.config.sign = config.sign | object.config.sign;
	//return_object.config.bit_width = std::max(config.bit_width, object.config.bit_width) + 1;
	//return_object.config.bit_width = config.bit_width, object.config.bit_width;
	return_object.check_bit_width(1);
	return return_object;
}

void FixedPointScalar::operator += (const FixedPointScalar &object) {
	assert(config.shift == object.config.shift);
	value_floating = value_floating + object.value_floating;
        mpz_add(big_value, big_value, object.big_value);
	config.sign = config.sign | object.config.sign;
	//config.bit_width = std::max(config.bit_width, object.config.bit_width) + 1;
	//config.bit_width = config.bit_width;
	check_bit_width(1);
}

// Ref: https://www.geeksforgeeks.org/operator-overloading-c/
FixedPointScalar FixedPointScalar::operator - (const FixedPointScalar &object) {
	//debug
	//if (shift != object.shift)
	//	printf("[FIXP_ERROR] shift (%i) != object.shift (%i)\n", shift, object.shift);
	assert(config.shift == object.config.shift);
	FixedPointScalar return_object(*this);
	return_object.value_floating = value_floating - object.value_floating;
        mpz_sub(return_object.big_value, big_value, object.big_value);
	return_object.config.sign = 1;
	//return_object.config.bit_width = std::max(config.bit_width, object.config.bit_width) + 1;
	//return_object.config.bit_width = config.bit_width;
	return_object.check_bit_width(2);
	return return_object;
}

void FixedPointScalar::operator -= (const FixedPointScalar &object) {
	assert(config.shift == object.config.shift);
	value_floating = value_floating - object.value_floating;
        mpz_sub(big_value, big_value, object.big_value);
	config.sign = 1;
	//config.bit_width = std::max(config.bit_width, object.config.bit_width) + 1;
	//config.bit_width = config.bit_width;
	check_bit_width(2);
}

// Ref: https://www.geeksforgeeks.org/operator-overloading-c/
FixedPointScalar FixedPointScalar::operator * (const FixedPointScalar &object) {
	FixedPointScalar return_object(*this);
	return_object.value_floating = value_floating * object.value_floating;
        mpz_mul(return_object.big_value, big_value, object.big_value);
        mpz_t shift_value;
        mpz_init(shift_value);
        mpz_set_si(shift_value, (int64_t)1);
        mpz_mul_2exp(shift_value, shift_value, config.shift);
        mpz_div(return_object.big_value, return_object.big_value, shift_value);
        mpz_clear(shift_value);
	return_object.config.sign = config.sign | object.config.sign;
	return_object.config.shift = config.shift;
	return_object.config.bit_width = config.bit_width;

	return_object.check_bit_width(3);
	return return_object;
}

// Ref: https://www.geeksforgeeks.org/operator-overloading-c/
FixedPointScalar FixedPointScalar::operator / (const FixedPointScalar &object) {
	FixedPointScalar return_object(*this);
	return_object.value_floating = value_floating / object.value_floating;
        mpz_t shift_value;
        mpz_init(shift_value);
        mpz_set_si(shift_value, (int64_t)1);
        mpz_mul_2exp(shift_value, shift_value, config.shift);
        mpz_mul(shift_value, big_value, shift_value);
        mpz_div(return_object.big_value, shift_value, object.big_value); 
        mpz_clear(shift_value);
	return_object.config.sign = config.sign | object.config.sign;
	//return_object.config.shift = config.shift - object.config.shift;
	//return_object.config.bit_width = config.bit_width;
	return_object.check_bit_width(4);
	return return_object;
}

FixedPointScalar FixedPointScalar::sqrt() {
	FixedPointScalar return_object(*this);
	return_object.value_floating = std::sqrt(value_floating);
        mpz_t shift_value;
        mpz_init(shift_value);
        mpz_set_si(shift_value, (int64_t)1);
        mpz_mul_2exp(shift_value, big_value, config.shift);
        mpz_sqrt(return_object.big_value, shift_value);
        mpz_clear(shift_value);

	return_object.config.bit_width = config.bit_width;
	return_object.config.shift = config.shift;
	return_object.check_bit_width(6);
	return return_object;
}

FixedPointScalar FixedPointScalar::abs() {
	FixedPointScalar return_object(*this);
	return_object.value_floating = std::abs(value_floating);
        mpz_abs(return_object.big_value, big_value);
	return_object.config.sign = config.sign;
	return_object.check_bit_width(7);
	return return_object;
}

FIXP_SCALAR_TYPE FixedPointScalar::to_floating() {
        //mpz_t shift_value;
        //mpz_t tmp_value;
        //mpz_init(shift_value);
        //mpz_init(tmp_value);
        //mpz_set_si(shift_value, (int64_t)1);
        //mpz_mul_2exp(shift_value, shift_value, config.shift);
        //mpz_div(tmp_value, big_value, shift_value); 
        //int64_t bvalue_get = mpz_get_si(tmp_value);
        //mpz_clear(shift_value);
        //mpz_clear(tmp_value);
        //return FIXP_SCALAR_TYPE(bvalue_get);
        int64_t bvalue_get = mpz_get_si(big_value);
        double bvalue_double = (double)bvalue_get / (double)(1LL << config.shift);
        return FIXP_SCALAR_TYPE(bvalue_double);
}

/*
void FixedPointScalar::print_big_value() {
      gmp_printf("%Zd\n", big_value);
}
*/

std::vector<FixedPointScalar> f_Mat2Vec(const Mat& in_mat, FixedPointConfig config) {
    if(in_mat.depth() != CV_32F)
        CV_Error(Error::StsBadSize, "Input Mat depth has to be FIXP_SCALAR_TYPE in Mat2Vec.");
    int rows = in_mat.rows;
    int cols = in_mat.cols;

    std::vector<FixedPointScalar> out_vec;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            FIXP_SCALAR_TYPE value = in_mat.at<FIXP_SCALAR_TYPE>(r, c);
            FixedPointScalar temp(value, config);
            out_vec.push_back(temp);
        }
    }

    return out_vec;
}

Mat Vec2Mat_f(const std::vector<FixedPointScalar>& in_vec, int rows, int cols) {

    Mat out_mat;
    out_mat.create(rows, cols, CV_32FC1);
  
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            FixedPointScalar temp = in_vec[r*cols + c];
            out_mat.at<FIXP_SCALAR_TYPE>(r, c) = temp.to_floating();
            //out_mat.at<FIXP_SCALAR_TYPE>(r, c) = temp.value_floating;
        }
    }

    return out_mat;
}

Mat PVec2Mat_f(const std::vector<FixedPointVector>& in_vec, int rows, int cols) {

    Mat out_mat;
    out_mat.create(rows, cols, CV_32FC3);
  
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            FixedPointVector temp = in_vec[r*cols + c];
            out_mat.at<Point3f>(r, c).x = temp.x.to_floating();
            out_mat.at<Point3f>(r, c).y = temp.y.to_floating();
            out_mat.at<Point3f>(r, c).z = temp.z.to_floating();
            //out_mat.at<Point3f>(r, c).x = temp.x.value_floating;
            //out_mat.at<Point3f>(r, c).y = temp.y.value_floating;
            //out_mat.at<Point3f>(r, c).z = temp.z.value_floating;
        }
    }

    return out_mat;
}
std::vector<FixedPointVector> f_PMat2Vec(const Mat& in_mat, FixedPointConfig config) {
    int rows = in_mat.rows;
    int cols = in_mat.cols;

    std::vector<FixedPointVector> out_vec;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            FIXP_SCALAR_TYPE value_x = in_mat.at<Vec3f>(r, c)[0];
            FIXP_SCALAR_TYPE value_y = in_mat.at<Vec3f>(r, c)[1];
            FIXP_SCALAR_TYPE value_z = in_mat.at<Vec3f>(r, c)[2];
            FixedPointScalar temp_x(value_x, config);
            FixedPointScalar temp_y(value_y, config);
            FixedPointScalar temp_z(value_z, config);
            FixedPointVector temp(temp_x, temp_y, temp_z);
            out_vec.push_back(temp);
        }
    }

    return out_vec;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
