/*
 * common.h
 *
 *  Created on: May 9, 2021
 *      Author: Grant
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_


typedef uint_fast8_t uint8_ft;

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define compile_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }
// From: http://www.pixelbeat.org/programming/gcc/static_assert.html


#endif /* INC_COMMON_H_ */
