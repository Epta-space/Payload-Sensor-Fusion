/*
 * quaternion.h
 *
 *  Created on: Feb 10, 2025
 *      Author: de4lerr
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Estrutura do Quaternion
typedef struct {
    float w, x, y, z;
} Quaternion;

void quaternion_init(Quaternion *q, float w, float x, float y, float z);
void normalizeVector(float *x, float *y, float *z);
void normalizeQuaternion(Quaternion *q);
void quaternionToEuler(Quaternion *q, float *roll, float *pitch, float *yaw);


#endif /* INC_QUATERNION_H_ */
