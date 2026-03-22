/*
 * quaternion.c
 *
 *  Created on: Feb 10, 2025
 *      Author: de4lerr
 */
#include "main.h"
#include "quaternion.h"

void quaternion_init(Quaternion *q, float w, float x, float y, float z) {
    if (q == NULL) return;  // Evita ponteiro nulo
    q->w = w;
    q->x = x;
    q->y = y;
    q->z = z;
}

void normalizeVector(float *x, float *y, float *z) {
    float norm = sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (norm > 0.0f) {
        *x /= norm;
        *y /= norm;
        *z /= norm;
    }
}

void normalizeQuaternion(Quaternion *q) {
    float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

void quaternionToEuler(Quaternion *q, float *roll, float *pitch, float *yaw) {
    *roll  = atan2(2.0f * (q->w * q->x + q->y * q->z), 1.0f - 2.0f * (q->x * q->x + q->y * q->y));
    *pitch = asin(2.0f * (q->w * q->y - q->z * q->x));
    *yaw   = atan2(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z));
}
