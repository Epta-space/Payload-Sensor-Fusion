/*
 * IMU_FUSION.c
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Biblioteca de fusão sensorial e filtros para IMU
 */

#include "IMU_FUSION.h"
#include "ROCKET.h"

/* ========================================================================
 * INICIALIZAÇÃO
 * ======================================================================== */

void FUSION_Init(IMU_Fusion *fusion, float sample_freq)
{
    memset(fusion, 0, sizeof(IMU_Fusion));
    
    /* Configuração padrão */
    fusion->alpha_complementary = 0.98f;
    fusion->use_lowpass = 1;
    fusion->use_highpass = 0;
    
    /* Inicializa filtros passa-baixa (20 Hz) */
    for (int i = 0; i < 3; i++) {
        FILTER_InitLowPass(&fusion->acc_lpf[i], 20.0f, sample_freq);
        FILTER_InitLowPass(&fusion->gyro_lpf[i], 20.0f, sample_freq);
        FILTER_InitLowPass(&fusion->mag_lpf[i], 5.0f, sample_freq);
    }
    
    /* Inicializa filtros passa-alta (0.5 Hz) */
    for (int i = 0; i < 3; i++) {
        FILTER_InitHighPass(&fusion->gyro_hpf[i], 0.5f, sample_freq);
    }
    
    /* Inicializa filtros de Kalman */
    KALMAN_Init(&fusion->kf_roll, 0.01f, 0.01f, 0.05f);
    KALMAN_Init(&fusion->kf_pitch, 0.01f, 0.01f, 0.05f);
    KALMAN_Init(&fusion->kf_yaw, 0.01f, 0.01f, 0.05f);
    
    /* Inicializa Madgwick */
    MADGWICK_Init(&fusion->madgwick, 0.1f, sample_freq);
}

void FUSION_Update(IMU_Fusion *fusion, IMU *imu)
{
    if (!imu->is_valid) return;
    
    /* Aplica filtros digitais */
    FUSION_ApplyFilters(fusion, imu);
    
    /* Filtro complementar */
    FUSION_ComplementaryFilter(fusion, imu);
    
    /* Filtro de Kalman */
    fusion->roll_kf = KALMAN_Update(&fusion->kf_roll, 
                                    imu->roll_acc, 
                                    imu->gyro_x * M_PI / 180.0f, 
                                    imu->dt);
    fusion->pitch_kf = KALMAN_Update(&fusion->kf_pitch, 
                                     imu->pitch_acc, 
                                     imu->gyro_y * M_PI / 180.0f, 
                                     imu->dt);
    fusion->yaw_kf = KALMAN_Update(&fusion->kf_yaw, 
                                   imu->yaw_mag, 
                                   imu->gyro_z * M_PI / 180.0f, 
                                   imu->dt);
    
    /* Filtro Madgwick */
//    MADGWICK_Update(&fusion->madgwick,
//                    imu->gyro_x * M_PI / 180.0f,
//                    imu->gyro_y * M_PI / 180.0f,
//                    imu->gyro_z * M_PI / 180.0f,
//                    imu->acc_x, imu->acc_y, imu->acc_z,
//                    imu->mag_x, imu->mag_y, imu->mag_z,
//                    imu->dt);

    MADGWICK_Update(&fusion->madgwick,
    		fusion->gyro_x_f * M_PI / 180.0f,
			fusion->gyro_y_f * M_PI / 180.0f,
			fusion->gyro_z_f * M_PI / 180.0f,
			fusion->acc_x_f, fusion->acc_y_f, fusion->acc_z_f,
			fusion->mag_x_f, fusion->mag_y_f, fusion->mag_z_f,
                    imu->dt);
    
    QUAT_ToEuler(&fusion->madgwick.q, 
                 &fusion->roll_mw, 
                 &fusion->pitch_mw, 
                 &fusion->yaw_mw);
}

/* ========================================================================
 * FILTROS DIGITAIS IIR
 * ======================================================================== */

void FILTER_InitLowPass(IIR_Filter *filter, float cutoff_freq, float sample_freq)
{
    /* Filtro Butterworth de 2ª ordem */
    float omega = 2.0f * M_PI * cutoff_freq / sample_freq;
    float k = tanf(omega / 2.0f);
    float norm = 1.0f / (1.0f + k * sqrtf(2.0f) + k * k);
    
    filter->a[0] = k * k * norm;
    filter->a[1] = 2.0f * filter->a[0];
    filter->a[2] = filter->a[0];
    filter->a[3] = 0.0f;
    
    filter->b[0] = 1.0f;
    filter->b[1] = 2.0f * (k * k - 1.0f) * norm;
    filter->b[2] = (1.0f - k * sqrtf(2.0f) + k * k) * norm;
    filter->b[3] = 0.0f;
    
    FILTER_Reset(filter);
}

void FILTER_InitHighPass(IIR_Filter *filter, float cutoff_freq, float sample_freq)
{
    /* Filtro Butterworth de 2ª ordem */
    float omega = 2.0f * M_PI * cutoff_freq / sample_freq;
    float k = tanf(omega / 2.0f);
    float norm = 1.0f / (1.0f + k * sqrtf(2.0f) + k * k);
    
    filter->a[0] = norm;
    filter->a[1] = -2.0f * norm;
    filter->a[2] = norm;
    filter->a[3] = 0.0f;
    
    filter->b[0] = 1.0f;
    filter->b[1] = 2.0f * (k * k - 1.0f) * norm;
    filter->b[2] = (1.0f - k * sqrtf(2.0f) + k * k) * norm;
    filter->b[3] = 0.0f;
    
    FILTER_Reset(filter);
}

float FILTER_Apply(IIR_Filter *filter, float input)
{
    /* Desloca histórico */
    filter->x[2] = filter->x[1];
    filter->x[1] = filter->x[0];
    filter->x[0] = input;
    
    filter->y[2] = filter->y[1];
    filter->y[1] = filter->y[0];
    
    /* Calcula saída */
    filter->y[0] = filter->a[0] * filter->x[0] + 
                   filter->a[1] * filter->x[1] + 
                   filter->a[2] * filter->x[2] -
                   filter->b[1] * filter->y[1] - 
                   filter->b[2] * filter->y[2];
    
    return filter->y[0];
}

void FILTER_Reset(IIR_Filter *filter)
{
    for (int i = 0; i < 4; i++) {
        filter->x[i] = 0.0f;
        filter->y[i] = 0.0f;
    }
}

/* ========================================================================
 * FILTRO COMPLEMENTAR
 * ======================================================================== */

void FUSION_ComplementaryFilter(IMU_Fusion *fusion, IMU *imu)
{
    float alpha = fusion->alpha_complementary;
    
    /* Integração do giroscópio */
    float gyro_roll = fusion->roll_cf + (fusion->gyro_x_f * imu->dt * M_PI / 180.0f);
    float gyro_pitch = fusion->pitch_cf + (fusion->gyro_y_f * imu->dt * M_PI / 180.0f);
    float gyro_yaw = fusion->yaw_cf + (fusion->gyro_z_f * imu->dt * M_PI / 180.0f);
    
    /* Fusão complementar */
    fusion->roll_cf = alpha * gyro_roll + (1.0f - alpha) * imu->roll_acc;
    fusion->pitch_cf = alpha * gyro_pitch + (1.0f - alpha) * imu->pitch_acc;
    fusion->yaw_cf = alpha * gyro_yaw + (1.0f - alpha) * imu->yaw_mag;
}

/* ========================================================================
 * FILTRO DE KALMAN
 * ======================================================================== */

void KALMAN_Init(Kalman_Filter *kf, float q_angle, float q_bias, float r_measure)
{
    kf->Q_angle = q_angle;
    kf->Q_bias = q_bias;
    kf->R_measure = r_measure;
    
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}

float KALMAN_Update(Kalman_Filter *kf, float measurement, float rate, float dt)
{
    /* Predição */
    kf->x[0] += dt * (rate - kf->x[1]);
    
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    /* Inovação */
    float y = measurement - kf->x[0];
    float S = kf->P[0][0] + kf->R_measure;
    
    /* Ganho de Kalman */
    kf->K[0] = kf->P[0][0] / S;
    kf->K[1] = kf->P[1][0] / S;
    
    /* Correção */
    kf->x[0] += kf->K[0] * y;
    kf->x[1] += kf->K[1] * y;
    
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= kf->K[0] * P00_temp;
    kf->P[0][1] -= kf->K[0] * P01_temp;
    kf->P[1][0] -= kf->K[1] * P00_temp;
    kf->P[1][1] -= kf->K[1] * P01_temp;
    
    return kf->x[0];
}

void KALMAN_Reset(Kalman_Filter *kf)
{
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}

/* ========================================================================
 * FILTRO MADGWICK
 * ======================================================================== */

void MADGWICK_Init(Madgwick_Filter *mw, float beta, float sample_freq)
{
    QUAT_Init(&mw->q, 1.0f, 0.0f, 0.0f, 0.0f);
    mw->beta = beta;
    mw->sample_freq = sample_freq;
}

void MADGWICK_Update(Madgwick_Filter *mw, float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    float q0 = mw->q.w;
    float q1 = mw->q.x;
    float q2 = mw->q.y;
    float q3 = mw->q.z;

    /* Normaliza acelerômetro */
    recipNorm = QUAT_InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* Normaliza magnetômetro */
    recipNorm = QUAT_InvSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    /* Valores auxiliares */
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    /* Campo magnético de referência */
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
         _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
         my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
           _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    /* Gradiente descendente */
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    /* Normaliza */
    recipNorm = QUAT_InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    /* Taxa de mudança do quaternion */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - mw->beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - mw->beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - mw->beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - mw->beta * s3;

    /* Integra */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normaliza quaternion */
    mw->q.w = q0;
    mw->q.x = q1;
    mw->q.y = q2;
    mw->q.z = q3;
    QUAT_Normalize(&mw->q);
}

void MADGWICK_UpdateIMU(Madgwick_Filter *mw, float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    float q0 = mw->q.w;
    float q1 = mw->q.x;
    float q2 = mw->q.y;
    float q3 = mw->q.z;

    /* Normaliza acelerômetro */
    recipNorm = QUAT_InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* Valores auxiliares */
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    /* Gradiente descendente */
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    /* Normaliza */
    recipNorm = QUAT_InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    /* Taxa de mudança do quaternion */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - mw->beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - mw->beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - mw->beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - mw->beta * s3;

    /* Integra */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* Normaliza quaternion */
    mw->q.w = q0;
    mw->q.x = q1;
    mw->q.y = q2;
    mw->q.z = q3;
    QUAT_Normalize(&mw->q);
}

/* ========================================================================
 * QUATERNION
 * ======================================================================== */

void QUAT_Init(Quaternion *q, float w, float x, float y, float z)
{
    q->w = w;
    q->x = x;
    q->y = y;
    q->z = z;
}

void QUAT_Normalize(Quaternion *q)
{
    float recipNorm = QUAT_InvSqrt(q->w * q->w + q->x * q->x +
                                   q->y * q->y + q->z * q->z);
    q->w *= recipNorm;
    q->x *= recipNorm;
    q->y *= recipNorm;
    q->z *= recipNorm;
}

void QUAT_ToEuler(Quaternion *q, float *roll, float *pitch, float *yaw)
{
    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(M_PI / 2.0f, sinp);
    else
        *pitch = asinf(sinp);

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void QUAT_FromEuler(Quaternion *q, float roll, float pitch, float yaw)
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

float QUAT_InvSqrt(float x)
{
    /* Fast inverse square root */
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* ========================================================================
 * UTILIDADES
 * ======================================================================== */

void FUSION_ApplyFilters(IMU_Fusion *fusion, IMU *imu)
{
    if (fusion->use_lowpass) {
        /* Aplica filtro passa-baixa */
        fusion->acc_x_f = FILTER_Apply(&fusion->acc_lpf[0], imu->acc_x);
        fusion->acc_y_f = FILTER_Apply(&fusion->acc_lpf[1], imu->acc_y);
        fusion->acc_z_f = FILTER_Apply(&fusion->acc_lpf[2], imu->acc_z);

        fusion->gyro_x_f = FILTER_Apply(&fusion->gyro_lpf[0], imu->gyro_x);
        fusion->gyro_y_f = FILTER_Apply(&fusion->gyro_lpf[1], imu->gyro_y);
        fusion->gyro_z_f = FILTER_Apply(&fusion->gyro_lpf[2], imu->gyro_z);

        fusion->mag_x_f = FILTER_Apply(&fusion->mag_lpf[0], imu->mag_x);
        fusion->mag_y_f = FILTER_Apply(&fusion->mag_lpf[1], imu->mag_y);
        fusion->mag_z_f = FILTER_Apply(&fusion->mag_lpf[2], imu->mag_z);
    } else {
        fusion->acc_x_f = imu->acc_x;
        fusion->acc_y_f = imu->acc_y;
        fusion->acc_z_f = imu->acc_z;

        fusion->gyro_x_f = imu->gyro_x;
        fusion->gyro_y_f = imu->gyro_y;
        fusion->gyro_z_f = imu->gyro_z;

        fusion->mag_x_f = imu->mag_x;
        fusion->mag_y_f = imu->mag_y;
        fusion->mag_z_f = imu->mag_z;
    }

    if (fusion->use_highpass) {
        /* Aplica filtro passa-alta no giroscópio */
        fusion->gyro_x_f = FILTER_Apply(&fusion->gyro_hpf[0], fusion->gyro_x_f);
        fusion->gyro_y_f = FILTER_Apply(&fusion->gyro_hpf[1], fusion->gyro_y_f);
        fusion->gyro_z_f = FILTER_Apply(&fusion->gyro_hpf[2], fusion->gyro_z_f);
    }
}

void FUSION_SetComplementaryAlpha(IMU_Fusion *fusion, float alpha)
{
    if (alpha >= 0.0f && alpha <= 1.0f) {
        fusion->alpha_complementary = alpha;
    }
}

void FUSION_SetMadgwickBeta(IMU_Fusion *fusion, float beta)
{
    if (beta > 0.0f) {
        fusion->madgwick.beta = beta;
    }
}

void FUSION_PrintAngles(IMU_Fusion *fusion)
{
    printf("\n--- FUSAO SENSORIAL ---\n");
    printf("Complementar - Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n",
           fusion->roll_cf * 180.0f / M_PI,
           fusion->pitch_cf * 180.0f / M_PI,
           fusion->yaw_cf * 180.0f / M_PI);

    printf("Kalman       - Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n",
           fusion->roll_kf * 180.0f / M_PI,
           fusion->pitch_kf * 180.0f / M_PI,
           fusion->yaw_kf * 180.0f / M_PI);

    printf("Madgwick     - Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n",
           fusion->roll_mw * 180.0f / M_PI,
           fusion->pitch_mw * 180.0f / M_PI,
           fusion->yaw_mw * 180.0f / M_PI);
}

/* ========================================================================
 * FUSÃO SENSORIAL ADAPTATIVA
 * ======================================================================== */

/**
 * @brief Atualiza fusão de sensores com lógica adaptativa por fase
 * @param fusion Estrutura de fusão
 * @param imu Dados do IMU
 * @param fsm Máquina de estados (para determinar fase)
 */
void FUSION_Update_Adaptive(IMU_Fusion *fusion, IMU *imu, ROCKET_FSM_t *fsm)
{
    if (!imu->is_valid) return;

    // Determina se está em fase de BOOST (motor queimando)
    uint8_t is_boost_phase = (fsm->current_state == FLIGHT_BOOST ||
                              fsm->current_state == FLIGHT_COAST);

    if (is_boost_phase) {
        /* ============================================================
         * DURANTE BOOST: Apenas Giroscópio
         * ============================================================
         *
         * O acelerômetro mede ~10g do motor, não da gravidade.
         * Não podemos usá-lo para estimar atitude.
         *
         * Estratégia:
         * 1. Integra apenas giroscópio
         * 2. Mantém quaternion do último estado válido
         * 3. Aguarda estabilização pós-burnout
         */

        FUSION_Update_GyroOnly(fusion, imu);

    } else {
        /* ============================================================
         * FORA DO BOOST: Madgwick Completo
         * ============================================================
         *
         * Ambiente "normal" (gravidade dominante)
         * Podem usar acelerômetro para corrigir drift do giroscópio
         */

        // Aplica filtros (passa-baixa)
        FUSION_ApplyFilters(fusion, imu);

        // Madgwick com todos os sensores
        MADGWICK_Update(&fusion->madgwick,
                        fusion->gyro_x_f * M_PI / 180.0f,
                        fusion->gyro_y_f * M_PI / 180.0f,
                        fusion->gyro_z_f * M_PI / 180.0f,
                        fusion->acc_x_f, fusion->acc_y_f, fusion->acc_z_f,
                        fusion->mag_x_f, fusion->mag_y_f, fusion->mag_z_f,
                        imu->dt);

        // Converte quaternion para Euler
        QUAT_ToEuler(&fusion->madgwick.q,
                     &fusion->roll_mw,
                     &fusion->pitch_mw,
                     &fusion->yaw_mw);

        // Também rodar filtro complementar como backup
        FUSION_ComplementaryFilter(fusion, imu);
    }
}

/**
 * @brief Integra APENAS giroscópio (sem acelerômetro)
 *
 * Usado durante BOOST quando aceleração do motor domina.
 * Mantém estimativa anterior e integra mudanças de rotação.
 *
 * @param fusion Estrutura de fusão
 * @param imu Dados do IMU
 */
void FUSION_Update_GyroOnly(IMU_Fusion *fusion, IMU *imu)
{
    static uint8_t first_boost = 1;
    static float roll_at_liftoff, pitch_at_liftoff, yaw_at_liftoff;

    // Na primeira vez que entra em BOOST, salva atitude
    if (first_boost) {
        roll_at_liftoff = fusion->roll_mw;
        pitch_at_liftoff = fusion->pitch_mw;
        yaw_at_liftoff = fusion->yaw_mw;
        first_boost = 0;

        printf("[IMU] BOOST: Congelando atitude em R:%.1f P:%.1f Y:%.1f\r\n",
               roll_at_liftoff * 180/M_PI,
               pitch_at_liftoff * 180/M_PI,
               yaw_at_liftoff * 180/M_PI);

        return;
    }

    // Integra APENAS giroscópio (delta de rotação)
    float dt = imu->dt;

    // Taxa de rotação em rad/s
    float wx = imu->gyro_x_f * M_PI / 180.0f;
    float wy = imu->gyro_y_f * M_PI / 180.0f;
    float wz = imu->gyro_z_f * M_PI / 180.0f;

    // Integração simples (ou Runge-Kutta se precisar de precisão)
    fusion->roll_mw += wx * dt;
    fusion->pitch_mw += wy * dt;
    fusion->yaw_mw += wz * dt;

    // Opcional: Aplica limites para evitar divergência
    #define MAX_PITCH_BOOST (M_PI / 2.0f)  // ±90°
    if (fusion->pitch_mw > MAX_PITCH_BOOST) {
        fusion->pitch_mw = MAX_PITCH_BOOST;
    } else if (fusion->pitch_mw < -MAX_PITCH_BOOST) {
        fusion->pitch_mw = -MAX_PITCH_BOOST;
    }
}

/**
 * @brief Recupera atitude após BOOST (transição suave)
 *
 * Quando sai de BOOST, o giroscópio pode ter acumulado erro.
 * Esta função faz uma transição suave para o Madgwick quando
 * o acelerômetro volta a ser confiável.
 *
 * @param fusion Estrutura de fusão
 * @param imu Dados do IMU
 */
void FUSION_Recover_From_Boost(IMU_Fusion *fusion, IMU *imu)
{
    static uint8_t recovery_counter = 0;
    static float roll_boost, pitch_boost, yaw_boost;

    // Primeira iteração pós-boost: salva atitude estimada pelo giroscópio
    if (recovery_counter == 0) {
        roll_boost = fusion->roll_mw;
        pitch_boost = fusion->pitch_mw;
        yaw_boost = fusion->yaw_mw;

        printf("[IMU] Saído de BOOST. Atitude gyro: R:%.1f P:%.1f Y:%.1f\r\n",
               roll_boost * 180/M_PI,
               pitch_boost * 180/M_PI,
               yaw_boost * 180/M_PI);
    }

    // Madgwick agora é confiável novamente
    MADGWICK_Update(&fusion->madgwick,
                    imu->gyro_x_f * M_PI / 180.0f,
                    imu->gyro_y_f * M_PI / 180.0f,
                    imu->gyro_z_f * M_PI / 180.0f,
                    imu->acc_x_f, imu->acc_y_f, imu->acc_z_f,
                    imu->mag_x_f, imu->mag_y_f, imu->mag_z_f,
                    imu->dt);

    QUAT_ToEuler(&fusion->madgwick.q,
                 &fusion->roll_mw,
                 &fusion->pitch_mw,
                 &fusion->yaw_mw);

    recovery_counter++;

    // Recuperação: 100 iterações (0.5s @ 200Hz)
    #define RECOVERY_ITERATIONS 100
    if (recovery_counter >= RECOVERY_ITERATIONS) {
        recovery_counter = 0;
        printf("[IMU] Recuperação completa. Atitude Madgwick: R:%.1f P:%.1f Y:%.1f\r\n",
               fusion->roll_mw * 180/M_PI,
               fusion->pitch_mw * 180/M_PI,
               fusion->yaw_mw * 180/M_PI);
    }
}

/* ========================================================================
 * KALMAN FILTER ADAPTATIVO (Alternativa ao Madgwick)
 * ======================================================================== */

/**
 * @brief Kalman filter que ignora acelerômetro durante BOOST
 *
 * Se preferir Kalman ao invés de Madgwick, esta versão é mais robusta.
 */
void FUSION_Kalman_Adaptive(IMU_Fusion *fusion, IMU *imu, ROCKET_FSM_t *fsm)
{
    uint8_t is_boost_phase = (fsm->current_state == FLIGHT_BOOST);

    if (is_boost_phase) {
        /* Durante BOOST: Apenas giroscópio como predição */
        float dt = imu->dt;

        fusion->roll_kf += imu->gyro_x_f * M_PI / 180.0f * dt;
        fusion->pitch_kf += imu->gyro_y_f * M_PI / 180.0f * dt;
        fusion->yaw_kf += imu->gyro_z_f * M_PI / 180.0f * dt;

        // NÃO fazer update com acelerômetro (seu_measurement = gyro apenas)

    } else {
        /* Fora de BOOST: Kalman completo com acelerômetro */

        float dt = imu->dt;

        // Medições confiáveis
        float roll_meas = atan2f(imu->acc_y_f, imu->acc_z_f);
        float pitch_meas = atan2f(-imu->acc_x_f,
                                  sqrtf(imu->acc_y_f * imu->acc_y_f +
                                       imu->acc_z_f * imu->acc_z_f));

        // Atualiza Kalman
        fusion->roll_kf = KALMAN_Update(&fusion->kf_roll,
                                        roll_meas,
                                        imu->gyro_x_f * M_PI / 180.0f,
                                        dt);

        fusion->pitch_kf = KALMAN_Update(&fusion->kf_pitch,
                                         pitch_meas,
                                         imu->gyro_y_f * M_PI / 180.0f,
                                         dt);

        fusion->yaw_kf = KALMAN_Update(&fusion->kf_yaw,
                                       imu->yaw_mag, // Magnetômetro
                                       imu->gyro_z_f * M_PI / 180.0f,
                                       dt);
    }
}

/* ========================================================================
 * FILTRO COMPLEMENTAR ADAPTATIVO
 * ======================================================================== */

void FUSION_ComplementaryFilter_Adaptive(IMU_Fusion *fusion, IMU *imu,
                                        ROCKET_FSM_t *fsm)
{
    uint8_t is_boost_phase = (fsm->current_state == FLIGHT_BOOST);

    // Alpha adaptativo: durante BOOST, confia MUITO mais no giroscópio
    float alpha = is_boost_phase ? 0.99f : fusion->alpha_complementary;

    float dt = imu->dt;

    // Integração do giroscópio
    float gyro_roll = fusion->roll_cf + (imu->gyro_x_f * M_PI / 180.0f * dt);
    float gyro_pitch = fusion->pitch_cf + (imu->gyro_y_f * M_PI / 180.0f * dt);
    float gyro_yaw = fusion->yaw_cf + (imu->gyro_z_f * M_PI / 180.0f * dt);

    if (!is_boost_phase) {
        // Acelerômetro confiável
        float roll_acc = atan2f(imu->acc_y_f, imu->acc_z_f);
        float pitch_acc = atan2f(-imu->acc_x_f,
                                sqrtf(imu->acc_y_f * imu->acc_y_f +
                                     imu->acc_z_f * imu->acc_z_f));

        fusion->roll_cf = alpha * gyro_roll + (1.0f - alpha) * roll_acc;
        fusion->pitch_cf = alpha * gyro_pitch + (1.0f - alpha) * pitch_acc;
        fusion->yaw_cf = alpha * gyro_yaw + (1.0f - alpha) * imu->yaw_mag;
    } else {
        // Durante BOOST: apenas giroscópio
        fusion->roll_cf = gyro_roll;
        fusion->pitch_cf = gyro_pitch;
        fusion->yaw_cf = gyro_yaw;
    }
}

/* ========================================================================
 * VALIDAÇÃO DE ATITUDE
 * ======================================================================== */

/**
 * @brief Verifica se estimativa de atitude é confiável
 *
 * @return 1 se confiável, 0 se suspeita
 */
uint8_t FUSION_IsAttitudeReliable(IMU_Fusion *fusion, IMU *imu, ROCKET_FSM_t *fsm)
{
    // Durante BOOST: atitude do giroscópio tem erro acumulado
    if (fsm->current_state == FLIGHT_BOOST) {
        return 0;  // Não confie plenamente
    }

    // Verifica magnitude de aceleração
    float acc_mag = sqrtf(imu->acc_x * imu->acc_x +
                          imu->acc_y * imu->acc_y +
                          imu->acc_z * imu->acc_z);

    // Se aceleração muito diferente de 1g, Madgwick pode estar errado
    if (acc_mag < 0.5f || acc_mag > 1.5f) {
        return 0;  // Não confie
    }

    // Verifica magnitude de taxa de rotação
    float gyro_mag = sqrtf(imu->gyro_x * imu->gyro_x +
                           imu->gyro_y * imu->gyro_y +
                           imu->gyro_z * imu->gyro_z);

    // Taxa de rotação acima de 500 °/s indica instabilidade
    if (gyro_mag > 500.0f) {
        return 0;  // Não confie
    }

    return 1;  // Confiável
}

/* ========================================================================
 * INTEGRAÇÃO NO IMU_UPDATE
 * ======================================================================== */

/**
 * @brief Replaces IMU_CalculateAttitude() with adaptive version
 *
 * Chamar no IMU_Update() ao invés da versão anterior
 */
void IMU_CalculateAttitude_Adaptive(IMU *imu, IMU_Fusion *fusion, ROCKET_FSM_t *fsm)
{
    if (!imu->is_valid || !fusion) {
        return;
    }

    static FlightState_t last_phase = FLIGHT_IDLE;

    // Detecta transição de fase
    if (fsm->current_state != last_phase) {

        // Saiu de BOOST
        if (last_phase == FLIGHT_BOOST &&
            fsm->current_state == FLIGHT_COAST) {
            printf("[IMU] Transição BOOST->COAST: Iniciando recuperação\r\n");
            // Reseta Madgwick para começar limpo
            QUAT_Init(&fusion->madgwick.q, 1.0f, 0.0f, 0.0f, 0.0f);
        }

        last_phase = fsm->current_state;
    }

    // Escolhe estratégia conforme a fase
    switch(fsm->current_state) {

        case FLIGHT_BOOST:
            // Apenas integração de giroscópio
            FUSION_Update_GyroOnly(fusion, imu);
            break;

        case FLIGHT_COAST:
            // Recuperação: transição para Madgwick
            FUSION_Recover_From_Boost(fusion, imu);
            break;

        default:
            // Resto das fases: Madgwick normal
            FUSION_Update_Adaptive(fusion, imu, fsm);
            break;
    }

    // Valida confiabilidade
    imu->attitude_reliable = FUSION_IsAttitudeReliable(fusion, imu, fsm);
}

