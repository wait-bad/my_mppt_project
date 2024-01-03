#ifndef CURRENT_PROCESSING_H
#define CURRENT_PROCESSING_H

#include <stdint.h>

#define DATA_SIZE 30  // ���������С
#define TRIM_PERCENTAGE 20  // Ҫȥ����ͷβ���ݵİٷֱ�

// ��������
void QuickSort(uint16_t arr[], int32_t low, int32_t high);

// ������λ��
uint16_t Median(uint16_t arr[], int32_t size);

// ����ƽ��ֵ
double CalculateAverage(uint16_t arr[], int32_t size);

// �����������
double ProcessCurrentData(uint16_t current_data[], int32_t data_size);

#endif  // CURRENT_PROCESSING_H
