#ifndef CURRENT_PROCESSING_H
#define CURRENT_PROCESSING_H

#include <stdint.h>

#define DATA_SIZE 30  // 数据数组大小
#define TRIM_PERCENTAGE 20  // 要去除的头尾数据的百分比

// 快速排序
void QuickSort(uint16_t arr[], int32_t low, int32_t high);

// 计算中位数
uint16_t Median(uint16_t arr[], int32_t size);

// 计算平均值
uint16_t CalculateAverage(uint16_t arr[], int32_t size,uint8_t remove);

// 处理电流数据
uint16_t ProcessCurrentData(uint16_t current_data[], int32_t data_size);

#endif  // CURRENT_PROCESSING_H
