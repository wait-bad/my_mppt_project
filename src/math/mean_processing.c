#include "current_processing.h"
#include <stdio.h>

// 快速排序实现
void QuickSort(uint16_t arr[], int32_t low, int32_t high) {
    if (low < high) {
        // 找到分区点
        int32_t pivotIndex = low + (high - low) / 2;
        uint16_t pivotValue = arr[pivotIndex];

        // 将数组分为两部分，并递归排序
        int32_t i = low;
        int32_t j = high;

        while (i <= j) {
            while (arr[i] < pivotValue) {
                i++;
            }

            while (arr[j] > pivotValue) {
                j--;
            }

            if (i <= j) {
                // 交换元素
                uint16_t temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;

                i++;
                j--;
            }
        }

        // 递归排序左半部分
        QuickSort(arr, low, j);

        // 递归排序右半部分
        QuickSort(arr, i, high);
    }
}

// 计算中位数
uint16_t Median(uint16_t arr[], int32_t size) {
    if (size % 2 == 0) {
        return (arr[size / 2 - 1] + arr[size / 2]) / 2;
    } else {
        return arr[size / 2];
    }
}

// 计算平均值
double CalculateAverage(uint16_t arr[], int32_t size) {
    double sum = 0;

    for (int32_t i = 0; i < size; ++i) {
        sum += arr[i];
    }

    return sum / size;
}

// 处理电流数据
double ProcessCurrentData(uint16_t current_data[], int32_t data_size) {
    // 执行快速排序
    QuickSort(current_data, 0, data_size - 1);

    // 计算需要去除的数据数量
    int32_t trim_count = (TRIM_PERCENTAGE * data_size) / 100;

    // 去除头尾数据
    int32_t trimmed_size = data_size - (2 * trim_count);
    uint16_t trimmed_data[trimmed_size];

    for (int32_t i = trim_count; i < data_size - trim_count; ++i) {
        trimmed_data[i - trim_count] = current_data[i];
    }

    // 计算去除头尾后的平均值
    return CalculateAverage(trimmed_data, trimmed_size);
}
