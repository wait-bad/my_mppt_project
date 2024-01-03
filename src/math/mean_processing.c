#include "mean_processing.h"
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
uint16_t CalculateAverage(uint16_t arr[], int32_t size,uint8_t remove) {
    uint32_t sum = 0;

    for (int32_t i = remove; i < size-remove; ++i) {
        sum += arr[i];
    }
    return sum/(size-remove);
}

// 处理电流数据
uint16_t ProcessCurrentData(uint16_t current_data[], int32_t data_size) {
    // 执行快速排序
    QuickSort(current_data, 0, data_size - 1);

    // 计算去除头尾后的平均值
    return CalculateAverage(current_data, data_size-1,5);
}
