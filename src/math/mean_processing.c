#include "current_processing.h"
#include <stdio.h>

// ��������ʵ��
void QuickSort(uint16_t arr[], int32_t low, int32_t high) {
    if (low < high) {
        // �ҵ�������
        int32_t pivotIndex = low + (high - low) / 2;
        uint16_t pivotValue = arr[pivotIndex];

        // �������Ϊ�����֣����ݹ�����
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
                // ����Ԫ��
                uint16_t temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;

                i++;
                j--;
            }
        }

        // �ݹ�������벿��
        QuickSort(arr, low, j);

        // �ݹ������Ұ벿��
        QuickSort(arr, i, high);
    }
}

// ������λ��
uint16_t Median(uint16_t arr[], int32_t size) {
    if (size % 2 == 0) {
        return (arr[size / 2 - 1] + arr[size / 2]) / 2;
    } else {
        return arr[size / 2];
    }
}

// ����ƽ��ֵ
double CalculateAverage(uint16_t arr[], int32_t size) {
    double sum = 0;

    for (int32_t i = 0; i < size; ++i) {
        sum += arr[i];
    }

    return sum / size;
}

// �����������
double ProcessCurrentData(uint16_t current_data[], int32_t data_size) {
    // ִ�п�������
    QuickSort(current_data, 0, data_size - 1);

    // ������Ҫȥ������������
    int32_t trim_count = (TRIM_PERCENTAGE * data_size) / 100;

    // ȥ��ͷβ����
    int32_t trimmed_size = data_size - (2 * trim_count);
    uint16_t trimmed_data[trimmed_size];

    for (int32_t i = trim_count; i < data_size - trim_count; ++i) {
        trimmed_data[i - trim_count] = current_data[i];
    }

    // ����ȥ��ͷβ���ƽ��ֵ
    return CalculateAverage(trimmed_data, trimmed_size);
}
