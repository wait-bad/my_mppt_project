#include "mean_processing.h"
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
uint16_t CalculateAverage(uint16_t arr[], int32_t size,uint8_t remove) {
    uint32_t sum = 0;

    for (int32_t i = remove; i < size-remove; ++i) {
        sum += arr[i];
    }
    return sum/(size-remove);
}

// �����������
uint16_t ProcessCurrentData(uint16_t current_data[], int32_t data_size) {
    // ִ�п�������
    QuickSort(current_data, 0, data_size - 1);

    // ����ȥ��ͷβ���ƽ��ֵ
    return CalculateAverage(current_data, data_size-1,5);
}
