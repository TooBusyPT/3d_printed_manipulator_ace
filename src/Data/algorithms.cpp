#include "algorithms.h"

std::vector<float> moving_average_filter(const std::vector<float>& input, int window_size) 
{
    std::vector<float> output(input.size(), 0.0f);
    int half_window = window_size / 2;

    for (int i = 0; i < input.size(); i++) 
	{
        float sum = 0.0f;
        int count = 0;
        for (int j = -half_window; j <= half_window; j++) 
		{
            if (i + j >= 0 && i + j < input.size()) 
			{
                sum += input[i + j];
                count++;
            }
        }
        output[i] = sum / count;
    }
    return output;
}

std::vector<int> find_local_minima(const std::vector<float>& data) 
{
    std::vector<int> minima_indices;
    for (int i = 1; i < data.size() - 1; i++) 
	{
        if (data[i] < data[i - 1] && data[i] < data[i + 1] && data[i] <= 240) 
		{
            minima_indices.push_back(i);
        }
    }
    return minima_indices;
}